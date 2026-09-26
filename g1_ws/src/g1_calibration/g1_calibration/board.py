"""Chessboard model, detection with a canonical corner order, and board pose (PnP).

Board frame (also used by the printed board from `make_board`): origin at the first inner corner,
x along the `cols` inner corners, y along the `rows` inner corners, z = x × y pointing INTO the
board (away from a camera that looks at its printed side). The square diagonal to the origin,
outside the inner-corner grid, is dark; so is square (0, 0) between the first four corners.

Canonical order: OpenCV may return the corners of a chessboard starting from either end. With
`cols + rows` odd the pattern is not symmetric under a 180° rotation, so the colour of square
(0, 0) tells the two orders apart. Two cameras that see the same board then agree on its frame,
which camera-to-camera calibration needs.
"""
from dataclasses import dataclass
import math

import cv2
import numpy as np

from .geometry import make_transform

SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)


@dataclass
class Board:
    cols: int            # inner corners along the board x axis
    rows: int            # inner corners along the board y axis
    square: float        # square edge length in metres (measure the print!)
    border: float = 0.0  # white margin around the squares, metres (for the LiDAR extent)

    def __post_init__(self):
        if self.cols < 3 or self.rows < 3:
            raise ValueError("the board needs at least 3 x 3 inner corners")
        if (self.cols + self.rows) % 2 == 0:
            raise ValueError(
                f"board {self.cols}x{self.rows} inner corners is symmetric under a 180° "
                "rotation: its corner order is ambiguous. Use one odd and one even count "
                "(e.g. 8x5 or 9x6 inner corners).")
        if self.square <= 0:
            raise ValueError("square must be > 0 m")

    @classmethod
    def from_config(cls, cfg):
        return cls(int(cfg["cols"]), int(cfg["rows"]), float(cfg["square"]),
                   float(cfg.get("border", 0.0)))

    @property
    def pattern_size(self):
        return (self.cols, self.rows)

    def object_points(self):
        """Inner corners in the board frame, OpenCV order (row by row), (N, 3) float32."""
        xs, ys = np.meshgrid(np.arange(self.cols), np.arange(self.rows))
        pts = np.stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)], axis=1) * self.square
        return pts.astype(np.float32)

    def extent(self):
        """(xmin, xmax, ymin, ymax) of the physical board (squares + border) in the board frame."""
        s, b = self.square, self.border
        return (-s - b, self.cols * s + b, -s - b, self.rows * s + b)

    def outline(self):
        """The four physical corners of the board, (4, 3), board frame."""
        x0, x1, y0, y1 = self.extent()
        return np.array([[x0, y0, 0.0], [x1, y0, 0.0], [x1, y1, 0.0], [x0, y1, 0.0]])

    def render(self, px_per_m):
        """Grey image of the physical board: (image, H_board_from_pixel 3x3)."""
        x0, x1, y0, y1 = self.extent()
        w, h = int(round((x1 - x0) * px_per_m)), int(round((y1 - y0) * px_per_m))
        u, v = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
        x, y = u / px_per_m + x0, v / px_per_m + y0
        s = self.square
        inside = (x >= -s) & (x < self.cols * s) & (y >= -s) & (y < self.rows * s)
        dark = ((np.floor(x / s) + np.floor(y / s)) % 2 == 0) & inside
        img = np.where(dark, 0, 255).astype(np.uint8)
        h_board_px = np.array([[1.0 / px_per_m, 0.0, x0], [0.0, 1.0 / px_per_m, y0],
                               [0.0, 0.0, 1.0]])
        return img, h_board_px


def to_gray(image):
    if image.ndim == 2:
        return image
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def detect_corners(image, board):
    """Inner corners in canonical order, (N, 2) float64 pixels, or None if not found."""
    gray = to_gray(image)
    found, corners = False, None
    if hasattr(cv2, "findChessboardCornersSB"):
        found, corners = cv2.findChessboardCornersSB(
            gray, board.pattern_size, flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if not found:
        found, corners = cv2.findChessboardCorners(
            gray, board.pattern_size,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if found:
            corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), SUBPIX_CRITERIA)
    if not found:
        return None
    return canonical_order(gray, corners.reshape(-1, 2).astype(np.float64), board)


def _square_mean(gray, h_img_board, board, i, j):
    """Mean intensity of the inner half of square (i, j) (x from i*s to (i+1)*s)."""
    s = board.square
    a = np.linspace(0.3, 0.7, 5)
    gx, gy = np.meshgrid((i + a) * s, (j + a) * s)
    pts = cv2.perspectiveTransform(np.stack([gx.ravel(), gy.ravel()], 1)[None], h_img_board)[0]
    u = np.clip(np.round(pts[:, 0]).astype(int), 0, gray.shape[1] - 1)
    v = np.clip(np.round(pts[:, 1]).astype(int), 0, gray.shape[0] - 1)
    return float(gray[v, u].mean())


def canonical_order(gray, corners, board):
    """Reorder detected corners so the board frame is right-handed (front view) and square
    (0, 0) is dark."""
    g = corners.reshape(board.rows, board.cols, 2).copy()
    x_dir, y_dir = g[0, -1] - g[0, 0], g[-1, 0] - g[0, 0]
    if x_dir[0] * y_dir[1] - x_dir[1] * y_dir[0] < 0:  # mirrored order: flip x
        g = g[:, ::-1]
    obj = board.object_points()[:, :2]
    h, _ = cv2.findHomography(obj, g.reshape(-1, 2))
    if _square_mean(gray, h, board, 0, 0) > _square_mean(gray, h, board, 1, 0):
        g = g[::-1, ::-1]  # started from the other end: rotate 180°
    return g.reshape(-1, 2)


def board_pose(corners, board, k, d):
    """T_camera_board from canonical corners; returns (T, reprojection RMS px)."""
    obj = board.object_points().astype(np.float64)
    img = np.asarray(corners, dtype=np.float64).reshape(-1, 1, 2)
    k, d = np.asarray(k, dtype=np.float64), np.asarray(d, dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, img, k, d, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        raise RuntimeError("solvePnP failed")
    rvec, tvec = cv2.solvePnPRefineLM(obj, img, k, d, rvec, tvec)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, k, d)
    rms = float(np.sqrt(np.mean(np.sum((proj - img) ** 2, axis=2))))
    tf = make_transform(cv2.Rodrigues(rvec)[0], tvec.ravel())
    return tf, rms


def board_plane(t_cam_board):
    """Board plane in the camera frame: unit normal n (board z, away from the camera) and
    distance d with n · x = d for points x on the board."""
    n = t_cam_board[:3, 2].copy()
    d = float(n @ t_cam_board[:3, 3])
    if d < 0:  # the camera is behind the board: keep the "away from the sensor" convention
        n, d = -n, -d
    return n, d


def view_angle_deg(t_cam_board):
    """Angle between the camera's optical axis and the board normal (0 = frontal)."""
    n = t_cam_board[:3, 2]
    return math.degrees(math.acos(min(1.0, abs(float(n[2])))))


def draw(image, corners, board, ok_text=None):
    out = image.copy() if image.ndim == 3 else cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if corners is not None:
        cv2.drawChessboardCorners(out, board.pattern_size,
                                  corners.reshape(-1, 1, 2).astype(np.float32), True)
        cv2.circle(out, tuple(int(v) for v in corners[0]), 8, (0, 0, 255), 2)  # origin
    if ok_text:
        cv2.putText(out, ok_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 200, 0) if corners is not None else (0, 0, 255), 2)
    return out
