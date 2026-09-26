"""AprilTag (36h11) detection and pose estimation with OpenCV, for any camera.

The same code measures the tag from the robot camera (OAK-D / RealSense, in ``map``) and from
the Spectacles camera frames (in the AR world), so both sides share one tag convention:

* tag frame = OpenCV ArUco: origin at the tag centre, x right, y up (as printed), z out of
  the tag towards the viewer.
* ``size_m`` = edge of the **black square** (what the detector finds). For 36h11 printed with its
  1-module white margin the black square is 80 % of the printed edge (upstream ``g1.py``).

Works with OpenCV 4.5 (Ubuntu 22.04 / ROS Humble, old ``cv2.aruco`` API) up to 5.x.
"""
from dataclasses import dataclass

import cv2
import numpy as np

from .geometry import inv_T, make_T, so3_exp, transform_points

TAG_DICTIONARY = "DICT_APRILTAG_36h11"


class TagDetector:
    """Detects AprilTags; returns ``[(tag_id, corners (4, 2) float64 px)]`` in ArUco corner
    order (top-left, top-right, bottom-right, bottom-left of the printed tag)."""

    def __init__(self, dictionary=TAG_DICTIONARY, subpix=True, refine_edges=True):
        self.refine_edges = refine_edges
        aruco = cv2.aruco
        self._dict = aruco.getPredefinedDictionary(getattr(aruco, dictionary))
        create = getattr(aruco, "DetectorParameters_create", None)
        params = create() if create is not None else aruco.DetectorParameters()
        if subpix:
            params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self._detector = None
        if hasattr(aruco, "ArucoDetector"):
            self._detector = aruco.ArucoDetector(self._dict, params)
        else:
            # OpenCV < 4.7 (Ubuntu 22.04's 4.5.4): with a thin white margin (trimmed print, dark
            # wall) its close-candidate filter drops the real black square. Keep all candidates;
            # detect() then keeps the largest square per id, which is the black border.
            params.minMarkerDistanceRate = 0.01
        self._params = params

    def detect(self, gray):
        if gray is None or gray.size == 0:
            return []
        if gray.ndim == 3:
            gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self._dict, parameters=self._params)
        if ids is None or len(ids) == 0:
            return []
        best = {}
        for c, i in zip(corners, np.asarray(ids).reshape(-1)):
            c = np.asarray(c, dtype=np.float64).reshape(4, 2)
            size = tag_side_px(c)
            if int(i) not in best or size > best[int(i)][0]:
                best[int(i)] = (size, c)
        out = [(i, c) for i, (_, c) in sorted(best.items())]
        if self.refine_edges:
            img = cv2.GaussianBlur(gray, (3, 3), 0.8).astype(np.float32)
            out = [(i, refine_corners_by_edges(img, c)) for i, c in out]
        return out


def _bilinear(img, pts):
    """Sample a float image at (..., 2) x/y positions (clamped to the border)."""
    h, w = img.shape
    x = np.clip(pts[..., 0], 0.0, w - 1.001)
    y = np.clip(pts[..., 1], 0.0, h - 1.001)
    x0, y0 = np.floor(x).astype(np.int64), np.floor(y).astype(np.int64)
    fx, fy = x - x0, y - y0
    a, b = img[y0, x0], img[y0, x0 + 1]
    c, d = img[y0 + 1, x0], img[y0 + 1, x0 + 1]
    return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy


def refine_corners_by_edges(img, corners, step=0.25):
    """Corners as intersections of the four black-square edges, each fitted to sub-pixel edge
    points (strongest dark-to-bright step along the outward normal), like AprilTag's own edge
    refinement. Makes the corners independent of the OpenCV version's corner refiner (4.5's
    is biased by ~0.2 px). Returns the input unchanged when the fit is not trustworthy."""
    c = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    side = tag_side_px(c)
    if side < 16:
        return c
    centre = c.mean(axis=0)
    # stay within half a module (side / 8 per module) so data cells never enter the search
    r = float(np.clip(side / 16.0, 1.5, 6.0))
    offsets = np.arange(-r, r + 1e-9, step)
    lines = []
    for i in range(4):
        p0, p1 = c[i], c[(i + 1) % 4]
        d = p1 - p0
        length = float(np.linalg.norm(d))
        t = d / length
        n = np.array([-t[1], t[0]])
        if np.dot(n, (p0 + p1) / 2.0 - centre) < 0:
            n = -n                                      # outward: black inside, white outside
        s = np.linspace(0.12 * length, 0.88 * length, max(8, int(length / 2)))
        base = p0 + s[:, None] * t
        probe = base[:, None, :] + offsets[None, :, None] * n
        prof = _bilinear(img, probe)
        # edge = where the profile crosses half-way between the black and the white level
        # (unbiased for a symmetric blur; robust to the piecewise-linear bilinear profile)
        nq = max(2, len(offsets) // 5)
        dark, bright = prof[:, :nq].mean(axis=1), prof[:, -nq:].mean(axis=1)
        mid = (dark + bright) / 2.0
        above = prof >= mid[:, None]
        cross = above[:, 1:] & ~above[:, :-1]
        j = np.argmax(cross, axis=1)
        rows = np.arange(len(j))
        a, b = prof[rows, j], prof[rows, j + 1]
        frac = np.where(b > a, (mid - a) / np.maximum(b - a, 1e-9), 0.5)
        pos = offsets[j] + np.clip(frac, 0.0, 1.0) * step
        contrast = bright - dark
        ok = (cross.any(axis=1) & (cross.sum(axis=1) == 1) & (contrast > 10.0)
              & (contrast > 0.5 * np.median(contrast)))
        if ok.sum() < 5:
            return c
        pts = base[ok] + pos[ok, None] * n
        mean = pts.mean(axis=0)
        _, _, vt = np.linalg.svd(pts - mean, full_matrices=False)
        lines.append((mean, vt[0]))
    out = np.empty_like(c)
    for i in range(4):
        (a, u), (b, v) = lines[(i - 1) % 4], lines[i]
        A = np.array([u, -v]).T
        if abs(np.linalg.det(A)) < 1e-6:
            return c
        t1, _ = np.linalg.solve(A, b - a)
        out[i] = a + t1 * u
    if np.max(np.linalg.norm(out - c, axis=1)) > max(1.5, 0.05 * side):
        return c
    return out


def tag_object_points(size_m):
    h = size_m / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])


def project(K, T_cam_world, points):
    """Pinhole projection (no distortion) of (N, 3) world points; returns (N, 2) and depths."""
    pc = transform_points(T_cam_world, points)
    z = pc[:, 2]
    zs = np.where(z > 1e-6, z, 1e-6)
    uv = np.stack([K[0, 0] * pc[:, 0] / zs + K[0, 2], K[1, 1] * pc[:, 1] / zs + K[1, 2]], axis=1)
    return uv, z


def rvec_tvec_to_T(rvec, tvec):
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    return make_T(R, np.asarray(tvec, dtype=np.float64).reshape(3))


@dataclass
class TagPose:
    T_cam_tag: np.ndarray      # OpenCV camera <- tag
    reproj_px: float           # RMS corner reprojection error of the chosen solution
    ambiguity: float           # 2nd-best / best reprojection error (>= 1; ~1 = ambiguous)


def estimate_tag_pose(corners, size_m, K, dist=None):
    """Single-view tag pose with IPPE (both square-planar solutions tried, best kept)."""
    obj = tag_object_points(size_m).astype(np.float32)
    img = np.asarray(corners, dtype=np.float32).reshape(4, 1, 2)
    d = np.zeros(5) if dist is None or len(dist) == 0 else np.asarray(dist, dtype=np.float64)
    try:
        ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, img, np.asarray(K, dtype=np.float64), d,
                                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    except cv2.error:
        return None
    if not ok or len(rvecs) == 0:
        return None
    cands = []
    for rv, tv in zip(rvecs, tvecs):
        T = rvec_tvec_to_T(rv, tv)
        if T[2, 3] <= 0:
            continue
        uv, _ = project(np.asarray(K, dtype=np.float64), T, tag_object_points(size_m))
        err = float(np.sqrt(np.mean(np.sum((uv - np.asarray(corners).reshape(4, 2)) ** 2, 1))))
        cands.append((err, T))
    if not cands:
        return None
    cands.sort(key=lambda c: c[0])
    best_err, best_T = cands[0]
    amb = cands[1][0] / max(best_err, 1e-9) if len(cands) > 1 else float("inf")
    return TagPose(best_T, best_err, max(1.0, amb))


def tag_side_px(corners):
    c = np.asarray(corners).reshape(4, 2)
    return float(np.mean(np.linalg.norm(c - np.roll(c, 1, axis=0), axis=1)))


# --- multi-view refinement ---------------------------------------------------------------


def _residuals(views, obj, R0, t0, x):
    R = R0 @ so3_exp(x[:3])
    t = t0 + x[3:]
    pts_w = obj @ R.T + t
    res = []
    for K, T_world_cv, corners in views:
        uv, z = project(K, inv_T(T_world_cv), pts_w)
        r = (uv - corners).reshape(-1)
        if np.any(z <= 1e-3):
            r = r + 1e3
        res.append(r)
    return np.concatenate(res)


def refine_tag_pose(views, size_m, T_world_tag0, iterations=20, huber_px=1.5):
    """Refine a static tag's pose from several views with known camera poses.

    ``views`` = ``[(K 3x3, T_world_cv 4x4 (OpenCV camera -> world), corners (4, 2))]``.
    Minimises the corner reprojection error over all views (Levenberg-Marquardt, Huber),
    which triangulates the tag when the camera moved (Spectacles) and averages the pixel
    noise when it did not (robot standing). Returns ``(T_world_tag, rms_px, per_view_rms)``.
    """
    obj = tag_object_points(size_m)
    views = [(np.asarray(K, float), np.asarray(T, float), np.asarray(c, float).reshape(4, 2))
             for K, T, c in views]
    R0, t0 = T_world_tag0[:3, :3].copy(), T_world_tag0[:3, 3].copy()
    x = np.zeros(6)
    lam = 1e-3

    def cost(r):
        a = np.abs(r)
        return float(np.sum(np.where(a <= huber_px, 0.5 * a * a, huber_px * (a - 0.5 * huber_px))))

    r = _residuals(views, obj, R0, t0, x)
    c = cost(r)
    eps = np.array([1e-6] * 3 + [1e-6] * 3)
    for _ in range(iterations):
        J = np.empty((r.size, 6))
        for k in range(6):
            dx = np.zeros(6)
            dx[k] = eps[k]
            J[:, k] = (_residuals(views, obj, R0, t0, x + dx)
                       - _residuals(views, obj, R0, t0, x - dx)) / (2 * eps[k])
        a = np.abs(r)
        w = np.where(a <= huber_px, 1.0, huber_px / np.maximum(a, 1e-12))
        A = J.T @ (w[:, None] * J)
        g = J.T @ (w * r)
        improved = False
        for _ in range(8):
            try:
                step = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-9), -g)
            except np.linalg.LinAlgError:
                lam *= 10
                continue
            r_new = _residuals(views, obj, R0, t0, x + step)
            c_new = cost(r_new)
            if c_new < c:
                x, r, c = x + step, r_new, c_new
                lam = max(lam * 0.3, 1e-9)
                improved = True
                break
            lam *= 10
        if not improved or np.linalg.norm(step) < 1e-9:
            break
    T = make_T(R0 @ so3_exp(x[:3]), t0 + x[3:])
    per_view = [float(np.sqrt(np.mean(r[8 * i:8 * i + 8] ** 2))) for i in range(len(views))]
    return T, float(np.sqrt(np.mean(r ** 2))), per_view


# --- depth (robot RGB-D camera): the tag plane from aligned depth ------------------------


def quad_pixels(corners, shape, scale=1.3):
    """(rows, cols) of the pixels inside the tag quad grown by ``scale`` about its centre
    (1.25 covers the printed white margin, which lies on the same plane)."""
    c = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    centre = c.mean(axis=0)
    grown = centre + (c - centre) * scale
    mask = np.zeros(shape[:2], dtype=np.uint8)
    cv2.fillConvexPoly(mask, np.round(grown).astype(np.int32), 1)
    return np.nonzero(mask)


def depth_to_metres(depth, encoding):
    """Depth image as float32 metres (16UC1 = mm, 32FC1 = m); invalid -> nan."""
    d = np.asarray(depth)
    if encoding in ("16UC1", "mono16") or d.dtype == np.uint16:
        out = d.astype(np.float32) / 1000.0
    else:
        out = d.astype(np.float32)
    out[~np.isfinite(out) | (out <= 0.0)] = np.nan
    return out


def backproject(depth_m, K, rows, cols, max_range_m=6.0):
    z = depth_m[rows, cols]
    ok = np.isfinite(z) & (z > 0.05) & (z < max_range_m)
    z, u, v = z[ok], cols[ok].astype(np.float64), rows[ok].astype(np.float64)
    x = (u - K[0, 2]) * z / K[0, 0]
    y = (v - K[1, 2]) * z / K[1, 1]
    return np.stack([x, y, z], axis=1)


def fit_plane(points, inlier_m=0.01, iterations=4, min_points=50):
    """Least-squares plane with iterative outlier rejection.

    Returns ``(unit normal, point on plane, rms_m, n_inliers)`` or ``None``.
    """
    P = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if len(P) < min_points:
        return None
    keep = np.ones(len(P), dtype=bool)
    for _ in range(iterations):
        Q = P[keep]
        if len(Q) < min_points:
            return None
        c = Q.mean(axis=0)
        _, _, vt = np.linalg.svd(Q - c, full_matrices=False)
        n = vt[2]
        d = np.abs((P - c) @ n)
        thr = max(inlier_m, 2.5 * float(np.median(d[keep])) * 1.4826)
        new_keep = d <= thr
        if new_keep.sum() < min_points:
            return None
        if np.array_equal(new_keep, keep):
            break
        keep = new_keep
    Q = P[keep]
    c = Q.mean(axis=0)
    _, _, vt = np.linalg.svd(Q - c, full_matrices=False)
    n = vt[2]
    rms = float(np.sqrt(np.mean(((Q - c) @ n) ** 2)))
    return n / np.linalg.norm(n), c, rms, int(keep.sum())


def snap_tag_to_plane(T_world_tag, normal, plane_point, cam_centre, max_angle_deg=15.0):
    """Correct a tag pose with a measured plane: tag +z becomes the plane normal (minimal
    rotation, in-plane rotation kept) and the centre moves along the camera ray onto the plane.

    Returns ``(T_new or None, angle_deg)``; ``None`` when PnP and depth disagree by more than
    ``max_angle_deg`` (the depth plane is then not the tag's, e.g. a wall edge).
    """
    n = np.asarray(normal, dtype=np.float64)
    c = np.asarray(cam_centre, dtype=np.float64)
    p0 = np.asarray(plane_point, dtype=np.float64)
    if np.dot(n, c - p0) < 0:
        n = -n
    z_old = T_world_tag[:3, 2]
    cosang = float(np.clip(np.dot(z_old, n), -1.0, 1.0))
    angle = float(np.degrees(np.arccos(cosang)))
    if angle > max_angle_deg:
        return None, angle
    axis = np.cross(z_old, n)
    s = float(np.linalg.norm(axis))
    R_fix = np.eye(3) if s < 1e-12 else so3_exp(axis / s * np.arctan2(s, cosang))
    R_new = R_fix @ T_world_tag[:3, :3]
    p = T_world_tag[:3, 3]
    d = (p - c) / np.linalg.norm(p - c)
    denom = float(np.dot(n, d))
    if abs(denom) < 1e-6:
        return None, angle
    p_new = c + d * float(np.dot(n, p0 - c)) / denom
    return make_T(R_new, p_new), angle
