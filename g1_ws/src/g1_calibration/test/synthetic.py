"""A synthetic G1 calibration rig with known ground truth, for the tests.

Frames (body = torso_link): the MID-360 and the head RealSense at their G1 URDF mounts, the OAK
on the chest at TRUE_BODY_OAK. Boards are held low in front of the robot, tilted towards the
head, so both cameras and the LiDAR see them (as in the real procedure).
"""
import math

import cv2
import numpy as np

from g1_calibration.board import Board
from g1_calibration.dataset import Intrinsics, write_sample
from g1_calibration.geometry import (T_BODY_OPTICAL, invert, rpy_to_matrix, transform_from_xyz_rpy,
                                     transform_points)

BOARD = Board(cols=8, rows=5, square=0.08, border=0.03)
BODY_LIDAR = transform_from_xyz_rpy([0.0002835, 0.00003, 0.428434],
                                    [math.pi, 0.05112069379091391, 0.0])
BODY_RS_OPT = transform_from_xyz_rpy([0.0576235, 0.01753, 0.42987],
                                     [0.0, 0.8307767239493009, 0.0]) @ T_BODY_OPTICAL
TRUE_BODY_OAK = transform_from_xyz_rpy([0.12, 0.01, 0.22], [0.01, 0.30, -0.02])  # oak root
TRUE_BODY_OAK_OPT = TRUE_BODY_OAK @ T_BODY_OPTICAL
OAK = Intrinsics(800, 600, np.array([[560.0, 0, 400.0], [0, 560.0, 300.0], [0, 0, 1]]),
                 np.zeros(5))
RS = Intrinsics(640, 480, np.array([[615.0, 0, 320.0], [0, 615.0, 240.0], [0, 0, 1]]),
                np.zeros(5))
FRAMES = {"oak": "oak_rgb_camera_optical_frame", "realsense": "camera_color_optical_frame"}
TEXTURE, H_BOARD_PX = BOARD.render(2000.0)


def config():
    return {
        "board": {"cols": BOARD.cols, "rows": BOARD.rows, "square": BOARD.square,
                  "border": BOARD.border},
        "cameras": {"oak": {"image": "/oak/rgb/image_raw", "camera_info": "/oak/rgb/camera_info"},
                    "realsense": {"image": "/camera/color/image_raw",
                                  "camera_info": "/camera/color/camera_info"}},
        "lidar": {"topic": "/utlidar/cloud_livox_mid360", "frame": "livox_frame", "seconds": 3.0,
                  "range_min": 0.3, "range_max": 6.0},
        "frames": {"body": "torso_link", "oak_root": "oak-d-base-frame"},
        "fallback_transforms": {"body_to_lidar": {"xyz": [0.0002835, 0.00003, 0.428434],
                                                  "rpy": [math.pi, 0.05112069379091391, 0.0]},
                                "body_to_realsense_optical": None},
        "initial_guess": {"body_to_oak_body": {"xyz": [0.10, 0.0, 0.25], "rpy": [0.0, 0.26, 0.0]}},
        "extrinsics": {"final": "auto", "max_reprojection_px": 1.0,
                       "lidar_search": [{"margin": 0.30, "depth_margin": 0.30},
                                        {"margin": 0.10, "depth_margin": 0.10},
                                        {"margin": 0.05, "depth_margin": 0.06}],
                       "plane_threshold": 0.03, "min_board_points": 40},
        "quality": {"reprojection_px": 0.5, "lidar_plane_rms_m": 0.025, "methods_agree_m": 0.02,
                    "methods_agree_deg": 1.0},
    }


def project(intr, t_cam_body, pts_body):
    p = transform_points(t_cam_body, pts_body)
    uv = p[:, :2] / p[:, 2:3] * [intr.k[0, 0], intr.k[1, 1]] + [intr.k[0, 2], intr.k[1, 2]]
    return uv, p[:, 2]


def visible(intr, t_cam_body, pts_body, margin=15):
    uv, z = project(intr, t_cam_body, pts_body)
    return bool((z > 0.3).all() and (uv[:, 0] > margin).all() and (uv[:, 1] > margin).all()
                and (uv[:, 0] < intr.width - margin).all()
                and (uv[:, 1] < intr.height - margin).all())


def board_poses(n, rng):
    """T_body_board for n boards seen by both cameras, with varied tilts."""
    x0, x1, y0, y1 = BOARD.extent()
    center_b = np.array([(x0 + x1) / 2, (y0 + y1) / 2, 0.0])
    # board x -> robot's right (-y), board y -> down (-z), board z -> forward (+x)
    base = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    poses, tries = [], 0
    while len(poses) < n:
        tries += 1
        if tries > 20000:
            raise RuntimeError(f"only {len(poses)} visible board poses in {tries} tries")
        # Tilt the face up towards the head cameras: the board normal (into the board) then
        # points forward and DOWN, along the cameras' viewing direction.
        tilt_up = rng.uniform(0.35, 0.95)
        yaw = rng.uniform(-0.45, 0.45)
        roll = rng.uniform(-0.3, 0.3)
        r = rpy_to_matrix(0.0, 0.0, yaw) @ rpy_to_matrix(0.0, tilt_up, 0.0) @ base \
            @ rpy_to_matrix(0.0, 0.0, roll)
        # centre 1.0-1.6 m ahead, 0.25-0.45 m above the floor (torso_link is ~0.85 m up): the
        # head RealSense looks ~48 deg down and only sees low boards
        c = np.array([rng.uniform(1.0, 1.6), rng.uniform(-0.3, 0.3), rng.uniform(-0.6, -0.4)])
        t = np.eye(4)
        t[:3, :3] = r
        t[:3, 3] = c - r @ center_b
        outline = transform_points(t, BOARD.outline())
        if (visible(OAK, invert(TRUE_BODY_OAK_OPT), outline)
                and visible(RS, invert(BODY_RS_OPT), outline)):
            poses.append(t)
    return poses


def render(intr, t_cam_board, rng):
    h = intr.k @ np.column_stack([t_cam_board[:3, 0], t_cam_board[:3, 1], t_cam_board[:3, 3]])
    img = cv2.warpPerspective(TEXTURE, h @ H_BOARD_PX, (intr.width, intr.height),
                              flags=cv2.INTER_AREA, borderValue=150)
    img = cv2.GaussianBlur(img, (3, 3), 0.7).astype(np.float64)
    img += rng.normal(0.0, 2.0, img.shape)
    return cv2.cvtColor(np.clip(img, 0, 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)


def lidar_points(t_body_board, rng, n_board=600, noise=0.012):
    x0, x1, y0, y1 = BOARD.extent()
    pb = np.column_stack([rng.uniform(x0, x1, n_board), rng.uniform(y0, y1, n_board),
                          np.zeros(n_board)])
    board = transform_points(t_body_board, pb)
    floor = np.column_stack([rng.uniform(0.3, 3.0, 3000), rng.uniform(-2, 2, 3000),
                             np.full(3000, -0.85)])
    wall = np.column_stack([np.full(2000, 3.0), rng.uniform(-2, 2, 2000),
                            rng.uniform(-0.85, 1.0, 2000)])
    pts = transform_points(invert(BODY_LIDAR), np.vstack([board, floor, wall]))
    r = np.linalg.norm(pts, axis=1, keepdims=True)
    return pts * (1.0 + rng.normal(0.0, noise, (len(pts), 1)) / r)  # range noise


def write_dataset(root, n=12, seed=0, with_tf=True, with_realsense=True):
    rng = np.random.default_rng(seed)
    for i, t_body_board in enumerate(board_poses(n, rng)):
        images = {"oak": render(OAK, invert(TRUE_BODY_OAK_OPT) @ t_body_board, rng)}
        infos = {"oak": OAK}
        if with_realsense:
            images["realsense"] = render(RS, invert(BODY_RS_OPT) @ t_body_board, rng)
            infos["realsense"] = RS
        tfs = []
        if with_tf:
            tfs = [("torso_link", "livox_frame", BODY_LIDAR),
                   ("livox_frame", FRAMES["realsense"], invert(BODY_LIDAR) @ BODY_RS_OPT),
                   ("oak-d-base-frame", FRAMES["oak"], T_BODY_OPTICAL)]
        write_sample(root, i, images, infos, {c: FRAMES[c] for c in images},
                     lidar_points(t_body_board, rng), "livox_frame", tfs)
    return root
