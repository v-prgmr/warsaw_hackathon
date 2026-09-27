"""Synthetic cameras and rendered AprilTags for the tests (no hardware, no ROS)."""
import math

import cv2
import numpy as np

from g1_ar_bridge.geometry import FLIP_YZ, inv_T, make_T, transform_points


def K_from_hfov(width, height, hfov_deg):
    f = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    return np.array([[f, 0.0, width / 2.0], [0.0, f, height / 2.0], [0.0, 0.0, 1.0]])


def look_at_cv(position, target, up):
    """OpenCV camera (x right, y down, z forward) at ``position`` looking at ``target``;
    ``up`` is the world's up axis. Returns T_world_cv."""
    p = np.asarray(position, float)
    z = np.asarray(target, float) - p
    z /= np.linalg.norm(z)
    x = np.cross(z, np.asarray(up, float))
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return make_T(np.column_stack([x, y, z]), p)


def glcam_from_cv(T_world_cv):
    """Lens (GL) camera pose from an OpenCV camera pose (FLIP_YZ is its own inverse)."""
    return T_world_cv @ FLIP_YZ


def tag_texture(tag_id=0, module_px=24):
    """Printed 36h11 tag: 8x8-module black square + 1-module white margin (10x10 modules)."""
    aruco = cv2.aruco
    d = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
    side = 8 * module_px
    if hasattr(aruco, "generateImageMarker"):
        img = aruco.generateImageMarker(d, tag_id, side, borderBits=1)
    else:
        img = aruco.drawMarker(d, tag_id, side, borderBits=1)
    return cv2.copyMakeBorder(img, module_px, module_px, module_px, module_px,
                              cv2.BORDER_CONSTANT, value=255)


def render_tag(K, width, height, T_cam_tag, black_size_m, tag_id=0, background=150,
               noise_sigma=2.0, seed=0, texture=None):
    """Grey image of a printed tag seen by a pinhole camera (OpenCV axes)."""
    tex = tag_texture(tag_id) if texture is None else texture
    s = tex.shape[0]
    half = black_size_m / 0.8 / 2.0                     # printed (outer) half size
    obj = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]])
    pc = transform_points(T_cam_tag, obj)
    if np.any(pc[:, 2] <= 0.05):
        raise ValueError("tag behind the camera")
    uv = np.stack([K[0, 0] * pc[:, 0] / pc[:, 2] + K[0, 2],
                   K[1, 1] * pc[:, 1] / pc[:, 2] + K[1, 2]], axis=1).astype(np.float32)
    # pixel centres are integers in OpenCV: the texture's outer edge is at -0.5 and s - 0.5
    src = np.array([[0, 0], [s, 0], [s, s], [0, s]], dtype=np.float32) - 0.5
    H = cv2.getPerspectiveTransform(src, uv)
    warped = cv2.warpPerspective(tex, H, (width, height), flags=cv2.INTER_LINEAR,
                                 borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    mask = cv2.warpPerspective(np.full_like(tex, 255), H, (width, height),
                               flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                               borderValue=0).astype(np.float32) / 255.0
    rng = np.random.default_rng(seed)
    img = background * (1 - mask) + warped.astype(np.float32) * mask
    img += rng.normal(0.0, noise_sigma, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def render_depth(K, width, height, T_cam_plane, noise_m=0.002, seed=0):
    """Metric depth (float32 m) of an infinite plane z=0 of frame ``plane`` (e.g. the wall)."""
    R, t = T_cam_plane[:3, :3], T_cam_plane[:3, 3]
    n = R[:, 2]                                         # plane normal in the camera frame
    d = float(n @ t)
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    rays = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u, float)],
                    axis=-1)
    denom = rays @ n
    with np.errstate(divide="ignore", invalid="ignore"):
        z = d / denom
    z[(denom == 0) | (z <= 0)] = np.nan
    rng = np.random.default_rng(seed)
    return (z + rng.normal(0.0, noise_m, z.shape)).astype(np.float32)


def wall_tag_in_map(distance=1.5, height_above_floor=1.0, floor_z=-0.78, lateral=0.0):
    """Tag on a wall ``distance`` m in front of the map origin, facing back towards it.

    Tag axes (ArUco): x right as seen by the viewer (= -y map), y up (= +z map), z towards the
    viewer (= -x map).
    """
    R = np.column_stack([[0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]])
    return make_T(R, [distance, lateral, floor_z + height_above_floor])


def cam_tag(T_world_cv, T_world_tag):
    return inv_T(T_world_cv) @ T_world_tag
