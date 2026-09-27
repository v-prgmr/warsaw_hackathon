"""3D bounding box of a detected object from its mask + aligned depth (no ROS).

Steps: mask pixels -> depth (robust band around the median, so background pixels at the mask
edge are dropped) -> 3D points in the camera optical frame -> (transformed to ``map`` by the
caller) -> a gravity-aligned box: yaw from the main horizontal direction of the points, extent
from percentiles. A single view only sees the object's front surface, so the horizontal extent
along the viewing direction is extended away from the camera to at least the object's width
(cups, bottles, boxes are about as deep as wide).
"""
import math

import numpy as np

from .backproject import _region_mask


def region_points(depth_raw, intr, box_xyxy, mask=None, depth_scale=0.001, min_depth_m=0.2,
                  max_depth_m=6.0, max_points=4000, band_min_m=0.10):
    """Nx3 points (optical frame: x right, y down, z forward) of the object, or None."""
    depth = np.asarray(depth_raw)
    h, w = depth.shape[:2]
    region = _region_mask((h, w), box_xyxy, mask)
    vs, us = np.nonzero(region)
    if len(us) == 0:
        return None
    if len(us) > max_points:
        pick = np.random.default_rng(0).choice(len(us), max_points, replace=False)
        us, vs = us[pick], vs[pick]
    z = depth[vs, us].astype(np.float64) * depth_scale
    ok = np.isfinite(z) & (z >= min_depth_m) & (z <= max_depth_m)
    us, vs, z = us[ok], vs[ok], z[ok]
    if len(z) < 10:
        return None
    med = float(np.median(z))
    mad = float(np.median(np.abs(z - med))) * 1.4826
    keep = np.abs(z - med) <= max(band_min_m, 3.0 * mad)
    us, vs, z = us[keep], vs[keep], z[keep]
    if len(z) < 10:
        return None
    x = (us - intr.cx) * z / intr.fx
    y = (vs - intr.cy) * z / intr.fy
    return np.column_stack([x, y, z])


def gravity_box(points, camera_xyz=None, lo=5.0, hi=95.0, min_size=0.03):
    """Box around map-frame points (z up): dict(center, size, yaw) or None.

    ``camera_xyz`` (map) enables the front-surface extension described in the module doc.
    """
    P = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if len(P) < 10:
        return None
    xy = P[:, :2] - P[:, :2].mean(axis=0)
    cov = np.cov(xy.T) if len(P) > 2 else np.eye(2)
    evals, evecs = np.linalg.eigh(cov)
    major = evecs[:, int(np.argmax(evals))]
    yaw = math.atan2(major[1], major[0])
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s], [s, c]])                      # box axes in map
    local = P[:, :2] @ R                                 # coordinates along the box axes
    mins = np.percentile(local, lo, axis=0)
    maxs = np.percentile(local, hi, axis=0)
    zmin, zmax = np.percentile(P[:, 2], lo), np.percentile(P[:, 2], hi)
    if camera_xyz is not None:
        view = P[:, :2].mean(axis=0) - np.asarray(camera_xyz, dtype=np.float64)[:2]
        n = np.linalg.norm(view)
        if n > 1e-6:
            view_local = (view / n) @ R                  # viewing direction in box axes
            k = int(np.argmax(np.abs(view_local)))       # axis most along the view
            other = 1 - k
            want = max(maxs[other] - mins[other], maxs[k] - mins[k])
            if view_local[k] > 0:                        # extend away from the camera
                maxs[k] = mins[k] + want
            else:
                mins[k] = maxs[k] - want
    size = np.maximum(np.array([maxs[0] - mins[0], maxs[1] - mins[1], zmax - zmin]), min_size)
    centre_local = (mins + maxs) / 2.0
    centre_xy = R @ centre_local
    return {"center": [float(centre_xy[0]), float(centre_xy[1]), float((zmin + zmax) / 2.0)],
            "size": [float(v) for v in size], "yaw": float(yaw)}


def transform_points(T, points):
    P = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    return P @ np.asarray(T)[:3, :3].T + np.asarray(T)[:3, 3]
