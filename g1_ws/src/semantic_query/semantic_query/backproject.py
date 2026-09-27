"""Lift a 2D detection into a 3D point in the camera optical frame using aligned depth.

Aligned depth means depth pixel (u, v) corresponds to RGB pixel (u, v), so the detector's box /
mask indexes straight into the depth image. We take a robust (median) depth over the mask (or the
box centre region), reject invalid/out-of-range samples, and apply the pinhole model.
"""
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_k(cls, k):
        # k is the row-major 3x3 CameraInfo.k (9 floats)
        return cls(fx=float(k[0]), fy=float(k[4]), cx=float(k[2]), cy=float(k[5]))


def _region_mask(shape, box_xyxy, mask):
    """Boolean HxW sample region: the SAM2 mask if given, else the central half of the box."""
    h, w = shape
    if mask is not None:
        m = np.asarray(mask, dtype=bool)
        if m.shape == (h, w) and m.any():
            return m
    x0, y0, x1, y1 = box_xyxy
    # shrink to the central 50% of the box to avoid edges / background
    bx = (x1 - x0) * 0.25
    by = (y1 - y0) * 0.25
    x0i, x1i = int(round(x0 + bx)), int(round(x1 - bx))
    y0i, y1i = int(round(y0 + by)), int(round(y1 - by))
    x0i, x1i = max(0, x0i), min(w, max(x0i + 1, x1i))
    y0i, y1i = max(0, y0i), min(h, max(y0i + 1, y1i))
    m = np.zeros((h, w), dtype=bool)
    m[y0i:y1i, x0i:x1i] = True
    return m


def backproject(depth_raw, intr: CameraIntrinsics, box_xyxy, mask=None,
                depth_scale: float = 0.001, min_depth_m: float = 0.2,
                max_depth_m: float = 6.0) -> Optional[Tuple[float, float, float]]:
    """Return (x, y, z) in the optical frame, or None if there is no valid depth.

    depth_raw: HxW array (e.g. uint16 mm). depth_scale converts to metres.
    """
    depth = np.asarray(depth_raw)
    h, w = depth.shape[:2]
    region = _region_mask((h, w), box_xyxy, mask)

    d = depth[region].astype(np.float64) * depth_scale
    d = d[np.isfinite(d)]
    d = d[(d >= min_depth_m) & (d <= max_depth_m)]
    if d.size == 0:
        return None
    z = float(np.median(d))

    # pixel centroid of the sample region (use mask centroid, not box centre, for masks)
    ys, xs = np.nonzero(region)
    u = float(xs.mean())
    v = float(ys.mean())

    x = (u - intr.cx) * z / intr.fx
    y = (v - intr.cy) * z / intr.fy
    return (x, y, z)
