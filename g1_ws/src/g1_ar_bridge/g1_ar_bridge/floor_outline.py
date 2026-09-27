"""Wall outlines of a 2D occupancy grid, as polylines on the floor (no ROS).

The AR glasses draw at most ~1500 LiDAR points per frame; lines are cheap for them. Tracing the
occupied cells of the RTAB-Map /map into a few simplified polylines gives a clear room outline on
the floor: occupied cells -> small closing (joins gaps) -> contours -> Douglas-Peucker
simplification -> the longest ones, capped in count and points.
"""
import math

import cv2
import numpy as np


def outline_polylines(data, width, height, resolution, origin_xy, origin_yaw=0.0,
                      occupied=65, simplify_m=0.05, min_length_m=0.4, max_lines=60,
                      max_points=1200):
    """Polylines ((N, 2) arrays, map x/y in metres, closed ones repeat their first point)."""
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)
    occ = (grid >= occupied).astype(np.uint8) * 255
    if not occ.any():
        return []
    occ = cv2.morphologyEx(occ, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    found = cv2.findContours(occ, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = found[0] if len(found) == 2 else found[1]          # OpenCV 4 / 3
    eps = max(simplify_m / resolution, 0.5)
    lines = []
    for c in contours:
        length = cv2.arcLength(c, True) * resolution
        if length < min_length_m:
            continue
        poly = cv2.approxPolyDP(c, eps, True).reshape(-1, 2).astype(np.float64)
        if len(poly) < 2:
            continue
        poly = np.vstack([poly, poly[:1]])                         # closed
        lines.append((length, poly))
    lines.sort(key=lambda item: -item[0])
    c, s = math.cos(origin_yaw), math.sin(origin_yaw)
    out, total = [], 0
    for _, poly in lines[:max_lines]:
        if total + len(poly) > max_points:
            break
        total += len(poly)
        gx = (poly[:, 0] + 0.5) * resolution                      # cell centres, grid frame
        gy = (poly[:, 1] + 0.5) * resolution
        out.append(np.column_stack([origin_xy[0] + c * gx - s * gy,
                                    origin_xy[1] + s * gx + c * gy]))
    return out


def signature(data):
    """Cheap change detector for an occupancy grid."""
    arr = np.asarray(data, dtype=np.int16)
    return (arr.size, int((arr >= 65).sum()), int(np.flatnonzero(arr >= 65)[:: 97].sum()))
