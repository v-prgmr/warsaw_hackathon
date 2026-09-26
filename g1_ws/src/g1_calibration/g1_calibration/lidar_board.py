"""Find the calibration board in an accumulated LiDAR cloud.

The MID-360's non-repetitive scan pattern fills in a static board over a few seconds, so each
sample accumulates ~2-3 s of scans while the robot and the board stand still. The board is cut
out around its PREDICTED pose (from the current extrinsic estimate and the camera's board pose),
then a RANSAC plane is fitted and refined. The predicted normal rejects walls and the floor.
"""
import math

import numpy as np

from .geometry import invert, transform_points


def fit_plane(points):
    """Least-squares plane: unit normal n, distance d (n · x = d, d >= 0: n points away from the
    sensor origin), RMS point-to-plane distance."""
    c = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - c, full_matrices=False)
    n = vt[2]
    d = float(n @ c)
    if d < 0:
        n, d = -n, -d
    rms = float(np.sqrt(np.mean((points @ n - d) ** 2)))
    return n, d, rms


def ransac_plane(points, threshold, iterations=300, rng=None):
    """Inlier mask of the best plane (most points within `threshold` metres)."""
    rng = rng if rng is not None else np.random.default_rng(0)
    best = np.zeros(len(points), dtype=bool)
    if len(points) < 3:
        return best
    for _ in range(iterations):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        mask = np.abs((points - a) @ n) < threshold
        if mask.sum() > best.sum():
            best = mask
    return best


def extract_board(points, t_lidar_board, board, margin=0.10, depth_margin=0.15,
                  threshold=0.03, min_points=30, max_normal_angle_deg=25.0, rng=None):
    """Board points around the predicted pose t_lidar_board.

    Returns dict(points, n, d, rms, n_crop) in the LiDAR frame, or None with a reason string in
    dict(error=...).
    """
    pts_b = transform_points(invert(t_lidar_board), points)
    x0, x1, y0, y1 = board.extent()
    sel = ((pts_b[:, 0] > x0 - margin) & (pts_b[:, 0] < x1 + margin)
           & (pts_b[:, 1] > y0 - margin) & (pts_b[:, 1] < y1 + margin)
           & (np.abs(pts_b[:, 2]) < depth_margin))
    crop = points[sel]
    if len(crop) < min_points:
        return {"error": f"only {len(crop)} LiDAR points near the predicted board pose"}
    mask = ransac_plane(crop, threshold, rng=rng)
    n, d, _ = fit_plane(crop[mask])
    mask = np.abs(crop @ n - d) < threshold  # refine once with the least-squares plane
    if mask.sum() < min_points:
        return {"error": f"only {mask.sum()} LiDAR points on the board plane"}
    inliers = crop[mask]
    n, d, rms = fit_plane(inliers)
    expected = t_lidar_board[:3, 2]
    angle = math.degrees(math.acos(min(1.0, abs(float(n @ expected)))))
    if angle > max_normal_angle_deg:
        return {"error": f"best plane is {angle:.0f} deg off the predicted board normal "
                         "(wall or floor?)"}
    return {"points": inliers, "n": n, "d": d, "rms": rms, "n_crop": int(len(crop))}


def fraction_inside(points_lidar, t_lidar_board, board, tolerance=0.03):
    """Share of LiDAR board points that fall inside the physical board outline (validation)."""
    pts_b = transform_points(invert(t_lidar_board), points_lidar)
    x0, x1, y0, y1 = board.extent()
    inside = ((pts_b[:, 0] > x0 - tolerance) & (pts_b[:, 0] < x1 + tolerance)
              & (pts_b[:, 1] > y0 - tolerance) & (pts_b[:, 1] < y1 + tolerance))
    return float(inside.mean()) if len(pts_b) else 0.0
