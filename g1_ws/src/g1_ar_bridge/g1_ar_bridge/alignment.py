"""Wall-tag alignment: one static AprilTag seen by the robot camera and by the Spectacles.

    robot camera:   map <- camera (TF)  x  camera <- tag (PnP)   =>  T_map_tag   (the anchor)
    Spectacles:     ar  <- camera (Lens) x  camera <- tag (PnP)  =>  T_ar_tag
    result:         T_ar_map = T_ar_tag @ inv(T_map_tag), levelled (both worlds know "up")

``TagPoseEstimator`` accumulates views of the tag in one world (``map`` or AR) and returns a
robust multi-view estimate; ``solve_ar_map`` combines the two sides. Nothing here needs ROS.
"""
import math
from dataclasses import dataclass, field

import numpy as np

from .apriltag import (estimate_tag_pose, project, refine_tag_pose, tag_object_points,
                       tag_side_px)
from .geometry import inv_T, level_ar_from_map, make_T, rotation_angle_deg


@dataclass
class TagView:
    K: np.ndarray
    T_world_cv: np.ndarray        # OpenCV camera -> world
    corners: np.ndarray           # (4, 2) px
    T_world_tag: np.ndarray       # single-view estimate
    reproj_px: float
    ambiguity: float
    distance_m: float
    side_px: float
    stamp: float = 0.0

    @property
    def cam_centre(self):
        return self.T_world_cv[:3, 3]


@dataclass
class TagEstimate:
    T_world_tag: np.ndarray
    n_views: int                  # views used (inliers)
    n_total: int                  # views collected
    rms_px: float                 # reprojection RMS over the used views
    baseline_m: float             # largest distance between the used camera centres
    pos_spread_m: float           # spread of the single-view positions about the estimate
    rot_spread_deg: float
    extra: dict = field(default_factory=dict)


class TagPoseEstimator:
    """Collects views of one static tag in one world frame and estimates its pose."""

    def __init__(self, size_m, max_views=40, max_reproj_px=3.0, max_distance_m=8.0,
                 min_side_px=12.0, inlier_pos_m=0.10, inlier_rot_deg=15.0):
        self.size_m = float(size_m)
        self.max_views = int(max_views)
        self.max_reproj_px = float(max_reproj_px)
        self.max_distance_m = float(max_distance_m)
        self.min_side_px = float(min_side_px)
        self.inlier_pos_m = float(inlier_pos_m)
        self.inlier_rot_deg = float(inlier_rot_deg)
        self.views = []

    def reset(self):
        self.views = []

    def make_view(self, corners, K, T_world_cv, stamp=0.0):
        """Single-view PnP for one detection. Returns ``(TagView or None, reason)``."""
        side = tag_side_px(corners)
        if side < self.min_side_px:
            return None, f"tag too small ({side:.0f} px)"
        pose = estimate_tag_pose(corners, self.size_m, K)
        if pose is None:
            return None, "PnP failed"
        if pose.reproj_px > self.max_reproj_px:
            return None, f"reprojection {pose.reproj_px:.1f} px"
        dist = float(np.linalg.norm(pose.T_cam_tag[:3, 3]))
        if dist > self.max_distance_m:
            return None, f"too far ({dist:.1f} m)"
        T_world_tag = np.asarray(T_world_cv) @ pose.T_cam_tag
        return TagView(np.asarray(K, float), np.asarray(T_world_cv, float),
                       np.asarray(corners, float).reshape(4, 2), T_world_tag, pose.reproj_px,
                       pose.ambiguity, dist, side, stamp), "ok"

    def add(self, view):
        self.views.append(view)
        if len(self.views) > self.max_views:
            self.views.pop(0)

    def _consistent(self, a, b):
        dp = float(np.linalg.norm(a.T_world_tag[:3, 3] - b.T_world_tag[:3, 3]))
        return (dp <= self.inlier_pos_m
                and rotation_angle_deg(a.T_world_tag[:3, :3], b.T_world_tag[:3, :3])
                <= self.inlier_rot_deg)

    def estimate(self, refine=True):
        views = self.views
        if not views:
            return None
        # consensus: the view that agrees with the most others (then lowest reprojection error)
        best, best_score = None, None
        for v in views:
            n = sum(1 for w in views if self._consistent(v, w))
            score = (n, -v.reproj_px)
            if best_score is None or score > best_score:
                best, best_score = v, score
        inliers = [w for w in views if self._consistent(best, w)]
        T0 = best.T_world_tag
        if refine and len(inliers) >= 1:
            # IPPE can pick the mirrored solution in some views; start from the candidate that
            # explains all inlier views best
            T0 = min((w.T_world_tag for w in inliers),
                     key=lambda T: _median_reproj(inliers, self.size_m, T))
            T, rms, _ = refine_tag_pose([(w.K, w.T_world_cv, w.corners) for w in inliers],
                                        self.size_m, T0)
        else:
            T, rms = T0, float(np.sqrt(np.mean([w.reproj_px ** 2 for w in inliers])))
        centres = np.array([w.cam_centre for w in inliers])
        baseline = 0.0
        if len(centres) > 1:
            diff = centres[:, None, :] - centres[None, :, :]
            baseline = float(np.max(np.linalg.norm(diff, axis=2)))
        pos = np.array([w.T_world_tag[:3, 3] for w in inliers])
        pos_spread = float(np.sqrt(np.mean(np.sum((pos - T[:3, 3]) ** 2, axis=1))))
        angles = [rotation_angle_deg(w.T_world_tag[:3, :3], T[:3, :3]) for w in inliers]
        rot_spread = float(np.sqrt(np.mean(np.square(angles))))
        return TagEstimate(T, len(inliers), len(views), rms, baseline, pos_spread, rot_spread)


def _median_reproj(views, size_m, T_world_tag):
    pts = tag_object_points(size_m) @ T_world_tag[:3, :3].T + T_world_tag[:3, 3]
    errs = []
    for w in views:
        uv, z = project(w.K, inv_T(w.T_world_cv), pts)
        if np.any(z <= 0):
            errs.append(1e6)
            continue
        errs.append(float(np.sqrt(np.mean(np.sum((uv - w.corners) ** 2, axis=1)))))
    return float(np.median(errs))


@dataclass
class ArMapSolution:
    T_ar_map: np.ndarray
    yaw_deg: float
    tilt_deg: float               # disagreement of the two "up" directions before levelling


def solve_ar_map(T_ar_tag, T_map_tag):
    """``T_ar_map`` from the tag seen in both worlds, gravity-levelled (yaw + translation).

    The rotation is the least-squares yaw of ``T_ar_tag @ inv(T_map_tag)``; the translation
    then puts the tag centre exactly where both sides measured it (the best-known point).
    """
    T_raw = np.asarray(T_ar_tag) @ inv_T(np.asarray(T_map_tag))
    T_level, yaw, tilt = level_ar_from_map(T_raw)
    t = T_ar_tag[:3, 3] - T_level[:3, :3] @ T_map_tag[:3, 3]
    return ArMapSolution(make_T(T_level[:3, :3], t), math.degrees(yaw), tilt)


def registration_progress(est, min_views, min_baseline_m):
    """0-100 progress for the Lens (views collected and camera movement)."""
    if est is None:
        return 0
    views = min(1.0, est.n_views / float(min_views))
    move = min(1.0, est.baseline_m / max(min_baseline_m, 1e-6))
    return int(round(100 * views * (0.6 + 0.4 * move)))
