"""ROS-independent projection and multi-view evidence fusion.

All transforms follow T_target_source and use metres. Camera coordinates are
ROS optical coordinates: +x right, +y down, +z forward. Input images are RGB.
"""

from dataclasses import dataclass, field

import numpy as np


def transform_matrix(translation, quaternion_xyzw):
    """Return a homogeneous T_target_source from a ROS translation/quaternion."""
    x, y, z, w = np.asarray(quaternion_xyzw, dtype=np.float64)
    n = x*x + y*y + z*z + w*w
    if n < 1e-12:
        raise ValueError("zero-length quaternion")
    s = 2.0 / n
    rot = np.array([
        [1-s*(y*y+z*z), s*(x*y-z*w), s*(x*z+y*w)],
        [s*(x*y+z*w), 1-s*(x*x+z*z), s*(y*z-x*w)],
        [s*(x*z-y*w), s*(y*z+x*w), 1-s*(x*x+y*y)],
    ])
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rot
    result[:3, 3] = translation
    return result


def project_visible(points, t_camera_cloud, intrinsics, depth_m,
                    tolerance_m=0.10, relative_tolerance=0.04):
    """Return (point indices, pixel u, pixel v) for depth-consistent LiDAR points.

    `intrinsics` is (fx, fy, cx, cy) of a rectified RGB image with aligned depth.
    Invalid depth is rejected: RGB/semantics are never painted through occluders.
    """
    pts = np.asarray(points, dtype=np.float64)
    depth = np.asarray(depth_m, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 3 or depth.ndim != 2:
        raise ValueError("expected Nx3 points and HxW depth")
    fx, fy, cx, cy = intrinsics
    if fx <= 0 or fy <= 0:
        raise ValueError("focal lengths must be positive")
    t = np.asarray(t_camera_cloud, dtype=np.float64)
    if t.shape != (4, 4):
        raise ValueError("transform must be 4x4")
    cam = pts @ t[:3, :3].T + t[:3, 3]
    z = cam[:, 2]
    candidates = np.flatnonzero(np.isfinite(cam).all(axis=1) & (z > 0))
    zc = z[candidates]
    u = np.rint(fx * cam[candidates, 0] / zc + cx).astype(np.int64)
    v = np.rint(fy * cam[candidates, 1] / zc + cy).astype(np.int64)
    height, width = depth.shape
    inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    candidates, u, v, zc = candidates[inside], u[inside], v[inside], zc[inside]
    measured = depth[v, u]
    visible = (np.isfinite(measured) & (measured > 0) &
               (np.abs(measured - zc) <= np.maximum(tolerance_m,
                                                    relative_tolerance * zc)))
    return candidates[visible], u[visible], v[visible]


@dataclass
class Evidence:
    color: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    weight: float = 0.0
    labels: dict = field(default_factory=dict)
    last_observation_s: float = 0.0


class VoxelFusion:
    """Bounded, revisable evidence keyed by fixed map-frame voxels.

    A whole image contributes at most one vote per voxel and class. An RTAB-Map
    cloud replacement can therefore reuse evidence at unchanged map locations;
    moved geometry will not display stale voxels and can be re-observed later.
    """

    def __init__(self, voxel_m=0.04, evidence_cap=20.0, max_age_s=300.0):
        if voxel_m <= 0 or evidence_cap <= 0 or max_age_s <= 0:
            raise ValueError("fusion parameters must be positive")
        self.voxel_m = voxel_m
        self.evidence_cap = evidence_cap
        self.max_age_s = max_age_s
        self.voxels = {}

    def keys(self, points):
        return np.floor(np.asarray(points) / self.voxel_m).astype(np.int64)

    def observe(self, points, indices, colors, labels, confidences, stamp_s):
        """Fuse visible points; label 0 means no COCO object evidence."""
        keys = self.keys(np.asarray(points)[indices])
        frame_votes = {}
        for key, rgb, label, confidence in zip(keys, colors, labels, confidences):
            k = tuple(key)
            vote = frame_votes.setdefault(k, [np.zeros(3), 0, {}])
            vote[0] += rgb
            vote[1] += 1
            if label > 0 and confidence > 0:
                vote[2][int(label)] = max(vote[2].get(int(label), 0.0),
                                          float(confidence))
        for key, (color_sum, count, scores) in frame_votes.items():
            item = self.voxels.setdefault(key, Evidence())
            if stamp_s < item.last_observation_s:
                continue
            age = stamp_s - item.last_observation_s
            decay = 0.5 ** (age / self.max_age_s) if item.weight else 1.0
            item.color *= decay
            item.weight *= decay
            # A later view with no object mask is negative evidence, not a vote
            # for an immutable label. Keep some inertia against one bad frame.
            item.labels = {k: v * decay * 0.9 for k, v in item.labels.items()
                           if v * decay * 0.9 >= 0.01}
            # Saturate evidence so later views can revise an early wrong color.
            if item.weight >= self.evidence_cap:
                factor = (self.evidence_cap - 1.0) / item.weight
                item.color *= factor
                item.weight *= factor
            item.color += color_sum / count
            item.weight += 1.0
            for label, score in scores.items():
                item.labels[label] = min(self.evidence_cap,
                                         item.labels.get(label, 0.0) + score)
            item.last_observation_s = stamp_s
        self.prune(stamp_s)

    def prune(self, stamp_s):
        old = [k for k, v in self.voxels.items()
               if stamp_s - v.last_observation_s > self.max_age_s * 4]
        for key in old:
            del self.voxels[key]

    def render(self, points):
        """Return RGB uint8, COCO label uint16 and normalized label confidence."""
        keys = self.keys(points)
        unique, inverse = np.unique(keys, axis=0, return_inverse=True)
        rgb = np.full((len(unique), 3), 128, dtype=np.uint8)
        labels = np.zeros(len(unique), dtype=np.uint16)
        confidence = np.zeros(len(unique), dtype=np.float32)
        for i, key in enumerate(unique):
            item = self.voxels.get(tuple(key))
            if item is None or item.weight <= 0:
                continue
            rgb[i] = np.clip(np.rint(item.color / item.weight), 0, 255).astype(np.uint8)
            if item.labels:
                label, score = max(item.labels.items(), key=lambda pair: pair[1])
                labels[i] = label
                confidence[i] = score / max(sum(item.labels.values()), 1.0)
        return rgb[inverse], labels[inverse], confidence[inverse]
