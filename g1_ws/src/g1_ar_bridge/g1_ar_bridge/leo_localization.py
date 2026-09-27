"""Leo Rover in the G1 map through the shared wall AprilTag (no ROS; g1_ws/docs/leo_g1_laptop_integration.md).

The same physical tag (36h11 ID 0) is anchored in the G1 map by ``tag_anchor``
(``T_map_tag``, ArUco axes: x right, y up, z out of the wall). Leo's OAK-D detector reports the
tag in its camera (``T_cam_tag``). With Leo's camera mount ``T_base_cam``::

    T_map_base = T_map_tag · inv(T_cam_tag) · inv(T_base_cam)

At each accepted sighting the correction ``T_map_odom = T_map_base · inv(T_odom_base(t))`` is
updated from Leo's own odometry at the same (Leo-clock) stamp; between sightings Leo's odometry
carries the pose: ``T_map_base(now) = T_map_odom · T_odom_base(now)``. Leo's odometry is never
modified. ``T_A_B`` maps coordinates in frame B into frame A.
"""
import collections
import math

import numpy as np

from .geometry import inv_T, make_T, quat_to_rot, rot_to_quat, so3_log

# Leo's OAK-D looks forward, mounted UPSIDE DOWN (the integration doc): optical x = Leo +y,
# optical y = Leo +z, optical z = Leo +x (forward). NOT MEASURED: position unknown.
R_BASE_OPTICAL_UPSIDE_DOWN = np.column_stack([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])
# apriltag conventions whose tag z axis points INTO the tag: rotate 180 deg about x to ours
FLIP_TAG_Z = np.diag([1.0, -1.0, -1.0, 1.0])


def rpy_to_rot(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rot_to_rpy(R):
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    return math.atan2(R[2, 1], R[2, 2]), pitch, math.atan2(R[1, 0], R[0, 0])


DEFAULT_CAMERA_RPY = rot_to_rpy(R_BASE_OPTICAL_UPSIDE_DOWN)


def camera_mount(xyz, rpy):
    """``T_base_cam`` from a URDF-style xyz + rpy (radians)."""
    return make_T(rpy_to_rot(*rpy), xyz)


def to_our_tag_convention(T_cam_tag):
    """Return ``(T_cam_tag, flipped)`` with the tag's z axis pointing out of the tag.

    A detected tag always faces the camera, so its outward normal points back towards it:
    ``z_tag · (camera - tag) > 0``. Detectors with z into the tag (y down) fail that test and
    are rotated 180 deg about x, which gives ours (x right, y up, z out).
    """
    T = np.asarray(T_cam_tag, dtype=np.float64)
    if float(T[:3, 2] @ (-T[:3, 3])) < 0.0:
        return T @ FLIP_TAG_Z, True
    return T, False


def leo_base_in_map(T_map_tag, T_cam_tag, T_base_cam):
    return np.asarray(T_map_tag) @ inv_T(np.asarray(T_cam_tag)) @ inv_T(np.asarray(T_base_cam))


def _interp_T(T0, T1, a):
    """Pose between two odometry samples (linear position, slerp-free small-angle rotation)."""
    R_rel = T0[:3, :3].T @ T1[:3, :3]
    w = so3_log(R_rel) * a
    th = float(np.linalg.norm(w))
    if th < 1e-9:
        R = T0[:3, :3]
    else:
        k = w / th
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        R = T0[:3, :3] @ (np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K)
    return make_T(R, (1 - a) * T0[:3, 3] + a * T1[:3, 3])


class LeoLocalizer:
    """Leo's pose in the G1 ``map`` from tag sightings + Leo odometry (all Leo-clock stamps)."""

    def __init__(self, T_base_cam, min_distance_m=0.2, max_distance_m=4.0,
                 max_odom_gap_s=0.25, odom_history_s=10.0):
        self.T_base_cam = np.asarray(T_base_cam, dtype=np.float64)
        self.min_distance_m, self.max_distance_m = min_distance_m, max_distance_m
        self.max_odom_gap_s = max_odom_gap_s
        self.odom_history_s = odom_history_s
        self.odom = collections.deque()          # (stamp, T_odom_base)
        self.odom_frames = None                  # (frame_id, child_frame_id) from Leo
        self.T_map_odom = None                   # correction, when odometry is available
        self.T_map_base_tag = None               # last sighting without odometry
        self.last_sighting = None                # dict with details of the last accepted one
        self.sightings = 0
        self.rejected = collections.Counter()
        self.flipped = None

    def add_odom(self, stamp, position, orientation, frames=None):
        T = make_T(quat_to_rot(orientation), position)
        if self.odom and stamp < self.odom[-1][0]:
            self.odom.clear()                    # Leo restarted / clock jumped
        self.odom.append((float(stamp), T))
        while self.odom and self.odom[-1][0] - self.odom[0][0] > self.odom_history_s:
            self.odom.popleft()
        if frames:
            self.odom_frames = tuple(frames)

    def odom_at(self, stamp):
        """``T_odom_base`` at a Leo stamp (interpolated), or None when not covered."""
        if not self.odom:
            return None
        if stamp <= self.odom[0][0] or stamp >= self.odom[-1][0]:
            t, T = min((self.odom[0], self.odom[-1]), key=lambda s: abs(s[0] - stamp))
            return T if abs(t - stamp) <= self.max_odom_gap_s else None
        prev = self.odom[0]
        for cur in self.odom:
            if cur[0] >= stamp:
                if cur[0] - prev[0] > 2 * self.max_odom_gap_s:
                    return None
                a = (stamp - prev[0]) / max(cur[0] - prev[0], 1e-9)
                return _interp_T(prev[1], cur[1], a)
            prev = cur
        return None

    def add_sighting(self, stamp, T_cam_tag, T_map_tag):
        """One Leo detection of the shared tag. Returns the accepted ``T_map_base`` or None."""
        T_cam_tag, flipped = to_our_tag_convention(T_cam_tag)
        self.flipped = flipped
        distance = float(np.linalg.norm(T_cam_tag[:3, 3]))
        if not self.min_distance_m <= distance <= self.max_distance_m:
            self.rejected["distance"] += 1
            return None
        T_map_base = leo_base_in_map(T_map_tag, T_cam_tag, self.T_base_cam)
        T_odom_base = self.odom_at(stamp)
        if T_odom_base is not None:
            self.T_map_odom = T_map_base @ inv_T(T_odom_base)
        elif self.odom:
            self.rejected["no odometry at the tag stamp"] += 1
            return None
        else:
            self.T_map_base_tag = T_map_base
        self.sightings += 1
        self.last_sighting = {"stamp": float(stamp), "distance_m": round(distance, 3),
                              "flipped_tag_convention": flipped}
        return T_map_base

    def pose(self):
        """Current ``T_map_base`` (odometry-propagated when available), or None."""
        if self.T_map_odom is not None and self.odom:
            return self.T_map_odom @ self.odom[-1][1]
        return self.T_map_base_tag


def floor_pose(T_map_base):
    """(x, y, yaw) of Leo on the floor."""
    return float(T_map_base[0, 3]), float(T_map_base[1, 3]), math.atan2(T_map_base[1, 0],
                                                                        T_map_base[0, 0])


def quat_of(T):
    return rot_to_quat(np.asarray(T)[:3, :3])
