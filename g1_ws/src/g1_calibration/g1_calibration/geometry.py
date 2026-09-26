"""Rigid transforms (4x4 numpy), in the conventions ROS uses.

Naming: ``T_a_b`` is the pose of frame b in frame a; it maps points from b to a
(``p_a = T_a_b @ p_b``), i.e. the TF transform with parent a and child b.
Euler angles follow tf2 / static_transform_publisher: ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``.
Quaternions are (x, y, z, w), as in geometry_msgs.
"""
import math

import numpy as np


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def matrix_to_rpy(r):
    """Inverse of rpy_to_matrix (pitch in [-pi/2, pi/2])."""
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(r[2, 1], r[2, 2])
        yaw = math.atan2(r[1, 0], r[0, 0])
    else:  # gimbal lock: only roll - yaw (or roll + yaw) is defined
        roll = 0.0
        yaw = math.atan2(-r[0, 1], r[1, 1])
    return roll, pitch, yaw


def quaternion_from_matrix(r):
    """(x, y, z, w) with w >= 0."""
    m = np.asarray(r, dtype=float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = 2.0 * math.sqrt(tr + 1.0)
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s]
    q = np.array(q)
    q /= np.linalg.norm(q)
    return q if q[3] >= 0 else -q


def matrix_from_quaternion(q):
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def make_transform(r=None, t=None):
    tf = np.eye(4)
    if r is not None:
        tf[:3, :3] = r
    if t is not None:
        tf[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return tf


def transform_from_xyz_rpy(xyz, rpy):
    return make_transform(rpy_to_matrix(*rpy), xyz)


def transform_from_xyz_quat(xyz, quat_xyzw):
    return make_transform(matrix_from_quaternion(quat_xyzw), xyz)


def invert(tf):
    inv = np.eye(4)
    inv[:3, :3] = tf[:3, :3].T
    inv[:3, 3] = -tf[:3, :3].T @ tf[:3, 3]
    return inv


def transform_points(tf, points):
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    return p @ tf[:3, :3].T + tf[:3, 3]


def skew(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def so3_exp(w):
    """Rotation matrix of the rotation vector w (Rodrigues)."""
    angle = float(np.linalg.norm(w))
    if angle < 1e-12:
        return np.eye(3) + skew(w)
    k = skew(np.asarray(w) / angle)
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def rotation_angle_deg(r):
    c = (np.trace(r) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def transform_difference(ta, tb):
    """(translation m, rotation deg) between two transforms of the same pair of frames."""
    d = invert(ta) @ tb
    return float(np.linalg.norm(d[:3, 3])), rotation_angle_deg(d[:3, :3])


def average_transforms(transforms):
    """Mean translation + chordal mean rotation (Markley quaternion average)."""
    qs = np.array([quaternion_from_matrix(t[:3, :3]) for t in transforms])
    m = qs.T @ qs
    q = np.linalg.eigh(m)[1][:, -1]
    return make_transform(matrix_from_quaternion(q),
                          np.mean([t[:3, 3] for t in transforms], axis=0))


def to_dict(tf, parent=None, child=None):
    """YAML-friendly transform: xyz, rpy (tf2 convention) and quaternion (x, y, z, w)."""
    out = {}
    if parent is not None:
        out["parent"] = parent
    if child is not None:
        out["child"] = child
    out["xyz"] = [round(float(v), 6) for v in tf[:3, 3]]
    out["rpy"] = [round(float(v), 6) for v in matrix_to_rpy(tf[:3, :3])]
    out["quat_xyzw"] = [round(float(v), 8) for v in quaternion_from_matrix(tf[:3, :3])]
    return out


def from_dict(d):
    if "quat_xyzw" in d:
        return transform_from_xyz_quat(d["xyz"], d["quat_xyzw"])
    return transform_from_xyz_rpy(d["xyz"], d["rpy"])


# A camera "body" frame (x forward, y left, z up; e.g. camera_link, oak-d-base-frame) to its
# optical frame (z forward, x right, y down; REP-103): the usual rpy (-pi/2, 0, -pi/2).
R_BODY_OPTICAL = rpy_to_matrix(-math.pi / 2, 0.0, -math.pi / 2)
T_BODY_OPTICAL = make_transform(R_BODY_OPTICAL)
