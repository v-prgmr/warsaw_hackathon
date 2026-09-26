"""Rigid transforms shared by the AR bridge (numpy only, no ROS).

Conventions (AGENTS.md §27, upstream dimos-ar ``tag_tracking/solve.py``):

* 4x4 homogeneous matrices; ``T_a_b`` maps points from frame ``b`` into frame ``a``.
* Quaternions are ``[x, y, z, w]`` (ROS geometry_msgs and the Lens protocol).
* ROS frames (``map``, ``robot_center``, cameras' ``*_link``): x forward, y left, z up.
* AR world (the Lens): metres, **Y up**, right-handed; a robot marker's local +X is forward.
  ``R_ALIGN`` maps ROS axes into AR axes: (x, y, z) -> (x, z, -y).
* Lens camera (GL): x right, y up, looks along -Z. OpenCV camera: x right, y down, looks along
  +Z. ``FLIP_YZ`` converts between them: ``T_ar_cv = T_ar_glcam @ FLIP_YZ``.
* Yaw in the AR world is about +Y, measured from +X towards -Z (upstream ``yaw_from_T``):
  forward(yaw) = (cos yaw, 0, -sin yaw).
"""
import math

import numpy as np

R_ALIGN = np.array([[1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0],
                    [0.0, -1.0, 0.0]])
FLIP_YZ = np.diag([1.0, -1.0, -1.0, 1.0])


def quat_to_rot(q):
    """3x3 rotation matrix of a quaternion ``[x, y, z, w]`` (normalised here)."""
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        raise ValueError("zero quaternion")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_to_quat(R):
    """Quaternion ``[x, y, z, w]`` (w >= 0) of a 3x3 rotation matrix."""
    R = np.asarray(R, dtype=np.float64)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    if q[3] < 0:
        q = -q
    return [float(v) for v in q]


def make_T(R=None, t=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def pose_to_T(position, quat):
    return make_T(quat_to_rot(quat), position)


def T_to_pose(T):
    """(position [x, y, z], quaternion [x, y, z, w]) of a 4x4 transform."""
    return [float(v) for v in T[:3, 3]], rot_to_quat(T[:3, :3])


def inv_T(T):
    R, t = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ t)


def transform_points(T, points):
    """Apply a 4x4 transform to an (N, 3) array."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    return pts @ T[:3, :3].T + T[:3, 3]


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def so3_exp(w):
    """Rotation matrix of a rotation vector (Rodrigues)."""
    w = np.asarray(w, dtype=np.float64).reshape(3)
    th = float(np.linalg.norm(w))
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)


def so3_log(R):
    """Rotation vector of a rotation matrix."""
    R = np.asarray(R, dtype=np.float64)
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    th = math.acos(c)
    if th < 1e-9:
        return np.zeros(3)
    if math.pi - th < 1e-6:
        # 180 deg: axis from the diagonal of (R + I) / 2
        M = (R + np.eye(3)) / 2.0
        k = np.sqrt(np.clip(np.diag(M), 0.0, None))
        i = int(np.argmax(k))
        k[i] = math.sqrt(M[i, i])
        for j in range(3):
            if j != i:
                k[j] = M[i, j] / k[i]
        return th * k / np.linalg.norm(k)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return th * v / (2.0 * math.sin(th))


def rotation_angle_deg(Ra, Rb):
    """Angle of the relative rotation between two rotation matrices, degrees."""
    return math.degrees(float(np.linalg.norm(so3_log(np.asarray(Ra).T @ np.asarray(Rb)))))


def average_rotations(rotations, weights=None):
    """Weighted chordal mean of rotation matrices (quaternion eigenvector method)."""
    Rs = list(rotations)
    if not Rs:
        raise ValueError("no rotations")
    w = np.ones(len(Rs)) if weights is None else np.asarray(weights, dtype=np.float64)
    M = np.zeros((4, 4))
    for R, wi in zip(Rs, w):
        q = np.asarray(rot_to_quat(R))
        M += wi * np.outer(q, q)
    vals, vecs = np.linalg.eigh(M)
    q = vecs[:, int(np.argmax(vals))]
    return quat_to_rot(q)


# --- AR world helpers ---------------------------------------------------------------------


def ar_yaw_quat(yaw):
    """Quaternion ``[x, y, z, w]`` of a rotation by ``yaw`` about the AR world's +Y axis."""
    return [0.0, math.sin(yaw / 2.0), 0.0, math.cos(yaw / 2.0)]


def ar_yaw_of_axis(v):
    """AR yaw of a direction's horizontal part: atan2(-z, x). None if (near) vertical."""
    x, z = float(v[0]), float(v[2])
    if math.hypot(x, z) < 1e-9:
        return None
    return math.atan2(-z, x)


def level_ar_from_map(T_ar_map):
    """Closest gravity-aligned transform to ``T_ar_map``.

    Both worlds know "up" (the Spectacles from their IMU, RTAB-Map's ``map`` from the robot
    IMU), so ``T_ar_map`` should be ``[rot_y(yaw) @ R_ALIGN, t]``. Returns ``(T_level, yaw,
    tilt_deg)`` where yaw is the least-squares (Frobenius) fit and ``tilt_deg`` is how far the
    input's map +Z axis was from AR +Y (large = inconsistent inputs).
    """
    R = T_ar_map[:3, :3] @ R_ALIGN.T          # should be a rotation about Y
    yaw = math.atan2(R[0, 2] - R[2, 0], R[0, 0] + R[2, 2])
    up = T_ar_map[:3, :3] @ np.array([0.0, 0.0, 1.0])
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(up[1]) / np.linalg.norm(up)))))
    return make_T(rot_y(yaw) @ R_ALIGN, T_ar_map[:3, 3]), yaw, tilt


def ar_from_map_yaw_t(yaw, t):
    return make_T(rot_y(yaw) @ R_ALIGN, t)


def level_ros_pose(T):
    """A ROS-frame pose flattened to heading only (roll and pitch removed, position kept)."""
    x_axis = T[:3, 0]
    yaw = math.atan2(float(x_axis[1]), float(x_axis[0]))
    return make_T(rot_z(yaw), T[:3, 3])


def ar_marker_pose(T_ar_robot):
    """(position, yaw-only quaternion) of a robot pose in the AR world, as the Lens wants it.

    The Lens keeps only the heading of the marker's +X axis (``yawRotationFromWorldRotation``),
    so the robot's roll and pitch are dropped here.
    """
    yaw = ar_yaw_of_axis(T_ar_robot[:3, 0])
    if yaw is None:
        yaw = 0.0
    return [float(v) for v in T_ar_robot[:3, 3]], ar_yaw_quat(yaw), yaw


def ar_marker_to_T(position, quat):
    """Robot base pose in the AR world from a Lens marker pose (yaw only) -> ROS-axis base.

    Upstream ``registration/session/flows.py``: the marker's yaw about Y, composed with
    ``R_ALIGN`` so the returned frame has ROS axes (x forward, y left, z up) in AR coordinates.
    """
    R = quat_to_rot(quat)
    yaw = ar_yaw_of_axis(R[:, 0])
    if yaw is None:
        yaw = 0.0
    return ar_from_map_yaw_t(yaw, position), yaw
