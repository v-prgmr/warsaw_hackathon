import math

import numpy as np
import pytest

from g1_ar_bridge import geometry as g


def random_rot(rng):
    return g.so3_exp(rng.normal(size=3))


def test_quaternion_round_trip_and_conventions():
    rng = np.random.default_rng(1)
    for _ in range(50):
        R = random_rot(rng)
        q = g.rot_to_quat(R)
        assert q[3] >= 0 and np.allclose(g.quat_to_rot(q), R, atol=1e-9)
    # 90 deg about z: [0, 0, sin45, cos45]
    assert np.allclose(g.rot_to_quat(g.rot_z(math.pi / 2)), [0, 0, math.sqrt(0.5), math.sqrt(0.5)])
    assert np.allclose(g.T_to_pose(g.pose_to_T([1, 2, 3], [0, 0, 0, 1]))[0], [1, 2, 3])


def test_so3_exp_log_and_rotation_average():
    rng = np.random.default_rng(2)
    for _ in range(50):
        w = rng.normal(size=3)
        w *= min(1.0, 3.0 / np.linalg.norm(w))
        assert np.allclose(g.so3_log(g.so3_exp(w)), w, atol=1e-7)
    assert np.allclose(g.so3_log(g.rot_x(math.pi)), [math.pi, 0, 0], atol=1e-6)
    R = random_rot(rng)
    noisy = [R @ g.so3_exp(rng.normal(scale=0.02, size=3)) for _ in range(200)]
    assert g.rotation_angle_deg(g.average_rotations(noisy), R) < 0.3


def test_inverse_and_point_transform():
    rng = np.random.default_rng(3)
    T = g.make_T(random_rot(rng), rng.normal(size=3))
    assert np.allclose(T @ g.inv_T(T), np.eye(4), atol=1e-12)
    p = rng.normal(size=(5, 3))
    assert np.allclose(g.transform_points(g.inv_T(T), g.transform_points(T, p)), p)


def test_ros_to_ar_axes():
    # ROS forward / left / up -> AR +X / -Z / +Y (upstream R_ALIGN)
    assert np.allclose(g.R_ALIGN @ [1, 0, 0], [1, 0, 0])
    assert np.allclose(g.R_ALIGN @ [0, 1, 0], [0, 0, -1])
    assert np.allclose(g.R_ALIGN @ [0, 0, 1], [0, 1, 0])
    assert np.isclose(np.linalg.det(g.R_ALIGN), 1.0)
    # AR yaw convention: forward(yaw) = (cos, 0, -sin)
    for yaw in (-2.0, -0.4, 0.0, 0.9, 3.0):
        f = g.rot_y(yaw) @ [1, 0, 0]
        assert np.allclose(f, [math.cos(yaw), 0, -math.sin(yaw)])
        assert g.ar_yaw_of_axis(f) == pytest.approx(yaw)
        assert np.allclose(g.quat_to_rot(g.ar_yaw_quat(yaw)), g.rot_y(yaw))


def test_level_ar_from_map_recovers_yaw_and_removes_tilt():
    T = g.ar_from_map_yaw_t(0.7, [1.0, 2.0, 3.0])
    T_level, yaw, tilt = g.level_ar_from_map(T)
    assert yaw == pytest.approx(0.7) and tilt == pytest.approx(0.0, abs=1e-9)
    assert np.allclose(T_level, T)
    tilted = T.copy()
    tilted[:3, :3] = g.rot_x(math.radians(4)) @ T[:3, :3]
    T_level, yaw, tilt = g.level_ar_from_map(tilted)
    assert yaw == pytest.approx(0.7, abs=math.radians(0.5)) and tilt == pytest.approx(4, abs=0.2)
    # the levelled map +Z is exactly AR +Y
    assert np.allclose(T_level[:3, :3] @ [0, 0, 1], [0, 1, 0])


def test_robot_marker_pose_round_trip():
    T_ar_map = g.ar_from_map_yaw_t(0.3, [0.5, 1.0, -2.0])
    # robot 1 m ahead in map, facing +y (left), slightly pitched
    T_map_robot = g.make_T(g.rot_z(math.pi / 2) @ g.rot_y(0.05), [1.0, 0.0, 0.0])
    pos, quat, yaw = g.ar_marker_pose(T_ar_map @ T_map_robot)
    assert np.allclose(pos, g.transform_points(T_ar_map, [[1, 0, 0]])[0])
    assert yaw == pytest.approx(0.3 + math.pi / 2, abs=1e-6)
    T_back, yaw_back = g.ar_marker_to_T(pos, quat)
    assert yaw_back == pytest.approx(yaw)
    # the base frame rebuilt from the marker has ROS axes: its +Z is AR up
    assert np.allclose(T_back[:3, :3] @ [0, 0, 1], [0, 1, 0])
    assert np.allclose(T_back @ g.inv_T(g.level_ros_pose(T_map_robot)), T_ar_map, atol=1e-9)
