import math

import numpy as np
import pytest

from g1_calibration import geometry as g


@pytest.mark.parametrize("seed", range(20))
def test_rpy_and_quaternion_roundtrip(seed):
    rng = np.random.default_rng(seed)
    rpy = rng.uniform([-math.pi, -1.5, -math.pi], [math.pi, 1.5, math.pi])
    r = g.rpy_to_matrix(*rpy)
    np.testing.assert_allclose(g.rpy_to_matrix(*g.matrix_to_rpy(r)), r, atol=1e-12)
    np.testing.assert_allclose(g.matrix_from_quaternion(g.quaternion_from_matrix(r)), r,
                               atol=1e-12)


def test_tf2_convention():
    # static_transform_publisher --yaw 90deg maps child x onto parent y
    r = g.rpy_to_matrix(0, 0, math.pi / 2)
    np.testing.assert_allclose(r @ [1, 0, 0], [0, 1, 0], atol=1e-12)
    # roll then pitch then yaw, applied to the child frame: R = Rz Ry Rx
    r = g.rpy_to_matrix(0.1, 0.2, 0.3)
    np.testing.assert_allclose(
        r, g.rpy_to_matrix(0, 0, 0.3) @ g.rpy_to_matrix(0, 0.2, 0) @ g.rpy_to_matrix(0.1, 0, 0))
    q = g.quaternion_from_matrix(g.rpy_to_matrix(0, 0, math.pi / 2))
    np.testing.assert_allclose(q, [0, 0, math.sqrt(0.5), math.sqrt(0.5)], atol=1e-12)


def test_optical_frame():
    r = g.R_BODY_OPTICAL  # columns: optical axes in the body frame
    np.testing.assert_allclose(r[:, 2], [1, 0, 0], atol=1e-12)   # optical z = forward
    np.testing.assert_allclose(r[:, 0], [0, -1, 0], atol=1e-12)  # optical x = right
    np.testing.assert_allclose(r[:, 1], [0, 0, -1], atol=1e-12)  # optical y = down


def test_invert_transform_points_and_difference():
    t = g.transform_from_xyz_rpy([1, 2, 3], [0.3, -0.2, 1.0])
    np.testing.assert_allclose(g.invert(t) @ t, np.eye(4), atol=1e-12)
    p = np.array([[0.5, -1.0, 2.0]])
    np.testing.assert_allclose(g.transform_points(g.invert(t), g.transform_points(t, p)), p)
    d = g.transform_from_xyz_rpy([0.01, 0, 0], [0, 0, math.radians(2)])
    dt, dr = g.transform_difference(t, t @ d)
    assert dt == pytest.approx(0.01) and dr == pytest.approx(2.0)


def test_so3_exp_and_average():
    w = np.array([0.1, -0.2, 0.3])
    r = g.so3_exp(w)
    assert g.rotation_angle_deg(r) == pytest.approx(math.degrees(np.linalg.norm(w)))
    base = g.transform_from_xyz_rpy([1, 0, 0], [0.1, 0.2, 0.3])
    ts = [base @ g.make_transform(g.so3_exp(s * np.array([0.01, 0, 0])), [s * 0.01, 0, 0])
          for s in (-1, 1)]
    dt, dr = g.transform_difference(g.average_transforms(ts), base)
    assert dt < 1e-9 and dr < 1e-6


def test_dict_roundtrip():
    t = g.transform_from_xyz_rpy([0.1, -0.2, 0.3], [0.4, -0.5, 0.6])
    d = g.to_dict(t, "a", "b")
    assert d["parent"] == "a" and d["child"] == "b"
    np.testing.assert_allclose(g.from_dict(d), t, atol=1e-6)
    np.testing.assert_allclose(g.from_dict({"xyz": d["xyz"], "rpy": d["rpy"]}), t, atol=1e-5)
