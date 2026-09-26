"""Solvers on synthetic data with known ground truth."""
import math

import numpy as np
import pytest

from g1_calibration.board import board_plane
from g1_calibration.geometry import (invert, transform_difference, transform_from_xyz_rpy,
                                     transform_points)
from g1_calibration.lidar_board import extract_board, fit_plane, fraction_inside
from g1_calibration.solvers import (CalibrationError, calibrate_camera_lidar,
                                    calibrate_camera_pair, calibrate_intrinsics,
                                    reprojection_rms)

from synthetic import (BODY_LIDAR, BODY_RS_OPT, BOARD, OAK, RS, TRUE_BODY_OAK_OPT, board_poses,
                       lidar_points, project)

TRUE_LIDAR_OAK = invert(BODY_LIDAR) @ TRUE_BODY_OAK_OPT


def observations(n=12, seed=1, noise=0.012):
    rng = np.random.default_rng(seed)
    obs = []
    for t_body_board in board_poses(n, rng):
        t_oak_board = invert(TRUE_BODY_OAK_OPT) @ t_body_board
        pts = lidar_points(t_body_board, rng, noise=noise)
        found = extract_board(pts, TRUE_LIDAR_OAK @ t_oak_board, BOARD, rng=rng)
        n_c, d_c = board_plane(t_oak_board)
        obs.append({"n_c": n_c, "d_c": d_c, "points": found["points"], "T": t_oak_board})
    return obs


def test_fit_plane():
    rng = np.random.default_rng(0)
    p = np.column_stack([rng.uniform(-1, 1, 500), rng.uniform(-1, 1, 500), np.full(500, -2.0)])
    n, d, rms = fit_plane(p)
    assert d == pytest.approx(2.0) and n @ [0, 0, -1] == pytest.approx(1.0) and rms < 1e-9


def test_extract_board_rejects_walls_and_keeps_board():
    rng = np.random.default_rng(3)
    (t_body_board,) = board_poses(1, rng)
    pts = lidar_points(t_body_board, rng)
    t_lidar_board = invert(BODY_LIDAR) @ t_body_board
    # coarse search window: may take a few floor points next to a low board ...
    found = extract_board(pts, t_lidar_board, BOARD, margin=0.3, depth_margin=0.3, rng=rng)
    assert 400 < len(found["points"]) and found["rms"] < 0.02
    assert fraction_inside(found["points"], t_lidar_board, BOARD) > 0.9
    # ... the fine window (last stage of calibrate_extrinsics) keeps the board only
    found = extract_board(pts, t_lidar_board, BOARD, margin=0.05, depth_margin=0.06, rng=rng)
    assert 400 < len(found["points"]) <= 600 and found["rms"] < 0.02
    assert fraction_inside(found["points"], t_lidar_board, BOARD) > 0.98
    # predicted far off (e.g. 1 m to the side): nothing sensible there
    off = t_lidar_board.copy()
    off[:3, 3] += [0.0, 1.5, 0.0]
    assert "error" in extract_board(pts, off, BOARD, rng=rng)


@pytest.mark.parametrize("noise,max_deg", [(0.005, 0.3), (0.02, 0.5)])
def test_camera_lidar_recovers_truth(noise, max_deg):
    obs = observations(noise=noise)
    res = calibrate_camera_lidar(obs)
    dt, dr = transform_difference(invert(res["T_cam_lidar"]), TRUE_LIDAR_OAK)
    assert dt < 0.01 and dr < max_deg, (dt, dr)
    assert res["rms_m"] == pytest.approx(noise, rel=0.35)


def test_camera_lidar_closed_form_is_independent_of_a_bad_guess():
    obs = observations()
    bad = TRUE_LIDAR_OAK @ transform_from_xyz_rpy([0.2, -0.1, 0.1], [0.2, -0.2, 0.3])
    res = calibrate_camera_lidar(obs, t_init=invert(bad))
    dt, dr = transform_difference(invert(res["T_cam_lidar"]), TRUE_LIDAR_OAK)
    assert dt < 0.01 and dr < 0.3


def test_camera_lidar_degenerate_poses_raise():
    obs = observations(n=4)
    for o in obs[1:]:
        o["n_c"], o["d_c"] = obs[0]["n_c"], obs[0]["d_c"]
    with pytest.raises(CalibrationError, match="too similar"):
        calibrate_camera_lidar(obs)
    with pytest.raises(CalibrationError, match="need >= 3"):
        calibrate_camera_lidar(obs[:2])


def _corners(intr, t_cam_board, rng, sigma=0.2):
    uv, _ = project(intr, t_cam_board, BOARD.object_points())
    return uv + rng.normal(0.0, sigma, uv.shape)


def test_camera_pair_recovers_truth():
    rng = np.random.default_rng(5)
    poses = board_poses(10, rng)
    t_oak = [invert(TRUE_BODY_OAK_OPT) @ p for p in poses]
    t_rs = [invert(BODY_RS_OPT) @ p for p in poses]
    res = calibrate_camera_pair([BOARD.object_points()] * 10,
                                [_corners(OAK, t, rng) for t in t_oak],
                                [_corners(RS, t, rng) for t in t_rs],
                                OAK.k, OAK.d, RS.k, RS.d, OAK.size, t_oak, t_rs)
    dt, dr = transform_difference(res["T_b_a"], invert(BODY_RS_OPT) @ TRUE_BODY_OAK_OPT)
    assert dt < 0.002 and dr < 0.05 and res["rms_px"] < 0.35
    assert res["spread"]["translation_m_max"] < 0.005


def test_intrinsics_recover_k_and_factory_check():
    rng = np.random.default_rng(7)
    obj, img = [], []
    for _ in range(15):
        r = transform_from_xyz_rpy([rng.uniform(-0.3, 0.1), rng.uniform(-0.2, 0.0),
                                    rng.uniform(0.9, 1.6)],
                                   [rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5),
                                    rng.uniform(-math.pi, math.pi)])
        obj.append(BOARD.object_points())
        img.append(_corners(OAK, r, rng, sigma=0.15))
    res = calibrate_intrinsics(obj, img, OAK.size)
    np.testing.assert_allclose(res["k"], OAK.k, atol=2.5)
    assert res["rms"] < 0.25
    assert reprojection_rms(obj, img, OAK.k, OAK.d) < 0.25
    wrong = OAK.k.copy()
    wrong[0, 0] *= 1.05
    assert reprojection_rms(obj, img, wrong, OAK.d) > 0.5  # a 5 % focal error shows up
    with pytest.raises(CalibrationError):
        calibrate_intrinsics(obj[:5], img[:5], OAK.size)


def test_projection_helper_consistency():
    t = transform_from_xyz_rpy([0, 0, 1.0], [0, 0, 0])
    uv, z = project(OAK, t, np.zeros((1, 3)))
    np.testing.assert_allclose(uv, [[400.0, 300.0]])
    np.testing.assert_allclose(transform_points(t, np.zeros((1, 3))), [[0, 0, 1.0]])
