"""End-to-end math of the wall-tag alignment with rendered images from both cameras."""
import math

import numpy as np
from synthetic import K_from_hfov, cam_tag, look_at_cv, render_tag, wall_tag_in_map

from g1_ar_bridge.alignment import TagPoseEstimator, registration_progress, solve_ar_map
from g1_ar_bridge.apriltag import TagDetector
from g1_ar_bridge.geometry import (ar_from_map_yaw_t, inv_T, make_T, rotation_angle_deg,
                                   transform_points)

TAG = 0.17                          # black square of an A4 print
FLOOR = -0.78                       # map z of the floor (robot_center 0.78 m above it)
T_MAP_TAG = wall_tag_in_map(distance=1.6, height_above_floor=1.1, floor_z=FLOOR, lateral=0.2)
T_AR_MAP = ar_from_map_yaw_t(1.1, [0.4, 0.9, -1.3])   # ground truth the solver must recover
RW, RH = 1280, 720
RK = K_from_hfov(RW, RH, 70.0)       # robot RGB camera (OAK-D-like)
GW, GH = 1008, 756
GK = K_from_hfov(GW, GH, 83.0)       # Spectacles setup stream


def robot_views(est, n=15, jitter=0.0, seed=0):
    rng = np.random.default_rng(seed)
    det = TagDetector()
    T_map_cv = look_at_cv([0.12, 0.0, 0.28], T_MAP_TAG[:3, 3] + [0, 0.1, -0.1], (0, 0, 1))
    for i in range(n):
        T = T_map_cv.copy()
        T[:3, 3] += rng.normal(scale=jitter, size=3)
        img = render_tag(RK, RW, RH, cam_tag(T, T_MAP_TAG), TAG, noise_sigma=2.0, seed=i)
        (_, corners), = det.detect(img)
        view, why = est.make_view(corners, RK, T)
        assert view is not None, why
        est.add(view)


def glasses_views(est, positions, T_ar_map=T_AR_MAP, glitch=None):
    """The person walks around at eye height; the Lens reports T_ar_cam (here via OpenCV)."""
    det = TagDetector()
    for i, p in enumerate(positions):
        T_map_cv = look_at_cv(p, T_MAP_TAG[:3, 3], (0, 0, 1))
        img = render_tag(GK, GW, GH, cam_tag(T_map_cv, T_MAP_TAG), TAG, noise_sigma=3.0,
                         seed=100 + i)
        found = det.detect(img)
        if not found:
            continue
        T_ar_cv = T_ar_map @ T_map_cv
        if glitch is not None and i == glitch:
            T_ar_cv = T_ar_cv @ make_T(t=[0.4, 0.0, 0.3])     # a tracking jump
        view, _ = est.make_view(found[0][1], GK, T_ar_cv)
        if view is not None:
            est.add(view)


def eye_positions(n=8, seed=3):
    rng = np.random.default_rng(seed)
    eye = FLOOR + 1.6
    return [[rng.uniform(-0.4, 0.4), rng.uniform(-1.0, 1.2), eye + rng.uniform(-0.1, 0.1)]
            for _ in range(n)]


def test_robot_anchor_from_a_standing_robot():
    est = TagPoseEstimator(TAG)
    robot_views(est)
    anchor = est.estimate()
    assert anchor.n_views == 15 and anchor.rms_px < 0.8
    assert np.linalg.norm(anchor.T_world_tag[:3, 3] - T_MAP_TAG[:3, 3]) < 0.01
    assert rotation_angle_deg(anchor.T_world_tag[:3, :3], T_MAP_TAG[:3, :3]) < 1.5
    assert anchor.baseline_m < 1e-9             # one viewpoint: nothing triangulated


def test_both_cameras_give_the_ar_map_transform():
    anchor_est = TagPoseEstimator(TAG)
    robot_views(anchor_est)
    glasses_est = TagPoseEstimator(TAG)
    glasses_views(glasses_est, eye_positions())
    anchor, glasses = anchor_est.estimate(), glasses_est.estimate()
    assert glasses.n_views >= 6 and glasses.baseline_m > 0.5
    sol = solve_ar_map(glasses.T_world_tag, anchor.T_world_tag)
    assert sol.tilt_deg < 2.0
    yaw_err = rotation_angle_deg(sol.T_ar_map[:3, :3], T_AR_MAP[:3, :3])
    assert yaw_err < 1.0
    # what the person sees: the robot (map origin) and a POI 3 m away land where they are
    pts = np.array([[0.0, 0.0, 0.0], [3.0, -1.0, 0.0], [0.0, 0.0, FLOOR]])
    err = np.linalg.norm(transform_points(sol.T_ar_map, pts) - transform_points(T_AR_MAP, pts),
                         axis=1)
    assert np.all(err < 0.05), err
    # levelled: map up is exactly AR up
    assert np.allclose(sol.T_ar_map[:3, :3] @ [0, 0, 1], [0, 1, 0])


def test_a_tracking_glitch_is_rejected():
    est = TagPoseEstimator(TAG)
    glasses_views(est, eye_positions(10, seed=4), glitch=2)
    got = est.estimate()
    assert got.n_total == 10 and got.n_views <= 9
    T_ar_tag_true = T_AR_MAP @ T_MAP_TAG
    assert np.linalg.norm(got.T_world_tag[:3, 3] - T_ar_tag_true[:3, 3]) < 0.02


def test_solve_is_exact_on_perfect_inputs_and_levels_a_tilt():
    T_ar_tag = T_AR_MAP @ T_MAP_TAG
    sol = solve_ar_map(T_ar_tag, T_MAP_TAG)
    assert np.allclose(sol.T_ar_map, T_AR_MAP, atol=1e-9)
    assert sol.yaw_deg == np.degrees(1.1).item() or math.isclose(sol.yaw_deg, math.degrees(1.1))
    # a 3 deg tilt in the Spectacles' tag estimate is removed, the tag centre is kept exact
    tilted = T_ar_tag.copy()
    tilted[:3, :3] = tilted[:3, :3] @ np.array([[1, 0, 0],
                                                [0, math.cos(0.05), -math.sin(0.05)],
                                                [0, math.sin(0.05), math.cos(0.05)]])
    sol = solve_ar_map(tilted, T_MAP_TAG)
    assert 2.0 < sol.tilt_deg < 4.0
    assert np.allclose(transform_points(sol.T_ar_map, [T_MAP_TAG[:3, 3]])[0], T_ar_tag[:3, 3])


def test_progress_grows_with_views_and_movement():
    assert registration_progress(None, 5, 0.3) == 0
    est = TagPoseEstimator(TAG)
    glasses_views(est, eye_positions(2, seed=5))
    p2 = registration_progress(est.estimate(), 6, 0.3)
    glasses_views(est, eye_positions(6, seed=6))
    p8 = registration_progress(est.estimate(), 6, 0.3)
    assert 0 < p2 < p8 <= 100


def test_inverse_consistency():
    # the glasses' camera pose in map, from the solved transform
    T_ar_cam = T_AR_MAP @ look_at_cv([0.2, 0.5, 0.8], T_MAP_TAG[:3, 3], (0, 0, 1))
    T_map_cam = inv_T(T_AR_MAP) @ T_ar_cam
    assert np.allclose(T_map_cam[:3, 3], [0.2, 0.5, 0.8])
