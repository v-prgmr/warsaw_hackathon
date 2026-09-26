import math

import numpy as np
import pytest
from synthetic import (K_from_hfov, cam_tag, look_at_cv, render_depth, render_tag,
                       wall_tag_in_map)

from g1_ar_bridge import apriltag as at
from g1_ar_bridge.geometry import inv_T, make_T, rotation_angle_deg, so3_exp

W, H = 1008, 756          # Spectacles setup-stream resolution (upstream PROTOCOL.md)
K = K_from_hfov(W, H, 83.0)
TAG = 0.16                # black square, metres


def view(position, T_world_tag, noise=2.0, seed=0, up=(0, 0, 1), size=TAG):
    T_world_cv = look_at_cv(position, T_world_tag[:3, 3], up)
    img = render_tag(K, W, H, cam_tag(T_world_cv, T_world_tag), size, noise_sigma=noise,
                     seed=seed)
    return T_world_cv, img


def test_detects_tag_zero_with_corner_order_of_the_printed_tag():
    T_tag = wall_tag_in_map()
    T_world_cv, img = view([0.0, 0.0, 0.2], T_tag)
    dets = at.TagDetector().detect(img)
    assert [d[0] for d in dets] == [0]
    corners = dets[0][1]
    expect, _ = at.project(K, inv_T(T_world_cv), at.tag_object_points(TAG)
                           @ T_tag[:3, :3].T + T_tag[:3, 3])
    assert np.max(np.linalg.norm(corners - expect, axis=1)) < 1.0   # px, same order
    assert at.TagDetector().detect(np.full((100, 100), 128, np.uint8)) == []


@pytest.mark.parametrize("position", [[0.0, 0.0, 0.2], [0.3, 0.8, 0.0], [-0.5, -1.0, 0.5]])
def test_single_view_pose_is_accurate(position):
    T_tag = wall_tag_in_map()
    T_world_cv, img = view(position, T_tag)
    (tag_id, corners), = at.TagDetector().detect(img)
    pose = at.estimate_tag_pose(corners, TAG, K)
    assert pose is not None and pose.reproj_px < 1.0
    T_est = T_world_cv @ pose.T_cam_tag
    dist = np.linalg.norm(T_tag[:3, 3] - T_world_cv[:3, 3])
    assert np.linalg.norm(T_est[:3, 3] - T_tag[:3, 3]) < 0.02 * dist   # single view: ~1-2 %
    assert rotation_angle_deg(T_est[:3, :3], T_tag[:3, :3]) < 5.0


def test_multiview_refinement_beats_single_views():
    rng = np.random.default_rng(5)
    T_tag = wall_tag_in_map(distance=2.5)
    det = at.TagDetector()
    views, singles = [], []
    for i in range(8):
        pos = [rng.uniform(-0.3, 0.8), rng.uniform(-1.2, 1.2), rng.uniform(-0.2, 0.5)]
        T_world_cv, img = view(pos, T_tag, noise=4.0, seed=i)
        found = det.detect(img)
        if not found:
            continue
        corners = found[0][1]
        pose = at.estimate_tag_pose(corners, TAG, K)
        singles.append(T_world_cv @ pose.T_cam_tag)
        views.append((K, T_world_cv, corners))
    assert len(views) >= 6
    T0 = singles[0]
    T, rms, per_view = at.refine_tag_pose(views, TAG, T0)
    assert rms < 1.0 and len(per_view) == len(views)
    err_single = np.median([rotation_angle_deg(S[:3, :3], T_tag[:3, :3]) for S in singles])
    err_multi = rotation_angle_deg(T[:3, :3], T_tag[:3, :3])
    assert err_multi < 1.0 and err_multi <= err_single + 1e-6
    assert np.linalg.norm(T[:3, 3] - T_tag[:3, 3]) < 0.01


def test_refinement_recovers_from_a_perturbed_start():
    T_tag = wall_tag_in_map(distance=2.0)
    det = at.TagDetector()
    views = []
    for i, pos in enumerate([[0, -0.8, 0.1], [0, 0.8, 0.1], [0.5, 0, 0.6]]):
        T_world_cv, img = view(pos, T_tag, seed=i)
        views.append((K, T_world_cv, det.detect(img)[0][1]))
    T0 = make_T(T_tag[:3, :3] @ so3_exp([0.1, -0.05, 0.08]), T_tag[:3, 3] + [0.05, -0.04, 0.03])
    T, rms, _ = at.refine_tag_pose(views, TAG, T0)
    assert rms < 1.0
    assert rotation_angle_deg(T[:3, :3], T_tag[:3, :3]) < 1.0
    assert np.linalg.norm(T[:3, 3] - T_tag[:3, 3]) < 0.01


def test_plane_fit_and_snap_fix_a_tilted_pose():
    T_tag = wall_tag_in_map(distance=1.4)
    T_world_cv = look_at_cv([0.0, 0.1, 0.25], T_tag[:3, 3], (0, 0, 1))
    img = render_tag(K, W, H, cam_tag(T_world_cv, T_tag), TAG)
    depth = render_depth(K, W, H, cam_tag(T_world_cv, T_tag), noise_m=0.004)
    (_, corners), = at.TagDetector().detect(img)
    rows, cols = at.quad_pixels(corners, depth.shape)
    pts_cam = at.backproject(depth, K, rows, cols)
    assert len(pts_cam) > 500
    n, p0, rms, n_in = at.fit_plane(pts_cam)
    assert rms < 0.01 and n_in > 0.9 * len(pts_cam)
    n_true = cam_tag(T_world_cv, T_tag)[:3, 2]
    assert abs(abs(float(n @ n_true)) - 1.0) < 1e-3
    # a PnP pose tilted by 6 deg and 3 cm too far is pulled back onto the plane
    T_true_cam = cam_tag(T_world_cv, T_tag)
    bad = make_T(so3_exp([0.0, math.radians(6), 0.0]) @ T_true_cam[:3, :3],
                 T_true_cam[:3, 3] * (1 + 0.03 / np.linalg.norm(T_true_cam[:3, 3])))
    fixed, angle = at.snap_tag_to_plane(bad, n, p0, np.zeros(3))
    assert angle == pytest.approx(6.0, abs=0.5)
    assert rotation_angle_deg(fixed[:3, :3], T_true_cam[:3, :3]) < 0.5
    assert np.linalg.norm(fixed[:3, 3] - T_true_cam[:3, 3]) < 0.005
    # a plane far from the tag's orientation is refused
    none, angle = at.snap_tag_to_plane(bad, [1.0, 0.0, 0.0], p0, np.zeros(3))
    assert none is None and angle > 15


def test_depth_encodings():
    mm = np.array([[0, 1500]], dtype=np.uint16)
    m = at.depth_to_metres(mm, "16UC1")
    assert np.isnan(m[0, 0]) and m[0, 1] == pytest.approx(1.5)
    f = at.depth_to_metres(np.array([[np.nan, 2.0]], dtype=np.float32), "32FC1")
    assert np.isnan(f[0, 0]) and f[0, 1] == pytest.approx(2.0)
