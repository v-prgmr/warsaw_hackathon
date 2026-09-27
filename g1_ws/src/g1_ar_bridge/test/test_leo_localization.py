"""Leo Rover in the G1 map through the shared wall tag (leo_localization, no ROS)."""
import math

import numpy as np
import pytest

from g1_ar_bridge.geometry import inv_T, make_T, rot_to_quat, rot_z, rotation_angle_deg
from g1_ar_bridge.leo_localization import (DEFAULT_CAMERA_RPY, FLIP_TAG_Z,
                                           R_BASE_OPTICAL_UPSIDE_DOWN, LeoLocalizer,
                                           camera_mount, floor_pose, leo_base_in_map,
                                           rpy_to_rot, to_our_tag_convention)
from g1_ar_bridge.world import wall_tag_pose

T_MAP_TAG = wall_tag_pose(3.0, 1.0, -0.78)                 # wall tag 3 m ahead, facing -x
T_BASE_CAM = camera_mount([0.15, 0.0, 0.12], DEFAULT_CAMERA_RPY)


def leo_at(x, y, yaw):
    return make_T(rot_z(yaw), [x, y, -0.70])


def seen_tag(T_map_base):
    """What Leo's detector reports: the tag in its camera optical frame (ours: z out)."""
    return inv_T(T_map_base @ T_BASE_CAM) @ T_MAP_TAG


def close(A, B, pos=1e-6, deg=1e-4):
    return (np.linalg.norm(A[:3, 3] - B[:3, 3]) < pos
            and rotation_angle_deg(A[:3, :3], B[:3, :3]) < deg)


def test_upside_down_mount_is_a_rotation_looking_forward():
    R = R_BASE_OPTICAL_UPSIDE_DOWN
    assert np.allclose(R @ R.T, np.eye(3)) and np.isclose(np.linalg.det(R), 1.0)
    assert np.allclose(R[:, 2], [1, 0, 0])          # optical z = Leo forward
    assert np.allclose(R[:, 1], [0, 0, 1])          # optical y (image down) = Leo up: upside down
    assert np.allclose(rpy_to_rot(*DEFAULT_CAMERA_RPY), R)


def test_leo_pose_from_the_shared_tag():
    truth = leo_at(1.2, 0.4, math.radians(15))
    assert close(leo_base_in_map(T_MAP_TAG, seen_tag(truth), T_BASE_CAM), truth)


def test_detector_convention_with_z_into_the_tag_is_detected_and_fixed():
    truth = leo_at(1.0, -0.3, math.radians(-20))
    ours = seen_tag(truth)
    fixed, flipped = to_our_tag_convention(ours @ FLIP_TAG_Z)    # y down, z into the tag
    assert flipped and close(fixed, ours)
    assert to_our_tag_convention(ours) == (pytest.approx(ours), False)
    loc = LeoLocalizer(T_BASE_CAM)
    assert close(loc.add_sighting(0.0, ours @ FLIP_TAG_Z, T_MAP_TAG), truth)
    assert loc.flipped is True


def test_odometry_carries_the_pose_between_sightings():
    loc = LeoLocalizer(T_BASE_CAM)
    # Leo's own odometry frame is unrelated to the G1 map (rotated and offset)
    T_map_leoodom = make_T(rot_z(math.radians(70)), [-2.0, 5.0, -0.70])
    def odom_of(T_map_base):
        T = inv_T(T_map_leoodom) @ T_map_base
        return T[:3, 3], rot_to_quat(T[:3, :3])
    start = leo_at(1.0, 0.0, 0.0)
    for k in range(11):                                  # standing, 10 Hz odometry
        loc.add_odom(10.0 + 0.1 * k, *odom_of(start), frames=("odom", "base_link"))
    assert close(loc.add_sighting(10.5, seen_tag(start), T_MAP_TAG), start)
    assert close(loc.T_map_odom, T_map_leoodom)
    # Leo drives 1 m forward and turns, without seeing the tag
    moved = leo_at(2.0, 0.3, math.radians(30))
    loc.add_odom(12.0, *odom_of(moved))
    assert close(loc.pose(), moved)
    x, y, yaw = floor_pose(loc.pose())
    assert (x, y) == pytest.approx((2.0, 0.3)) and yaw == pytest.approx(math.radians(30))
    assert loc.odom_frames == ("odom", "base_link")


def test_sighting_uses_odometry_interpolated_at_the_tag_stamp():
    loc = LeoLocalizer(T_BASE_CAM)
    a, b = leo_at(1.0, 0.0, 0.0), leo_at(1.2, 0.0, 0.0)      # moving 2 m/s along x
    loc.add_odom(0.0, a[:3, 3], rot_to_quat(a[:3, :3]))
    loc.add_odom(0.1, b[:3, 3], rot_to_quat(b[:3, :3]))
    mid = leo_at(1.1, 0.0, 0.0)
    loc.add_sighting(0.05, seen_tag(mid), T_MAP_TAG)
    assert close(loc.T_map_odom, np.eye(4), pos=1e-6)       # odom == map here


def test_sightings_are_rejected_when_implausible_or_without_matching_odometry():
    loc = LeoLocalizer(T_BASE_CAM, max_distance_m=2.5)
    far = leo_at(0.0, 0.0, 0.0)                              # 3 m from the tag
    assert loc.add_sighting(0.0, seen_tag(far), T_MAP_TAG) is None
    assert loc.rejected["distance"] == 1
    near = leo_at(1.5, 0.0, 0.0)
    loc.add_odom(100.0, near[:3, 3], rot_to_quat(near[:3, :3]))
    assert loc.add_sighting(50.0, seen_tag(near), T_MAP_TAG) is None   # odom 50 s away
    assert loc.rejected["no odometry at the tag stamp"] == 1
    assert loc.pose() is None
