import math

from humanoid_nav_bridge.mujoco_to_gazebo_node import estimate_body_twist


def estimate(previous, stamp, x, y, yaw):
    return estimate_body_twist(previous, stamp, x, y, yaw, 0.02, 1.0, 2.0)


def test_repeated_clock_does_not_create_velocity_spikes_or_lose_baseline():
    previous, twist = estimate(None, 1.0, 0.0, 0.0, 0.0)
    assert twist == (0.0, 0.0, 0.0)

    repeated, twist = estimate(previous, 1.0, 0.02, 0.0, 0.01)
    assert repeated == previous
    assert twist == (0.0, 0.0, 0.0)

    too_soon, twist = estimate(previous, 1.01, 0.03, 0.0, 0.02)
    assert too_soon == previous
    assert twist == (0.0, 0.0, 0.0)

    updated, twist = estimate(previous, 1.1, 0.05, 0.0, 0.0)
    assert updated[0] == 1.1
    assert math.isclose(twist[0], 0.5)
    assert twist[1:] == (0.0, 0.0)


def test_velocity_is_expressed_in_base_frame_and_yaw_wraps():
    _, (vx, vy, wz) = estimate((0.0, 0.0, 0.0, math.pi / 2), 1.0, 0.0, 1.0, math.pi / 2)
    assert math.isclose(vx, 1.0)
    assert math.isclose(vy, 0.0, abs_tol=1e-9)
    assert wz == 0.0

    _, (_, _, wz) = estimate((1.0, 0.0, 0.0, math.pi - 0.1), 1.1, 0.0, 0.0, -math.pi + 0.1)
    assert math.isclose(wz, 2.0)


def test_clock_reset_and_pose_jump_do_not_publish_implausible_twist():
    current, twist = estimate((5.0, 2.0, 1.0, 0.0), 1.0, 0.0, 0.0, 0.0)
    assert current == (1.0, 0.0, 0.0, 0.0)
    assert twist == (0.0, 0.0, 0.0)

    _, (vx, vy, wz) = estimate(current, 1.1, 2.0, 2.0, 2.0)
    assert math.isclose(math.hypot(vx, vy), 1.0)
    assert math.isclose(wz, 2.0)
