"""The Nav2 TF relay must never restamp or replay missing robot chains."""

from geometry_msgs.msg import TransformStamped
from rclpy.clock import ClockType
from rclpy.time import Time

from g1_nav2.navigation_tf_relay import current_navigation_transforms


def transform(parent, child, seconds):
    message = TransformStamped()
    message.header.frame_id = parent
    message.child_frame_id = child
    message.header.stamp = Time(seconds=seconds).to_msg()
    message.transform.rotation.w = 1.0
    return message


def test_relay_keeps_original_stamps_only_with_fresh_complete_chain():
    now = Time(seconds=12, clock_type=ClockType.ROS_TIME)
    map_odom = transform('map', 'odom', 12)
    odom_base = transform('odom', 'base_footprint', 12)
    waist_yaw = transform('pelvis', 'waist_yaw_link', 12)
    waist_roll = transform('waist_yaw_link', 'waist_roll_link', 12)
    torso = transform('waist_roll_link', 'torso_link', 12)
    old_joint = transform('base_footprint', 'old_joint', 10)
    frames = {'odom': map_odom, 'base_footprint': odom_base,
              'waist_yaw_link': waist_yaw, 'waist_roll_link': waist_roll,
              'torso_link': torso,
              'old_joint': old_joint}
    result = current_navigation_transforms(frames, now)
    assert result == [map_odom, odom_base, waist_yaw, waist_roll, torso]
    assert result[0].header.stamp == map_odom.header.stamp

    frames['base_footprint'] = transform('odom', 'base_footprint', 10)
    assert current_navigation_transforms(frames, now) == []

    frames['base_footprint'] = odom_base
    del frames['torso_link']
    assert current_navigation_transforms(frames, now) == []

    frames['torso_link'] = transform('wrong_parent', 'torso_link', 12)
    assert current_navigation_transforms(frames, now) == []
