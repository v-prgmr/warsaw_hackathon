"""Synthetic moving wearer and AprilTag observations for RViz/offline testing."""

import json
from math import cos, sin

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from .protocol import pose_dict
from .transforms import Transform, compose, inverse


def yaw(angle):
    return (0.0, 0.0, sin(angle / 2), cos(angle / 2))


MAP_WORLD = Transform((1.1, -0.7, 0.15), yaw(0.43))
MAP_TAG = Transform((2.0, 1.0, 1.2), yaw(-0.3))


def sample(index):
    t = index * 0.05
    local = Transform((0.3 + 0.25*t, 0.12*sin(0.7*t), 1.65),
                      yaw(0.08*sin(0.4*t)))
    device_tag = compose(inverse(local), compose(inverse(MAP_WORLD), MAP_TAG))
    # Marked observations stop after ~1.25 s. Tracking continues without the tag.
    tag_seen = index < 25
    payload = {
        "schema_version": 1,
        "coordinate_system": "ros_rh_m",
        "session_id": "mock_session_1",
        "timestamp_sec": t,
        "tracking_valid": True,
        "device_pose": pose_dict(local),
        "tag_id": 0 if tag_seen else None,
        "tag_pose_device": pose_dict(device_tag) if tag_seen else None,
    }
    return payload, compose(MAP_WORLD, local)


class MockPublisher(Node):
    def __init__(self):
        super().__init__("spectacles_mock")
        self.observations = self.create_publisher(String, "/spectacles/observation", 10)
        self.truth = self.create_publisher(PoseStamped, "/spectacles/mock_truth", 10)
        self.counter = 0
        self.create_timer(0.05, self.tick)

    def tick(self):
        payload, truth = sample(self.counter)
        msg = String()
        msg.data = json.dumps(payload)
        self.observations.publish(msg)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = truth.position
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = truth.orientation
        self.truth.publish(pose)
        self.counter += 1


def main():
    rclpy.init()
    node = MockPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
