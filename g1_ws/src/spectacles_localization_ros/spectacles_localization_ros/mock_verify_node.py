"""Report PASS/FAIL comparing ROS map pose to synthetic ground truth."""

from collections import deque
from math import degrees
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from .transforms import rotation_distance, translation_distance


class Verify(Node):
    def __init__(self):
        super().__init__("spectacles_mock_verify")
        self.truth = None
        self.state = "UNLOCALIZED"
        self.errors = deque(maxlen=30)
        self.started = time.monotonic()
        self.reported = False
        self.create_subscription(PoseStamped, "/spectacles/mock_truth", self.on_truth, 10)
        self.create_subscription(PoseStamped, "/spectacles/pose", self.on_pose, 10)
        self.create_subscription(String, "/spectacles/localization_status", self.on_status, 10)
        self.create_timer(1.0, self.check)

    def on_truth(self, msg):
        self.truth = msg

    def on_status(self, msg):
        import json
        self.state = json.loads(msg.data).get("state", "UNKNOWN")

    def on_pose(self, msg):
        if self.truth is None or self.state != "LOCALIZED":
            return
        # Both mock streams publish at 20 Hz. The latest truth can be one tick apart.
        a, b = msg.pose, self.truth.pose
        trans = translation_distance((a.position.x, a.position.y, a.position.z),
                                     (b.position.x, b.position.y, b.position.z))
        angle = degrees(rotation_distance(
            (a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w),
            (b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w)))
        self.errors.append((trans, angle))

    def check(self):
        if self.reported:
            return
        if len(self.errors) >= 15:
            max_trans = max(x for x, _ in self.errors)
            max_angle = max(y for _, y in self.errors)
            if max_trans < 0.05 and max_angle < 2.0:
                self.get_logger().info(
                    f"MOCK PASS: wearer pose <{max_trans:.3f} m, "
                    f"<{max_angle:.2f} deg from ground truth")
            else:
                self.get_logger().error(
                    f"MOCK FAIL: wearer pose error {max_trans:.3f} m, "
                    f"{max_angle:.2f} deg")
            self.reported = True
        elif time.monotonic() - self.started > 8.0:
            self.get_logger().error("MOCK FAIL: no localized pose within 8 seconds")
            self.reported = True


def main():
    rclpy.init()
    node = Verify()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
