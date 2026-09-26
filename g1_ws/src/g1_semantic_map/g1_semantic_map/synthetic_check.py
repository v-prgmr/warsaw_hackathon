"""Fail-fast ROS smoke assertion for the synthetic color projection fixture."""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class SyntheticCheck(Node):
    def __init__(self):
        super().__init__("g1_semantic_map_synthetic_check")
        self.complete = False
        self.success = False
        self.create_subscription(PointCloud2, "/g1_semantic_map/cloud", self.on_cloud, 1)

    def on_cloud(self, msg):
        points = point_cloud2.read_points(
            msg, field_names=("x", "y", "z", "rgb", "label"), skip_nans=True)
        if len(points) != 3172 or msg.header.frame_id != "map":
            return
        rgb_bits = points["rgb"].view(np.uint32)
        rgb = np.column_stack(((rgb_bits >> 16) & 255,
                               (rgb_bits >> 8) & 255,
                               rgb_bits & 255))
        red = np.count_nonzero((rgb[:, 0] > 180) & (rgb[:, 1] < 70))
        cyan = np.count_nonzero((rgb[:, 2] > 180) & (rgb[:, 1] > 110))
        gray = np.count_nonzero(np.all(rgb == 80, axis=1))
        chair = np.count_nonzero(points["label"] == 62)
        table = np.count_nonzero(points["label"] == 67)
        hidden = points["z"] > 2.5
        occlusion_ok = (np.count_nonzero(hidden) == 100 and
                        np.all(points["label"][hidden] == 0) and
                        np.all(rgb[hidden] == 128))
        self.success = (red > 300 and cyan > 300 and gray > 1000 and
                        chair > 300 and table > 300 and occlusion_ok)
        self.complete = True
        self.get_logger().info(f"synthetic colors: red={red}, cyan={cyan}, gray={gray}; "
                               f"classes: chair={chair}, table={table}; "
                               f"hidden occlusion={'PASS' if occlusion_ok else 'FAIL'}; "
                               f"{'PASS' if self.success else 'FAIL'}")


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticCheck()
    deadline = node.get_clock().now().nanoseconds + 10_000_000_000
    while not node.complete and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    success = node.success
    node.destroy_node()
    rclpy.shutdown()
    if not success:
        raise SystemExit("synthetic color projection failed or timed out")


if __name__ == "__main__":
    main()
