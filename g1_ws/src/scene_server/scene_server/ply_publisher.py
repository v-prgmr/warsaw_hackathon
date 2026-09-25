"""STUB scene publisher (owned by D).

Publishes a synthetic colored point cloud on /scene_cloud in the `map` frame, so RViz + the M3
topic/frame contract exist before the real metric map is ready. The real scene_server must keep
the same topic name and frame.

No open3d/plyfile dependency: the cloud is generated with numpy (a gray floor plane + a colored
box). If a real .ply path is passed via the `ply_path` parameter later, load that instead.
"""
import struct

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def _pack_rgb(r, g, b):
    """Pack 0-255 ints into the float32 'rgb' field RViz expects."""
    rgb_uint32 = (int(r) << 16) | (int(g) << 8) | int(b)
    return struct.unpack("f", struct.pack("I", rgb_uint32))[0]


def _make_synthetic_cloud():
    """Return a list of (x, y, z, rgb_float) points: a floor plane + a box."""
    pts = []

    # 5 m x 5 m floor grid at 5 cm resolution, gray.
    gray = _pack_rgb(120, 120, 120)
    coords = np.arange(-2.5, 2.5, 0.05)
    for x in coords:
        for y in coords:
            pts.append((float(x), float(y), 0.0, gray))

    # A 0.6 m colored box sitting on the floor near the origin (a stand-in "object").
    box = np.arange(0.0, 0.6, 0.02)
    orange = _pack_rgb(230, 120, 30)
    for x in box:
        for y in box:
            for z in box:
                # keep only the shell to save points
                on_shell = (
                    x < 0.02 or x > 0.58 or y < 0.02 or y > 0.58 or z < 0.02 or z > 0.58
                )
                if on_shell:
                    pts.append((float(x + 1.0), float(y - 0.3), float(z), orange))
    return pts


class PlyPublisher(Node):
    def __init__(self):
        super().__init__("scene_server_stub")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("rate_hz", 1.0)
        self.frame_id = self.get_parameter("frame_id").value

        # Latch the cloud so RViz opened after startup still receives it.
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(PointCloud2, "/scene_cloud", qos)

        self.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        self.points = _make_synthetic_cloud()
        self.get_logger().info(
            f"Synthetic scene: {len(self.points)} points in frame '{self.frame_id}'"
        )

        rate = float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self._publish_cloud)

    def _publish_cloud(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        msg = point_cloud2.create_cloud(header, self.fields, self.points)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
