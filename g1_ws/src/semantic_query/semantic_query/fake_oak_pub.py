"""Synthetic OAK-D RGB+depth+camera_info publisher for OFFLINE testing of poi_node (mock backend).

Publishes on the OAK topic names with matching stamps so the ApproximateTimeSynchronizer fires:
a textured RGB frame and a flat depth (default 1.5 m) so backprojection yields a point ~1.5 m in
front of the camera optical frame. Not for robot use.
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class FakeOak(Node):
    def __init__(self):
        super().__init__("fake_oak_pub")
        self.rgb = self.create_publisher(Image, "/oak/rgb/image_raw", 10)
        self.depth = self.create_publisher(Image, "/oak/stereo/image_raw", 10)
        self.info = self.create_publisher(CameraInfo, "/oak/rgb/camera_info", 10)
        self.bridge = CvBridge()
        self.depth_m = float(self.declare_parameter("depth_m", 1.5).value)
        self.i = 0
        self.create_timer(0.2, self.tick)  # 5 Hz
        self.get_logger().info("fake_oak_pub: /oak/rgb/image_raw + /oak/stereo/image_raw (flat depth)")

    def tick(self):
        now = self.get_clock().now().to_msg()
        w, h = 640, 480
        rng = np.random.default_rng(self.i)
        img = cv2.resize(rng.integers(0, 255, size=(60, 80, 3)).astype(np.uint8),
                         (w, h), interpolation=cv2.INTER_NEAREST)
        rgb = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        rgb.header.stamp = now
        rgb.header.frame_id = "oak_rgb_camera_optical_frame"

        depth_mm = np.full((h, w), int(self.depth_m * 1000), dtype=np.uint16)
        depth = self.bridge.cv2_to_imgmsg(depth_mm, encoding="16UC1")
        depth.header = rgb.header

        info = CameraInfo()
        info.header = rgb.header
        info.width, info.height = w, h
        info.distortion_model = "plumb_bob"
        info.k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.rgb.publish(rgb)
        self.depth.publish(depth)
        self.info.publish(info)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = FakeOak()
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
