"""Synthetic RGB-D + odom publisher for OFFLINE testing of keyframe_node (R3).

Publishes color, aligned depth (16UC1 mm), camera_info and odom on the same topics keyframe_node
subscribes to, with matching timestamps so ApproximateTimeSynchronizer fires. Every Nth frame is
deliberately blurred and the odom pose advances in +x, so you can watch keyframe_node accept sharp,
well-separated frames and reject blurry ones. Not for robot use.
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class FakeRgbd(Node):
    def __init__(self):
        super().__init__("fake_rgbd_pub")
        self.color_pub = self.create_publisher(Image, "/camera/color/image_raw", 10)
        self.depth_pub = self.create_publisher(
            Image, "/camera/aligned_depth_to_color/image_raw", 10)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/color/camera_info", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.bridge = CvBridge()
        self.i = 0
        self.timer = self.create_timer(0.2, self.tick)  # 5 Hz
        self.get_logger().info("fake_rgbd_pub: every 3rd frame is blurred; odom advances in +x")

    def tick(self):
        now = self.get_clock().now().to_msg()
        w, h = 640, 480

        # A textured scene (random-ish blocks) so Laplacian variance is high when sharp.
        rng = np.random.default_rng(self.i)
        img = (rng.integers(0, 255, size=(h, w, 3))).astype(np.uint8)
        img = cv2.resize(cv2.resize(img, (80, 60)), (w, h), interpolation=cv2.INTER_NEAREST)
        blurred = (self.i % 3 == 2)
        if blurred:
            img = cv2.GaussianBlur(img, (31, 31), 12)

        color = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        color.header.stamp = now
        color.header.frame_id = "camera_color_optical_frame"

        depth_mm = np.full((h, w), 1500, dtype=np.uint16)  # flat 1.5 m wall
        depth = self.bridge.cv2_to_imgmsg(depth_mm, encoding="16UC1")
        depth.header = color.header

        info = CameraInfo()
        info.header = color.header
        info.width, info.height = w, h
        info.distortion_model = "plumb_bob"
        info.k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base"
        odom.pose.pose.position.x = 0.1 * self.i  # 10 cm per frame
        odom.pose.pose.orientation.w = 1.0

        self.color_pub.publish(color)
        self.depth_pub.publish(depth)
        self.info_pub.publish(info)
        self.odom_pub.publish(odom)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = FakeRgbd()
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
