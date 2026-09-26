"""Small ROS simulation for the projection path, without G1 or OAK-D hardware.

Publishes a color-coded planar scene and a matching LiDAR-like map cloud. The
camera optical and map frames coincide only in this fixture; real TF is used by
the annotator exactly as it would be with the calibrated chest camera.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class SyntheticScene(Node):
    def __init__(self):
        super().__init__("g1_semantic_map_synthetic_scene")
        self.width, self.height = 64, 48
        self.fx = self.fy = 50.0
        self.cx, self.cy = 31.5, 23.5
        self.rgb_pub = self.create_publisher(Image, "/synthetic/oak/rgb", 1)
        self.depth_pub = self.create_publisher(Image, "/synthetic/oak/depth", 1)
        self.info_pub = self.create_publisher(CameraInfo, "/synthetic/oak/info", 1)
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/synthetic/cloud_map", 1)
        self.tf_pub = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "oak_rgb_optical_frame"
        transform.transform.rotation.w = 1.0
        self.tf_pub.sendTransform(transform)
        self.create_timer(0.5, self.publish_scene)

    def publish_scene(self):
        header = Header(stamp=self.get_clock().now().to_msg(),
                        frame_id="oak_rgb_optical_frame")
        rgb = np.full((self.height, self.width, 3), [80, 80, 80], dtype=np.uint8)
        rgb[12:36, 8:26] = [220, 40, 30]    # red fixture surface
        rgb[20:40, 36:58] = [30, 160, 210]  # cyan fixture surface
        depth = np.full((self.height, self.width), 2.0, dtype=np.float32)
        color = Image(header=header, height=self.height, width=self.width,
                      encoding="rgb8", is_bigendian=False, step=self.width*3,
                      data=rgb.tobytes())
        depth_msg = Image(header=header, height=self.height, width=self.width,
                          encoding="32FC1", is_bigendian=False, step=self.width*4,
                          data=depth.tobytes())
        info = CameraInfo(header=header, width=self.width, height=self.height,
                          distortion_model="plumb_bob",
                          k=[self.fx, 0.0, self.cx, 0.0, self.fy, self.cy,
                             0.0, 0.0, 1.0],
                          p=[self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy,
                             0.0, 0.0, 0.0, 1.0, 0.0])
        uu, vv = np.meshgrid(np.arange(self.width), np.arange(self.height))
        points = np.stack(((uu-self.cx)*2/self.fx, (vv-self.cy)*2/self.fy,
                           np.full_like(uu, 2.0)), axis=-1).reshape(-1, 3)
        # A hidden 10x10 patch behind the red surface must remain gray/unlabeled.
        hidden_u, hidden_v = np.meshgrid(np.arange(12, 22), np.arange(18, 28))
        hidden = np.stack(((hidden_u-self.cx)*3/self.fx,
                           (hidden_v-self.cy)*3/self.fy,
                           np.full_like(hidden_u, 3.0)), axis=-1).reshape(-1, 3)
        points = np.concatenate((points, hidden), axis=0)
        cloud_header = Header(stamp=header.stamp, frame_id="map")
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(cloud_header,
                                                                points.tolist()))
        self.rgb_pub.publish(color)
        self.depth_pub.publish(depth_msg)
        self.info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticScene()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
