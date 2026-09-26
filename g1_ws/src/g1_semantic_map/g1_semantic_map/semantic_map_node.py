"""Project OAK-D pixels onto RTAB-Map's LiDAR cloud; never publish map TF."""

import json
import struct

import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener

from .fusion import VoxelFusion, project_visible, transform_matrix


FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="label", offset=16, datatype=PointField.UINT16, count=1),
    PointField(name="confidence", offset=20, datatype=PointField.FLOAT32, count=1),
]


def pack_rgb(rgb):
    value = (int(rgb[0]) << 16) | (int(rgb[1]) << 8) | int(rgb[2])
    return struct.unpack("<f", struct.pack("<I", value))[0]


class SemanticMapNode(Node):
    def __init__(self):
        super().__init__("g1_semantic_map")
        for name, default in (
            ("cloud_topic", "/cloud_map"), ("rgb_topic", ""),
            ("depth_topic", ""), ("camera_info_topic", ""),
            ("output_topic", "/g1_semantic_map/cloud"),
            ("detector", "coco_maskrcnn"), ("device", "cpu"),
            ("max_rate_hz", 1.0), ("max_points", 200000),
            ("voxel_m", 0.04), ("depth_tolerance_m", 0.10),
            ("relative_depth_tolerance", 0.04), ("score_threshold", 0.65),
            ("sync_slop_s", 0.05),
        ):
            self.declare_parameter(name, default)
        p = lambda name: self.get_parameter(name).value
        for name in ("rgb_topic", "depth_topic", "camera_info_topic"):
            if not p(name):
                raise ValueError(f"{name} must be set to a verified OAK-D ROS topic")
        if p("max_rate_hz") <= 0 or p("max_points") <= 0:
            raise ValueError("max_rate_hz and max_points must be positive")
        self.fusion = VoxelFusion(voxel_m=float(p("voxel_m")))
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cloud = None
        self.cloud_frame = None
        self.last_stamp_s = None
        mode = p("detector")
        if mode == "coco_maskrcnn":
            from .detector import CocoMaskRCNN
            self.detector = CocoMaskRCNN(score_threshold=p("score_threshold"),
                                         device=p("device"))
            self.get_logger().info("COCO Mask R-CNN loaded")
        elif mode == "none":
            self.detector = None
            self.get_logger().warn("detector=none: RGB only; no semantic labels")
        elif mode == "synthetic_oracle":
            from .detector import SyntheticPaletteDetector
            self.detector = SyntheticPaletteDetector()
            self.get_logger().warn("Fixture-only synthetic_oracle; not real perception")
        else:
            raise ValueError("detector must be coco_maskrcnn, none or synthetic_oracle")
        self.publisher = self.create_publisher(PointCloud2, p("output_topic"), 1)
        if self.detector is not None:
            class_qos = QoSProfile(depth=1)
            class_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.classes_publisher = self.create_publisher(
                String, "/g1_semantic_map/class_names", class_qos)
            categories = self.detector.categories
            names = (categories if isinstance(categories, dict) else
                     {i: name for i, name in enumerate(categories)})
            self.classes_publisher.publish(String(data=json.dumps(
                {str(i): name for i, name in names.items()})))
        self.create_subscription(PointCloud2, p("cloud_topic"), self.on_cloud,
                                 qos_profile_sensor_data)
        subscriptions = [Subscriber(self, typ, topic, qos_profile=qos_profile_sensor_data)
                         for typ, topic in ((Image, p("rgb_topic")),
                                            (Image, p("depth_topic")),
                                            (CameraInfo, p("camera_info_topic")))]
        self.sync = ApproximateTimeSynchronizer(subscriptions, queue_size=5,
                                                slop=float(p("sync_slop_s")))
        self.sync.registerCallback(self.on_rgbd)
        self.get_logger().info("Waiting for /cloud_map and synchronized rectified OAK-D RGB/depth/info")

    def on_cloud(self, msg):
        if not msg.header.frame_id:
            self.get_logger().warn("Ignoring cloud without frame_id")
            return
        points = point_cloud2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=True).astype(np.float32)
        if not len(points):
            return
        points = points.reshape(-1, 3)
        stride = max(1, int(np.ceil(len(points) / self.get_parameter("max_points").value)))
        self.cloud = points[::stride].copy()
        self.cloud_frame = msg.header.frame_id

    def on_rgbd(self, rgb_msg, depth_msg, info_msg):
        if self.cloud is None:
            return
        stamp = rgb_msg.header.stamp
        stamp_s = stamp.sec + stamp.nanosec * 1e-9
        interval = 1.0 / self.get_parameter("max_rate_hz").value
        if self.last_stamp_s is not None:
            if stamp_s < self.last_stamp_s:
                self.fusion.voxels.clear()  # rosbag /clock rewind
            elif stamp_s - self.last_stamp_s < interval:
                return
        frame = info_msg.header.frame_id
        if not frame or frame != rgb_msg.header.frame_id:
            self.get_logger().warn("RGB and CameraInfo optical frame IDs must match")
            return
        if abs((depth_msg.header.stamp.sec + depth_msg.header.stamp.nanosec * 1e-9) - stamp_s) > self.get_parameter("sync_slop_s").value:
            self.get_logger().warn("Depth timestamp too far from RGB")
            return
        try:
            tf = self.tf_buffer.lookup_transform(frame, self.cloud_frame,
                                                 Time.from_msg(stamp),
                                                 timeout=Duration(seconds=0.2))
        except TransformException as exc:
            self.get_logger().warn(f"No camera<-cloud TF at image time: {exc}")
            return
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
        depth = np.asarray(self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough"))
        if depth.shape != rgb.shape[:2] or rgb.shape[1] != info_msg.width or rgb.shape[0] != info_msg.height:
            self.get_logger().warn("RGB, aligned depth and CameraInfo dimensions differ")
            return
        if depth_msg.encoding == "16UC1":
            depth = depth.astype(np.float32) * 0.001
        elif depth_msg.encoding == "32FC1":
            depth = depth.astype(np.float32)
        else:
            self.get_logger().warn(f"Unsupported depth encoding {depth_msg.encoding}")
            return
        # Rectified CameraInfo P matches the undistorted RGB image. Distorted raw
        # frames are not supported; projection with K alone would be wrong there.
        if info_msg.p[0] <= 0 or info_msg.p[5] <= 0:
            self.get_logger().warn("CameraInfo P is missing; use a rectified RGB stream")
            return
        intrinsics = (info_msg.p[0], info_msg.p[5], info_msg.p[2], info_msg.p[6])
        tr, q = tf.transform.translation, tf.transform.rotation
        t_camera_cloud = transform_matrix((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))
        indices, u, v = project_visible(
            self.cloud, t_camera_cloud, intrinsics, depth,
            tolerance_m=self.get_parameter("depth_tolerance_m").value,
            relative_tolerance=self.get_parameter("relative_depth_tolerance").value)
        if self.detector is None:
            labels = np.zeros(rgb.shape[:2], dtype=np.uint16)
            scores = np.zeros(rgb.shape[:2], dtype=np.float32)
        else:
            labels, scores = self.detector.predict(rgb)
        self.fusion.observe(self.cloud, indices, rgb[v, u], labels[v, u],
                            scores[v, u], stamp_s)
        self.last_stamp_s = stamp_s
        colors, classes, confidence = self.fusion.render(self.cloud)
        header = Header(stamp=stamp, frame_id=self.cloud_frame)
        rows = [(float(x), float(y), float(z), pack_rgb(c), int(label), float(score))
                for (x, y, z), c, label, score in zip(self.cloud, colors, classes, confidence)]
        self.publisher.publish(point_cloud2.create_cloud(header, FIELDS, rows))
        self.get_logger().info(f"Projected {len(indices)}/{len(self.cloud)} points; "
                               f"{int(np.count_nonzero(classes))} labeled")


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
