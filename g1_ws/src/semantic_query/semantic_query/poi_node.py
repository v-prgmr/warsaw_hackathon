"""semantic_query poi_node — on-demand NL object detection to a 3D POI in the map frame.

Send a query on /semantic_query/query (std_msgs/String), e.g. "red bottle". The node runs the
detector on the latest synced OAK-D RGB+depth frame, backprojects the detection with the aligned
depth, transforms it into the map frame, and publishes:
  * /ar_glasses/markers  (visualization_msgs/MarkerArray, ns="semantic_query"): a sphere + label
  * /semantic_query/poi  (std_msgs/String, JSON): {id,label,xyz,frame_id,confidence,stamp}

Grounding DINO + SAM2 run on a CUDA host; with no torch (or on load failure) and allow_mock:=true
the node falls back to the MockDetector so the pipeline still runs offline.
"""
import json

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

import tf2_geometry_msgs  # noqa: F401  (registers PointStamped do_transform)

from .backproject import CameraIntrinsics, backproject
from .detectors import MockDetector, make_detector

POI_COLOR = (0.1, 1.0, 0.3)   # green


class PoiNode(Node):
    def __init__(self):
        super().__init__("poi_node")
        g = self.declare_parameter
        self.rgb_topic = g("rgb_topic", "/oak/rgb/image_raw").value
        self.depth_topic = g("depth_topic", "/oak/stereo/image_raw").value
        self.info_topic = g("info_topic", "/oak/rgb/camera_info").value
        self.optical_frame = g("optical_frame", "oak_rgb_camera_optical_frame").value
        self.map_frame = g("map_frame", "map").value
        self.depth_scale = float(g("depth_scale", 0.001).value)
        self.max_depth_m = float(g("max_depth_m", 6.0).value)
        self.min_depth_m = float(g("min_depth_m", 0.2).value)
        self.backend = g("backend", "grounding_dino_sam2").value
        self.allow_mock = bool(g("allow_mock", True).value)
        self.min_confidence = float(g("min_confidence", 0.30).value)
        det_kwargs = dict(
            device=g("device", "cuda").value,
            box_threshold=float(g("box_threshold", 0.35).value),
            text_threshold=float(g("text_threshold", 0.25).value),
            use_sam2=bool(g("use_sam2", True).value),
            gdino_weights=g("gdino_weights", "").value,
            sam2_weights=g("sam2_weights", "").value,
        )
        self.sync_slop_s = float(g("sync_slop_s", 0.10).value)
        self.sync_queue = int(g("sync_queue", 15).value)

        self.detector = self._make_detector(det_kwargs)

        self.bridge = CvBridge()
        self.latest = None       # (bgr, depth_raw, CameraIntrinsics, stamp)
        self._label_ids = {}
        self._next_id = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        rgb = Subscriber(self, Image, self.rgb_topic, qos_profile=qos_profile_sensor_data)
        depth = Subscriber(self, Image, self.depth_topic, qos_profile=qos_profile_sensor_data)
        info = Subscriber(self, CameraInfo, self.info_topic, qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer(
            [rgb, depth, info], queue_size=self.sync_queue, slop=self.sync_slop_s)
        self.sync.registerCallback(self.on_frame)

        self.marker_pub = self.create_publisher(MarkerArray, "/ar_glasses/markers", 10)
        self.poi_pub = self.create_publisher(String, "/semantic_query/poi", 10)
        self.create_subscription(String, "/semantic_query/query", self.on_query, 10)

        self.get_logger().info(
            f"poi_node up (backend={self.backend}). rgb={self.rgb_topic} depth={self.depth_topic} "
            f"info={self.info_topic}; query on /semantic_query/query")

    def _make_detector(self, det_kwargs):
        if self.backend == "mock":
            return MockDetector()
        try:
            det = make_detector(self.backend, **det_kwargs).load()
            self.get_logger().info(f"loaded detector backend '{self.backend}'")
            return det
        except Exception as exc:  # noqa: BLE001
            if self.allow_mock:
                self.get_logger().warning(
                    f"detector '{self.backend}' unavailable ({exc}); falling back to MockDetector")
                return MockDetector()
            raise

    def on_frame(self, rgb_msg, depth_msg, info_msg):
        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth = np.asarray(self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough"))
        intr = CameraIntrinsics.from_k(info_msg.k)
        self.latest = (bgr, depth, intr, rgb_msg.header.stamp)

    def on_query(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        if self.latest is None:
            self.get_logger().warning("query received but no camera frame yet")
            return
        bgr, depth, intr, stamp = self.latest

        det = self.detector.detect(bgr, text)
        if det is None or det.score < self.min_confidence:
            self.get_logger().info(
                f"'{text}': no detection above {self.min_confidence:.2f}"
                + (f" (best {det.score:.2f})" if det else ""))
            return

        p = backproject(depth, intr, det.box_xyxy, det.mask, depth_scale=self.depth_scale,
                        min_depth_m=self.min_depth_m, max_depth_m=self.max_depth_m)
        if p is None:
            self.get_logger().warning(f"'{text}': detected but no valid depth in the region")
            return

        pt = PointStamped()
        pt.header.frame_id = self.optical_frame
        pt.header.stamp = rclpy.time.Time().to_msg()   # latest available transform
        pt.point.x, pt.point.y, pt.point.z = p
        try:
            pt_map = self.tf_buffer.transform(pt, self.map_frame, timeout=rclpy.duration.Duration(seconds=0.5))
        except TransformException as exc:
            self.get_logger().warning(f"TF {self.optical_frame}->{self.map_frame} failed: {exc}")
            return

        xyz = (pt_map.point.x, pt_map.point.y, pt_map.point.z)
        poi_id = self._id_for(det.label)
        self._publish_markers(poi_id, det.label, det.score, xyz)
        self._publish_poi(poi_id, det.label, det.score, xyz)
        self.get_logger().info(
            f"POI '{det.label}' ({det.score:.2f}) at map ({xyz[0]:.2f}, {xyz[1]:.2f}, {xyz[2]:.2f})")

    def _id_for(self, label):
        if label not in self._label_ids:
            self._label_ids[label] = self._next_id
            self._next_id += 1
        return self._label_ids[label]

    def _publish_markers(self, poi_id, label, score, xyz):
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()

        sphere = Marker()
        sphere.header.frame_id = self.map_frame
        sphere.header.stamp = now
        sphere.ns, sphere.id, sphere.type, sphere.action = "semantic_query", poi_id * 2, Marker.SPHERE, Marker.ADD
        sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = xyz
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.10
        sphere.color.r, sphere.color.g, sphere.color.b = POI_COLOR
        sphere.color.a = 0.9
        arr.markers.append(sphere)

        text = Marker()
        text.header.frame_id = self.map_frame
        text.header.stamp = now
        text.ns, text.id, text.type, text.action = "semantic_query", poi_id * 2 + 1, Marker.TEXT_VIEW_FACING, Marker.ADD
        text.pose.position.x, text.pose.position.y, text.pose.position.z = xyz[0], xyz[1], xyz[2] + 0.12
        text.pose.orientation.w = 1.0
        text.scale.z = 0.08
        text.color.r, text.color.g, text.color.b, text.color.a = POI_COLOR[0], POI_COLOR[1], POI_COLOR[2], 1.0
        text.text = f"{label} ({score:.2f})"
        arr.markers.append(text)

        self.marker_pub.publish(arr)

    def _publish_poi(self, poi_id, label, score, xyz):
        payload = {
            "id": poi_id,
            "label": label,
            "xyz": [round(v, 4) for v in xyz],
            "frame_id": self.map_frame,
            "confidence": round(float(score), 4),
            "stamp": self.get_clock().now().nanoseconds,
        }
        self.poi_pub.publish(String(data=json.dumps(payload)))


def main(args=None):
    rclpy.init(args=args)
    node = PoiNode()
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
