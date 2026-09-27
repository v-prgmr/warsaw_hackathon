"""semantic_query poi_node — search for an object by name and mark it with a 3D box.

Type "red cup" (or "search for a red cup") in the AR glasses, or publish it on
/semantic_query/query. The node then SEARCHES: it runs Grounding DINO + SAM2 on new OAK-D frames
(chest camera) for up to ``search_timeout_s`` until it finds the object, backprojects the SAM2
mask with the aligned depth, transforms the points into ``map`` and fits a gravity-aligned 3D
box. It publishes:
  * /ar_glasses/markers   (MarkerArray, ns="semantic_query"): the 3D box + label ("red cup
                          (0.62)"), drawn in the glasses by g1_ar_bridge; ns="semantic_status":
                          "searching: red cup" above the robot while searching
  * /ar_glasses/reply     (String): "Searching for red cup…", "Found red cup …", "No red cup …"
                          shown in the glasses' assistant panel
  * /semantic_query/poi   (String, JSON): {id,label,query,xyz,frame_id,confidence,box,stamp}
                          (AGENTS.md §12 POI contract, for the Leo handoff)
  * /semantic_query/image (Image): the last frame with the 2D box and mask, for RViz
"stop" stops a search; "clear" removes the boxes. Read-only towards the robot: it never moves.
Heavy inference runs in its own process / container (AGENTS.md §25.3), in a worker thread.
"""
import concurrent.futures
import json
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .backproject import CameraIntrinsics
from .box3d import gravity_box, region_points, transform_points
from .commands import parse_command
from .detectors import MockDetector, make_detector

BOX_COLOR = (0.1, 1.0, 0.3)       # green, like the demo scene's box
STATUS_COLOR = (1.0, 0.85, 0.2)


def tf_to_T(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = [t.x, t.y, t.z]
    return T


class PoiNode(Node):
    def __init__(self):
        super().__init__("poi_node")
        g = self.declare_parameter
        self.rgb_topic = g("rgb_topic", "/oak/rgb/image_raw").value
        self.depth_topic = g("depth_topic", "/oak/stereo/image_raw").value
        self.info_topic = g("info_topic", "/oak/rgb/camera_info").value
        self.optical_frame = g("optical_frame", "oak_rgb_camera_optical_frame").value
        self.map_frame = g("map_frame", "map").value
        self.robot_frame = g("robot_frame", "robot_center").value
        self.depth_scale = float(g("depth_scale", 0.001).value)
        self.max_depth_m = float(g("max_depth_m", 6.0).value)
        self.min_depth_m = float(g("min_depth_m", 0.2).value)
        self.backend = g("backend", "grounding_dino_sam2").value
        self.allow_mock = bool(g("allow_mock", True).value)
        self.min_confidence = float(g("min_confidence", 0.30).value)
        self.search_timeout_s = float(g("search_timeout_s", 20.0).value)
        self.search_period_s = float(g("search_period_s", 0.3).value)
        query_topics = list(g("query_topics", ["/semantic_query/query",
                                               "/ar_glasses/user_command"]).value)
        det_kwargs = dict(
            device=g("device", "auto").value,
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
        self.latest = None       # (bgr, depth_raw, CameraIntrinsics, stamp, frame_id)
        self.frame_seq = 0
        self._label_ids = {}
        self._next_id = 0
        self.search = None       # dict(query, started, attempts, used_seq, best) while searching
        self.pool = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self.job = None          # (future, frame tuple, search)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        rgb = Subscriber(self, Image, self.rgb_topic, qos_profile=qos_profile_sensor_data)
        depth = Subscriber(self, Image, self.depth_topic, qos_profile=qos_profile_sensor_data)
        info = Subscriber(self, CameraInfo, self.info_topic, qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer(
            [rgb, depth, info], queue_size=self.sync_queue, slop=self.sync_slop_s)
        self.sync.registerCallback(self.on_frame)

        self.marker_pub = self.create_publisher(MarkerArray, "/ar_glasses/markers", 10)
        self.reply_pub = self.create_publisher(
            String, g("reply_topic", "/ar_glasses/reply").value, 10)
        self.poi_pub = self.create_publisher(String, "/semantic_query/poi", 10)
        self.image_pub = self.create_publisher(Image, "/semantic_query/image", 1)
        for topic in query_topics:
            self.create_subscription(String, topic, self.on_query, 10)
        self.create_timer(0.05, self.search_tick)

        self.get_logger().info(
            f"poi_node up (detector={getattr(self.detector, 'name', self.backend)}, device="
            f"{getattr(self.detector, 'device', '-')}). rgb={self.rgb_topic} "
            f"depth={self.depth_topic}; queries on {', '.join(query_topics)}")

    def _make_detector(self, det_kwargs):
        if self.backend == "mock":
            return MockDetector()
        try:
            det = make_detector(self.backend, **det_kwargs).load()
            self.get_logger().info(f"loaded detector backend '{self.backend}' on {det.device}"
                                   + (f" (SAM2 off: {det.sam2_error})" if det.sam2_error else ""))
            return det
        except Exception as exc:  # noqa: BLE001
            if self.allow_mock:
                self.get_logger().warning(
                    f"detector '{self.backend}' unavailable ({exc}); falling back to MockDetector")
                return MockDetector()
            raise

    # --- inputs --------------------------------------------------------------------------

    def on_frame(self, rgb_msg, depth_msg, info_msg):
        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth = np.asarray(self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough"))
        intr = CameraIntrinsics.from_k(info_msg.k)
        self.latest = (bgr, depth, intr, rgb_msg.header.stamp,
                       rgb_msg.header.frame_id or self.optical_frame)
        self.frame_seq += 1

    def on_query(self, msg: String):
        action, what = parse_command(msg.data)
        if action == "stop":
            if self.search:
                self.reply(f"Stopped searching for {self.search['query']}.")
            self.search = None
            self.status_marker(None)
        elif action == "clear":
            self.search = None
            self.status_marker(None)
            arr = MarkerArray()
            for ns in ("semantic_query", "semantic_status"):
                m = Marker()
                m.header.frame_id = self.map_frame
                m.ns, m.action = ns, Marker.DELETEALL
                arr.markers.append(m)
            self.marker_pub.publish(arr)
            self.reply("Cleared the found objects.")
        elif action == "search":
            self.search = {"query": what, "started": time.monotonic(), "attempts": 0,
                           "used_seq": self.frame_seq, "best": None}
            self.reply(f"Searching for {what}…" + ("" if self.latest is not None
                                                   else " (no camera image yet)"))
            self.status_marker(f"searching: {what}")
            self.get_logger().info(f"search '{what}' started")

    # --- search loop ---------------------------------------------------------------------

    def search_tick(self):
        if self.job is not None:
            future, frame, search = self.job
            if not future.done():
                return
            self.job = None
            try:
                det = future.result()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"detector failed: {exc}")
                det = None
            if search is self.search:
                self.on_result(search, frame, det)
        s = self.search
        if s is None:
            return
        now = time.monotonic()
        if now - s["started"] > self.search_timeout_s:
            best = s["best"]
            self.reply(f"No {s['query']} found in {self.search_timeout_s:.0f} s"
                       + (f" (best guess {best:.2f}, below {self.min_confidence:.2f})."
                          if best is not None else ".") + " Turn the robot and try again.")
            self.get_logger().info(f"search '{s['query']}': not found after {s['attempts']} frames")
            self.search = None
            self.status_marker(None)
            return
        if self.latest is None or self.frame_seq <= s["used_seq"]:
            return
        if now - s.get("last_attempt", 0.0) < self.search_period_s:
            return
        s["used_seq"], s["last_attempt"] = self.frame_seq, now
        s["attempts"] += 1
        frame = self.latest
        self.job = (self.pool.submit(self.detector.detect, frame[0], s["query"]), frame, s)

    def on_result(self, search, frame, det):
        bgr, depth, intr, stamp, frame_id = frame
        query = search["query"]
        if det is not None:
            search["best"] = max(search["best"] or 0.0, det.score)
        self.publish_debug(bgr, det, query)
        if det is None or det.score < self.min_confidence:
            return
        pts = region_points(depth, intr, det.box_xyxy, det.mask, depth_scale=self.depth_scale,
                            min_depth_m=self.min_depth_m, max_depth_m=self.max_depth_m)
        if pts is None:
            self.get_logger().info(f"'{query}': seen ({det.score:.2f}) but no valid depth")
            return
        T = self.lookup(frame_id, stamp)
        if T is None:
            return
        box = gravity_box(transform_points(T, pts), camera_xyz=T[:3, 3])
        if box is None:
            return
        poi_id = self._id_for(query)
        label = f"{query} ({det.score:.2f})"
        self.publish_box(poi_id, label, box)
        dist = self.distance_from_robot(box["center"])
        self.publish_poi(poi_id, det, query, box, len(pts))
        size_cm = " x ".join(f"{v * 100:.0f}" for v in box["size"])
        self.reply(f"Found {query} ({det.score:.2f})"
                   + (f", {dist:.1f} m from the robot" if dist is not None else "")
                   + f". Box {size_cm} cm.")
        self.get_logger().info(
            f"found '{query}' ({det.score:.2f}) after {search['attempts']} frames at map "
            f"({box['center'][0]:.2f}, {box['center'][1]:.2f}, {box['center'][2]:.2f}), "
            f"size {box['size']}, mask={'SAM2' if det.mask is not None else 'box'}")
        self.search = None
        self.status_marker(None)

    def lookup(self, frame_id, stamp):
        for when in (Time.from_msg(stamp), Time()):      # at the image stamp, else latest
            try:
                return tf_to_T(self.tf_buffer.lookup_transform(
                    self.map_frame, frame_id, when, timeout=Duration(seconds=0.1)))
            except TransformException as exc:
                err = exc
        self.get_logger().warning(f"TF {frame_id} -> {self.map_frame} failed: {err}",
                                  throttle_duration_sec=5.0)
        return None

    def distance_from_robot(self, xyz):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, Time())
        except TransformException:
            return None
        p = t.transform.translation
        return math.hypot(xyz[0] - p.x, xyz[1] - p.y)

    def _id_for(self, label):
        if label not in self._label_ids:
            self._label_ids[label] = self._next_id
            self._next_id += 1
        return self._label_ids[label]

    # --- outputs -------------------------------------------------------------------------

    def reply(self, text):
        self.reply_pub.publish(String(data=text))

    def marker(self, ns, mid, mtype, color):
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 1.0
        return m

    def publish_box(self, poi_id, label, box):
        m = self.marker("semantic_query", poi_id, Marker.CUBE, BOX_COLOR)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = box["center"]
        m.pose.orientation.z = math.sin(box["yaw"] / 2)
        m.pose.orientation.w = math.cos(box["yaw"] / 2)
        m.scale.x, m.scale.y, m.scale.z = box["size"]
        m.color.a = 0.5
        m.text = label              # g1_ar_bridge draws the box edges and this label
        self.marker_pub.publish(MarkerArray(markers=[m]))

    def status_marker(self, text):
        """"searching: red cup" floating above the robot; None removes it."""
        m = self.marker("semantic_status", 0, Marker.TEXT_VIEW_FACING, STATUS_COLOR)
        if text is None:
            m.action = Marker.DELETE
        else:
            try:
                t = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, Time())
            except TransformException:
                return
            p = t.transform.translation
            m.pose.position.x, m.pose.position.y, m.pose.position.z = p.x, p.y, p.z + 0.9
            m.scale.z = 0.08
            m.text = text
        self.marker_pub.publish(MarkerArray(markers=[m]))

    def publish_poi(self, poi_id, det, query, box, n_points):
        payload = {
            "id": poi_id, "label": query, "detector_label": det.label, "query": query,
            "xyz": [round(v, 4) for v in box["center"]], "frame_id": self.map_frame,
            "confidence": round(float(det.score), 4),
            "box": {"center": [round(v, 4) for v in box["center"]],
                    "size": [round(v, 4) for v in box["size"]], "yaw": round(box["yaw"], 4)},
            "points": int(n_points), "mask": "sam2" if det.mask is not None else "box",
            "stamp": self.get_clock().now().nanoseconds,
        }
        self.poi_pub.publish(String(data=json.dumps(payload)))

    def publish_debug(self, bgr, det, query):
        if self.image_pub.get_subscription_count() == 0:
            return
        img = bgr.copy()
        if det is not None:
            if det.mask is not None and det.mask.shape == img.shape[:2]:
                img[det.mask] = (0.5 * img[det.mask] + 0.5 * np.array([60, 220, 60])).astype(
                    np.uint8)
            x0, y0, x1, y1 = (int(round(v)) for v in det.box_xyxy)
            ok = det.score >= self.min_confidence
            color = (60, 220, 60) if ok else (0, 160, 255)
            cv2.rectangle(img, (x0, y0), (x1, y1), color, 2)
            cv2.putText(img, f"{query} {det.score:.2f}", (x0, max(20, y0 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        else:
            cv2.putText(img, f"searching: {query}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 160, 255), 2)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, encoding="bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = PoiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pool.shutdown(wait=False, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
