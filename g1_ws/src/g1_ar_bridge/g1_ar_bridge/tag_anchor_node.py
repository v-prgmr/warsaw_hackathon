"""Robot side of the wall-tag alignment: measure the AprilTag in ``map`` with a robot camera.

    camera image + CameraInfo (+ aligned depth)  ->  AprilTag 36h11, PnP (multi-frame)
    TF map <- camera optical frame (RTAB-Map + g1_sensors + camera driver)
    =>  static TF  map -> ar_tag_<id>   (+ /ar_glasses/anchor_status, optional YAML)

The robot must **stand still** facing the tag (1-2 m) for a few seconds: views are only
collected while the camera's map pose is steady, which also makes the result independent of
the camera's clock (AGENTS.md §7 Clock). With aligned depth, the tag plane is fitted to the
depth pixels around the tag and the PnP orientation is corrected with it.

Read-only: subscribes to camera topics and TF, publishes TF / status. No robot commands.
"""
import collections
import json
import math
import os
import time

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

from .alignment import TagPoseEstimator
from .apriltag import TagDetector, backproject, fit_plane, quad_pixels, snap_tag_to_plane
from .geometry import T_to_pose, pose_to_T, quat_to_rot, rotation_angle_deg
from .lifetime import exit_with_parent
from .ros_util import camera_info_intrinsics, depth_image_to_m, image_to_gray, transform_msg_to_T


def T_to_transform_msg(T, parent, child, stamp):
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    (x, y, z), (qx, qy, qz, qw) = T_to_pose(T)
    msg.transform.translation.x, msg.transform.translation.y = x, y
    msg.transform.translation.z = z
    msg.transform.rotation.x, msg.transform.rotation.y = qx, qy
    msg.transform.rotation.z, msg.transform.rotation.w = qz, qw
    return msg


class TagAnchorNode(Node):
    def __init__(self):
        super().__init__("ar_tag_anchor")
        p = self.declare_parameter
        self.map_frame = p("map_frame", "map").value
        self.tag_id = int(p("tag_id", 0).value)
        self.tag_size = float(p("tag_black_size_m", 0.16).value)
        self.child_frame = p("tag_frame_prefix", "ar_tag_").value + str(self.tag_id)
        self.min_views = int(p("min_views", 10).value)
        self.window_s = float(p("window_s", 3.0).value)
        self.max_rate = float(p("max_rate_hz", 5.0).value)
        self.max_reproj = float(p("max_reproj_px", 2.0).value)
        self.max_distance = float(p("max_distance_m", 4.0).value)
        self.still_pos = float(p("stationary_pos_m", 0.01).value)
        self.still_rot = float(p("stationary_rot_deg", 0.5).value)
        self.use_depth = bool(p("use_depth", True).value)
        self.depth_max_angle = float(p("depth_max_angle_deg", 15.0).value)
        self.update_pos = float(p("republish_pos_m", 0.01).value)
        self.update_rot = float(p("republish_rot_deg", 0.5).value)
        self.anchor_file = os.path.expanduser(p("anchor_file", "").value)
        load_saved = bool(p("load_saved", False).value)
        image_topic = p("image_topic", "/camera/color/image_raw").value
        info_topic = p("camera_info_topic", "/camera/color/camera_info").value
        depth_topic = p("depth_topic", "/camera/aligned_depth_to_color/image_raw").value

        self.detector = TagDetector()
        self.est = TagPoseEstimator(self.tag_size, max_views=40, max_reproj_px=self.max_reproj,
                                    max_distance_m=self.max_distance)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.status_pub = self.create_publisher(String, "/ar_glasses/anchor_status", 10)
        self.K = self.D = None
        self.depth = None                     # (stamp_s, metres array)
        self.depth_points = collections.deque(maxlen=8)   # map-frame points per frame
        self.cam_poses = collections.deque()  # (monotonic t, T_map_cv)
        self.last_processed = 0.0
        self.anchor = None                    # T_map_tag
        self.anchor_info = {}
        self.state = "searching"
        self.warned = set()

        self.create_subscription(CameraInfo, info_topic, self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
        if self.use_depth and depth_topic:
            self.create_subscription(Image, depth_topic, self.on_depth, qos_profile_sensor_data)
        self.create_timer(1.0, self.publish_status)
        if load_saved and self.anchor_file and os.path.exists(self.anchor_file):
            self.load_anchor()
        self.get_logger().info(
            f"measuring AprilTag 36h11 ID {self.tag_id} (black square {self.tag_size} m) on "
            f"{image_topic}; publishes {self.map_frame} -> {self.child_frame}. Stand the robot "
            f"still 1-2 m in front of the tag.")

    # --- inputs ----------------------------------------------------------------------------

    def on_info(self, msg):
        self.K, self.D, _ = camera_info_intrinsics(msg)

    def on_depth(self, msg):
        d = depth_image_to_m(msg)
        if d is None:
            self.warn_once("depthenc", f"unsupported depth encoding {msg.encoding}: no depth")
            return
        self.depth = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, d)

    def warn_once(self, key, text):
        if key not in self.warned:
            self.warned.add(key)
            self.get_logger().warn(text)

    def on_image(self, msg):
        now = time.monotonic()
        if self.K is None or now - self.last_processed < 1.0 / self.max_rate:
            return
        self.last_processed = now
        frame = msg.header.frame_id
        if "optical" not in frame:
            self.warn_once("optical", f"image frame '{frame}' is not an optical frame: the tag "
                                      "pose assumes OpenCV axes (z forward, y down)")
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, frame, Time())
        except TransformException as exc:
            self.state = "no_tf"
            self.warn_once("tf", f"no TF {self.map_frame} <- {frame} yet ({exc}); is g1_mapping "
                                 "+ g1_sensors running with the camera mount?")
            return
        T_map_cv = transform_msg_to_T(tf.transform)
        if not self.update_stillness(now, T_map_cv):
            if self.est.views:
                self.est.reset()
                self.depth_points.clear()
            self.state = "robot_moving"
            return
        gray = image_to_gray(msg)
        if gray is None:
            self.warn_once("enc", f"unsupported image encoding {msg.encoding}")
            return
        found = [c for i, c in self.detector.detect(gray) if i == self.tag_id]
        if not found:
            self.state = "searching" if self.anchor is None else "anchored"
            return
        corners = self.undistort(found[0])
        view, why = self.est.make_view(corners, self.K, T_map_cv, stamp=now)
        if view is None:
            self.state = f"rejected: {why}"
            return
        self.est.add(view)
        self.collect_depth(msg, corners, T_map_cv)
        if len(self.est.views) >= self.min_views:
            self.solve(T_map_cv)
        else:
            self.state = f"collecting {len(self.est.views)}/{self.min_views}"

    def undistort(self, corners):
        if self.D is None or not np.any(np.abs(self.D) > 1e-9):
            return corners
        pts = cv2.undistortPoints(corners.reshape(-1, 1, 2).astype(np.float64), self.K, self.D,
                                  P=self.K)
        return pts.reshape(4, 2)

    def update_stillness(self, now, T_map_cv):
        """True when the camera has not moved within the window (views are then comparable)."""
        self.cam_poses.append((now, T_map_cv))
        while self.cam_poses and now - self.cam_poses[0][0] > self.window_s:
            self.cam_poses.popleft()
        for _, T in self.cam_poses:
            if (np.linalg.norm(T[:3, 3] - T_map_cv[:3, 3]) > self.still_pos
                    or rotation_angle_deg(T[:3, :3], T_map_cv[:3, :3]) > self.still_rot):
                self.cam_poses.clear()
                self.cam_poses.append((now, T_map_cv))
                return False
        return True

    def collect_depth(self, msg, corners, T_map_cv):
        if self.depth is None:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        d_stamp, depth = self.depth
        if abs(d_stamp - stamp) > 0.5 or depth.shape != (msg.height, msg.width):
            self.warn_once("depthsize", "depth not aligned to the colour image (size or time): "
                                        "tag plane from PnP only")
            return
        rows, cols = quad_pixels(corners, depth.shape)
        pts = backproject(depth, self.K, rows, cols)
        if len(pts) > 2000:
            pts = pts[np.random.default_rng(0).choice(len(pts), 2000, replace=False)]
        self.depth_points.append(pts @ T_map_cv[:3, :3].T + T_map_cv[:3, 3])

    # --- estimate and publish --------------------------------------------------------------

    def solve(self, T_map_cv):
        est = self.est.estimate()
        if est is None or est.n_views < self.min_views:
            return
        T = est.T_world_tag
        info = {"views": est.n_views, "rms_px": round(est.rms_px, 2),
                "distance_m": round(float(np.linalg.norm(T[:3, 3] - T_map_cv[:3, 3])), 2),
                "depth": "off"}
        if self.depth_points:
            plane = fit_plane(np.concatenate(list(self.depth_points)), inlier_m=0.01)
            if plane is None:
                info["depth"] = "no plane"
            else:
                n, p0, rms, n_in = plane
                snapped, angle = snap_tag_to_plane(T, n, p0, T_map_cv[:3, 3],
                                                   self.depth_max_angle)
                info["depth_angle_deg"] = round(angle, 2)
                info["plane_rms_m"] = round(rms, 4)
                if snapped is None:
                    info["depth"] = "disagrees with PnP: ignored"
                else:
                    T, info["depth"] = snapped, "used"
        self.publish_anchor(T, info)

    def publish_anchor(self, T, info, force=False):
        if (not force and self.anchor is not None
                and np.linalg.norm(T[:3, 3] - self.anchor[:3, 3]) < self.update_pos
                and rotation_angle_deg(T[:3, :3], self.anchor[:3, :3]) < self.update_rot):
            self.state = "anchored"
            return
        first = self.anchor is None
        self.anchor, self.anchor_info = T, dict(info, time=time.strftime("%Y-%m-%d %H:%M:%S"))
        self.state = "anchored"
        self.static_tf.sendTransform(T_to_transform_msg(T, self.map_frame, self.child_frame,
                                                        self.get_clock().now().to_msg()))
        x, y, z = T[:3, 3]
        self.get_logger().info(
            f"{'anchored' if first else 'updated'} {self.map_frame} -> {self.child_frame}: "
            f"({x:.3f}, {y:.3f}, {z:.3f}) m, {info}")
        self.save_anchor()

    def save_anchor(self):
        if not self.anchor_file:
            return
        pos, quat = T_to_pose(self.anchor)
        data = {"frame_id": self.map_frame, "child_frame_id": self.child_frame,
                "translation": [round(v, 5) for v in pos],
                "rotation_xyzw": [round(v, 6) for v in quat], "info": self.anchor_info}
        try:
            os.makedirs(os.path.dirname(self.anchor_file) or ".", exist_ok=True)
            with open(self.anchor_file, "w") as f:
                yaml.safe_dump(data, f, sort_keys=False)
        except OSError as exc:
            self.warn_once("save", f"could not write {self.anchor_file}: {exc}")

    def load_anchor(self):
        with open(self.anchor_file) as f:
            data = yaml.safe_load(f)
        if data.get("frame_id") != self.map_frame or data.get("child_frame_id") != \
                self.child_frame:
            self.get_logger().warn(f"{self.anchor_file} is for another frame pair: not loaded")
            return
        T = pose_to_T(data["translation"], data["rotation_xyzw"])
        self.get_logger().warn(
            f"loaded a saved anchor from {self.anchor_file}: valid only with the SAME RTAB-Map "
            "map (localization mode on the saved database)")
        self.publish_anchor(T, {"loaded": self.anchor_file}, force=True)

    def publish_status(self):
        out = {"tag_id": self.tag_id, "state": self.state, "views": len(self.est.views),
               "frame": self.child_frame, "have_camera_info": self.K is not None,
               "have_depth": self.depth is not None}
        if self.anchor is not None:
            pos, quat = T_to_pose(self.anchor)
            R = quat_to_rot(quat)
            # direction the tag faces (its +z), as a map heading: ~180 deg when the robot at
            # the map origin faces the wall along +x
            facing = math.degrees(math.atan2(R[1, 2], R[0, 2]))
            out["anchor"] = {"xyz": [round(v, 3) for v in pos],
                             "facing_heading_deg": round(facing, 1), **self.anchor_info}
        self.status_pub.publish(String(data=json.dumps(out)))


def main():
    exit_with_parent()
    rclpy.init()
    node = None
    try:
        node = TagAnchorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
