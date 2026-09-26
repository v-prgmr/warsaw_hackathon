"""Select ~8-20 RGB-D keyframes from a live stream or replayed bag and export them.

Selection criteria (all must pass to accept a synchronized RGB-D triplet):
  * sharpness   - variance of the Laplacian >= blur_threshold (rejects motion blur)
  * temporal    - >= min_time_gap_s since the last accepted keyframe
  * baseline    - >= min_translation_m of camera/base travel since last accepted (needs odom;
                  skipped if no odom has been received)
Stops after max_keyframes.

Output (the FROZEN keyframe struct that C/D/E consume):
  <output_dir>/
    manifest.json                     # {count, keyframes:[...]} updated after each accept
    keyframe_00/
      rgb.png                         # color, bgr8
      depth.npy                       # uint16 millimetres, aligned to rgb
      camera_info.yaml                # k, d, width, height, distortion_model
      meta.yaml                       # id, stamp, frame_id, blur_var, depth_units, odom_pose?
"""
import json
import os

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def _stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class KeyframeNode(Node):
    def __init__(self):
        super().__init__("keyframe_node")
        p = self.declare_parameters(
            "",
            [
                ("color_topic", "/camera/color/image_raw"),
                ("depth_topic", "/camera/aligned_depth_to_color/image_raw"),
                ("info_topic", "/camera/color/camera_info"),
                ("odom_topic", "/odom"),
                ("output_dir", "keyframes"),
                ("max_keyframes", 20),
                ("blur_threshold", 100.0),
                ("min_translation_m", 0.15),
                ("min_time_gap_s", 0.5),
                ("sync_slop_s", 0.10),
                ("sync_queue", 30),
            ],
        )
        (self.color_topic, self.depth_topic, self.info_topic, self.odom_topic,
         self.output_dir, self.max_keyframes, self.blur_threshold, self.min_translation_m,
         self.min_time_gap_s, self.sync_slop_s, self.sync_queue) = [x.value for x in p]

        os.makedirs(self.output_dir, exist_ok=True)
        self.bridge = CvBridge()
        self.manifest = []
        self.last_accept_t = None
        self.last_accept_xyz = None
        self.latest_odom_xyz = None
        self.latest_odom_frame = None

        # Sensor QoS (best_effort) matches both reliable and best_effort publishers.
        qos = qos_profile_sensor_data
        self.color_sub = Subscriber(self, Image, self.color_topic, qos_profile=qos)
        self.depth_sub = Subscriber(self, Image, self.depth_topic, qos_profile=qos)
        self.info_sub = Subscriber(self, CameraInfo, self.info_topic, qos_profile=qos)
        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.info_sub],
            queue_size=int(self.sync_queue), slop=float(self.sync_slop_s),
        )
        self.sync.registerCallback(self.on_rgbd)

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)

        self.get_logger().info(
            f"keyframe_node up. color={self.color_topic} depth={self.depth_topic} "
            f"info={self.info_topic} odom={self.odom_topic} -> {os.path.abspath(self.output_dir)}"
        )
        self.get_logger().info(
            f"thresholds: blur>={self.blur_threshold} baseline>={self.min_translation_m}m "
            f"gap>={self.min_time_gap_s}s max={self.max_keyframes}"
        )

    def on_odom(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.latest_odom_xyz = np.array([pos.x, pos.y, pos.z])
        self.latest_odom_frame = msg.header.frame_id or "odom"

    def on_rgbd(self, color_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        if len(self.manifest) >= self.max_keyframes:
            return

        t = _stamp_to_sec(color_msg.header.stamp)

        # --- temporal spacing (a negative gap means the bag restarted: accept) ---
        if self.last_accept_t is not None and 0.0 <= (t - self.last_accept_t) < self.min_time_gap_s:
            return

        # --- sharpness ---
        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        blur_var = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        if blur_var < self.blur_threshold:
            self.get_logger().info(f"reject (blur {blur_var:.1f} < {self.blur_threshold})")
            return

        # --- translational baseline (only if odom is available) ---
        cur_xyz = self.latest_odom_xyz
        if cur_xyz is not None and self.last_accept_xyz is not None:
            baseline = float(np.linalg.norm(cur_xyz - self.last_accept_xyz))
            if baseline < self.min_translation_m:
                self.get_logger().info(
                    f"reject (baseline {baseline:.3f} < {self.min_translation_m})"
                )
                return

        self._save_keyframe(color, depth_msg, info_msg, blur_var, cur_xyz)
        self.last_accept_t = t
        if cur_xyz is not None:
            self.last_accept_xyz = cur_xyz.copy()

        if len(self.manifest) >= self.max_keyframes:
            self.get_logger().info(f"reached max_keyframes={self.max_keyframes}; done.")

    def _save_keyframe(self, color, depth_msg, info_msg, blur_var, cur_xyz):
        kid = f"{len(self.manifest):02d}"
        kdir = os.path.join(self.output_dir, f"keyframe_{kid}")
        os.makedirs(kdir, exist_ok=True)

        cv2.imwrite(os.path.join(kdir, "rgb.png"), color)

        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        depth = np.asarray(depth)
        if np.issubdtype(depth.dtype, np.floating):
            # metres -> millimetres; invalid / out-of-range (> 65.535 m) -> 0 = no depth
            depth_mm = np.nan_to_num(depth * 1000.0, nan=0.0, posinf=0.0, neginf=0.0)
            depth_mm[(depth_mm < 0) | (depth_mm > 65535)] = 0
            depth_mm = np.round(depth_mm).astype(np.uint16)
        else:
            depth_mm = depth.astype(np.uint16)
        np.save(os.path.join(kdir, "depth.npy"), depth_mm)

        with open(os.path.join(kdir, "camera_info.yaml"), "w") as f:
            yaml.safe_dump({
                "width": int(info_msg.width),
                "height": int(info_msg.height),
                "distortion_model": info_msg.distortion_model,
                "k": [float(x) for x in info_msg.k],
                "d": [float(x) for x in info_msg.d],
            }, f)

        meta = {
            "id": kid,
            "stamp": {"sec": int(depth_msg.header.stamp.sec),
                      "nanosec": int(depth_msg.header.stamp.nanosec)},
            "frame_id": info_msg.header.frame_id,
            "blur_var": blur_var,
            "depth_units": "mm",
            "depth_encoding": depth_msg.encoding,
        }
        if cur_xyz is not None:
            meta["odom_pose"] = {"frame": self.latest_odom_frame,
                                 "position": [float(v) for v in cur_xyz]}
        with open(os.path.join(kdir, "meta.yaml"), "w") as f:
            yaml.safe_dump(meta, f)

        self.manifest.append({"id": kid, "dir": f"keyframe_{kid}", "blur_var": blur_var,
                              "frame_id": info_msg.header.frame_id})
        self._write_manifest()
        self.get_logger().info(
            f"accepted keyframe_{kid} (blur {blur_var:.1f}) [{len(self.manifest)}]")

    def _write_manifest(self):
        with open(os.path.join(self.output_dir, "manifest.json"), "w") as f:
            json.dump({"count": len(self.manifest), "keyframes": self.manifest}, f, indent=2)


def main(args=None):
    rclpy.init(args=args)
    node = KeyframeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._write_manifest()
        node.get_logger().info(f"wrote {len(node.manifest)} keyframes to {node.output_dir}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
