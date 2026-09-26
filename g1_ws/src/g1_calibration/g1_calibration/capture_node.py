"""Record calibration samples: OAK + RealSense images, accumulated LiDAR, and TF.

Read-only towards the robot (subscribes only). Each `~/capture` call (std_srvs/Trigger, or the
`calib_trigger` helper) waits `lidar.seconds`, then writes one sample (see dataset.py) with:
  * the latest image + CameraInfo of every camera in the config, received AFTER the call
  * all LiDAR points received during those seconds (the board must stand still)
  * TF at capture: body <- lidar, lidar <- each camera's optical frame, oak_root <- oak optical

Live feedback: ~/debug/<camera> shows the detected board (origin circled) at ~2 Hz.
Nothing is synchronized by timestamp on purpose: robot and board stand still, so the OAK on the
laptop clock and the robot sensors on the robot clock need no common time base.
"""
from collections import deque
import os
import threading
import time

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_srvs.srv import Trigger
import tf2_ros

from .board import Board, board_pose, detect_corners, draw
from .dataset import Intrinsics, load_config, next_index, write_sample
from .geometry import transform_from_xyz_quat

_NP = {7: "<f4", 8: "<f8"}  # PointField FLOAT32 / FLOAT64


def cloud_xyz(msg):
    fields = {f.name: f for f in msg.fields}
    dt = np.dtype({"names": ["x", "y", "z"],
                   "formats": [_NP[fields[c].datatype] for c in "xyz"],
                   "offsets": [fields[c].offset for c in "xyz"], "itemsize": msg.point_step})
    a = np.frombuffer(bytes(msg.data), dtype=dt, count=msg.width * msg.height)
    return np.stack([a["x"], a["y"], a["z"]], axis=1).astype(np.float32)


class CalibCapture(Node):
    def __init__(self):
        super().__init__("calib_capture")
        cfg_path = self.declare_parameter("config", "").value
        if not cfg_path:
            from .calibrate_extrinsics import default_config
            cfg_path = default_config()
        self.cfg = load_config(cfg_path)
        self.out_dir = os.path.abspath(self.declare_parameter("output_dir", "calib_data").value)
        lidar = self.cfg["lidar"]
        self.seconds = float(self.declare_parameter("lidar_seconds",
                                                    float(lidar["seconds"])).value)
        self.range_min, self.range_max = float(lidar["range_min"]), float(lidar["range_max"])
        self.board = Board.from_config(self.cfg["board"])
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest = {}   # camera -> (image msg, info msg, receive ns)
        self.infos = {}
        self.clouds = deque()  # (receive ns, xyz)
        self.debug_pubs = {}
        sensors = MutuallyExclusiveCallbackGroup()
        for cam, topics in self.cfg["cameras"].items():
            self.create_subscription(Image, topics["image"],
                                     lambda m, c=cam: self.on_image(c, m),
                                     qos_profile_sensor_data, callback_group=sensors)
            self.create_subscription(CameraInfo, topics["camera_info"],
                                     lambda m, c=cam: self.infos.__setitem__(c, m),
                                     qos_profile_sensor_data, callback_group=sensors)
            self.debug_pubs[cam] = self.create_publisher(Image, f"~/debug/{cam}", 1)
        self.create_subscription(PointCloud2, lidar["topic"], self.on_cloud,
                                 qos_profile_sensor_data, callback_group=sensors)
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.create_service(Trigger, "~/capture", self.on_capture,
                            callback_group=ReentrantCallbackGroup())
        self.create_timer(0.5, self.publish_debug, callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(
            f"calib_capture: {list(self.cfg['cameras'])} + {lidar['topic']} -> {self.out_dir}. "
            f"Call ~/capture (or `ros2 run g1_calibration calib_trigger`) with the robot and the "
            f"board STILL; each capture takes {self.seconds:.1f} s.")

    def now_ns(self):
        return time.monotonic_ns()

    def on_image(self, cam, msg):
        with self.lock:
            self.latest[cam] = (msg, self.infos.get(cam), self.now_ns())

    def on_cloud(self, msg):
        xyz = cloud_xyz(msg)
        r = np.linalg.norm(xyz, axis=1)
        xyz = xyz[np.isfinite(r) & (r >= self.range_min) & (r <= self.range_max)]
        now = self.now_ns()
        with self.lock:
            self.clouds.append((now, xyz, msg.header.frame_id))
            while self.clouds and self.clouds[0][0] < now - int(30e9):
                self.clouds.popleft()

    def publish_debug(self):
        for cam, pub in self.debug_pubs.items():
            if pub.get_subscription_count() == 0 or cam not in self.latest:
                continue
            msg, info, _ = self.latest[cam]
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            corners = detect_corners(img, self.board)
            text = "board OK" if corners is not None else "no board"
            if corners is not None and info is not None:
                intr = Intrinsics.from_msg(info)
                tf, rms = board_pose(corners, self.board, intr.k, intr.d)
                text = f"board {np.linalg.norm(tf[:3, 3]):.2f} m, {rms:.2f} px"
            out = self.bridge.cv2_to_imgmsg(draw(img, corners, self.board, text), "bgr8")
            out.header = msg.header
            pub.publish(out)

    def lookup(self, parent, child):
        if not parent or not child:
            return None
        try:
            t = self.tf.lookup_transform(parent, child, Time())
        except tf2_ros.TransformException:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return transform_from_xyz_quat([tr.x, tr.y, tr.z], [q.x, q.y, q.z, q.w])

    def on_capture(self, request, response):
        start = self.now_ns()
        time.sleep(self.seconds)
        end = self.now_ns()
        with self.lock:
            latest = dict(self.latest)
            clouds = [c for c in self.clouds if start <= c[0] <= end]
        images, infos, frames, problems, summary = {}, {}, {}, [], []
        for cam in self.cfg["cameras"]:
            if cam not in latest or latest[cam][2] < start:
                problems.append(f"no new image from {cam}")
                continue
            msg, info, _ = latest[cam]
            if info is None:
                problems.append(f"no camera_info from {cam}")
                continue
            images[cam] = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            infos[cam] = Intrinsics.from_msg(info)
            frames[cam] = msg.header.frame_id
            corners = detect_corners(images[cam], self.board)
            summary.append(f"{cam}: " + ("board OK" if corners is not None else "NO BOARD"))
        if not clouds:
            problems.append("no LiDAR scans during the capture")
        lidar_frame = clouds[0][2] if clouds else self.cfg["lidar"]["frame"]
        points = np.concatenate([c[1] for c in clouds]) if clouds else None
        body, oak_root = self.cfg["frames"]["body"], self.cfg["frames"].get("oak_root")
        # lidar <- reference cameras (not the OAK: its pose is what we calibrate)
        wanted = [(body, lidar_frame)] + [(lidar_frame, f) for c, f in frames.items()
                                          if c != "oak"]
        if "oak" in frames and oak_root:
            wanted.append((oak_root, frames["oak"]))
        looked_up = {(p, c): self.lookup(p, c) for p, c in wanted}
        transforms = [(p, c, t) for (p, c), t in looked_up.items() if t is not None]
        missing = [f"{p}<-{c}" for (p, c), t in looked_up.items() if t is None]
        if not images:
            response.success = False
            response.message = "; ".join(problems)
            return response
        index = next_index(self.out_dir)
        os.makedirs(self.out_dir, exist_ok=True)
        path = write_sample(self.out_dir, index, images, infos, frames, points, lidar_frame,
                            transforms, extra={"lidar_scans": len(clouds),
                                               "capture_seconds": self.seconds})
        n_pts = 0 if points is None else len(points)
        msg = (f"{os.path.basename(path)}: {', '.join(summary)}; LiDAR {len(clouds)} scans / "
               f"{n_pts} points; TF {len(transforms)}/{len(wanted)}")
        if missing:
            msg += f" (missing {', '.join(missing)})"
        if problems:
            msg += f"; WARN {'; '.join(problems)}"
        self.get_logger().info(msg)
        response.success = not problems
        response.message = msg
        return response


def main():
    rclpy.init()
    node = CalibCapture()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
