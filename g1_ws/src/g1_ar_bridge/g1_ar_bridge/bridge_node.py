"""ROS 2 node: the Spectacles AR bridge on the robot laptop (AGENTS.md §27).

WebSocket ``ws://<this laptop>:8787`` for the Dimensional OS Lens (protocol v19), fed by our
stack over the robot Ethernet:

    TF   map -> robot_center            robot box in the glasses        (g1_mapping, g1_sensors)
    TF   map -> ar_tag_<id>             wall-tag anchor                  (tag_anchor node)
    /cloud_map  (PointCloud2)           LiDAR map, voxelised, <= 1500 pts (g1_mapping)
    /plan       (nav_msgs/Path)         Nav2 path                        (Nav2, when running)
    /ar_glasses/markers (MarkerArray)   POIs / 3D boxes                  (semantic_query, tools)

Publishes, after registration:

    TF   map -> ar_world  (static)      the Spectacles' world in our map
    TF   ar_world -> spectacles         the glasses (x forward, y left, z up), ~2 Hz
    /ar_glasses/hmd_pose (PoseStamped, map), /ar_glasses/status (JSON String),
    /ar_glasses/user_command (String)   voice / typed commands from the glasses

Never commands the robot: goals and e-stop from the glasses are refused (§19, §25).
"""
import asyncio
import json
import queue
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import (Buffer, StaticTransformBroadcaster, TransformBroadcaster,
                     TransformException, TransformListener)
from visualization_msgs.msg import MarkerArray

from .geometry import T_to_pose, inv_T, transform_points
from .lifetime import exit_with_parent
from .ros_util import (apply_marker_array, pointcloud2_xyz, pose_msg_to_T, transform_msg_to_T,
                       voxel_downsample)
from .server import ArBridgeServer, BridgeConfig, serve
from .tag_anchor_node import T_to_transform_msg
from .world import World

# the Lens camera (x right, y up, looks along -Z) -> REP-103 body axes (x forward, y left, z up)
GL_TO_BODY = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]])


class RosWorld(World):
    """``World`` backed by TF and topics; thread-safe for the bridge's asyncio thread."""

    def __init__(self, node, map_frame, robot_frame, tag_prefix, base_height, voxel_m,
                 max_points):
        self.node = node
        self.map_frame, self.robot_frame, self.tag_prefix = map_frame, robot_frame, tag_prefix
        self.base_height = base_height
        self.voxel_m, self.max_points = voxel_m, max_points
        self.lock = threading.Lock()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self._points = np.zeros((0, 3))
        self._path = []
        self._markers = {}
        self.outbox = queue.SimpleQueue()     # messages for the ROS thread to publish
        self.registered = False

    def lookup(self, target, source):
        try:
            tf = self.tf_buffer.lookup_transform(target, source, Time())
        except TransformException:
            return None
        return transform_msg_to_T(tf.transform)

    # --- World interface -------------------------------------------------------------------

    def robot_pose(self):
        return self.lookup(self.map_frame, self.robot_frame)

    def anchor(self, tag_id):
        return self.lookup(self.map_frame, f"{self.tag_prefix}{tag_id}")

    def floor_z(self):
        T = self.robot_pose()
        return None if T is None else float(T[2, 3]) - self.base_height

    def lidar_points(self):
        with self.lock:
            return self._points

    def path(self):
        with self.lock:
            return list(self._path)

    def annotations(self):
        with self.lock:
            return {a.id: a for anns in self._markers.values() for a in anns}

    def on_registered(self, T_ar_map, method, info):
        self.registered = True
        self.outbox.put(("registered", inv_T(T_ar_map), info))

    def on_unregistered(self):
        if self.registered:
            self.outbox.put(("unregistered", None, None))
        self.registered = False

    def on_hmd_pose(self, T_map_hmd, T_ar_hmd):
        T_ar_body = T_ar_hmd.copy()
        T_ar_body[:3, :3] = T_ar_hmd[:3, :3] @ GL_TO_BODY
        T_map_body = T_map_hmd.copy()
        T_map_body[:3, :3] = T_map_hmd[:3, :3] @ GL_TO_BODY
        self.outbox.put(("hmd", T_ar_body, T_map_body))

    def on_user_command(self, text):
        self.outbox.put(("user_command", text, None))
        return f"Sent to the robot: {text}"

    # --- ROS callbacks (executor thread) ---------------------------------------------------

    def on_cloud(self, msg):
        pts = pointcloud2_xyz(msg)
        frame = msg.header.frame_id or self.map_frame
        if frame != self.map_frame:
            T = self.lookup(self.map_frame, frame)
            if T is None:
                return
            pts = transform_points(T, pts)
        pts = voxel_downsample(pts, self.voxel_m)
        if len(pts) > self.max_points:
            pts = pts[np.random.default_rng(0).choice(len(pts), self.max_points, replace=False)]
        with self.lock:
            self._points = pts

    def on_path(self, msg):
        frame = msg.header.frame_id or self.map_frame
        T = np.eye(4) if frame == self.map_frame else self.lookup(self.map_frame, frame)
        if T is None:
            return
        pts = [(T @ pose_msg_to_T(p.pose))[:3, 3] for p in msg.poses]
        with self.lock:
            self._path = [list(map(float, p)) for p in pts]

    def on_markers(self, msg):
        def lookup(frame):
            frame = frame or self.map_frame
            return np.eye(4) if frame == self.map_frame else self.lookup(self.map_frame, frame)
        with self.lock:
            apply_marker_array(self._markers, msg.markers, lookup)


class ArBridgeNode(Node):
    def __init__(self):
        super().__init__("ar_bridge")
        p = self.declare_parameter
        self.host = p("host", "0.0.0.0").value
        self.port = int(p("port", 8787).value)
        self.map_frame = p("map_frame", "map").value
        self.ar_frame = p("ar_world_frame", "ar_world").value
        self.hmd_frame = p("spectacles_frame", "spectacles").value
        cfg = BridgeConfig(
            display_name=p("display_name", "Unitree G1").value,
            base_height_m=float(p("base_height_m", 0.78).value),
            tag_id=int(p("tag_id", 0).value),
            tag_black_size_m=float(p("tag_black_size_m", 0.16).value),
            min_views=int(p("min_views", 6).value),
            min_baseline_m=float(p("min_baseline_m", 0.3).value),
            max_view_reproj_px=float(p("max_view_reproj_px", 4.0).value),
            max_rms_px=float(p("max_rms_px", 3.0).value),
            max_tilt_deg=float(p("max_tilt_deg", 10.0).value),
            registration_timeout_s=float(p("registration_timeout_s", 180.0).value),
            pose_hz=float(p("pose_hz", 10.0).value),
            lidar_hz=float(p("lidar_hz", 2.0).value),
            hmd_hz=float(p("hmd_hz", 2.0).value))
        self.world = RosWorld(self, self.map_frame, p("robot_frame", "robot_center").value,
                              p("tag_frame_prefix", "ar_tag_").value, cfg.base_height_m,
                              float(p("cloud_voxel_m", 0.08).value),
                              int(p("cloud_max_points", 30000).value))
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(PointCloud2, p("cloud_topic", "/cloud_map").value,
                                 self.world.on_cloud, best_effort)
        self.create_subscription(Path, p("path_topic", "/plan").value, self.world.on_path, 10)
        self.create_subscription(MarkerArray, p("markers_topic", "/ar_glasses/markers").value,
                                 self.world.on_markers, 10)
        self.cmd_pub = self.create_publisher(
            String, p("user_command_topic", "/ar_glasses/user_command").value, 10)
        self.hmd_pub = self.create_publisher(
            PoseStamped, p("hmd_pose_topic", "/ar_glasses/hmd_pose").value, 10)
        self.status_pub = self.create_publisher(
            String, p("status_topic", "/ar_glasses/status").value, 10)
        self.static_tf = StaticTransformBroadcaster(self)
        self.tf = TransformBroadcaster(self)
        self.bridge = ArBridgeServer(self.world, cfg, log=self.get_logger().info)
        self.create_timer(0.05, self.drain_outbox)
        self.create_timer(1.0, self.publish_status)
        self.loop = None
        self.task = None
        self.thread = threading.Thread(target=self.run_bridge, daemon=True)
        self.thread.start()

    def run_bridge(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self.loop = loop

        async def main():
            server = await serve(self.bridge, self.host, self.port)
            self.get_logger().info(
                f"Spectacles bridge listening on ws://{self.host}:{self.port} (type this "
                "laptop's Wi-Fi IP into the Lens). No actuation from the glasses.")
            try:
                await self.bridge.run()
            finally:
                server.close()
                await server.wait_closed()

        self.task = loop.create_task(main())
        try:
            loop.run_until_complete(self.task)
        except asyncio.CancelledError:
            pass
        except OSError as exc:
            self.get_logger().error(f"cannot open port {self.port}: {exc}")
        finally:
            loop.close()

    def stop_bridge(self):
        if self.loop is not None and self.task is not None and not self.loop.is_closed():
            self.loop.call_soon_threadsafe(self.task.cancel)
        self.thread.join(timeout=3.0)

    def drain_outbox(self):
        while True:
            try:
                kind, a, b = self.world.outbox.get_nowait()
            except queue.Empty:
                return
            stamp = self.get_clock().now().to_msg()
            if kind == "registered":
                self.static_tf.sendTransform(
                    T_to_transform_msg(a, self.map_frame, self.ar_frame, stamp))
                self.get_logger().info(f"registered: published {self.map_frame} -> "
                                       f"{self.ar_frame} ({b})")
            elif kind == "hmd":
                self.tf.sendTransform(T_to_transform_msg(a, self.ar_frame, self.hmd_frame,
                                                         stamp))
                msg = PoseStamped()
                msg.header.stamp, msg.header.frame_id = stamp, self.map_frame
                (x, y, z), (qx, qy, qz, qw) = T_to_pose(b)
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
                o = msg.pose.orientation
                o.x, o.y, o.z, o.w = qx, qy, qz, qw
                self.hmd_pub.publish(msg)
            elif kind == "user_command":
                self.cmd_pub.publish(String(data=a))
            elif kind == "unregistered":
                self.get_logger().info("Lens gone: registration cleared (map -> ar_world stale)")

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps(self.bridge.status(), default=str)))


def main():
    exit_with_parent()
    rclpy.init()
    node = None
    try:
        node = ArBridgeNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop_bridge()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
