"""Leo Rover in the G1 map through the shared wall tag, and marked in the AR glasses.

Laptop side of g1_ws/docs/leo_g1_laptop_integration.md §4. Receives ``leo_relay`` (running on
Leo, read-only) over UDP: Leo's detections of the shared AprilTag (36h11 ID 0) in its OAK-D
camera, and Leo's odometry. With the G1 anchor ``map -> ar_tag_0`` (``tag_anchor``) it computes
Leo's pose in ``map`` (``leo_localization``) and publishes, as the single owner of these frames:

    TF    map -> leo_odom -> leo_base   (leo_base only from the tag when no odometry arrives)
    /ar_glasses/markers   ns "leo": a box of Leo's size, a heading line and a "Leo Rover" label
    /leo_in_g1/status     JSON: relay, sightings, rejections, pose, warnings

Leo's own topics and TF are untouched; its frame names never enter our graph (``leo_odom`` and
``leo_base`` are ours). Nothing here commands either robot. The camera mount on Leo is NOT
measured (default: forward-looking, upside down, at Leo's origin): the pose is provisional.

    ros2 run g1_ar_bridge leo_in_map        (or ar_bridge.launch.py leo:=true)
"""
import json
import math
import socket
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .leo_localization import DEFAULT_CAMERA_RPY, LeoLocalizer, camera_mount, floor_pose, quat_of
from .lifetime import exit_with_parent
from .ros_util import transform_msg_to_T

LEO_COLOR = (0.2, 0.55, 1.0)


def T_to_tf(T, parent, child, stamp):
    msg = TransformStamped()
    msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, parent, child
    t = msg.transform.translation
    t.x, t.y, t.z = (float(v) for v in T[:3, 3])
    q = msg.transform.rotation
    q.x, q.y, q.z, q.w = (float(v) for v in quat_of(T))
    return msg


class LeoInMap(Node):
    def __init__(self):
        super().__init__("leo_in_map")
        p = self.declare_parameter
        self.map_frame = p("map_frame", "map").value
        self.tag_frame = p("tag_frame", "ar_tag_0").value
        self.robot_frame = p("robot_frame", "robot_center").value
        self.base_height = float(p("base_height_m", 0.78).value)
        self.odom_frame = p("leo_odom_frame", "leo_odom").value
        self.base_frame = p("leo_base_frame", "leo_base").value
        self.size = [float(v) for v in p("leo_size_m", [0.45, 0.43, 0.25]).value]
        xyz = [float(v) for v in p("leo_camera_xyz", [0.0, 0.0, 0.0]).value]
        rpy = [float(v) for v in p("leo_camera_rpy", list(DEFAULT_CAMERA_RPY)).value]
        self.mount_measured = bool(p("leo_camera_measured", False).value)
        self.stale_s = float(p("stale_s", 3.0).value)
        self.loc = LeoLocalizer(camera_mount(xyz, rpy),
                                min_distance_m=float(p("min_distance_m", 0.2).value),
                                max_distance_m=float(p("max_distance_m", 4.0).value))
        port = int(p("udp_port", 8791).value)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(
            MarkerArray, p("markers_topic", "/ar_glasses/markers").value, 10)
        self.status_pub = self.create_publisher(
            String, p("status_topic", "/leo_in_g1/status").value, 10)
        self.counts = {"tag": 0, "odom": 0, "hello": 0, "bad": 0}
        self.last_rx = {}
        self.relay_from = None
        self.last_label = None
        self.create_timer(0.02, self.receive)
        self.create_timer(0.1, self.publish_tf)
        self.create_timer(0.5, self.publish_markers)
        self.create_timer(1.0, self.publish_status)
        self.get_logger().info(
            f"waiting for leo_relay on UDP {port}; anchor {self.map_frame} -> {self.tag_frame}"
            + ("" if self.mount_measured else "; Leo camera mount NOT measured (provisional)"))

    # --- input ---------------------------------------------------------------------------

    def anchor(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.tag_frame, Time())
        except TransformException:
            return None
        return transform_msg_to_T(tf.transform)

    def receive(self):
        for _ in range(200):
            try:
                data, addr = self.sock.recvfrom(65536)
            except BlockingIOError:
                return
            except OSError:
                return
            for line in data.decode(errors="replace").splitlines():
                self.on_message(line, addr)

    def on_message(self, line, addr):
        try:
            msg = json.loads(line)
            kind = msg["type"]
        except (ValueError, KeyError, TypeError):
            self.counts["bad"] += 1
            return
        if self.relay_from != addr[0]:
            self.relay_from = addr[0]
            self.get_logger().info(f"leo_relay connected from {addr[0]}")
        self.last_rx[kind] = time.monotonic()
        if kind == "odom":
            self.counts["odom"] += 1
            self.loc.add_odom(float(msg["stamp"]), msg["position"], msg["orientation"],
                              frames=(msg.get("frame_id"), msg.get("child_frame_id")))
        elif kind == "tag":
            self.counts["tag"] += 1
            T_map_tag = self.anchor()
            if T_map_tag is None:
                self.loc.rejected["no G1 anchor (map -> ar_tag_0) yet"] += 1
                return
            first = self.loc.sightings == 0
            if self.loc.add_sighting(float(msg["stamp"]), np.asarray(msg["T"], dtype=float),
                                     T_map_tag) is not None and first:
                x, y, yaw = floor_pose(self.loc.pose())
                self.get_logger().info(
                    f"Leo placed in {self.map_frame}: ({x:.2f}, {y:.2f}) m, yaw "
                    f"{math.degrees(yaw):.0f} deg, {self.loc.last_sighting}")
        elif kind == "hello":
            self.counts["hello"] += 1

    # --- output --------------------------------------------------------------------------

    def publish_tf(self):
        stamp = self.get_clock().now().to_msg()
        if self.loc.T_map_odom is not None and self.loc.odom:
            self.tf_pub.sendTransform([
                T_to_tf(self.loc.T_map_odom, self.map_frame, self.odom_frame, stamp),
                T_to_tf(self.loc.odom[-1][1], self.odom_frame, self.base_frame, stamp)])
        elif self.loc.T_map_base_tag is not None:
            self.tf_pub.sendTransform(
                T_to_tf(self.loc.T_map_base_tag, self.map_frame, self.base_frame, stamp))

    def floor_z(self, T_map_base):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, Time())
            return tf.transform.translation.z - self.base_height
        except TransformException:
            return float(T_map_base[2, 3])

    def stale(self):
        newest = max((self.last_rx.get(k, -1e9) for k in ("tag", "odom")), default=-1e9)
        return time.monotonic() - newest > self.stale_s

    def marker(self, mid, mtype):
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = "leo", mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b = LEO_COLOR
        m.color.a = 1.0
        return m

    def publish_markers(self):
        T = self.loc.pose()
        if T is None:
            return
        x, y, yaw = floor_pose(T)
        floor = self.floor_z(T)
        L, W, H = self.size
        box = self.marker(0, Marker.CUBE)
        box.pose.position.x, box.pose.position.y, box.pose.position.z = x, y, floor + H / 2
        box.pose.orientation.z, box.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        box.scale.x, box.scale.y, box.scale.z = L, W, H
        box.color.a = 0.6
        heading = self.marker(1, Marker.LINE_STRIP)
        heading.scale.x = 0.02
        heading.points = [Point(x=x, y=y, z=floor + H), Point(
            x=x + 0.45 * math.cos(yaw), y=y + 0.45 * math.sin(yaw), z=floor + H)]
        label = self.marker(2, Marker.TEXT_VIEW_FACING)
        label.pose.position.x, label.pose.position.y = x, y
        label.pose.position.z = floor + H + 0.2
        label.scale.z = 0.08
        label.text = "Leo Rover" + (" (no update)" if self.stale() else "")
        self.marker_pub.publish(MarkerArray(markers=[box, heading, label]))

    def publish_status(self):
        now = time.monotonic()
        T = self.loc.pose()
        status = {
            "relay": self.relay_from,
            "age_s": {k: round(now - v, 1) for k, v in self.last_rx.items()},
            "received": dict(self.counts), "sightings": self.loc.sightings,
            "rejected": dict(self.loc.rejected), "last_sighting": self.loc.last_sighting,
            "leo_odom_frames": self.loc.odom_frames, "odometry": self.loc.T_map_odom is not None,
            "g1_anchor": self.anchor() is not None,
            "camera_mount_measured": self.mount_measured,
        }
        if T is not None:
            x, y, yaw = floor_pose(T)
            status["pose_in_map"] = {"x": round(x, 3), "y": round(y, 3),
                                     "yaw_deg": round(math.degrees(yaw), 1)}
        self.status_pub.publish(String(data=json.dumps(status)))


def main():
    exit_with_parent()
    rclpy.init()
    node = None
    try:
        node = LeoInMap()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
