"""Labels, pins and floor lines for the AR glasses (and RViz), from what the robot already knows.

Publishes on /ar_glasses/markers (frame ``map``), each in its own ``ns`` so they can be replaced
without touching other nodes' markers:
  * ``floor_outline``: the walls of the RTAB-Map /map traced into polylines on the floor
    (``floor_outline.py``); updated when the map changes, at most every ``outline_period_s``
  * ``robot_label``: a pin above the robot, "Unitree G1 · 70%" (battery from /battery_state)
  * ``tag_label``: a pin on the wall AprilTag once the robot has anchored it (map -> ar_tag_0)
The detected objects' boxes and pins come from semantic_query (ns ``semantic_query``).
Read-only towards the robot.

    ros2 run g1_ar_bridge scene_markers          (or ar_bridge.launch.py scene_markers:=true)
"""
import math

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .floor_outline import outline_polylines, signature
from .lifetime import exit_with_parent

OUTLINE_COLOR = (0.2, 0.8, 1.0)
ROBOT_COLOR = (0.95, 0.95, 0.95)
TAG_COLOR = (1.0, 0.85, 0.2)


class SceneMarkers(Node):
    def __init__(self):
        super().__init__("ar_scene_markers")
        p = self.declare_parameter
        self.map_frame = p("map_frame", "map").value
        self.robot_frame = p("robot_frame", "robot_center").value
        self.tag_frame = p("tag_frame", "ar_tag_0").value
        self.base_height = float(p("base_height_m", 0.78).value)
        self.robot_name = p("robot_name", "Unitree G1").value
        self.label_height = float(p("robot_label_height_m", 0.65).value)
        self.outline = bool(p("floor_outline", True).value)
        self.outline_period = float(p("outline_period_s", 5.0).value)
        self.outline_kwargs = dict(
            simplify_m=float(p("outline_simplify_m", 0.05).value),
            min_length_m=float(p("outline_min_length_m", 0.4).value),
            max_lines=int(p("outline_max_lines", 60).value),
            max_points=int(p("outline_max_points", 1200).value))
        self.pub = self.create_publisher(
            MarkerArray, p("markers_topic", "/ar_glasses/markers").value, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # VOLATILE matches latched and plain /map publishers; RTAB-Map republishes it anyway
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
        if self.outline:
            self.create_subscription(OccupancyGrid, p("map_topic", "/map").value,
                                     self.on_map, map_qos)
        self.create_subscription(BatteryState, p("battery_topic", "/battery_state").value,
                                 self.on_battery, 10)
        self.battery = None
        self.grid = None
        self.outline_sig = None
        self.outline_sent = -1e9
        self.outline_count = 0          # lines published last time (the rest get DELETE)
        self.robot_sent = None          # (text, x, y, z) last published
        self.tag_sent = None
        self.create_timer(1.0, self.tick)
        self.get_logger().info("publishing floor outline, robot and tag labels on "
                               "/ar_glasses/markers")

    # --- inputs ---------------------------------------------------------------------------

    def on_map(self, msg):
        self.grid = msg

    def on_battery(self, msg):
        if msg.percentage == msg.percentage:                    # not NaN
            self.battery = msg.percentage

    def lookup(self, frame):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, frame, Time()).transform
        except TransformException:
            return None
        return t

    # --- output ---------------------------------------------------------------------------

    def marker(self, ns, mid, mtype, color):
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b = color
        m.color.a = 1.0
        return m

    def label(self, ns, text, xyz, color):
        m = self.marker(ns, 0, Marker.TEXT_VIEW_FACING, color)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
        m.scale.z = 0.08
        m.text = text
        return m

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        robot = self.lookup(self.robot_frame)
        floor = (robot.translation.z - self.base_height) if robot is not None \
            else -self.base_height
        out = []
        if self.outline and self.grid is not None and now - self.outline_sent >= self.outline_period:
            sig = signature(self.grid.data)
            if sig != self.outline_sig:
                self.outline_sig, self.outline_sent = sig, now
                out += self.floor_outline(floor)
        if robot is not None:
            t = robot.translation
            text = self.robot_name + (f" · {self.battery * 100:.0f}%"
                                      if self.battery is not None else "")
            pos = (t.x, t.y, t.z + self.label_height)
            last = self.robot_sent
            if last is None or last[0] != text or math.dist(last[1:], pos) > 0.15:
                self.robot_sent = (text,) + pos
                out.append(self.label("robot_label", text, pos, ROBOT_COLOR))
        tag = self.lookup(self.tag_frame)
        if tag is not None:
            t = tag.translation
            pos = (t.x, t.y, t.z + 0.15)
            if self.tag_sent is None or math.dist(self.tag_sent, pos) > 0.02:
                self.tag_sent = pos
                out.append(self.label("tag_label", f"AprilTag {self.tag_frame.rsplit('_', 1)[-1]}",
                                      pos, TAG_COLOR))
        if out:
            self.pub.publish(MarkerArray(markers=out))

    def floor_outline(self, floor_z):
        g = self.grid
        o = g.info.origin
        yaw = 2.0 * math.atan2(o.orientation.z, o.orientation.w)
        lines = outline_polylines(g.data, g.info.width, g.info.height, g.info.resolution,
                                  (o.position.x, o.position.y), yaw, **self.outline_kwargs)
        out = []
        for i in range(len(lines), self.outline_count):   # old lines only; no DELETEALL, which
            gone = self.marker("floor_outline", i, Marker.LINE_STRIP, OUTLINE_COLOR)  # RViz may
            gone.action = Marker.DELETE                                # apply to every ns
            out.append(gone)
        self.outline_count = len(lines)
        for i, poly in enumerate(lines):
            m = self.marker("floor_outline", i, Marker.LINE_STRIP, OUTLINE_COLOR)
            m.scale.x = 0.02
            m.points = [Point(x=float(x), y=float(y), z=floor_z + 0.02) for x, y in poly]
            out.append(m)
        self.get_logger().info(f"floor outline: {len(lines)} lines, "
                               f"{sum(len(p) for p in lines)} points", throttle_duration_sec=30.0)
        return out


def main():
    exit_with_parent()
    rclpy.init()
    node = None
    try:
        node = SceneMarkers()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
