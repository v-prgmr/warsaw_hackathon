"""Publish the demo scene on /ar_glasses/markers (MarkerArray), placed relative to the robot.

The same template the home test (``sim_main``) shows, so the glasses look the same with the
robot: a table top 0.75 m high to the robot's front right, a green 3D box standing on it
("box on the table") and a "red bottle" label next to it. Placed once, relative to
``robot_center`` at the first TF lookup, in ``map``; the objects are virtual, not detections.
Started by ``ar_bridge.launch.py demo_pois:=true`` (``scripts/start_ar_glasses.sh`` does it
unless ``--no-demo``), or on its own:

    ros2 run g1_ar_bridge publish_demo_pois

Any node can drive the glasses the same way: publish Markers in ``map`` (TEXT, SPHERE, CUBE =
3D box, LINE_STRIP / LINE_LIST); ``ns`` + ``id`` identify them, DELETE / DELETEALL remove them.
Real POIs (``semantic_query``) should use their own ``ns``.
"""
import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .annotations import BOX_COLOR, POI_COLOR
from .lifetime import exit_with_parent
from .ros_util import transform_msg_to_T

# the sim_main scene (world.SimWorld), in the robot's frame: x ahead, y left, z above the floor
TABLE = ((0.6, -1.3), (1.2, -0.7), 0.75)              # top corners (x, y) and height
BOX = ((1.0, -1.1, 0.87), (0.3, 0.4, 0.24), 0.3)      # centre, size, yaw
BOX_LABEL = ("box on the table (0.74)", (1.0, -1.1, 0.95))
BOTTLE = ("red bottle (0.87)", (0.9, -0.85, 0.87))
TABLE_COLOR = (0.8, 0.8, 0.8)


class DemoPois(Node):
    def __init__(self):
        super().__init__("ar_demo_pois")
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.robot_frame = self.declare_parameter("robot_frame", "robot_center").value
        self.base_height = float(self.declare_parameter("base_height_m", 0.78).value)
        self.pub = self.create_publisher(MarkerArray, "/ar_glasses/markers", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.origin = None
        self.markers = None
        self.create_timer(1.0, self.tick)

    def tick(self):
        if self.origin is None:
            try:
                tf = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, Time())
            except TransformException:
                self.get_logger().info(f"waiting for TF {self.map_frame} -> {self.robot_frame}",
                                       throttle_duration_sec=5.0)
                return
            T = transform_msg_to_T(tf.transform)
            yaw = math.atan2(T[1, 0], T[0, 0])
            self.origin = (T[0, 3], T[1, 3], T[2, 3] - self.base_height, yaw)
            self.markers = self.build()
            self.get_logger().info("publishing the demo scene (table, green box, red bottle) "
                                   "in front of the robot")
        self.pub.publish(MarkerArray(markers=self.markers))

    def at(self, ahead, left, up):
        x0, y0, floor, yaw = self.origin
        return (x0 + ahead * math.cos(yaw) - left * math.sin(yaw),
                y0 + ahead * math.sin(yaw) + left * math.cos(yaw), floor + up)

    def marker(self, mid, mtype, color, alpha=1.0):
        m = Marker()
        m.header.frame_id = self.map_frame
        m.ns, m.id, m.type, m.action = "demo", mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b = (float(c) for c in color)
        m.color.a = alpha
        return m

    def label(self, mid, text, xyz):
        m = self.marker(mid, Marker.TEXT_VIEW_FACING, POI_COLOR)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = self.at(*xyz)
        m.scale.z = 0.08
        m.text = text
        return m

    def build(self):
        yaw = self.origin[3]
        (x1, y1), (x2, y2), h = TABLE
        table = self.marker(0, Marker.LINE_STRIP, TABLE_COLOR)
        table.scale.x = 0.01
        for x, y in ((x1, y1), (x2, y1), (x2, y2), (x1, y2), (x1, y1)):
            px, py, pz = self.at(x, y, h)
            table.points.append(Point(x=px, y=py, z=pz))
        centre, size, box_yaw = BOX
        box = self.marker(1, Marker.CUBE, BOX_COLOR, alpha=0.5)
        box.pose.position.x, box.pose.position.y, box.pose.position.z = self.at(*centre)
        box.pose.orientation.z = math.sin((yaw + box_yaw) / 2)
        box.pose.orientation.w = math.cos((yaw + box_yaw) / 2)
        box.scale.x, box.scale.y, box.scale.z = size
        return [table, box, self.label(2, *BOX_LABEL), self.label(3, *BOTTLE)]


def main():
    exit_with_parent()
    rclpy.init()
    node = None
    try:
        node = DemoPois()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
