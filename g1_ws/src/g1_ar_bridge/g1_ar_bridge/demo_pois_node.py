"""Publish demo POIs on /ar_glasses/markers (MarkerArray), placed relative to the robot.

For checking the glasses before ``semantic_query`` exists: a labelled "red bottle" marker and a
3D box, ``distance`` m in front of ``robot_center`` at the time of the first TF lookup, in
``map``. Any node can drive the glasses the same way: publish Markers in ``map`` (TEXT, SPHERE,
CUBE = 3D box, LINE_STRIP / LINE_LIST); ``ns`` + ``id`` identify them, DELETE / DELETEALL remove.

    ros2 run g1_ar_bridge publish_demo_pois --ros-args -p distance:=1.5
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .lifetime import exit_with_parent
from .ros_util import transform_msg_to_T


class DemoPois(Node):
    def __init__(self):
        super().__init__("ar_demo_pois")
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.robot_frame = self.declare_parameter("robot_frame", "robot_center").value
        self.distance = float(self.declare_parameter("distance", 1.5).value)
        self.base_height = float(self.declare_parameter("base_height_m", 0.78).value)
        self.pub = self.create_publisher(MarkerArray, "/ar_glasses/markers", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.origin = None
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
            self.get_logger().info("publishing demo POIs in front of the robot")
        x0, y0, floor, yaw = self.origin

        def at(ahead, left, up):
            return (x0 + ahead * math.cos(yaw) - left * math.sin(yaw),
                    y0 + ahead * math.sin(yaw) + left * math.cos(yaw), floor + up)

        text = Marker()
        text.header.frame_id = self.map_frame
        text.ns, text.id, text.type, text.action = "demo", 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        text.pose.position.x, text.pose.position.y, text.pose.position.z = at(
            self.distance, 0.3, 0.95)
        text.pose.orientation.w = 1.0
        text.scale.z = 0.08
        text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 0.3, 0.2, 1.0
        text.text = "red bottle (0.87)"
        box = Marker()
        box.header.frame_id = self.map_frame
        box.ns, box.id, box.type, box.action = "demo", 1, Marker.CUBE, Marker.ADD
        box.pose.position.x, box.pose.position.y, box.pose.position.z = at(
            self.distance, -0.3, 0.87)
        box.pose.orientation.z, box.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        box.scale.x, box.scale.y, box.scale.z = 0.3, 0.4, 0.24
        box.color.g, box.color.a = 0.9, 0.5
        box.text = "box on the table (0.74)"
        self.pub.publish(MarkerArray(markers=[text, box]))


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
