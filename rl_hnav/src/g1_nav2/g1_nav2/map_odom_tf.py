"""Expose current Nav2 map and TF from SLAM Toolbox's metric observations."""

import copy
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def yaw(quaternion):
    return math.atan2(
        2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def map_odom_pose(map_base, odom_base):
    """Return planar T_map_odom = T_map_base * inverse(T_odom_base)."""
    angle = yaw(map_base.orientation) - yaw(odom_base.rotation)
    c, s = math.cos(angle), math.sin(angle)
    x = map_base.position.x - c * odom_base.translation.x + s * odom_base.translation.y
    y = map_base.position.y - s * odom_base.translation.x - c * odom_base.translation.y
    return x, y, angle


class MapOdomTF(Node):
    def __init__(self):
        super().__init__("map_odom_tf")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.pending_pose = None
        self.correction = None
        self.map = None
        self.last_time = None
        # SLAM Toolbox advertises its relative "pose" topic at /pose (root namespace).
        self.create_subscription(PoseWithCovarianceStamped, "/pose", self.on_pose, 10)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, "/slam_toolbox/map_raw", self.on_map, map_qos)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", map_qos)
        self.create_timer(0.05, self.tick)
        self.create_timer(1.0, self.publish_map)

    def on_pose(self, message):
        if message.header.frame_id == self.map_frame:
            self.pending_pose = message

    def on_map(self, message):
        if message.header.frame_id == self.map_frame:
            self.map = message

    def publish_map(self):
        now = self.get_clock().now()
        if self.map is None or self.correction is None or now.nanoseconds == 0:
            return
        # Only the header changes: unknown/free/occupied cells remain exactly
        # SLAM Toolbox's output. This avoids old scan stamps on Nav2 paths.
        out = copy.deepcopy(self.map)
        out.header.stamp = now.to_msg()
        self.map_pub.publish(out)

    def tick(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return
        if self.last_time is not None and now < self.last_time:
            # Gazebo reset: never reuse a correction from the previous run.
            self.correction = None
            self.pending_pose = None
            self.map = None
        self.last_time = now

        if self.pending_pose is not None:
            pose = self.pending_pose
            try:
                odom_base = self.buffer.lookup_transform(
                    self.odom_frame, self.base_frame, Time.from_msg(pose.header.stamp)
                )
            except TransformException:
                pass  # Retry when TF for this SLAM observation arrives.
            else:
                self.correction = map_odom_pose(pose.pose.pose, odom_base.transform)
                self.pending_pose = None

        if self.correction is None:
            return
        try:
            odom_base = self.buffer.lookup_transform(self.odom_frame, self.base_frame, Time())
        except TransformException:
            return
        odom_stamp = Time.from_msg(odom_base.header.stamp)
        if abs((now - odom_stamp).nanoseconds) > Duration(nanoseconds=500_000_000).nanoseconds:
            return  # Do not hide a stalled odometry source with fresh-looking TF.

        x, y, angle = self.correction
        msg = TransformStamped()
        msg.header.stamp = (now + Duration(nanoseconds=100_000_000)).to_msg()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.odom_frame
        msg.transform.translation.x = x
        msg.transform.translation.y = y
        msg.transform.rotation.z = math.sin(angle / 2)
        msg.transform.rotation.w = math.cos(angle / 2)
        self.broadcaster.sendTransform(msg)


def main():
    rclpy.init()
    node = MapOdomTF()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
