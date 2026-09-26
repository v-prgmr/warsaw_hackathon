"""Publish TF (header.frame_id -> child_frame_id) from an Odometry topic, e.g. /dog_odom.

Used only for g1_mapping's `odom_source:=dog_odom` fallback on replayed bags. It mirrors the
role of rl_hnav's odom_tf_bridge (/dog_odom -> /odom + TF); prefer the team's bridge
(AGENTS.md task A) on the live robot. Optionally republishes the message on `odom_out`.

Topics (remap): odom_in -> source odometry, odom_out -> republished odometry.
"""
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from g1_mapping.lifetime import exit_with_parent


class OdomToTf(Node):
    def __init__(self):
        super().__init__("odom_to_tf")
        self.republish = self.declare_parameter("republish", True).value
        self.br = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, "odom_out", 50) if self.republish else None
        self.create_subscription(Odometry, "odom_in", self.cb, 100)

    def cb(self, msg):
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        p = msg.pose.pose.position
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)
        if self.pub:
            self.pub.publish(msg)


def main():
    exit_with_parent()
    rclpy.init()
    try:
        rclpy.spin(OdomToTf())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
