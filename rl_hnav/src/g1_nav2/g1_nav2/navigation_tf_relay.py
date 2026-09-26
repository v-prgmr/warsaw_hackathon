"""Relay only the dynamic G1 navigation and LiDAR TF chain to Nav2.

RobotStatePublisher sends the entire moving G1 skeleton on /tf. The simulated
laser is fixed to torso_link; its three waist joints are the only moving
joints in the base-to-laser chain. Static links remain on /tf_static. Do not
alter original transform timestamps or change the /tf owner.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_msgs.msg import TFMessage


NAVIGATION_PARENTS = {
    'odom': 'map',
    'base_footprint': 'odom',
    'waist_yaw_link': 'pelvis',
    'waist_roll_link': 'waist_yaw_link',
    'torso_link': 'waist_roll_link',
}


def current_navigation_transforms(transforms, now):
    """Only publish a complete, fresh base and laser chain."""
    if any(child not in transforms or transforms[child].header.frame_id != parent
           for child, parent in NAVIGATION_PARENTS.items()):
        return []
    current = [transforms[child] for child in NAVIGATION_PARENTS]
    if any(abs((now - Time.from_msg(item.header.stamp)).nanoseconds) > 500_000_000
           for item in current):
        return []
    return current


class NavigationTFRelay(Node):
    def __init__(self):
        super().__init__('navigation_tf_relay')
        self.transforms = {}
        self.last_now = None
        self.last_published_base = None
        self.create_subscription(TFMessage, '/tf', self.receive, 100)
        self.publisher = self.create_publisher(
            TFMessage, '/nav_tf', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.create_timer(0.1, self.publish)

    def receive(self, message):
        for transform in message.transforms:
            key = transform.child_frame_id
            if transform.header.frame_id != NAVIGATION_PARENTS.get(key):
                continue
            stamp = Time.from_msg(transform.header.stamp)
            previous = self.transforms.get(key)
            if previous is None or stamp >= Time.from_msg(previous.header.stamp):
                self.transforms[key] = transform

    def publish(self):
        now = self.get_clock().now()
        if self.last_now is not None and now < self.last_now:
            self.transforms.clear()
            self.last_published_base = None
        self.last_now = now
        if now.nanoseconds == 0:
            return

        current = current_navigation_transforms(self.transforms, now)
        if current:
            base_stamp = Time.from_msg(self.transforms['base_footprint'].header.stamp)
            if self.last_published_base is not None and base_stamp <= self.last_published_base:
                return
            self.last_published_base = base_stamp
            self.publisher.publish(TFMessage(transforms=current))


def main():
    rclpy.init()
    node = NavigationTFRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
