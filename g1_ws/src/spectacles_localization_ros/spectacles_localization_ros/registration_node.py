"""Register Spectacles' local tracking world to RTAB-Map's map frame."""

import json
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import (Buffer, StaticTransformBroadcaster, TransformBroadcaster,
                     TransformException, TransformListener)

from .protocol import parse
from .registration import Registrar
from .transforms import (Transform, compose, inverse, rotation_distance,
                         translation_distance)


def from_ros(msg):
    t, q = msg.translation, msg.rotation
    return Transform((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))


def pose_message(frame, stamp, value):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.header.stamp = stamp
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = value.position
    (msg.pose.orientation.x, msg.pose.orientation.y,
     msg.pose.orientation.z, msg.pose.orientation.w) = value.orientation
    return msg


def transform_message(parent, child, stamp, value):
    msg = TransformStamped()
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.header.stamp = stamp
    (msg.transform.translation.x, msg.transform.translation.y,
     msg.transform.translation.z) = value.position
    (msg.transform.rotation.x, msg.transform.rotation.y,
     msg.transform.rotation.z, msg.transform.rotation.w) = value.orientation
    return msg


class RegistrationNode(Node):
    def __init__(self):
        super().__init__("spectacles_registration")
        defaults = {
            "map_frame": "map", "tag_frame": "april_tag_0",
            "world_frame": "spectacles_world", "device_frame": "spectacles",
            "tag_id": 0, "tag_size_m": 0.16, "tag_pose_map_json": "[]",
            "publish_tag_tf": False, "minimum_detections": 10,
            "timeout_sec": 10.0, "tracking_timeout_sec": 1.0,
            "max_translation_residual_m": 0.25,
            "max_angular_residual_deg": 15.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        param = lambda key: self.get_parameter(key).value
        self.map_frame = param("map_frame")
        self.tag_frame = param("tag_frame")
        self.world_frame = param("world_frame")
        self.device_frame = param("device_frame")
        self.tag_id = int(param("tag_id"))
        self.tag_size_m = float(param("tag_size_m"))
        self.tracking_timeout = float(param("tracking_timeout_sec"))
        if self.tag_size_m <= 0 or self.tracking_timeout <= 0:
            raise ValueError("Tag size and tracking timeout must be positive")
        self.registrar = Registrar(param("minimum_detections"), param("timeout_sec"),
                                   param("max_translation_residual_m"),
                                   param("max_angular_residual_deg"))
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.tf = TransformBroadcaster(self)
        self.tag_tf = StaticTransformBroadcaster(self)
        values = json.loads(param("tag_pose_map_json"))
        if not isinstance(values, list):
            raise ValueError("tag_pose_map must be a list")
        self.configured_tag = (Transform(tuple(values[:3]), tuple(values[3:]))
                               if len(values) == 7 else None)
        if values and self.configured_tag is None:
            raise ValueError("tag_pose_map must be empty or [x,y,z,qx,qy,qz,qw]")
        if param("publish_tag_tf"):
            if self.configured_tag is None:
                raise ValueError("Cannot publish tag TF without measured tag_pose_map")
            self.tag_tf.sendTransform(transform_message(
                self.map_frame, self.tag_frame, self.get_clock().now().to_msg(),
                self.configured_tag))

        self.local_pub = self.create_publisher(PoseStamped,
                                                "/spectacles/device_pose_local", 10)
        self.tag_pub = self.create_publisher(PoseStamped,
                                              "/spectacles/tag_detection", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/spectacles/pose", 10)
        self.status_pub = self.create_publisher(String,
                                                 "/spectacles/localization_status", 10)
        self.create_subscription(String, "/spectacles/observation", self.on_observation, 20)
        self.create_timer(0.2, self.publish_status)
        self.session = None
        self.last_device_stamp = None
        self.last_seen = None
        self.registered_at = None
        self.last_drift_m = None
        self.last_drift_deg = None
        self.tracking_valid = False

    def map_tag(self):
        if self.configured_tag is not None:
            return self.configured_tag
        try:
            return from_ros(self.buffer.lookup_transform(
                self.map_frame, self.tag_frame, Time()).transform)
        except TransformException:
            return None

    def on_observation(self, msg):
        try:
            obs = parse(msg.data)
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warn(f"Invalid observation: {exc}")
            return
        if obs.session_id != self.session:
            self.session = obs.session_id
            self.registrar.reset()
            self.registered_at = None
            self.last_device_stamp = None
            self.get_logger().info("Tracking session changed; registration reset")
        if (self.last_device_stamp is not None
                and obs.timestamp_sec <= self.last_device_stamp):
            return
        self.last_device_stamp = obs.timestamp_sec
        self.last_seen = time.monotonic()
        self.tracking_valid = obs.tracking_valid
        if not obs.tracking_valid:
            return

        stamp = self.get_clock().now().to_msg()  # arrival time; clock sync is not assumed
        self.local_pub.publish(pose_message(self.world_frame, stamp, obs.world_device))
        if obs.tag_id == self.tag_id and obs.device_tag is not None:
            self.tag_pub.publish(pose_message(self.device_frame, stamp, obs.device_tag))
            map_tag = self.map_tag()
            if map_tag is not None:
                if self.registrar.result is None:
                    result = self.registrar.add(time.monotonic(), map_tag,
                                                obs.world_device, obs.device_tag)
                    if result is not None:
                        self.registered_at = time.monotonic()
                        self.get_logger().info(
                            f"LOCALIZED using {result.used} detections; "
                            f"position RMS {result.translation_stddev_m:.3f} m, "
                            f"angle RMS {result.angular_stddev_deg:.2f} deg")
                else:
                    predicted_tag = compose(self.registrar.map_device(obs.world_device),
                                            obs.device_tag)
                    self.last_drift_m = translation_distance(predicted_tag.position,
                                                              map_tag.position)
                    self.last_drift_deg = rotation_distance(
                        predicted_tag.orientation, map_tag.orientation) * 180 / 3.141592653589793
        mapped = self.registrar.map_device(obs.world_device)
        if mapped is not None:
            self.pose_pub.publish(pose_message(self.map_frame, stamp, mapped))
            self.tf.sendTransform(transform_message(self.map_frame, self.device_frame,
                                                    stamp, mapped))

    def publish_status(self):
        stale = self.last_seen is None or time.monotonic() - self.last_seen > self.tracking_timeout
        if stale or not self.tracking_valid:
            state = "TRACKING_LOST" if self.last_seen is not None else "UNLOCALIZED"
        elif self.registrar.result is not None:
            state = "LOCALIZED"
        else:
            state = "COLLECTING" if self.registrar.samples else "UNLOCALIZED"
        result = self.registrar.result
        status = {
            "state": state,
            "tracking_state": "VALID" if not stale and self.tracking_valid else "LOST",
            "detections": len(self.registrar.samples),
            "registration_translation_stddev_m": (result.translation_stddev_m if result else None),
            "registration_angular_stddev_deg": (result.angular_stddev_deg if result else None),
            "time_since_registration_sec": (round(time.monotonic() - self.registered_at, 2)
                                            if self.registered_at else None),
            "last_tag_revisit_error_m": self.last_drift_m,
            "last_tag_revisit_error_deg": self.last_drift_deg,
            "tag_pose_available": self.map_tag() is not None,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = RegistrationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
