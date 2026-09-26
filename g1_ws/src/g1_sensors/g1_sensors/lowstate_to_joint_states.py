"""Bridge Unitree /lowstate (unitree_hg/LowState) to /joint_states for robot_state_publisher.

Read-only: subscribes to the robot's state and publishes sensor_msgs/JointState. It never
publishes anything the robot consumes. Uses the unitree_hg ROS 2 message package over the normal
ROS 2 graph (no unitree_sdk2 in this process, AGENTS.md §5).

Stamps: LowState has no header. Joint states are stamped on the ROBOT's clock, so /tf lines up
with the LiDAR and IMU header stamps even when the host clock is off (it was ~73 s ahead). The
offset is the median of (header.stamp - receive time) of a robot-stamped reference topic
(the LiDAR IMU) over the last `clock_window` seconds. Works live and in replay (use_sim_time).
stamp_source:=receive uses the receive time instead.

CPU: inputs are taken as raw CDR bytes. Only the header stamp is read from the reference (any
message that starts with std_msgs/Header works), and LowState is deserialized only when a joint
state is published (publish_rate).

Topics (remap): lowstate -> unitree_hg/LowState (/lf/lowstate, 20 Hz, live and in bags),
clock_reference -> a header-stamped message (sensor_msgs/Imu), joint_states -> output.
"""
from collections import deque
import struct

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState
from unitree_hg.msg import LowState


class LowStateToJointStates(Node):
    def __init__(self):
        super().__init__("lowstate_to_joint_states")
        self.names = list(self.declare_parameter("joint_names", [""]).value)
        rate = float(self.declare_parameter("publish_rate", 50.0).value)
        self.stamp_source = self.declare_parameter("stamp_source", "robot_clock").value
        self.window_ns = int(float(self.declare_parameter("clock_window", 2.0).value) * 1e9)
        if not self.names or not all(self.names):
            raise RuntimeError("joint_names must list the LowState motor order")
        if self.stamp_source not in ("robot_clock", "receive"):
            raise RuntimeError("stamp_source must be 'robot_clock' or 'receive'")
        self.min_period_ns = int(1e9 / rate) if rate > 0 else 0
        self.last_pub_ns = None
        self.offsets = deque()  # (receive ns, header - receive ns)
        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_subscription(LowState, "lowstate", self.on_lowstate, qos_profile_sensor_data,
                                 raw=True)
        if self.stamp_source == "robot_clock":
            self.create_subscription(Imu, "clock_reference", self.on_reference,
                                     qos_profile_sensor_data, raw=True)
        self.warned = False

    def on_reference(self, raw):
        now = self.get_clock().now().nanoseconds
        if self.offsets and now < self.offsets[-1][0]:
            self.offsets.clear()  # time went backwards (bag restarted)
        # CDR: 4-byte encapsulation header (byte 1 == 1: little endian), then header.stamp
        sec, nanosec = struct.unpack_from("<iI" if raw[1] == 1 else ">iI", raw, 4)
        stamp = sec * 1_000_000_000 + nanosec
        self.offsets.append((now, stamp - now))
        while self.offsets[0][0] < now - self.window_ns:
            self.offsets.popleft()

    def on_lowstate(self, raw):
        now = self.get_clock().now().nanoseconds
        if self.last_pub_ns is not None and 0 <= now - self.last_pub_ns < self.min_period_ns:
            return
        if self.stamp_source == "robot_clock":
            if not self.offsets:
                if not self.warned:
                    self.get_logger().warn("waiting for clock_reference to learn the robot clock")
                    self.warned = True
                return
            offset = int(np.median([o for _, o in self.offsets]))
        else:
            offset = 0
        self.last_pub_ns = now
        msg = deserialize_message(raw, LowState)
        js = JointState()
        js.header.stamp = Time(nanoseconds=now + offset).to_msg()
        js.name = self.names
        motors = msg.motor_state[:len(self.names)]
        js.position = [float(m.q) for m in motors]
        js.velocity = [float(m.dq) for m in motors]
        js.effort = [float(m.tau_est) for m in motors]
        self.pub.publish(js)


def main():
    rclpy.init()
    try:
        rclpy.spin(LowStateToJointStates())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
