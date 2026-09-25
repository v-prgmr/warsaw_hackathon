"""Normalize the G1 MID-360 internal IMU (/utlidar/imu_livox_mid360) for imu_filter_madgwick.

The stream reports linear_acceleration in g (at rest |a| ~ 1.0; z ~ -1.0 because livox_frame is
upside down) and leaves orientation all zeros. This node scales the acceleration to m/s^2
(accel_scale, default 9.80665) and marks the orientation as unknown (orientation_covariance[0] =
-1, REP-145). imu_filter_madgwick then estimates the orientation. Gyro (rad/s), header, and frame
are passed through.

Topics (remap): input -> raw LiDAR IMU, output -> fixed IMU.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class LivoxImuFix(Node):
    def __init__(self):
        super().__init__("livox_imu_fix")
        self.scale = float(self.declare_parameter("accel_scale", 9.80665).value)
        self.pub = self.create_publisher(Imu, "output", 50)
        self.create_subscription(Imu, "input", self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        a = msg.linear_acceleration
        a.x *= self.scale
        a.y *= self.scale
        a.z *= self.scale
        msg.orientation_covariance[0] = -1.0
        self.pub.publish(msg)


def main():
    rclpy.init()
    try:
        rclpy.spin(LivoxImuFix())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
