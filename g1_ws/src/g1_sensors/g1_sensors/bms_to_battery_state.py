"""Read-only G1 BMS SOC bridge: /lf/bmsstate -> /battery_state.

Unitree's BmsState.soc is a percent in 0..100; the standard ROS
BatteryState.percentage is a fraction in 0..1. Unknown values fail closed.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from unitree_hg.msg import BmsState

from g1_sensors.lifetime import exit_with_parent


class BmsToBatteryState(Node):
    def __init__(self):
        super().__init__("g1_bms_to_battery_state")
        self.publisher = self.create_publisher(BatteryState, "battery_state", 10)
        self.subscription = self.create_subscription(
            BmsState, "bms", self.on_bms, qos_profile_sensor_data)
        self.last_soc = None

    def on_bms(self, msg):
        soc = int(msg.soc)
        battery = BatteryState()
        # BmsState has no stamp. The laptop clock is synchronized to the robot.
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.header.frame_id = "robot_center"
        battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        battery.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        battery.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        battery.voltage = math.nan  # Units of the BMS voltage fields are unverified.
        battery.current = math.nan
        battery.present = 0 <= soc <= 100
        battery.percentage = soc / 100.0 if battery.present else math.nan
        self.publisher.publish(battery)
        if soc != self.last_soc:
            if not battery.present:
                self.get_logger().warn(f"Invalid Unitree BMS SOC: {soc}")
            else:
                self.get_logger().info(f"Unitree BMS SOC: {soc}%")
            self.last_soc = soc


def main():
    exit_with_parent()
    rclpy.init()
    try:
        rclpy.spin(BmsToBatteryState())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
