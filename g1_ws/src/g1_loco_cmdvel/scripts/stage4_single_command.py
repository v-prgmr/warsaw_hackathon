#!/usr/bin/env python3
"""Stage 4: ONE tiny supervised high-level movement (see README.md, Stage 4).

Requests 0.05 m/s forward for about 0.25 s, then sends zero. Run it ONCE, only when the remote
operator says "ready". It sends nothing unless /battery_state is fresh (< 1 s) and >= 20 %, and
/cmd_vel has exactly one subscriber (g1_loco_cmdvel_gateway) and no other publisher. The finally
block always sends zero.

Offline dry run (domain 77, SDK client in dry-run, 2026-09-26): 5 x vx=0.05 then 6 zeros reach the
client; a stale bridge, a battery at 15 %, or an extra /cmd_vel subscriber sends no motion.
"""
import time
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from rclpy.node import Node

rclpy.init()
node = Node("g1_single_supervised_velocity_test")
state = {"percentage": None, "received": 0.0}

def on_battery(msg):
    state["percentage"] = msg.percentage
    state["received"] = time.monotonic()

battery_sub = node.create_subscription(
    BatteryState, "/battery_state", on_battery, 10)
publisher = node.create_publisher(Twist, "/cmd_vel", 10)

def battery_fresh():
    pct = state["percentage"]
    return (pct is not None and 0.20 <= pct <= 1.0 and
            time.monotonic() - state["received"] < 1.0)

try:
    deadline = time.monotonic() + 3.0
    while not battery_fresh() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not battery_fresh():
        raise RuntimeError("No fresh battery above 20%; no movement sent")

    deadline = time.monotonic() + 3.0
    while node.count_subscribers("/cmd_vel") != 1 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if (node.count_subscribers("/cmd_vel") != 1 or
            node.count_publishers("/cmd_vel") != 1):
        raise RuntimeError("Unexpected /cmd_vel endpoints; no movement sent")

    command = Twist()
    command.linear.x = 0.05
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.01)
        if (not battery_fresh() or
                node.count_subscribers("/cmd_vel") != 1 or
                node.count_publishers("/cmd_vel") != 1):
            raise RuntimeError("Battery or command graph changed; stopping")
        publisher.publish(command)
        time.sleep(0.05)
    print("Brief command sent; sending zero", flush=True)
finally:
    for _ in range(6):
        publisher.publish(Twist())
        time.sleep(0.05)
    node.destroy_node()
    rclpy.try_shutdown()
