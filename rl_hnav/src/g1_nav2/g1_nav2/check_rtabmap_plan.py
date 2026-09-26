"""Read-only G1 Nav2 preflight: verify a short path without sending motion goals.

Run with g1_sensors, g1_mapping and rtabmap_nav_dry_run.launch.py. The only
action called here is ComputePathToPose; no NavigateToPose or /cmd_vel output.
"""

from collections import deque
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PolygonStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, LaserScan
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    node = Node("g1_rtabmap_plan_check")
    state = {}
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, "/map", lambda m: state.update(map=m), latched)
    node.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                             lambda m: state.update(costmap=m), latched)
    node.create_subscription(Odometry, "/odom", lambda m: state.update(odom=m),
                             qos_profile_sensor_data)
    node.create_subscription(LaserScan, "/scan", lambda m: state.update(scan=m),
                             qos_profile_sensor_data)
    node.create_subscription(BatteryState, "/battery_state",
                             lambda m: state.update(battery=m), 10)
    node.create_subscription(PolygonStamped, "/local_costmap/published_footprint",
                             lambda m: state.update(footprint=m), 10)
    tf = Buffer(node=node)
    listener = TransformListener(tf, node)
    planner = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
    lifecycle = node.create_client(GetState, "/bt_navigator/get_state")

    def until(predicate, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def fresh(msg, max_age):
        stamp = msg.header.stamp
        age = (node.get_clock().now().nanoseconds -
               (stamp.sec * 1_000_000_000 + stamp.nanosec)) / 1e9
        return -0.1 <= age <= max_age

    try:
        required = ("map", "costmap", "odom", "scan", "battery", "footprint")
        if not until(lambda: all(name in state for name in required) and
                     tf.can_transform("map", "robot_center", Time()), 12):
            missing = [name for name in required if name not in state]
            raise RuntimeError(f"Missing inputs: {missing}; check TF map -> robot_center")
        if node.get_publishers_info_by_topic("/cmd_vel") or \
                node.get_subscriptions_info_by_topic("/cmd_vel"):
            raise RuntimeError("A live /cmd_vel publisher/subscriber is present; dry-run only")
        # RTAB-Map retains the last occupancy-grid stamp while the robot stands;
        # dynamic odom, scan and footprint are the live-data freshness checks.
        for name, max_age in (("odom", 0.8), ("scan", 0.8),
                              ("battery", 1), ("footprint", 0.8)):
            if not fresh(state[name], max_age):
                raise RuntimeError(f"Stale {name} stamp; check robot/laptop clock and sensors")
        if not math.isfinite(state["battery"].percentage) or \
                state["battery"].percentage < 0.20 or \
                state["battery"].percentage > 1.0:
            raise RuntimeError("Battery missing/invalid/below 20%")
        if not any(math.isfinite(r) for r in state["scan"].ranges):
            raise RuntimeError("/scan has no finite ranges")
        if not state["footprint"].polygon.points:
            raise RuntimeError("Local costmap has no robot footprint")
        if not lifecycle.wait_for_service(timeout_sec=3):
            raise RuntimeError("Nav2 lifecycle service unavailable")
        future = lifecycle.call_async(GetState.Request())
        if not until(future.done, 3):
            raise RuntimeError("Nav2 lifecycle request timed out")
        lifecycle_state = future.result()
        if lifecycle_state is None or lifecycle_state.current_state.label != "active":
            raise RuntimeError("bt_navigator is not active")

        transform = tf.lookup_transform("map", "robot_center", Time()).transform
        x, y = transform.translation.x, transform.translation.y
        rot = transform.rotation
        yaw = math.atan2(2 * (rot.w * rot.z + rot.x * rot.y),
                         1 - 2 * (rot.y * rot.y + rot.z * rot.z))
        grid = state["costmap"]
        width, height = grid.info.width, grid.info.height
        resolution = grid.info.resolution
        origin = grid.info.origin.position
        ix = int(math.floor((x - origin.x) / resolution))
        iy = int(math.floor((y - origin.y) / resolution))

        def traversable(cx, cy):
            if cx < 0 or cy < 0 or cx >= width or cy >= height:
                return False
            return 0 <= grid.data[cy * width + cx] < 99

        if not traversable(ix, iy):
            raise RuntimeError("Robot is outside known, non-lethal costmap cells")
        cells = deque([(ix, iy)])
        visited = {(ix, iy)}
        candidates = []
        while cells:
            cx, cy = cells.popleft()
            gx = origin.x + (cx + 0.5) * resolution
            gy = origin.y + (cy + 0.5) * resolution
            dx, dy = gx - x, gy - y
            forward = math.cos(yaw) * dx + math.sin(yaw) * dy
            lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
            if 0.60 <= math.hypot(dx, dy) <= 1.25 and forward >= 0.4 and \
                    abs(lateral) < 0.6 and grid.data[cy * width + cx] == 0:
                candidates.append((abs(lateral) + abs(forward - 0.8), gx, gy))
            for ox, oy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                neighbor = (cx + ox, cy + oy)
                if neighbor not in visited and traversable(*neighbor):
                    visited.add(neighbor)
                    cells.append(neighbor)
        print(f"Fresh inputs; battery={state['battery'].percentage:.0%}; "
              f"connected costmap cells={len(visited)}; candidate free cells={len(candidates)}",
              flush=True)
        if not candidates:
            raise RuntimeError("No reachable known-free short goal; survey with the vendor remote")
        if not planner.wait_for_server(timeout_sec=3):
            raise RuntimeError("ComputePathToPose unavailable")
        for _, gx, gy in sorted(candidates)[:8]:
            goal = ComputePathToPose.Goal()
            goal.goal.header.frame_id = "map"
            goal.goal.header.stamp = node.get_clock().now().to_msg()
            goal.goal.pose.position.x = gx
            goal.goal.pose.position.y = gy
            goal.goal.pose.orientation = rot
            goal.planner_id = "GridBased"
            response = planner.send_goal_async(goal)
            if not until(response.done, 3):
                continue
            handle = response.result()
            if handle is None or not handle.accepted:
                continue
            result_future = handle.get_result_async()
            if not until(result_future.done, 4):
                continue
            result = result_future.result()
            if result is not None and result.status == 4 and len(result.result.path.poses) > 1:
                print(f"PASS: computed {len(result.result.path.poses)}-pose "
                      f"path to ({gx:.2f}, {gy:.2f}) in map; NO motion goal sent", flush=True)
                return
        raise RuntimeError("Nav2 could not plan to any known-free short goal")
    except RuntimeError as exc:
        print(f"NOT READY: {exc}", file=sys.stderr, flush=True)
        raise SystemExit(1) from exc
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
