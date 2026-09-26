"""Bounded, simulation-only NavigateToPose acceptance check."""

import argparse
import math
import os
import time

import rclpy
from geometry_msgs.msg import PolygonStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Send the checked simulation goal')
    parser.add_argument('--preflight-only', action='store_true',
                        help='Check feedback without a goal or requiring a clear corridor')
    parser.add_argument('--distance', type=float, default=0.8)
    parser.add_argument('--timeout', type=float, default=12.0)
    args = parser.parse_args()
    if args.execute and args.preflight_only:
        parser.error('--execute and --preflight-only are mutually exclusive')
    if not 0.5 <= args.distance <= 1.0 or not 1 <= args.timeout <= 20:
        parser.error('distance must be 0.5–1 m and timeout 1–20 s')
    if (os.environ.get('ROS_DOMAIN_ID') != os.environ.get('G1_SIM_DOMAIN_ID', '76') or
            os.environ.get('ROS_LOCALHOST_ONLY') != '1' or
            os.environ.get('GAZEBO_MASTER_URI') != 'http://127.0.0.1:11476'):
        parser.error('run through sim_ros2 with the private simulation graph')

    rclpy.init()
    node = Node('g1_nav_goal_check', parameter_overrides=[Parameter('use_sim_time', value=True)])
    state = {'odom': None, 'costmap': None, 'footprint': None,
             'commands': 0, 'linear': 0, 'max_yaw': 0.0}
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Odometry, '/odom', lambda msg: state.update(odom=msg), 10)
    node.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                             lambda msg: state.update(costmap=msg), qos)
    node.create_subscription(PolygonStamped, '/local_costmap/published_footprint',
                             lambda msg: state.update(footprint=msg), 10)

    def on_command(msg):
        state['commands'] += 1
        state['linear'] += abs(msg.linear.x) > 0.01
        state['max_yaw'] = max(state['max_yaw'], abs(msg.angular.z))

    node.create_subscription(Twist, '/cmd_vel', on_command, 10)
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    lifecycle = node.create_client(GetState, '/bt_navigator/get_state')

    def until(predicate, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def pose():
        try:
            return buffer.lookup_transform('map', 'base_footprint', Time()).transform
        except TransformException:
            return None

    def feedback_ok():
        footprint = state['footprint']
        base = pose()
        if footprint is None or not footprint.polygon.points or base is None:
            return False
        age = (node.get_clock().now().nanoseconds -
               Time.from_msg(footprint.header.stamp).nanoseconds) / 1e9
        points = footprint.polygon.points
        error = math.hypot(sum(p.x for p in points) / len(points) - base.translation.x,
                           sum(p.y for p in points) / len(points) - base.translation.y)
        return -0.1 <= age < 0.7 and error < 0.2

    def cost(map_msg, x, y):
        ix = int((x - map_msg.info.origin.position.x) / map_msg.info.resolution)
        iy = int((y - map_msg.info.origin.position.y) / map_msg.info.resolution)
        if ix < 0 or iy < 0 or ix >= map_msg.info.width or iy >= map_msg.info.height:
            return 100
        return map_msg.data[iy * map_msg.info.width + ix]

    handle = None
    result = None
    try:
        assert lifecycle.wait_for_service(timeout_sec=8), 'Nav2 lifecycle unavailable'
        future = lifecycle.call_async(GetState.Request())
        assert until(lambda: future.done(), 3), 'Nav2 lifecycle timed out'
        assert future.result().current_state.label == 'active', 'Nav2 is not active'
        assert client.wait_for_server(timeout_sec=3), 'NavigateToPose is unavailable'
        assert until(lambda: state['odom'] is not None and state['costmap'] is not None and
                     feedback_ok(), 7), 'odom, costmap, or Nav2 footprint is stale'
        if args.preflight_only:
            print('Nav2 active; map, odometry, TF, and controller footprint fresh', flush=True)
            return

        base = pose()
        q = base.rotation
        heading = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
        x = base.translation.x + args.distance * math.cos(heading)
        y = base.translation.y + args.distance * math.sin(heading)
        costs = [cost(state['costmap'], base.translation.x + t * (x - base.translation.x),
                      base.translation.y + t * (y - base.translation.y))
                 for t in (0, .25, .5, .75, 1)]
        print(f'start=({base.translation.x:.3f}, {base.translation.y:.3f}) '
              f'goal=({x:.3f}, {y:.3f}) cost={costs}', flush=True)
        assert all(value == 0 for value in costs), 'goal corridor not entirely free'
        if not args.execute:
            print('Dry run only; add --execute to send this simulation goal', flush=True)
            return

        initial = state['odom'].pose.pose.position
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation = q
        sent = client.send_goal_async(goal)
        assert until(lambda: sent.done(), 5), 'goal acknowledgement timed out'
        handle = sent.result()
        assert handle.accepted, 'goal rejected'
        print('Goal accepted', flush=True)
        result = handle.get_result_async()
        reason = None
        end = time.monotonic() + args.timeout
        while time.monotonic() < end and not result.done():
            rclpy.spin_once(node, timeout_sec=0.1)
            current = state['odom'].pose.pose.position
            if math.hypot(current.x - initial.x, current.y - initial.y) > args.distance + 0.3:
                reason = 'displacement limit'
                break
            if not feedback_ok():
                reason = 'Nav2 feedback stale'
                break
        if not result.done():
            reason = reason or 'timeout'
            cancel = handle.cancel_goal_async()
            until(lambda: cancel.done(), 5)
            until(lambda: result.done(), 5)
            print(f'Cancelled: {reason}; acknowledgement={cancel.done()}', flush=True)
        current = state['odom'].pose.pose.position
        distance = math.hypot(current.x - initial.x, current.y - initial.y)
        print(f'action status={result.result().status if result.done() else "unknown"} '
              f'odom displacement={distance:.3f} m '
              f'cmd samples={state["commands"]} linear samples={state["linear"]} '
              f'max yaw={state["max_yaw"]:.3f}', flush=True)
    finally:
        if handle is not None and rclpy.ok() and (result is None or not result.done()):
            cancel = handle.cancel_goal_async()
            until(lambda: cancel.done(), 5)
            print(f'Goal cleanup cancellation acknowledgement={cancel.done()}', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
