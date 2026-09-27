"""Frontier exploration (explore_lite) on the RTAB-Map /map of the real G1 (AGENTS.md §15).

Start g1_sensors TF, g1_mapping, the /scan pipeline and rtabmap_nav_dry_run.launch.py (or
rtabmap_nav_live.launch.py) first; scripts/start_g1_navigation.sh --explore does all of it.
explore_lite sends NavigateToPose goals as soon as it starts:
  - dry run: Nav2 plans and its velocities go to /g1_nav2_dry_run/cmd_vel, nothing moves.
  - live: the G1 WALKS to the frontiers. Only after the dry run and the AGENTS.md §19 / §25
    gates, inside a prepared safety zone, with the remote's e-stop in hand.

preflight:=true (default) runs check_rtabmap_plan first (read-only; ComputePathToPose only) and
starts the explorer only if it passes. It refuses while /cmd_vel has a publisher or subscriber,
so live runs use preflight:=false after the same check passed in the dry run.
Pause / resume: ros2 topic pub --once /explore/resume std_msgs/msg/Bool '{data: false}' / true.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def _setup(context):
    config = os.path.join(get_package_share_directory('g1_nav2'), 'params',
                          'explore_g1_rtabmap.yaml')
    overrides = {}
    size = context.launch_configurations.get('min_frontier_size', '')
    if size:
        overrides['min_frontier_size'] = float(size)
    explorer = Node(package='explore_lite', executable='explore', name='explore_node',
                    parameters=[config, overrides], output='screen')
    if context.launch_configurations['preflight'].lower() not in ('true', '1'):
        return [LogInfo(msg='Preflight skipped; starting frontier exploration.'), explorer]

    preflight = ExecuteProcess(cmd=['ros2', 'run', 'g1_nav2', 'check_rtabmap_plan'],
                               output='screen')

    def after_preflight(event, _context):
        if event.returncode != 0:
            return [LogInfo(msg='Nav2 preflight NOT READY; exploration not started, no goals '
                                'sent. Survey more with the remote, then start again.')]
        return [LogInfo(msg='Nav2 preflight passed; starting frontier exploration.'), explorer]

    return [RegisterEventHandler(OnProcessExit(target_action=preflight,
                                               on_exit=after_preflight)),
            preflight]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('preflight', default_value='true',
                              description='Run check_rtabmap_plan first (dry run only).'),
        DeclareLaunchArgument('min_frontier_size', default_value='',
                              description='Override the YAML minimum frontier size (m).'),
        OpaqueFunction(function=_setup),
    ])
