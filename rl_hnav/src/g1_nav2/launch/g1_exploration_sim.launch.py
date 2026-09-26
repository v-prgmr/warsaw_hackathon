"""Opt-in frontier exploration attached to an already running G1 simulation.

Start MuJoCo and nav_amcl.launch.py first, including the operator's 0/1
initialization. A read-only Nav2 preflight must pass before explore_lite starts;
explore_lite immediately sends navigation goals when it starts.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

from g1_nav2.sim_environment import ros_library_path


def generate_launch_description():
    config = os.path.join(get_package_share_directory('g1_nav2'), 'params', 'explore_g1_sim.yaml')
    preflight = ExecuteProcess(
        cmd=['ros2', 'run', 'g1_nav2', 'check_nav_goal', '--preflight-only'],
        output='screen',
    )
    explorer = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        parameters=[config],
        output='screen',
    )

    def after_preflight(event, context):
        if event.returncode != 0:
            return [LogInfo(msg='Exploration preflight failed; no goals will be sent.')]
        return [LogInfo(msg='Nav2 preflight passed; starting autonomous frontier exploration.'),
                explorer]

    return LaunchDescription([
        SetEnvironmentVariable(
            'LD_LIBRARY_PATH', ros_library_path(os.environ.get('LD_LIBRARY_PATH', ''))
        ),
        RegisterEventHandler(OnProcessExit(target_action=preflight, on_exit=after_preflight)),
        preflight,
    ])
