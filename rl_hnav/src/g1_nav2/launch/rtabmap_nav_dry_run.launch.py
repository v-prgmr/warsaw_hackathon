"""Nav2 command-only bringup on RTAB-Map: velocity cannot reach /cmd_vel.

Start g1_sensors TF, g1_mapping (static_tf:=false), and the /scan-only pipeline
first. This launch has no SLAM Toolbox, /dog_odom bridge, or locomotion node.
Its output is deliberately isolated from any robot command subscriber.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('g1_nav2'), 'params', 'nav2_g1_slam_real.yaml')
    dry_cmd = '/g1_nav2_dry_run/cmd_vel'
    lifecycle_nodes = [
        'controller_server', 'smoother_server', 'planner_server',
        'behavior_server', 'bt_navigator', 'waypoint_follower',
        'velocity_smoother',
    ]
    executables = [
        ('nav2_controller', 'controller_server'),
        ('nav2_smoother', 'smoother_server'),
        ('nav2_planner', 'planner_server'),
        ('nav2_behaviors', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator'),
        ('nav2_waypoint_follower', 'waypoint_follower'),
        ('nav2_velocity_smoother', 'velocity_smoother'),
    ]

    nodes = []
    for package, executable in executables:
        remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
        if executable in ('controller_server', 'velocity_smoother'):
            remappings.append(('cmd_vel', '/g1_nav2_dry_run/cmd_vel_raw'))
        else:
            remappings.append(('cmd_vel', dry_cmd))
        if executable == 'velocity_smoother':
            remappings.append(('cmd_vel_smoothed', dry_cmd))
        nodes.append(Node(
            package=package, executable=executable, name=executable,
            output='screen', parameters=[params], remappings=remappings,
        ))

    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': False, 'autostart': True,
                     'node_names': lifecycle_nodes}],
    ))
    return LaunchDescription(nodes)
