"""Nav2 on the RTAB-Map /map for the real G1, shared by the dry-run and live launches.

Both launches start the same nodes with the same parameters (params/nav2_g1_rtabmap.yaml,
behavior_trees/*_g1.xml). Only the velocity output differs: every Nav2 velocity, from the
controller and from the recovery behaviors, goes to one raw topic, then through
velocity_smoother to the output topic. The dry run writes /g1_nav2_dry_run/cmd_vel; the live
launch writes /cmd_vel for g1_loco_cmdvel's cmd_vel_gateway. No SLAM Toolbox, odom bridge or
locomotion node: g1_mapping owns /map and map -> odom -> robot_center (AGENTS.md §6).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

# g1_loco_cmdvel cmd_vel_gateway clamps (AGENTS.md §13). Nav2 must plan with the speeds the
# robot actually gets: a plan at 0.3 m/s / 1.0 rad/s is clamped 3-5x by the gateway and tracked
# badly, and a 1.0 rad/s spin recovery at 0.2 rad/s outlasts its time allowance.
GATEWAY_MAX_VX = 0.10  # m/s
GATEWAY_MAX_WZ = 0.20  # rad/s
# Operator-requested ceilings for the multi-terminal bringup. Preserve the
# standalone defaults above; the script sets both Nav2 and gateway limits.
NAV_MAX_VX = 0.50  # m/s; matches SDK hard ceiling
NAV_MAX_WZ = 1.0  # rad/s; matches SDK hard ceiling

LIFECYCLE_NODES = [
    'controller_server', 'smoother_server', 'planner_server',
    'behavior_server', 'bt_navigator', 'waypoint_follower',
    'velocity_smoother',
]
EXECUTABLES = [
    ('nav2_controller', 'controller_server'),
    ('nav2_smoother', 'smoother_server'),
    ('nav2_planner', 'planner_server'),
    ('nav2_behaviors', 'behavior_server'),
    ('nav2_bt_navigator', 'bt_navigator'),
    ('nav2_waypoint_follower', 'waypoint_follower'),
    ('nav2_velocity_smoother', 'velocity_smoother'),
]


def speed_limits(max_vx, max_wz):
    """Parse and check the launch speed limits: positive and within the gateway clamps."""
    max_vx, max_wz = float(max_vx), float(max_wz)
    if not 0.0 < max_vx <= NAV_MAX_VX:
        raise ValueError(
            f'max_vx must be in (0, {NAV_MAX_VX}] m/s; match the gateway limit, got {max_vx}')
    if not 0.0 < max_wz <= NAV_MAX_WZ:
        raise ValueError(
            f'max_wz must be in (0, {NAV_MAX_WZ}] rad/s; match the gateway limit, got {max_wz}')
    return max_vx, max_wz


def speed_overrides(max_vx, max_wz):
    """Per-node parameter overrides that apply one speed limit to every velocity source."""
    return {
        'controller_server': {
            'FollowPath.desired_linear_vel': max_vx,
            'FollowPath.rotate_to_heading_angular_vel': max_wz,
        },
        'velocity_smoother': {
            'max_velocity': [max_vx, 0.0, max_wz],
            'min_velocity': [0.0, 0.0, -max_wz],  # no reversing (min vx 0)
        },
        'behavior_server': {
            'max_rotational_vel': max_wz,
            # Preserve explicitly lower launch ceilings for command-only tests.
            'min_rotational_vel': min(0.11, max_wz),
        },
    }


def nav2_nodes(raw_cmd_vel, out_cmd_vel, max_vx, max_wz):
    """Nav2 nodes + lifecycle manager; all velocity -> raw_cmd_vel -> smoother -> out_cmd_vel."""
    max_vx, max_wz = speed_limits(max_vx, max_wz)
    share = get_package_share_directory('g1_nav2')
    params = os.path.join(share, 'params', 'nav2_g1_rtabmap.yaml')
    bt_dir = os.path.join(share, 'behavior_trees')
    overrides = speed_overrides(max_vx, max_wz)
    # Behavior trees without BackUp: the G1 does not walk backwards in the survey zone.
    overrides['bt_navigator'] = {
        'default_nav_to_pose_bt_xml': os.path.join(bt_dir, 'navigate_to_pose_g1.xml'),
        'default_nav_through_poses_bt_xml': os.path.join(bt_dir, 'navigate_through_poses_g1.xml'),
    }

    nodes = []
    for package, executable in EXECUTABLES:
        remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static'), ('cmd_vel', raw_cmd_vel)]
        if executable == 'velocity_smoother':
            remappings.append(('cmd_vel_smoothed', out_cmd_vel))
        nodes.append(Node(
            package=package, executable=executable, name=executable, output='screen',
            parameters=[params, overrides.get(executable, {})], remappings=remappings,
        ))
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': False, 'autostart': True,
                     'node_names': LIFECYCLE_NODES}],
    ))
    return nodes
