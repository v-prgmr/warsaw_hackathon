"""Nav2 on RTAB-Map with LIVE velocity output: the velocity smoother publishes /cmd_vel.

The live twin of rtabmap_nav_dry_run.launch.py: same nodes and parameters
(g1_nav2.rtabmap_nav), no SLAM Toolbox, /dog_odom bridge, or locomotion node.
g1_mapping owns /map and map -> odom -> robot_center. The controller and recovery
behaviors write /g1_nav2/cmd_vel_nav; only velocity_smoother publishes /cmd_vel,
limited to max_vx / max_wz (at most the cmd_vel_gateway clamps), never backwards.

This launch alone moves nothing: g1_loco_cmdvel's cmd_vel_gateway (enabled:=true)
and g1_loco_client (--enabled=true --i-accept-high-level-actuation=true) turn
/cmd_vel into Loco SetVelocity. Use it only after Stage 4, the G0-G3 gates
(g1_ws/NAV2_FIRST_STEP.md) and AGENTS.md §19 / §25, with the remote operator ready.
Run check_rtabmap_plan BEFORE this launch: it refuses while /cmd_vel exists.
For a first goal with the Stage 4 gateway limits, match them:
    ros2 launch g1_nav2 rtabmap_nav_live.launch.py max_vx:=0.05 max_wz:=0.10
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from g1_nav2.rtabmap_nav import GATEWAY_MAX_VX, GATEWAY_MAX_WZ, nav2_nodes


def _nodes(context):
    return nav2_nodes('/g1_nav2/cmd_vel_nav', '/cmd_vel',
                      LaunchConfiguration('max_vx').perform(context),
                      LaunchConfiguration('max_wz').perform(context))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('max_vx', default_value=str(GATEWAY_MAX_VX),
                              description='Forward speed limit (m/s), <= the gateway clamp. '
                                          'Match the gateway max_vx if it runs tighter.'),
        DeclareLaunchArgument('max_wz', default_value=str(GATEWAY_MAX_WZ),
                              description='Turn rate limit (rad/s), <= the gateway clamp. '
                                          'Match the gateway max_wz if it runs tighter.'),
        OpaqueFunction(function=_nodes),
    ])
