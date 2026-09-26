"""Nav2 command-only bringup on RTAB-Map: velocity cannot reach /cmd_vel.

Start g1_sensors TF, g1_mapping (static_tf:=false), and the /scan-only pipeline
first. This launch has no SLAM Toolbox, /dog_odom bridge, or locomotion node.
Its output is deliberately isolated from any robot command subscriber: the
controller and recovery behaviors write /g1_nav2_dry_run/cmd_vel_raw, and the
velocity smoother writes /g1_nav2_dry_run/cmd_vel.

Same nodes and parameters as rtabmap_nav_live.launch.py (g1_nav2.rtabmap_nav),
so this dry run tests exactly what the live launch runs.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from g1_nav2.rtabmap_nav import GATEWAY_MAX_VX, GATEWAY_MAX_WZ, nav2_nodes


def _nodes(context):
    return nav2_nodes('/g1_nav2_dry_run/cmd_vel_raw', '/g1_nav2_dry_run/cmd_vel',
                      LaunchConfiguration('max_vx').perform(context),
                      LaunchConfiguration('max_wz').perform(context))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('max_vx', default_value=str(GATEWAY_MAX_VX),
                              description='Forward speed limit (m/s), <= the gateway clamp.'),
        DeclareLaunchArgument('max_wz', default_value=str(GATEWAY_MAX_WZ),
                              description='Turn rate limit (rad/s), <= the gateway clamp.'),
        OpaqueFunction(function=_nodes),
    ])
