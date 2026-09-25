"""Run the keyframe_node with parameters from config/keyframe_params.yaml.

Override the output directory with `output_dir:=<path>` and use simulated time when replaying a
bag with `use_sim_time:=true` (then `ros2 bag play <bag> --clock`).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("keyframe_manager"), "config", "keyframe_params.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument("output_dir", default_value="keyframes"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="keyframe_manager",
            executable="keyframe_node",
            name="keyframe_node",
            output="screen",
            parameters=[
                params,
                {
                    "output_dir": LaunchConfiguration("output_dir"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                },
            ],
        ),
    ])
