"""Run the semantic_query poi_node with config/semantic_query.yaml.

Override the detector backend for offline testing with `backend:=mock`.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("semantic_query"), "config", "semantic_query.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="grounding_dino_sam2",
                              description="grounding_dino_sam2 | mock"),
        Node(
            package="semantic_query",
            executable="poi_node",
            name="poi_node",
            output="screen",
            parameters=[params, {"backend": LaunchConfiguration("backend")}],
        ),
    ])
