"""Launch the add-on cloud annotator; configure real OAK-D topics explicitly."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    names = {
        "cloud_topic": "/cloud_map",
        "rgb_topic": "",
        "depth_topic": "",
        "camera_info_topic": "",
        "detector": "coco_maskrcnn",
        "device": "cpu",
        "max_rate_hz": "1.0",
        "max_points": "200000",
        "voxel_m": "0.04",
        "depth_tolerance_m": "0.10",
        "score_threshold": "0.65",
        "use_sim_time": "false",
    }
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in names.items()]
    args.append(Node(
        package="g1_semantic_map", executable="semantic_map_node", output="screen",
        parameters=[{k: LaunchConfiguration(k) for k in names}],
    ))
    return LaunchDescription(args)
