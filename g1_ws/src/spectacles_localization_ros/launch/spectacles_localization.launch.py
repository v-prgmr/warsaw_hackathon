"""ROS-side Spectacles registration; mock mode needs no device or RTAB-Map."""

import json
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = get_package_share_directory("spectacles_localization_ros")
    mock = LaunchConfiguration("mock").perform(context).lower() in ("true", "1")
    config_path = LaunchConfiguration("config").perform(context)
    if not config_path:
        config_path = os.path.join(share, "config", "mock.yaml" if mock else
                                   "spectacles_localization.yaml")
    with open(config_path, encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream)
    tag, frames, reg = cfg["tag"], cfg["frames"], cfg["registration"]
    params = {
        "use_sim_time": LaunchConfiguration("use_sim_time").perform(context).lower() == "true",
        "map_frame": frames["map"], "tag_frame": frames["tag"],
        "world_frame": frames["world"], "device_frame": frames["spectacles"],
        "tag_id": int(tag["id"]), "tag_size_m": float(tag["size_m"]),
        "tag_pose_map_json": json.dumps(cfg.get("tag_pose_map") or []),
        "publish_tag_tf": bool(cfg.get("publish_tag_tf", False)),
        "minimum_detections": int(reg["minimum_detections"]),
        "timeout_sec": float(reg["timeout_sec"]),
        "tracking_timeout_sec": float(reg["tracking_timeout_sec"]),
        "max_translation_residual_m": float(reg["max_translation_residual_m"]),
        "max_angular_residual_deg": float(reg["max_angular_residual_deg"]),
    }
    nodes = [Node(package="spectacles_localization_ros", executable="registration_node",
                  output="screen", parameters=[params])]
    if mock:
        nodes += [Node(package="spectacles_localization_ros", executable="mock_node",
                       output="screen"),
                  Node(package="spectacles_localization_ros", executable="mock_verify_node",
                       output="screen")]
    else:
        nodes.append(Node(package="spectacles_localization_ros", executable="bridge_node",
                          output="screen", parameters=[{
                              "bind_host": LaunchConfiguration("bind_host").perform(context),
                              "port": int(LaunchConfiguration("port").perform(context)),
                              "token": LaunchConfiguration("token").perform(context),
                              "tag_id": int(tag["id"]),
                              "tag_size_m": float(tag["size_m"]),
                          }]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mock", default_value="false"),
        DeclareLaunchArgument("config", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("bind_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="8765"),
        DeclareLaunchArgument("token", default_value=""),
        OpaqueFunction(function=setup),
    ])
