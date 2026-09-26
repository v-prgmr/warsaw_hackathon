"""Calibration capture (read-only): calib_capture node + optional image viewers.

  ros2 launch g1_calibration capture.launch.py output_dir:=calib_$(date +%F)
  ros2 run g1_calibration calib_trigger            # second terminal: Enter = one sample

Needs, while capturing: the OAK-D driver, the RealSense driver, the LiDAR (robot), and
`ros2 launch g1_sensors tf_chain.launch.py` (URDF TF: LiDAR <- RealSense).
"""
import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("g1_calibration")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=os.path.join(share, "config",
                                                                   "calibration.yaml")),
        DeclareLaunchArgument("output_dir",
                              default_value=f"calib_{datetime.now():%Y%m%d_%H%M%S}",
                              description="Dataset directory (samples are appended)."),
        DeclareLaunchArgument("lidar_seconds", default_value="3.0",
                              description="LiDAR accumulation per sample (s)."),
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="true when capturing from a replayed bag."),
        DeclareLaunchArgument("viewer", default_value="true",
                              description="rqt_image_view on the board-detection images."),
        Node(package="g1_calibration", executable="calib_capture", name="calib_capture",
             output="screen",
             parameters=[{"config": LaunchConfiguration("config"),
                          "output_dir": LaunchConfiguration("output_dir"),
                          "lidar_seconds": LaunchConfiguration("lidar_seconds"),
                          "use_sim_time": LaunchConfiguration("use_sim_time")}]),
        Node(package="rqt_image_view", executable="rqt_image_view", output="log",
             arguments=["/calib_capture/debug/oak"],
             condition=IfCondition(LaunchConfiguration("viewer"))),
    ])
