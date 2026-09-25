"""Launch `ros2 bag record` (mcap) for the canonical G1 survey bag.

Reads the topic list from the installed config/topics.yaml and applies the QoS overrides in
config/qos_override.yaml (critically, transient_local for /tf_static). Output bag defaults to a
timestamped directory; override with `output:=<dir>`. Point at a different topic set with
`topics_file:=<abs path>` (used by the R1 synthetic test).
"""
import os
from datetime import datetime

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _launch_record(context, *args, **kwargs):
    share = get_package_share_directory("g1_recorder")
    topics_file = LaunchConfiguration("topics_file").perform(context) or os.path.join(
        share, "config", "topics.yaml"
    )
    qos_file = LaunchConfiguration("qos_file").perform(context) or os.path.join(
        share, "config", "qos_override.yaml"
    )
    output = LaunchConfiguration("output").perform(context)

    with open(topics_file) as f:
        topics = yaml.safe_load(f).get("topics", [])

    if not topics:
        raise RuntimeError(f"No topics found in {topics_file}")

    cmd = [
        "ros2", "bag", "record",
        "--storage", "mcap",
        "-o", output,
        "--qos-profile-overrides-path", qos_file,
        "--topics", *topics,
    ]
    return [ExecuteProcess(cmd=cmd, output="screen")]


def generate_launch_description():
    default_out = f"g1_survey_{datetime.now():%Y%m%d_%H%M%S}"
    return LaunchDescription([
        DeclareLaunchArgument("output", default_value=default_out,
                              description="Output bag directory"),
        DeclareLaunchArgument("topics_file", default_value="",
                              description="Abs path to a topics.yaml (defaults to installed one)"),
        DeclareLaunchArgument("qos_file", default_value="",
                              description="Abs path to a qos_override.yaml (defaults to installed)"),
        OpaqueFunction(function=_launch_record),
    ])
