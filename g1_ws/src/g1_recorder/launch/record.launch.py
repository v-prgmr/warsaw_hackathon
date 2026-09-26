"""Launch `ros2 bag record` (mcap) for the canonical G1 survey bag.

Reads the topic list from config/<profile>.yaml and applies the QoS overrides in
config/qos_override.yaml (critically, transient_local for /tf_static). Output bag defaults to a
timestamped directory; override with `output:=<dir>`. Point at a different topic set with
`topics_file:=<abs path>` (used by the R1 synthetic test and custom captures).
"""
import os
from datetime import datetime

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

PROFILES = ("rtab", "survey", "live_run", "full_survey")


def _launch_record(context, *args, **kwargs):
    share = get_package_share_directory("g1_recorder")
    profile = LaunchConfiguration("profile").perform(context)
    topics_file = LaunchConfiguration("topics_file").perform(context)
    if not topics_file:
        if profile not in PROFILES:
            raise RuntimeError(f"Unknown profile {profile!r}; choose one of {', '.join(PROFILES)}")
        topics_file = os.path.join(share, "config", f"{profile}.yaml")
    if not os.path.isfile(topics_file):
        raise RuntimeError(
            f"Topic profile not found: {topics_file}. "
            "Use a named profile or topics_file:=<absolute path>."
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
    ]
    if profile == "live_run" and not LaunchConfiguration("topics_file").perform(context):
        cmd.append("--include-hidden-topics")
    cmd.extend(topics)
    return [ExecuteProcess(cmd=cmd, output="screen")]


def generate_launch_description():
    default_out = f"g1_survey_{datetime.now():%Y%m%d_%H%M%S}"
    return LaunchDescription([
        DeclareLaunchArgument("output", default_value=default_out,
                              description="Output bag directory"),
        DeclareLaunchArgument("profile", default_value="survey",
                              description="Topic profile: survey (canonical), live_run, "
                                          "rtab (camera only), legacy full_survey"),
        DeclareLaunchArgument("topics_file", default_value="",
                              description="Absolute topic YAML path; overrides profile"),
        DeclareLaunchArgument("qos_file", default_value="",
                              description="Abs path to a qos_override.yaml "
                                          "(defaults to installed)"),
        OpaqueFunction(function=_launch_record),
    ])
