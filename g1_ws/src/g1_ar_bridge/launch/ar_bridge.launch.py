"""Spectacles AR bridge + wall-tag anchor (AGENTS.md §27). Read-only towards the robot.

Needs our stack for TF (g1_sensors tf_chain + g1_mapping) and a robot camera that sees the wall
tag. On the robot laptop (robot-connected container):

    ros2 launch g1_ar_bridge ar_bridge.launch.py tag_black_size_m:=0.16
    # chest OAK-D instead of the head RealSense (topics to verify on the robot first):
    ros2 launch g1_ar_bridge ar_bridge.launch.py image_topic:=<rgb> camera_info_topic:=<info> \
        depth_topic:=<depth aligned to rgb>

Then on the glasses: Dimensional OS -> this laptop's Wi-Fi IP -> Registration: AprilTag.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    use_sim_time = arg("use_sim_time").lower() in ("true", "1")
    common = {"use_sim_time": use_sim_time}
    if arg("tag_id"):
        common["tag_id"] = int(arg("tag_id"))
    if arg("tag_black_size_m"):
        common["tag_black_size_m"] = float(arg("tag_black_size_m"))
    bridge = dict(common)
    if arg("port"):
        bridge["port"] = int(arg("port"))
    anchor = dict(common)
    for name in ("image_topic", "camera_info_topic", "depth_topic", "anchor_file"):
        if arg(name):
            anchor[name] = arg(name)
    if arg("use_depth"):
        anchor["use_depth"] = arg("use_depth").lower() in ("true", "1")
    nodes = [Node(package="g1_ar_bridge", executable="ar_bridge", name="ar_bridge",
                  output="screen", parameters=[arg("config"), bridge])]
    if arg("anchor").lower() in ("true", "1"):
        nodes.append(Node(package="g1_ar_bridge", executable="tag_anchor", name="ar_tag_anchor",
                          output="screen", parameters=[arg("config"), anchor]))
    return nodes


def generate_launch_description():
    share = get_package_share_directory("g1_ar_bridge")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=f"{share}/config/ar_bridge.yaml",
                              description="ROS params file for both nodes"),
        DeclareLaunchArgument("anchor", default_value="true",
                              description="also run tag_anchor (robot camera: map -> ar_tag_N)"),
        DeclareLaunchArgument("tag_id", default_value="",
                              description="AprilTag 36h11 id (both nodes); '' = config"),
        DeclareLaunchArgument("tag_black_size_m", default_value="",
                              description="edge of the tag's black square in m; '' = config"),
        DeclareLaunchArgument("image_topic", default_value="",
                              description="robot camera RGB image; '' = config (RealSense)"),
        DeclareLaunchArgument("camera_info_topic", default_value="",
                              description="its CameraInfo; '' = config"),
        DeclareLaunchArgument("depth_topic", default_value="",
                              description="depth aligned to that image; '' = config"),
        DeclareLaunchArgument("use_depth", default_value="",
                              description="fit the tag plane in depth; '' = config"),
        DeclareLaunchArgument("anchor_file", default_value="",
                              description="YAML to save the anchor to; '' = config"),
        DeclareLaunchArgument("port", default_value="",
                              description="WebSocket port; the Lens always uses 8787"),
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="true on replayed bags (ros2 bag play --clock)"),
        OpaqueFunction(function=_launch_setup),
    ])
