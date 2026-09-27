"""Spectacles AR bridge + wall-tag anchor (AGENTS.md §27). Read-only towards the robot.

Needs our stack for TF (g1_sensors tf_chain + g1_mapping) and a robot camera that sees the wall
tag. On the robot laptop (robot-connected container):

    ros2 launch g1_ar_bridge ar_bridge.launch.py tag_black_size_m:=0.16               # RealSense
    ros2 launch g1_ar_bridge ar_bridge.launch.py tag_black_size_m:=0.16 camera:=oak   # chest OAK-D
    # any other camera: image_topic:=... camera_info_topic:=... depth_topic:=...

Then on the glasses: Dimensional OS -> this laptop's Wi-Fi IP -> Registration: AprilTag.

Home test of this ROS path without the robot (real glasses, tag on your wall):

    ros2 launch g1_ar_bridge ar_bridge.launch.py tag_black_size_m:=0.16 fake_robot:=true

publishes a fake robot at the map origin and the robot anchor map -> ar_tag_0 as a wall tag
``fake_tag_distance`` m in front of it, instead of running tag_anchor.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from g1_ar_bridge.geometry import rot_to_quat
from g1_ar_bridge.world import wall_tag_pose

# depthai-ros topics expected per OAKD_REALSENSE_TF.md (RGB-aligned depth); check them live with
# `ros2 topic list | grep /oak/` before relying on them. Explicit *_topic arguments win.
CAMERA_PRESETS = {
    "realsense": {},                                   # = config/ar_bridge.yaml
    "oak": {"image_topic": "/oak/rgb/image_raw", "camera_info_topic": "/oak/rgb/camera_info",
            "depth_topic": "/oak/stereo/image_raw"},
}


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
    if arg("record_dir"):
        bridge["record_dir"] = arg("record_dir")
    anchor = dict(common)
    camera = arg("camera").lower()
    if camera not in CAMERA_PRESETS:
        raise RuntimeError(f"camera:={camera}: use one of {sorted(CAMERA_PRESETS)}")
    anchor.update(CAMERA_PRESETS[camera])
    for name in ("image_topic", "camera_info_topic", "depth_topic", "anchor_file"):
        if arg(name):
            anchor[name] = arg(name)
    if arg("use_depth"):
        anchor["use_depth"] = arg("use_depth").lower() in ("true", "1")
    nodes = [Node(package="g1_ar_bridge", executable="ar_bridge", name="ar_bridge",
                  output="screen", parameters=[arg("config"), bridge])]
    if arg("fake_robot").lower() in ("true", "1"):
        # no robot: robot_center at the map origin (0.78 m above the floor), the wall tag
        # fake_tag_distance m ahead at fake_tag_height m above the floor, facing the robot
        base = 0.78
        T = wall_tag_pose(float(arg("fake_tag_distance")), float(arg("fake_tag_height")), -base)
        q = rot_to_quat(T[:3, :3])
        xyz = [f"{v:.4f}" for v in T[:3, 3]]
        quat = [f"{v:.6f}" for v in q]
        nodes += [
            Node(package="tf2_ros", executable="static_transform_publisher",
                 name="fake_robot_center",
                 arguments=["--frame-id", "map", "--child-frame-id", "robot_center"]),
            Node(package="tf2_ros", executable="static_transform_publisher",
                 name="fake_tag_anchor",
                 arguments=["--x", xyz[0], "--y", xyz[1], "--z", xyz[2], "--qx", quat[0],
                            "--qy", quat[1], "--qz", quat[2], "--qw", quat[3],
                            "--frame-id", "map", "--child-frame-id",
                            f"ar_tag_{arg('tag_id') or 0}"])]
        return nodes
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
        DeclareLaunchArgument("camera", default_value="realsense",
                              description="robot camera preset for tag_anchor: realsense | oak"),
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
        DeclareLaunchArgument("record_dir", default_value="",
                              description="where glasses registrations are recorded; '' = "
                                          "config (/ws/bags/ar_registration)"),
        DeclareLaunchArgument("fake_robot", default_value="false",
                              description="home test without the robot: fake robot pose + "
                                          "fake wall-tag anchor instead of tag_anchor"),
        DeclareLaunchArgument("fake_tag_distance", default_value="1.5",
                              description="fake_robot: wall tag this far in front of the robot"),
        DeclareLaunchArgument("fake_tag_height", default_value="1.0",
                              description="fake_robot: tag centre height above the floor"),
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="true on replayed bags (ros2 bag play --clock)"),
        OpaqueFunction(function=_launch_setup),
    ])
