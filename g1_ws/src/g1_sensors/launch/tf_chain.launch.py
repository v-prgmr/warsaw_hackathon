"""The G1's /tf chain (AGENTS.md §10.1): /lowstate -> /joint_states -> robot_state_publisher.

  robot_center -> pelvis -> waist joints -> torso_link -> mid360_link -> livox_frame
                                                       -> d435_link   -> camera_link
                        -> imu_in_pelvis -> dog_imu_link

Read-only towards the robot: subscribes to /lowstate and /dog_imu_raw, publishes /joint_states,
/tf, /tf_static and /robot_description. odom -> robot_center comes from g1_mapping (run it with
static_tf:=false). Topic names, joint order, and glue frames live in config/g1_sensors.yaml.

Live (robot-connected container):   ros2 launch g1_sensors tf_chain.launch.py
Replay (SIM=1 container, domain 77): ros2 launch g1_sensors tf_chain.launch.py use_sim_time:=true
                                     ros2 bag play <bag> --clock 200
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    share = get_package_share_directory("g1_sensors")
    with open(LaunchConfiguration("config").perform(context)) as f:
        cfg = yaml.safe_load(f)
    topics, bridge = cfg["topics"], cfg["bridge"]
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in ("true", "1")
    lowstate = LaunchConfiguration("lowstate_topic").perform(context) or topics["lowstate"]
    rate = float(bridge["publish_rate"])

    with open(LaunchConfiguration("urdf").perform(context)) as f:
        urdf = f.read()
    mesh_dir = LaunchConfiguration("mesh_dir").perform(context)
    if mesh_dir and os.path.isdir(mesh_dir):
        # URDF meshes are relative ("meshes/x.STL"); RViz's RobotModel needs absolute paths.
        urdf = urdf.replace('filename="meshes/', f'filename="file://{mesh_dir}/')

    nodes = [
        Node(package="g1_sensors", executable="lowstate_to_joint_states", output="screen",
             parameters=[{"use_sim_time": use_sim_time, "joint_names": cfg["joint_names"],
                          "publish_rate": rate, "stamp_source": bridge["stamp_source"],
                          "clock_window": float(bridge["clock_window"])}],
             remappings=[("lowstate", lowstate),
                         ("clock_reference", topics["clock_reference"]),
                         ("joint_states", topics["joint_states"])]),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="screen",
             parameters=[{"use_sim_time": use_sim_time, "robot_description": urdf,
                          "publish_frequency": rate}],
             remappings=[("joint_states", topics["joint_states"])]),
    ]
    if LaunchConfiguration("publish_battery_state").perform(context).lower() in ("true", "1"):
        nodes.append(Node(
            package="g1_sensors", executable="bms_to_battery_state", output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=[("bms", topics["bms"]),
                        ("battery_state", topics["battery_state"])],
        ))
    for tf in cfg.get("static_transforms", []):
        (x, y, z), (roll, pitch, yaw) = tf["xyz"], tf["rpy"]
        nodes.append(Node(
            package="tf2_ros", executable="static_transform_publisher",
            name=f"static_tf_{tf['child'].replace('-', '_')}", output="log",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=["--x", str(x), "--y", str(y), "--z", str(z),
                       "--roll", str(roll), "--pitch", str(pitch), "--yaw", str(yaw),
                       "--frame-id", tf["parent"], "--child-frame-id", tf["child"]]))
    if LaunchConfiguration("rviz").perform(context).lower() in ("true", "1"):
        nodes.append(Node(package="rviz2", executable="rviz2", output="log",
                          parameters=[{"use_sim_time": use_sim_time}],
                          arguments=["-d", os.path.join(share, "rviz", "tf_chain.rviz")]))
    return nodes


def generate_launch_description():
    share = get_package_share_directory("g1_sensors")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=os.path.join(share, "config",
                                                                   "g1_sensors.yaml")),
        DeclareLaunchArgument("urdf", default_value=os.path.join(share, "urdf",
                                                                 "g1_29dof_rev_1_0.urdf")),
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="true when replaying a bag with --clock."),
        DeclareLaunchArgument("lowstate_topic", default_value="",
                              description="Override the YAML's lowstate topic, e.g. "
                                          "/lowstate (1 kHz, more CPU)."),
        DeclareLaunchArgument("mesh_dir",
                              default_value="/ws/third_party/unitree_ros/robots/g1_description/"
                                            "meshes",
                              description="G1 meshes for RViz's RobotModel (optional)."),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("publish_battery_state", default_value="true",
                              description="Bridge /lf/bmsstate SOC into /battery_state."),
        OpaqueFunction(function=_launch_setup),
    ])
