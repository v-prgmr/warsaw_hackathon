"""The robot's x-ray skeleton on /ar_glasses/markers, on its own (no bridge, no glasses needed).

Draws the G1 as an articulated stick figure from the URDF link tree + /tf. robot_state_publisher
must be running so /robot_description is latched and /tf carries every link; the g1_sensors
tf_chain does that from /lf/lowstate. Handy for seeing the skeleton in RViz while replaying a bag,
before involving the glasses:

    ros2 launch g1_sensors tf_chain.launch.py use_sim_time:=true rviz:=true
    ros2 launch g1_ar_bridge skeleton.launch.py use_sim_time:=true
    ros2 bag play <bag> --clock 200

By default bones are expressed in ``map`` (where the glasses see everything). With tf_chain alone
(no mapper) there is no ``map``; set ``root_frame:=robot_center`` to view the skeleton on its own.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    params = {"use_sim_time": arg("use_sim_time").lower() in ("true", "1"),
              "root_frame": arg("root_frame")}
    if arg("urdf_path"):
        params["urdf_path"] = arg("urdf_path")
    if arg("rate_hz"):
        params["rate_hz"] = float(arg("rate_hz"))
    return [Node(package="g1_ar_bridge", executable="ar_skeleton", name="ar_skeleton",
                 output="screen", parameters=[params])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("root_frame", default_value="map",
                              description="frame the bones are expressed in (glasses use map; "
                                          "robot_center to view without a mapper)"),
        DeclareLaunchArgument("urdf_path", default_value="",
                              description="URDF file; '' = the latched /robot_description"),
        DeclareLaunchArgument("rate_hz", default_value="",
                              description="publish rate; '' = node default (10 Hz)"),
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="true on replayed bags (ros2 bag play --clock)"),
        OpaqueFunction(function=_launch_setup),
    ])
