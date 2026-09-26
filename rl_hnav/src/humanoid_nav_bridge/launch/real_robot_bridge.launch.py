"""
real_robot_bridge.launch.py

==============================================================
REAL ROBOT NAVIGATION BRIDGE (Production Version)
==============================================================

This launch file provides the minimal and clean bridge layer
between:

    • Real robot low-level drivers
    • SLAM Toolbox
    • Nav2 navigation stack

It performs 4 essential tasks:

1) /dog_odom  --->  /odom  + TF(odom -> base_frame)
   Standardizes odometry for Nav2 compatibility.

2) Static TF  base_frame -> lidar_frame
   Completes TF tree required by costmaps and SLAM.

3) PointCloud2 ---> LaserScan (/scan_raw)
   Converts 3D LiDAR to 2D scan for SLAM/Nav2.

4) Restamp /scan_raw ---> /scan
   Prevents TF message_filter timestamp errors
   caused by driver time inconsistencies.

--------------------------------------------------------------
Validated defaults (from real robot probes):

    /dog_odom.header.frame_id            = "odom"
    /dog_odom.child_frame_id             = "robot_center"
    /utlidar/cloud_livox_mid360.frame_id = "livox_frame"

So default TF chain becomes:

    map
      ↓
    odom
      ↓
    robot_center
      ↓
    livox_frame

--------------------------------------------------------------
Next to the team's g1_sensors + g1_mapping stack (AGENTS.md §6):

    publish_odom_tf:=false      g1_mapping owns odom -> robot_center (+ /odom)
    publish_lidar_tf:=false     g1_sensors owns livox_frame (G1 URDF chain)
    override_scan_stamp:=false  their /tf is stamped on the ROBOT clock; restamping
                                /scan with this PC's clock breaks the TF lookup

Their TF is on the robot clock, while Nav2 checks transforms against this PC's clock:
sync this PC's clock to the robot first (it was ~76 s ahead on 2026-09-26).
--------------------------------------------------------------
IMPORTANT:
This launch DOES NOT start debug/probe nodes.
It is strictly for production runtime.
--------------------------------------------------------------
"""

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ==========================================================
    # 1️⃣ Clock configuration
    # ==========================================================
    # Real robot does NOT use /clock unless explicitly provided.
    # Must remain false to avoid TF time drift.
    use_sim_time = LaunchConfiguration("use_sim_time")


    # ==========================================================
    # 2️⃣ Input topics from hardware
    # ==========================================================
    # Raw odometry published by robot driver
    input_odom_topic = LaunchConfiguration("input_odom_topic")

    # Raw 3D LiDAR cloud
    cloud_topic = LaunchConfiguration("cloud_topic")


    # ==========================================================
    # 3️⃣ TF Frame configuration
    # ==========================================================
    # These MUST match real robot frames
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    lidar_frame = LaunchConfiguration("lidar_frame")


    # ==========================================================
    # 4️⃣ Static transform parameters (extrinsics)
    # ==========================================================
    # Used to define LiDAR pose relative to base frame.
    # Default = G1 29-DoF rev 1.0 URDF chain, robot standing (g1_sensors, checked live):
    # the MID-360 is mounted UPSIDE DOWN, 0.47 m above robot_center (the pelvis).
    # The old all-zero default mirrored /scan left-right.
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")


    # ==========================================================
    # 5️⃣ Cloud -> Scan conversion parameters
    # ==========================================================
    min_height = LaunchConfiguration("min_height")
    max_height = LaunchConfiguration("max_height")
    range_min = LaunchConfiguration("range_min")
    range_max = LaunchConfiguration("range_max")
    transform_tolerance = LaunchConfiguration("transform_tolerance")

    # Switches to run next to g1_sensors + g1_mapping (see the docstring)
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    publish_lidar_tf = LaunchConfiguration("publish_lidar_tf")
    override_scan_stamp = LaunchConfiguration("override_scan_stamp")


    # ==========================================================
    # (1) ODOM BRIDGE NODE
    # ==========================================================
    # Converts robot-native odometry into Nav2-compatible format.
    #
    # Responsibilities:
    #   - Subscribes to /dog_odom
    #   - Republishes standardized /odom
    #   - Publishes TF: odom -> base_frame
    #
    # Without this node:
    #   Nav2 cannot compute local costmap transforms.
    #
    odom_tf_bridge = Node(
        package="humanoid_nav_bridge",
        executable="odom_tf_bridge",
        name="real_odom_tf_bridge",
        output="screen",
        condition=IfCondition(publish_odom_tf),
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_odom_topic": input_odom_topic,
            "output_odom_topic": "/odom",
            "odom_frame": odom_frame,
            "base_frame": base_frame,
            "zero_at_start": True,
        }],
    )


    # ==========================================================
    # (2) STATIC TF: base_frame -> lidar_frame
    # ==========================================================
    # Required so that:
    #   TF tree is complete
    #   LaserScan can be transformed correctly
    #
    # Uses ROS2 built-in static_transform_publisher
    #
    # Named arguments: the positional form is "x y z YAW PITCH ROLL", so passing
    # "x y z roll pitch yaw" positionally put the roll into the yaw.
    static_tf_base_to_lidar = ExecuteProcess(
        cmd=[
            "ros2", "run", "tf2_ros", "static_transform_publisher",
            "--x", x, "--y", y, "--z", z,                        # translation
            "--roll", roll, "--pitch", pitch, "--yaw", yaw,      # rotation (radians)
            "--frame-id", base_frame, "--child-frame-id", lidar_frame,
        ],
        output="screen",
        condition=IfCondition(publish_lidar_tf),
    )


    # ==========================================================
    # (3) POINTCLOUD2 -> LASERSCAN
    # ==========================================================
    # Converts 3D cloud into 2D planar scan for:
    #   - slam_toolbox
    #   - Nav2 costmaps
    #
    # Output is /scan_raw (NOT /scan yet).
    #
    cloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan_node",
        output="screen",
        remappings=[
            ("cloud_in", cloud_topic),
            ("scan", "/scan_raw"),   # intermediate topic
        ],
        parameters=[{
            "target_frame": base_frame,
            "transform_tolerance": transform_tolerance,
            "min_height": min_height,
            "max_height": max_height,
            "range_min": range_min,
            "range_max": range_max,
        }],
    )


    # ==========================================================
    # (4) SCAN RESTAMPER
    # ==========================================================
    # Fixes timestamp inconsistencies between:
    #   - LiDAR driver
    #   - TF buffer
    #
    # Without this, you get:
    #   "Message Filter dropping message"
    #
    # Strategy:
    #   Override header.stamp with current node time.
    #
    scan_restamper = Node(
        package="humanoid_nav_bridge",
        executable="scan_restamper",
        name="scan_restamper",
        output="screen",
        parameters=[{
            "input_scan_topic": "/scan_raw",
            "output_scan_topic": "/scan",
            "override_stamp": ParameterValue(override_scan_stamp, value_type=bool),
        }],
    )


    # ==========================================================
    # Launch arguments declaration
    # ==========================================================
    declared_args = [

        # Clock
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Real robot: keep false (no /clock topic).",
        ),

        # Input topics
        DeclareLaunchArgument("input_odom_topic", default_value="/dog_odom"),
        DeclareLaunchArgument("cloud_topic", default_value="/utlidar/cloud_livox_mid360"),

        # Frames
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="robot_center"),
        DeclareLaunchArgument("lidar_frame", default_value="livox_frame"),

        # Static TF: G1 URDF chain, robot standing (MID-360 upside down)
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.472"),
        DeclareLaunchArgument("roll", default_value="3.14159265"),
        DeclareLaunchArgument("pitch", default_value="0.065"),
        DeclareLaunchArgument("yaw", default_value="0.0"),

        # Switches for running next to g1_sensors + g1_mapping
        DeclareLaunchArgument("publish_odom_tf", default_value="true",
                              description="odom -> base_frame + /odom from /dog_odom. "
                                          "false when g1_mapping runs."),
        DeclareLaunchArgument("publish_lidar_tf", default_value="true",
                              description="static base_frame -> lidar_frame. "
                                          "false when g1_sensors runs."),
        DeclareLaunchArgument("override_scan_stamp", default_value="true",
                              description="restamp /scan with this PC's clock. "
                                          "false when /tf is on the robot clock."),

        # Cloud->scan tuning
        DeclareLaunchArgument("min_height", default_value="-0.2"),
        DeclareLaunchArgument("max_height", default_value="0.2"),
        DeclareLaunchArgument("range_min", default_value="0.2"),
        DeclareLaunchArgument("range_max", default_value="30.0"),
        DeclareLaunchArgument("transform_tolerance", default_value="0.1"),
    ]


    # ==========================================================
    # STARTUP ORDER (critical for stability)
    # ==========================================================
    #
    # 0.5s  → odom bridge (TF must exist first)
    # 1.0s  → static TF
    # 2.0s  → cloud -> scan conversion
    # 2.5s  → restamp (after scan_raw exists)
    #
    actions = [
        TimerAction(period=0.5, actions=[odom_tf_bridge]),
        TimerAction(period=1.0, actions=[static_tf_base_to_lidar]),
        TimerAction(period=2.0, actions=[cloud_to_scan]),
        TimerAction(period=2.5, actions=[scan_restamper]),
    ]

    return LaunchDescription(declared_args + actions)
