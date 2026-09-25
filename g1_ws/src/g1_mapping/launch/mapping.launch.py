"""RTAB-Map LiDAR-inertial mapping for the G1 (AGENTS.md §8).

Pipeline (odom_source:=icp, default):
  MID-360 cloud -> livox_cloud_fix (time ns->s, drop zeros) -> lidar_deskewing (IMU-stabilized
  frame from imu_to_tf) -> icp_odometry (odom -> base) -> rtabmap (map -> odom, /map, 3D map)
odom_source:=dog_odom replaces icp_odometry with /dog_odom -> TF (odom_to_tf); deskewing then
uses the odom frame.
use_rgbd:=true attaches RealSense RGB-D to map nodes (color only; the 2D grid stays LiDAR-only).

Topic / frame names and tuning come from config/g1_mapping.yaml.

Replay a bag:
  ros2 launch g1_mapping mapping.launch.py use_sim_time:=true
  ros2 bag play <bag> --clock
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1", "yes")


def _launch_setup(context):
    with open(LaunchConfiguration("config").perform(context)) as f:
        cfg = yaml.safe_load(f)
    topics, frames, lidar = cfg["topics"], cfg["frames"], cfg["lidar"]

    odom_source = LaunchConfiguration("odom_source").perform(context)
    if odom_source not in ("icp", "dog_odom"):
        raise RuntimeError(f"odom_source must be 'icp' or 'dog_odom', got '{odom_source}'")
    use_sim_time = _bool(context, "use_sim_time")
    use_imu = _bool(context, "use_imu")
    use_rgbd = _bool(context, "use_rgbd")
    deskewing = _bool(context, "deskewing")
    localization = _bool(context, "localization")

    base = frames["base"]
    voxel = float(lidar["voxel_size"])
    icp_params = dict(cfg.get("icp_parameters", {}))
    icp_params["Icp/VoxelSize"] = str(voxel)
    icp_params["Icp/MaxCorrespondenceDistance"] = str(voxel * 10.0)
    common = {"use_sim_time": use_sim_time, "frame_id": base, "wait_for_transform": 0.2}

    # Fixed frame for deskewing: IMU-stabilized base (icp) or the odometry frame (dog_odom).
    if odom_source == "dog_odom":
        fixed_frame = frames["odom"]
    elif use_imu:
        fixed_frame = base + "_stabilized"
    else:
        fixed_frame = ""  # icp_odometry deskews internally from its own motion estimate
    scan_topic = topics["cloud_fixed"]
    if deskewing and fixed_frame:
        scan_topic = topics["cloud_fixed"] + "/deskewed"

    nodes = []

    if _bool(context, "static_tf"):
        for tf in cfg.get("static_transforms", []):
            (x, y, z), (roll, pitch, yaw) = tf["xyz"], tf["rpy"]
            nodes.append(Node(
                package="tf2_ros", executable="static_transform_publisher",
                name=f"static_tf_{tf['child']}", output="log",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=["--x", str(x), "--y", str(y), "--z", str(z),
                           "--roll", str(roll), "--pitch", str(pitch), "--yaw", str(yaw),
                           "--frame-id", tf["parent"], "--child-frame-id", tf["child"]]))

    nodes.append(Node(
        package="g1_mapping", executable="livox_cloud_fix", output="screen",
        parameters=[{"use_sim_time": use_sim_time,
                     "time_field": lidar["time_field"],
                     "time_scale": float(lidar["time_scale"]),
                     "range_min": float(lidar["range_min"]),
                     "range_max": float(lidar["range_max"])}],
        remappings=[("input", topics["lidar"]), ("output", topics["cloud_fixed"])]))

    if odom_source == "icp":
        if use_imu and deskewing:
            nodes.append(Node(
                package="rtabmap_util", executable="imu_to_tf", output="screen",
                parameters=[{"use_sim_time": use_sim_time, "fixed_frame_id": fixed_frame,
                             "base_frame_id": base, "wait_for_transform_duration": 0.001}],
                remappings=[("imu/data", topics["imu"])]))
        odom_params = {
            "odom_frame_id": frames["odom"],
            "publish_tf": True,
            "expected_update_rate": float(lidar["expected_update_rate"]),
            "deskewing": deskewing and not fixed_frame,
            "deskewing_slerp": bool(cfg["deskewing"]["slerp"]),
            "guess_frame_id": fixed_frame if use_imu else "",
            "wait_imu_to_init": use_imu,
            "OdomF2M/ScanSubtractRadius": str(voxel),
        }
        odom_params.update(cfg.get("odometry_parameters", {}))
        nodes.append(Node(
            package="rtabmap_odom", executable="icp_odometry", output="screen",
            parameters=[common, icp_params, odom_params],
            remappings=[("scan_cloud", scan_topic), ("odom", topics["odom"]),
                        ("imu", topics["imu"] if use_imu else "imu_not_used")]))
    else:
        nodes.append(Node(
            package="g1_mapping", executable="odom_to_tf", output="screen",
            parameters=[{"use_sim_time": use_sim_time,
                         "republish": topics["odom"] != topics["dog_odom"]}],
            remappings=[("odom_in", topics["dog_odom"]), ("odom_out", topics["odom"])]))

    if deskewing and fixed_frame:
        nodes.append(Node(
            package="rtabmap_util", executable="lidar_deskewing", output="screen",
            parameters=[{"use_sim_time": use_sim_time, "fixed_frame_id": fixed_frame,
                         "wait_for_transform": 0.2, "slerp": bool(cfg["deskewing"]["slerp"])}],
            remappings=[("input_cloud", topics["cloud_fixed"])]))

    if use_rgbd:
        nodes.append(Node(
            package="rtabmap_sync", executable="rgbd_sync", output="screen",
            parameters=[{"use_sim_time": use_sim_time, "approx_sync": False}],
            remappings=[("rgb/image", topics["rgb"]), ("depth/image", topics["depth"]),
                        ("rgb/camera_info", topics["camera_info"]),
                        ("rgbd_image", topics["rgbd_image"])]))

    slam_params = {
        "subscribe_depth": False,
        "subscribe_rgb": False,
        "subscribe_rgbd": use_rgbd,
        "subscribe_scan_cloud": True,
        "map_frame_id": frames["map"],
        "database_path": os.path.expanduser(LaunchConfiguration("database_path").perform(context)),
        "approx_sync": use_rgbd,  # LiDAR and camera stamps differ; LiDAR + odom are exact
        "Icp/CorrespondenceRatio": str(lidar["min_loop_closure_overlap"]),
    }
    if odom_source == "icp":
        # odometry from icp_odometry's topic (same stamp as the scan -> exact sync)
        slam_params["subscribe_odom_info"] = True
        slam_params["odom_sensor_sync"] = use_rgbd
    else:
        # odometry from TF (odom -> base, bridged from /dog_odom)
        slam_params["odom_frame_id"] = frames["odom"]
        slam_params["subscribe_odom_info"] = False
    slam_params.update(cfg.get("rtabmap_parameters", {}))
    if localization:
        slam_params["Mem/IncrementalMemory"] = "false"
        slam_params["Mem/InitWMWithAllNodes"] = "true"
    slam_args = ["-d"] if (not localization and _bool(context, "delete_db")) else []
    slam_remaps = [("scan_cloud", scan_topic), ("odom", topics["odom"]),
                   ("rgbd_image", topics["rgbd_image"])]
    if use_imu:
        # IMU gravity constrains the pose graph (keeps the map level), as in lidar3d.launch.py
        slam_remaps.append(("imu", topics["imu"]))
    nodes.append(Node(
        package="rtabmap_slam", executable="rtabmap", output="screen",
        parameters=[common, icp_params, slam_params], remappings=slam_remaps,
        arguments=slam_args))

    if _bool(context, "rtabmap_viz"):
        nodes.append(Node(
            package="rtabmap_viz", executable="rtabmap_viz", output="screen",
            parameters=[common, slam_params,
                        {"odometry_node_name": "icp_odometry" if odom_source == "icp" else ""}],
            remappings=slam_remaps))
    if _bool(context, "rviz"):
        nodes.append(Node(
            package="rviz2", executable="rviz2", output="log",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=["-d", os.path.join(get_package_share_directory("g1_mapping"),
                                          "rviz", "mapping.rviz")]))
    return nodes


def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory("g1_mapping"), "config", "g1_mapping.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_cfg,
                              description="Topic/frame/tuning YAML."),
        DeclareLaunchArgument("odom_source", default_value="icp",
                              description="icp (LiDAR ICP odometry) or dog_odom "
                                          "(/dog_odom -> TF)."),
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="true when replaying a bag with --clock."),
        DeclareLaunchArgument("use_imu", default_value="true",
                              description="Use the IMU for the ICP motion guess and deskewing."),
        DeclareLaunchArgument("deskewing", default_value="true",
                              description="Deskew the LiDAR cloud with per-point time."),
        DeclareLaunchArgument("use_rgbd", default_value="false",
                              description="Attach RealSense RGB-D to map nodes (color)."),
        DeclareLaunchArgument("static_tf", default_value="true",
                              description="Publish the ESTIMATED fallback extrinsics from the "
                                          "YAML. Set false when /tf comes from the G1 URDF."),
        DeclareLaunchArgument("database_path", default_value="~/.ros/g1_rtabmap.db"),
        DeclareLaunchArgument("delete_db", default_value="true",
                              description="Start a new map (ignored in localization mode)."),
        DeclareLaunchArgument("localization", default_value="false"),
        DeclareLaunchArgument("rtabmap_viz", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=_launch_setup),
    ])
