import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
    LogInfo,
    GroupAction,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnShutdown, OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetParameter, SetRemap


from ament_index_python.packages import get_package_share_directory
from g1_nav2.sim_environment import ros_library_path


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    world_name = LaunchConfiguration("world_name")
    map_name = LaunchConfiguration("map_name")

    # Chemin cible de sauvegarde : ~/g1_ws/maps/<map_name>.yaml/.pgm
    maps_dir = os.path.join(os.path.expanduser("~"), "g1_ws", "maps")
    map_prefix = os.path.join(maps_dir, "")

    # ✅ Chemins ABSOLUS vers les YAML (évite les surprises avec substitutions)
    g1_nav2_share = get_package_share_directory("g1_nav2")
    nav2_params_path = os.path.join(g1_nav2_share, "params", "nav2_g1_slam.yaml")
    slam_params_path = os.path.join(g1_nav2_share, "params", "slam_toolbox_nav2_async.yaml")

    # Spawn robot
    g1_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("g1_gazebo"),
                "launch",
                "spawn_g1.launch.py",
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time, "world_name": world_name}.items(),
    )

    # SLAM toolbox online async
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params_path,
            "scan_queue_size": "250",
            "transform_timeout": "0.5",
        }.items(),
    )

    # SLAM Toolbox owns /map; its TF output is off in this launch. Keep a
    # single map->odom publisher whose timestamp stays current during turns.
    map_odom_tf = Node(
        package="g1_nav2",
        executable="map_odom_tf",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Keep Nav2's internal TF buffers off the high-volume full G1 joint TF
    # stream. The relay copies original stamps at a bounded rate; other nodes
    # (SLAM, RViz) continue to use the original /tf.
    navigation_tf_relay = Node(
        package="g1_nav2",
        executable="navigation_tf_relay",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # -----------------------------
    # rl_sar param_node (OBLIGATOIRE)
    # -----------------------------
    param_node = Node(
        package="rl_sar",
        executable="param_node",
        name="param_node",
        output="screen",
        parameters=[{
            "robot_name": "g1",         # ou le nom attendu par rl_sar (important)
            "gazebo_model_name": "g1",  # DOIT matcher le -model g1 du spawn
        }],
    )

    # -----------------------------
    # rl_sar locomotion (MuJoCo)
    # -----------------------------
    rl_sar_node = Node(
        package="rl_sar",
        executable="rl_sim_mujoco",    # si l'exécutable existe côté install
        name="rl_sim_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        additional_env={
            "RL_SAR_NAV_MODE": "1",   # force navigation_mode = true
        },
    )

    rl_mujoco = ExecuteProcess(
        cmd=[
            "ros2", "run", "rl_sar", "rl_mujoco", "g1", "scene_29dof",
            "--ros-args",
            "-p", "navigation_mode:=true",
            "-p", "cmd_vel_timeout_sec:=0.6",
            "-p", "cmd_vel_topic:=/cmd_vel",
        ],
        output="screen",
    )

    # -----------------------------
    # MuJoCo → Nav2 bridge
    # -----------------------------
    '''
        mujoco_bridge = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("humanoid_nav_bridge"),
                    "launch",
                    "mujoco_to_nav2_bridge.launch.py",
                )
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "mujoco_pose_topic": "/mujoco/base_pose",
                "odom_topic": "/mujoco/odom",
            }.items(),
        )
    '''
    mujoco_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("humanoid_nav_bridge"),
                "launch",
                "mujoco_to_gazebo_bridge.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "mujoco_pose_topic": "/mujoco/base_pose",
            "odom_topic": "/odom",   
            "publish_map_odom_identity": "false",
        }.items(),
    )

    # Nav2 bringup (navigation) - PAS de namespace pour l’instant (debug clair)
    nav2_bringup_share = get_package_share_directory("nav2_bringup")
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_path,
            # ✅ force no namespace (puisque ton YAML n’est pas sous nav2:)
            "namespace": "",
            "use_namespace": "false",
        }.items()
    )
    # The Nav2 RViz include shuts down its parent launch when RViz exits.
    # Run a lightweight viewer independently so a GUI freeze cannot stop SLAM/Nav2.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", os.path.join(g1_nav2_share, "rviz", "g1_nav_minimal.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    # ✅ Log clair des fichiers réellement passés
    log_params = LogInfo(msg=[
        "NAV2 params_file = ", nav2_params_path,
        " | SLAM params_file = ", slam_params_path
    ])

    # Sauvegarde auto de map au shutdown
    save_map_on_shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=[
                        "bash",
                        "-lc",
                        f"mkdir -p {maps_dir} && "
                        f"ros2 run nav2_map_server map_saver_cli -f {map_prefix}{map_name}",
                    ],
                    output="screen",
                )
            ]
        )
    )

    # ================================================================
    # STARTUP TIMING
    # ================================================================
    #
    # Timeline:
    #   t=0s   → gzserver + gzclient + RViz
    #   t=2s   → RSP + spawn (inside spawn_g1.launch.py)
    #   t=2s   → bridge (use_sim_time=false, publishes TF odom→base_footprint + /odom + /joint_states)
    #   t=10s  → SLAM toolbox (needs: /scan, TF odom→base_footprint, TF base_footprint→mid360_link)
    #   SLAM pose → map_odom_tf (the only map→odom publisher)
    #   t=16s  → Nav2 (needs: /map, TF map→odom→base_footprint)
    #
    # Note: bridge must run with use_sim_time=False because it needs to
    # publish TF with wall-clock timestamps before /clock is available.
    # All other nodes (SLAM, Nav2, RSP) run with use_sim_time=True.
    # ================================================================

    # ================================================================
    # FINAL LAUNCH DESCRIPTION
    # ================================================================
    return LaunchDescription([
        SetEnvironmentVariable('LD_LIBRARY_PATH', ros_library_path(os.environ.get('LD_LIBRARY_PATH', ''))),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("world_name", default_value="turtlebot3_house.world"),
        DeclareLaunchArgument("map_name", default_value="g1_map"),
        DeclareLaunchArgument(
            "publish_map_odom_identity",
            default_value="false",
            description="Deprecated for this simulation: map_odom_tf is the only TF owner"
        ),

        log_params,

        # Phase 1: Gazebo + spawn (gzserver, gzclient, RSP, spawn_entity)
        g1_spawn,

        # Phase 2: RViz (can start early)
        rviz,

        # Phase 3: Bridge (use_sim_time=false) — publishes immediately with default pose
        # Wrapped in GroupAction to isolate use_sim_time=False from other nodes
        GroupAction([
            SetParameter(name="use_sim_time", value=False),
            TimerAction(period=2.0, actions=[mujoco_bridge]),
        ]),

        # Phase 4: SLAM (use_sim_time=true) — needs /scan + TF chain
        TimerAction(period=10.0, actions=[slam]),
        map_odom_tf,

        # Phase 5: Nav2 (use_sim_time=true) — needs /map + full TF
        navigation_tf_relay,
        TimerAction(period=40.0, actions=[GroupAction([
            SetRemap(src='tf', dst='/nav_tf'),
            nav2,
        ])]),

        # Save map on shutdown
        save_map_on_shutdown,
    ])
