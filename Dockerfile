# Host-side ROS 2 Humble environment for talking to the Unitree G1 (AGENTS.md §4, §5).
# Runs on any host OS (e.g. Ubuntu 24.04 / Jazzy) — all robot-facing ROS traffic goes through here.
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-generator-dds-idl \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-tf2-tools \
    python3-pip \
    git \
    iproute2 \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# Mapping + navigation (AGENTS.md §8, §11): RTAB-Map LiDAR-inertial mapping (g1_mapping), Nav2 consumes
# its map. rtabmap-ros brings icp_odometry, imu_to_tf, lidar_deskewing and the lidar3d example.
# imu-tools (imu_complementary_filter): orientation + gyro bias for IMUs that publish only gyro/accel, e.g. the
# MID-360's internal /utlidar/imu_livox_mid360 (/dog_imu_raw already carries orientation).
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rtabmap-ros \
    ros-humble-imu-tools \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-pointcloud-to-laserscan \
    && rm -rf /var/lib/apt/lists/*

# Simulation: Gazebo Classic 11 + TurtleBot3 Waffle (2D LiDAR + RGB-D). Useful for Nav2 / RTAB-Map
# plumbing only. It CANNOT exercise the 3D-LiDAR path; test g1_mapping on replayed G1 bags instead.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-teleop \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*
COPY docker/patch_tb3_waffle_rgbd.py /tmp/patch_tb3_waffle_rgbd.py
RUN python3 /tmp/patch_tb3_waffle_rgbd.py \
        /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf \
    && rm /tmp/patch_tb3_waffle_rgbd.py
ENV TURTLEBOT3_MODEL=waffle \
    GAZEBO_MODEL_DATABASE_URI=""

# Unitree ROS 2 message packages (unitree_go / unitree_hg / unitree_api).
# On Humble the stock rmw_cyclonedds_cpp is used; no separate CycloneDDS build is needed.
ARG UNITREE_ROS2_REF=master
RUN git clone --depth 1 --branch ${UNITREE_ROS2_REF} https://github.com/unitreerobotics/unitree_ros2.git /opt/unitree_ros2 \
    && . /opt/ros/humble/setup.sh \
    && cd /opt/unitree_ros2/cyclonedds_ws \
    && colcon build --packages-select unitree_go unitree_hg unitree_api \
    && rm -rf build log

# DDS: CycloneDDS pinned to the robot-facing NIC. Override ROBOT_IFACE at `docker run`.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_DOMAIN_ID=0 \
    ROBOT_IFACE=enp2s0 \
    CYCLONEDDS_URI=file:///etc/cyclonedds/cyclonedds.xml
COPY docker/cyclonedds.xml /etc/cyclonedds/cyclonedds.xml

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh \
    && echo '. /ros_entrypoint.sh' >> /etc/bash.bashrc

WORKDIR /ws
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
