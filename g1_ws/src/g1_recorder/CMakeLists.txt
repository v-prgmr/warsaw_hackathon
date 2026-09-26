cmake_minimum_required(VERSION 3.10)
project(g1_recorder)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/discover_sensors.sh scripts/camera_calibration_capture
           scripts/calibrate_charuco_bag scripts/capture_frontier_snapshot
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
