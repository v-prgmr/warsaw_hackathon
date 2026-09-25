cmake_minimum_required(VERSION 3.10)
project(g1_recorder)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/discover_sensors.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
