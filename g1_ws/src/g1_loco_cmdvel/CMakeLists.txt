cmake_minimum_required(VERSION 3.8)
project(g1_loco_cmdvel)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(cmd_vel_gateway src/cmd_vel_gateway.cpp)
target_include_directories(cmd_vel_gateway PRIVATE include)
ament_target_dependencies(cmd_vel_gateway geometry_msgs rclcpp sensor_msgs)

set(UNITREE_SDK_ROOT "$ENV{UNITREE_SDK_ROOT}" CACHE PATH
  "Unitree SDK 2 root; required only for the SDK client executable")
if(NOT UNITREE_SDK_ROOT)
  message(WARNING "UNITREE_SDK_ROOT is not set; g1_loco_client will not be built")
else()
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(UNITREE_ARCH aarch64)
  else()
    set(UNITREE_ARCH x86_64)
  endif()

  set(UNITREE_SDK_LIB_DIR "${UNITREE_SDK_ROOT}/lib/${UNITREE_ARCH}")
  set(UNITREE_DDS_LIB_DIR "${UNITREE_SDK_ROOT}/thirdparty/lib/${UNITREE_ARCH}")
  set(UNITREE_SDK_LIB "${UNITREE_SDK_LIB_DIR}/libunitree_sdk2.a")
  set(UNITREE_DDS_LIB "${UNITREE_DDS_LIB_DIR}/libddsc.so")
  set(UNITREE_DDSXX_LIB "${UNITREE_DDS_LIB_DIR}/libddscxx.so")

  foreach(required_file IN ITEMS
      "${UNITREE_SDK_ROOT}/include/unitree/robot/g1/loco/g1_loco_client.hpp"
      "${UNITREE_SDK_LIB}" "${UNITREE_DDS_LIB}" "${UNITREE_DDSXX_LIB}")
    if(NOT EXISTS "${required_file}")
      message(FATAL_ERROR "Missing Unitree SDK file: ${required_file}")
    endif()
  endforeach()

  foreach(sdk_target IN ITEMS g1_loco_client g1_bms_probe)
    add_executable(${sdk_target} src/${sdk_target}.cpp)
    target_include_directories(${sdk_target} PRIVATE
      include "${UNITREE_SDK_ROOT}/include"
      "${UNITREE_SDK_ROOT}/thirdparty/include"
      "${UNITREE_SDK_ROOT}/thirdparty/include/ddscxx")
    target_link_libraries(${sdk_target} PRIVATE
      "${UNITREE_SDK_LIB}" "${UNITREE_DDSXX_LIB}" "${UNITREE_DDS_LIB}" pthread dl)
    # SDK CycloneDDS 0.10.2 must take precedence over ROS's 0.11.x even when
    # /opt/ros/humble/lib is in LD_LIBRARY_PATH (DT_RUNPATH loses to that env).
    target_link_options(${sdk_target} PRIVATE "LINKER:--disable-new-dtags")
    set_target_properties(${sdk_target} PROPERTIES
      BUILD_RPATH "${UNITREE_DDS_LIB_DIR}"
      INSTALL_RPATH "${UNITREE_DDS_LIB_DIR}")
  endforeach()
endif()

install(TARGETS cmd_vel_gateway
  DESTINATION lib/${PROJECT_NAME})
if(TARGET g1_loco_client)
  install(TARGETS g1_loco_client g1_bms_probe
    DESTINATION lib/${PROJECT_NAME})
endif()
install(PROGRAMS scripts/stage4_single_command.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
