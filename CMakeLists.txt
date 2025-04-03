cmake_minimum_required(VERSION 3.8)
project(data_aquisition)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem program_options)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Find CUDA and ZED SDK
find_package(CUDA REQUIRED)
set(ZED_DIR "/usr/local/zed/")
find_package(zed REQUIRED)

# Find nlohmann_json package using PkgConfig if available
include(FindPkgConfig)
if(PkgConfig_FOUND)
  pkg_check_modules(NLOHMANN_JSON QUIET nlohmann_json>=3.0.0)
endif()

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${NLOHMANN_JSON_INCLUDE_DIRS}
  ${ZED_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Camera components
add_library(camera_lib SHARED
  src/sensors/camera/camera_config.cpp
  src/sensors/camera/camera_manager.cpp 
  src/sensors/camera/zed_camera_driver.cpp
  src/sensors/camera/zed_camera_node.cpp
)

ament_target_dependencies(camera_lib
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  tf2
  tf2_ros
  lifecycle_msgs
  diagnostic_updater
)

# Link with ZED SDK libraries
target_link_libraries(camera_lib
  ${ZED_LIBRARIES}
  ${CUDA_LIBRARIES}
)

# GNSS components
add_library(gnss_lib SHARED
  src/sensors/gnss/gnss_config.cpp
  src/sensors/gnss/gnss_driver.cpp
  src/sensors/gnss/gnss_node.cpp
  src/sensors/gnss/gnss_manager.cpp
  src/sensors/gnss/nmea_parser.cpp
  src/sensors/gnss/base64.cpp
  src/sensors/gnss/ntrip_client.cpp
  src/sensors/gnss/rtk_processor.cpp
)

ament_target_dependencies(gnss_lib
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  tf2
  tf2_ros
  lifecycle_msgs
  diagnostic_updater
  nav_msgs
)

# LiDAR components
add_library(lidar_lib SHARED
  src/sensors/lidar/lidar_config.cpp
  src/sensors/lidar/lidar_driver.cpp
  src/sensors/lidar/lidar_manager.cpp
  src/sensors/lidar/livox_converter.cpp
  src/sensors/lidar/livox_lidar_node.cpp
)

ament_target_dependencies(lidar_lib
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  tf2
  tf2_ros
  lifecycle_msgs
  diagnostic_updater
)

# LiDAR Adapter
add_library(livox_adapter SHARED
  src/sensors/lidar/livox_adapter_node.cpp
  src/sensors/lidar/livox_converter.cpp
)

ament_target_dependencies(livox_adapter
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs)

# Recording components removed, using direct file recorder from src/recorder/ instead

# Camera node executable
add_executable(zed_camera_node
  src/sensors/camera/zed_camera_main.cpp
)

target_link_libraries(zed_camera_node camera_lib)

ament_target_dependencies(zed_camera_node
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
)

# GNSS node executable
add_executable(gnss_node
  src/sensors/gnss/gnss_main.cpp
)

target_link_libraries(gnss_node gnss_lib)

ament_target_dependencies(gnss_node
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  nav_msgs
)

# LiDAR node executable
add_executable(livox_lidar_node
  src/sensors/lidar/livox_lidar_main.cpp
)

target_link_libraries(livox_lidar_node lidar_lib)

ament_target_dependencies(livox_lidar_node
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
)

# LiDAR adapter node executable
add_executable(livox_adapter_node
  src/sensors/lidar/livox_adapter_main.cpp
)

target_link_libraries(livox_adapter_node livox_adapter)

ament_target_dependencies(livox_adapter_node
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
)

# Sync components
add_library(sync_lib SHARED
  src/sync/sensor_synchronizer.cpp
)

ament_target_dependencies(sync_lib
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  message_filters
)

# Sync node executable
add_executable(sync_node
  src/sync/sync_main.cpp
)

target_link_libraries(sync_node sync_lib)

ament_target_dependencies(sync_node
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  message_filters
)

# ROS2 Bag Recorder node executable removed, using data_recorder_node instead

# Data Recorder node
add_executable(data_recorder_node
  src/recorder/main.cpp
  src/recorder/data_recorder_node.cpp
)

ament_target_dependencies(data_recorder_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  nav_msgs
  message_filters
)

target_link_libraries(data_recorder_node
  ${OpenCV_LIBRARIES}
)

# Register components
rclcpp_components_register_nodes(camera_lib "sensors::camera::ZedCameraNode")
rclcpp_components_register_nodes(gnss_lib "sensors::gnss::GnssNode")
rclcpp_components_register_nodes(lidar_lib "sensors::lidar::LivoxLidarNode")
rclcpp_components_register_nodes(livox_adapter "sensors::lidar::LivoxAdapterNode")
rclcpp_components_register_nodes(sync_lib "data_aquisition::sync::SensorSynchronizer")

# Install targets
install(TARGETS
  camera_lib
  gnss_lib
  lidar_lib
  livox_adapter
  sync_lib
  zed_camera_node
  gnss_node
  livox_lidar_node
  livox_adapter_node
  sync_node
  data_recorder_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install configuration files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install scripts directory
install(DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# Ensure scripts are executable
execute_process(
  COMMAND chmod +x ${CMAKE_CURRENT_SOURCE_DIR}/scripts/recording/run_synchronized_recording.sh
)

ament_package()