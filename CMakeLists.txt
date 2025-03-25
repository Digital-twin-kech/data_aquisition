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
)

# Camera components
add_library(camera_lib SHARED
  src/camera/camera_config.cpp
  src/camera/camera_manager.cpp
  src/camera/zed_camera_driver.cpp
  src/camera/zed_camera_node.cpp
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
  src/gnss/gnss_config.cpp
  src/gnss/gnss_driver.cpp
  src/gnss/gnss_node.cpp
  src/gnss/gnss_manager.cpp
  src/gnss/nmea_parser.cpp
  src/gnss/base64.cpp
  src/gnss/ntrip_client.cpp
  src/gnss/rtk_processor.cpp
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

# Camera node executable
add_executable(zed_camera_node
  src/camera/zed_camera_main.cpp
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
  src/gnss/gnss_main.cpp
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

# Register components
rclcpp_components_register_nodes(camera_lib "sensors::camera::ZedCameraNode")
rclcpp_components_register_nodes(gnss_lib "sensors::gnss::GnssNode")

# Install targets
install(TARGETS
  camera_lib
  gnss_lib
  zed_camera_node
  gnss_node
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

ament_package()