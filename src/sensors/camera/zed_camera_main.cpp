#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensors/camera/zed_camera_node.h"

int main(int argc, char** argv) {
  // Initialize ROS2
  rclcpp::init(argc, argv);
  
  // Setup logging
  rclcpp::Logger logger = rclcpp::get_logger("zed_camera_main");
  RCLCPP_INFO(logger, "Starting ZED camera node");
  
  // Create node with default options
  rclcpp::NodeOptions options;
  auto node = std::make_shared<sensors::camera::ZedCameraNode>(options);
  
  // Spin the node
  rclcpp::spin(node);
  
  // Shutdown ROS2
  rclcpp::shutdown();
  
  RCLCPP_INFO(logger, "ZED camera node terminated");
  
  return 0;
}

