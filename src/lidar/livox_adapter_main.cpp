/**
 * @file livox_adapter_main.cpp
 * @brief Main entry point for Livox adapter node
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "sensors/lidar/livox_adapter_node.h"

int main(int argc, char** argv) {
  // Initialize ROS
  rclcpp::init(argc, argv);
  
  // Create node with default options
  rclcpp::NodeOptions options;
  auto node = std::make_shared<sensors::lidar::LivoxAdapterNode>(options);
  
  // Spin the node
  rclcpp::spin(node);
  
  // Clean up
  rclcpp::shutdown();
  return 0;
}