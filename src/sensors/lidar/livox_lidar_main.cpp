/**
 * @file livox_lidar_main.cpp
 * @brief Main entry point for Livox LiDAR node
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "sensors/lidar/livox_lidar_node.h"

int main(int argc, char** argv) {
  // Initialize ROS
  rclcpp::init(argc, argv);
  
  // Create node with default options
  rclcpp::NodeOptions options;
  auto node = std::make_shared<sensors::lidar::LivoxLidarNode>(options);
  
  // Spin the node
  rclcpp::spin(node->get_node_base_interface());
  
  // Clean up
  rclcpp::shutdown();
  return 0;
}