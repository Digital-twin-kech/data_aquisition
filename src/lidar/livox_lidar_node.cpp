/**
 * @file livox_lidar_node.cpp
 * @brief Implementation of ROS2 node for Livox LiDAR
 */

#include "sensors/lidar/livox_lidar_node.h"

namespace sensors {
namespace lidar {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

LivoxLidarNode::LivoxLidarNode(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("livox_lidar_node", options) {
  
  RCLCPP_INFO(get_logger(), "Creating LivoxLidarNode");
  
  // Create a private node for non-lifecycle publishers/subscribers
  private_node_ = std::make_shared<rclcpp::Node>(
      "_" + std::string(get_name()) + "_private",
      options);
}

LivoxLidarNode::~LivoxLidarNode() {
  RCLCPP_INFO(get_logger(), "Destroying LivoxLidarNode");
}

CallbackReturn LivoxLidarNode::on_configure(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Configuring LivoxLidarNode from %s state", state.label().c_str());
  
  // Create configuration
  config_ = std::make_shared<LidarConfig>(private_node_);
  
  // Create manager
  manager_ = std::make_shared<LidarManager>(private_node_, config_);
  
  // Initialize manager
  if (!manager_->initialize()) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize LiDAR manager");
    return CallbackReturn::FAILURE;
  }
  
  RCLCPP_INFO(get_logger(), "LivoxLidarNode configured successfully");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LivoxLidarNode::on_activate(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Activating LivoxLidarNode from %s state", state.label().c_str());
  
  // Start manager
  if (!manager_->start()) {
    RCLCPP_ERROR(get_logger(), "Failed to start LiDAR manager");
    return CallbackReturn::FAILURE;
  }
  
  RCLCPP_INFO(get_logger(), "LivoxLidarNode activated successfully");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LivoxLidarNode::on_deactivate(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Deactivating LivoxLidarNode from %s state", state.label().c_str());
  
  // Stop manager
  manager_->stop();
  
  RCLCPP_INFO(get_logger(), "LivoxLidarNode deactivated successfully");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LivoxLidarNode::on_cleanup(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Cleaning up LivoxLidarNode from %s state", state.label().c_str());
  
  // Cleanup resources
  manager_.reset();
  config_.reset();
  
  RCLCPP_INFO(get_logger(), "LivoxLidarNode cleaned up successfully");
  return CallbackReturn::SUCCESS;
}

CallbackReturn LivoxLidarNode::on_shutdown(const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Shutting down LivoxLidarNode from %s state", state.label().c_str());
  
  // Stop manager if still running
  if (manager_ && manager_->isRunning()) {
    manager_->stop();
  }
  
  RCLCPP_INFO(get_logger(), "LivoxLidarNode shut down successfully");
  return CallbackReturn::SUCCESS;
}

} // namespace lidar
} // namespace sensors

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sensors::lidar::LivoxLidarNode)