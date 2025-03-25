/**
 * @file lidar_config.cpp
 * @brief Implementation of configuration class for LiDAR sensors
 */

#include "sensors/lidar/lidar_config.h"
#include <iostream>

namespace sensors {
namespace lidar {

LidarConfig::LidarConfig(const rclcpp::Node::SharedPtr& node)
  : node_(node) {
  
  // Get namespace for this node
  std::string ns = node_->get_namespace();
  if (ns == "/") {
    ns = "/livox_lidar";
  }
  
  // Load parameters with defaults
  frame_id_ = node_->declare_parameter<std::string>("lidar.frame_id", "livox_frame");
  ip_address_ = node_->declare_parameter<std::string>("lidar.ip_address", "192.168.1.100");
  data_port_ = node_->declare_parameter<int>("lidar.data_port", 57000);
  command_port_ = node_->declare_parameter<int>("lidar.command_port", 56000);
  imu_port_ = node_->declare_parameter<int>("lidar.imu_port", 58000);
  use_dhcp_ = node_->declare_parameter<bool>("lidar.use_dhcp", false);
  host_ip_address_ = node_->declare_parameter<std::string>("lidar.host_ip_address", "192.168.1.5");
  publish_frequency_ = node_->declare_parameter<double>("lidar.publish_frequency", 10.0);
  point_cloud_type_ = node_->declare_parameter<int>("lidar.point_cloud_type", 0);
  broadcast_code_ = node_->declare_parameter<std::string>("lidar.broadcast_code", "livox0000000001");
  qos_reliability_ = node_->declare_parameter<bool>("lidar.qos_reliability", true);
  qos_history_depth_ = node_->declare_parameter<int>("lidar.qos_history_depth", 5);
  
  RCLCPP_INFO(node_->get_logger(), "LiDAR Configuration:");
  RCLCPP_INFO(node_->get_logger(), "  Namespace: %s", ns.c_str());
  RCLCPP_INFO(node_->get_logger(), "  Frame ID: %s", frame_id_.c_str());
  RCLCPP_INFO(node_->get_logger(), "  IP Address: %s", ip_address_.c_str());
  RCLCPP_INFO(node_->get_logger(), "  Host IP: %s", host_ip_address_.c_str());
  RCLCPP_INFO(node_->get_logger(), "  DHCP Mode: %s", use_dhcp_ ? "Enabled" : "Disabled");
}

std::string LidarConfig::getNamespace() const {
  return node_->get_namespace();
}

std::string LidarConfig::getFrameId() const {
  return frame_id_;
}

std::string LidarConfig::getIpAddress() const {
  return ip_address_;
}

int LidarConfig::getDataPort() const {
  return data_port_;
}

int LidarConfig::getCommandPort() const {
  return command_port_;
}

int LidarConfig::getImuPort() const {
  return imu_port_;
}

bool LidarConfig::useDhcp() const {
  return use_dhcp_;
}

std::string LidarConfig::getHostIpAddress() const {
  return host_ip_address_;
}

double LidarConfig::getPublishFrequency() const {
  return publish_frequency_;
}

int LidarConfig::getPointCloudType() const {
  return point_cloud_type_;
}

std::string LidarConfig::getBroadcastCode() const {
  return broadcast_code_;
}

bool LidarConfig::getQosReliability() const {
  return qos_reliability_;
}

int LidarConfig::getQosHistoryDepth() const {
  return qos_history_depth_;
}

} // namespace lidar
} // namespace sensors