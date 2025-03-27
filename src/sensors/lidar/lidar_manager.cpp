/**
 * @file lidar_manager.cpp
 * @brief Implementation of manager class for LiDAR sensors
 */

#include "sensors/lidar/lidar_manager.h"
#include <functional>

namespace sensors {
namespace lidar {

LidarManager::LidarManager(
    const rclcpp::Node::SharedPtr& node, 
    const LidarConfigPtr& config)
  : node_(node),
    config_(config),
    running_(false) {
  
  // Create driver
  driver_ = std::make_shared<LidarDriver>(config_);
}

LidarManager::~LidarManager() {
  stop();
}

bool LidarManager::initialize() {
  // Create publishers
  std::string ns = node_->get_namespace();
  
  // QoS settings
  rclcpp::QoS lidar_qos = config_->getQosReliability() 
      ? rclcpp::QoS(config_->getQosHistoryDepth()).reliable() 
      : rclcpp::QoS(config_->getQosHistoryDepth()).best_effort();
  
  // Publishers
  point_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      ns + "/point_cloud", lidar_qos);
  
  imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
      ns + "/imu", lidar_qos);
  
  status_pub_ = node_->create_publisher<std_msgs::msg::String>(
      ns + "/status", rclcpp::QoS(10));
  
  // Status timer
  status_timer_ = node_->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&LidarManager::statusTimerCallback, this));
  
  // Set callbacks for the driver
  driver_->setPointCloudCallback(
      std::bind(&LidarManager::handlePointCloud, this, std::placeholders::_1));
  
  driver_->setImuCallback(
      std::bind(&LidarManager::handleImu, this, std::placeholders::_1));
  
  // Initialize driver
  if (!driver_->initialize()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to initialize LiDAR driver");
    return false;
  }
  
  RCLCPP_INFO(node_->get_logger(), "LiDAR manager initialized");
  return true;
}

bool LidarManager::start() {
  if (running_) {
    // Already running
    return true;
  }
  
  RCLCPP_INFO(node_->get_logger(), "Starting LiDAR manager");
  
  // Connect to the LiDAR
  if (!driver_->isConnected()) {
    if (!driver_->connect()) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to connect to LiDAR");
      return false;
    }
  }
  
  // Start data acquisition
  if (!driver_->start()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to start LiDAR data acquisition");
    return false;
  }
  
  running_ = true;
  RCLCPP_INFO(node_->get_logger(), "LiDAR manager started");
  return true;
}

void LidarManager::stop() {
  if (!running_) {
    return;
  }
  
  RCLCPP_INFO(node_->get_logger(), "Stopping LiDAR manager");
  
  // Stop data acquisition
  driver_->stop();
  
  // Disconnect from the LiDAR
  driver_->disconnect();
  
  running_ = false;
  RCLCPP_INFO(node_->get_logger(), "LiDAR manager stopped");
}

bool LidarManager::isRunning() const {
  return running_;
}

void LidarManager::handlePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // Set the frame ID and timestamp
  msg->header.frame_id = config_->getFrameId();
  msg->header.stamp = node_->now();
  
  // Publish the point cloud
  point_cloud_pub_->publish(*msg);
}

void LidarManager::handleImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  // Set the frame ID and timestamp
  msg->header.frame_id = config_->getFrameId();
  msg->header.stamp = node_->now();
  
  // Publish the IMU data
  imu_pub_->publish(*msg);
}

void LidarManager::publishStatus() {
  std_msgs::msg::String status_msg;
  
  if (!driver_->isConnected()) {
    status_msg.data = "Disconnected";
  } else if (!driver_->isRunning()) {
    status_msg.data = "Connected, Not Running";
  } else {
    status_msg.data = "Running";
  }
  
  status_pub_->publish(status_msg);
}

void LidarManager::statusTimerCallback() {
  publishStatus();
}

} // namespace lidar
} // namespace sensors