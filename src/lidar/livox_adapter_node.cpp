/**
 * @file livox_adapter_node.cpp
 * @brief Implementation of adapter node for Livox ROS2 driver
 */

#include "sensors/lidar/livox_adapter_node.h"
#include "sensors/lidar/livox_converter.h"

namespace sensors {
namespace lidar {

LivoxAdapterNode::LivoxAdapterNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("livox_adapter_node", options) {
  
  RCLCPP_INFO(get_logger(), "Creating LivoxAdapterNode");
  
  // Initialize parameters
  initializeParameters();
  
  // Initialize topics
  initializeTopics();
  
  RCLCPP_INFO(get_logger(), "LivoxAdapterNode initialized");
  RCLCPP_INFO(get_logger(), "Subscribing to input topics:");
  RCLCPP_INFO(get_logger(), "  Point cloud: %s", input_point_cloud_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  IMU: %s", input_imu_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Publishing to output topics:");
  RCLCPP_INFO(get_logger(), "  Point cloud: %s", output_point_cloud_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  IMU: %s", output_imu_topic_.c_str());
}

LivoxAdapterNode::~LivoxAdapterNode() {
  RCLCPP_INFO(get_logger(), "Destroying LivoxAdapterNode");
}

void LivoxAdapterNode::initializeParameters() {
  // Input topics
  input_point_cloud_topic_ = declare_parameter<std::string>("input_point_cloud_topic", "/livox/lidar");
  input_imu_topic_ = declare_parameter<std::string>("input_imu_topic", "/livox/imu");
  
  // Output topics
  output_point_cloud_topic_ = declare_parameter<std::string>("output_point_cloud_topic", "~/point_cloud");
  output_imu_topic_ = declare_parameter<std::string>("output_imu_topic", "~/imu");
  
  // Frame ID
  frame_id_ = declare_parameter<std::string>("frame_id", "lidar_frame");
  
  // Filtering parameters
  filter_points_ = declare_parameter<bool>("filter_points", true);
  min_distance_ = declare_parameter<float>("min_distance", 0.1);  // 10cm
  max_distance_ = declare_parameter<float>("max_distance", 100.0);  // 100m
  
  // Downsampling parameters
  downsample_factor_ = declare_parameter<int>("downsample_factor", 1);  // No downsampling by default
}

void LivoxAdapterNode::initializeTopics() {
  // QoS settings
  auto qos = rclcpp::QoS(10);
  
  // Subscribers
  point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_point_cloud_topic_,
      qos,
      std::bind(&LivoxAdapterNode::handlePointCloudData, this, std::placeholders::_1));
  
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      input_imu_topic_,
      qos,
      std::bind(&LivoxAdapterNode::handleImuData, this, std::placeholders::_1));
  
  // Publishers
  point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_point_cloud_topic_,
      qos);
  
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      output_imu_topic_,
      qos);
}

void LivoxAdapterNode::handlePointCloudData(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // Make a copy of the message
  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
  
  // Set frame ID if needed
  if (!frame_id_.empty()) {
    cloud->header.frame_id = frame_id_;
  }
  
  // Apply processing if needed
  if (filter_points_) {
    cloud = LivoxConverter::filterPointCloud(cloud, min_distance_, max_distance_);
  }
  
  if (downsample_factor_ > 1) {
    cloud = LivoxConverter::downsamplePointCloud(cloud, downsample_factor_);
  }
  
  // Publish processed point cloud
  point_cloud_pub_->publish(*cloud);
}

void LivoxAdapterNode::handleImuData(const sensor_msgs::msg::Imu::SharedPtr msg) {
  // Make a copy of the message
  auto imu = std::make_shared<sensor_msgs::msg::Imu>(*msg);
  
  // Set frame ID if needed
  if (!frame_id_.empty()) {
    imu->header.frame_id = frame_id_;
  }
  
  // Publish IMU data
  imu_pub_->publish(*imu);
}

} // namespace lidar
} // namespace sensors

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sensors::lidar::LivoxAdapterNode)