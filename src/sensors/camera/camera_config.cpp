#include "sensors/camera/camera_config.h"
#include <string>
#include "rclcpp/rclcpp.hpp"

namespace sensors {
namespace camera {

CameraConfig::CameraConfig(rclcpp::Node::SharedPtr node) 
    : node_(node),
      camera_model_("ZED X"),
      resolution_("HD1080"),
      serial_number_(0),
      use_depth_(true),
      min_fps_(1.0f),
      max_fps_(60.0f),
      reliability_policy_(rclcpp::ReliabilityPolicy::BestEffort), // Default to BestEffort for real-time video
      qos_history_depth_(1), // Default to 1 for lowest latency
      pid_p_(0.5),
      pid_i_(0.1),
      pid_d_(0.01) {
  
  loadParameters();
}

void CameraConfig::loadParameters() {
  // Declare default parameters if they don't exist
  node_->declare_parameter<std::string>("camera.model", camera_model_);
  node_->declare_parameter<std::string>("camera.resolution", resolution_);
  node_->declare_parameter<int>("camera.serial_number", serial_number_);
  node_->declare_parameter<float>("camera.min_fps", min_fps_);
  node_->declare_parameter<float>("camera.max_fps", max_fps_);
  // Default to true (Reliable) for better compatibility with visualization tools
  node_->declare_parameter<bool>("camera.reliable_qos", true);
  // Use slightly higher depth for visualization tools
  node_->declare_parameter<int>("camera.qos_history_depth", 5);
  node_->declare_parameter<double>("camera.pid_p", pid_p_);
  node_->declare_parameter<double>("camera.pid_i", pid_i_);
  node_->declare_parameter<double>("camera.pid_d", pid_d_);
  
  // Depth parameters
  node_->declare_parameter<bool>("depth.enabled", true);
  node_->declare_parameter<bool>("point_cloud.enabled", true);
  
  // Load parameters
  camera_model_ = node_->get_parameter("camera.model").as_string();
  resolution_ = node_->get_parameter("camera.resolution").as_string();
  serial_number_ = node_->get_parameter("camera.serial_number").as_int();
  // Get depth.enabled parameter or fallback to point_cloud.enabled
  use_depth_ = node_->get_parameter("depth.enabled").as_bool() || 
               node_->get_parameter("point_cloud.enabled").as_bool();
  min_fps_ = node_->get_parameter("camera.min_fps").as_double();
  max_fps_ = node_->get_parameter("camera.max_fps").as_double();
  bool reliable_qos = node_->get_parameter("camera.reliable_qos").as_bool();
  qos_history_depth_ = node_->get_parameter("camera.qos_history_depth").as_int();
  pid_p_ = node_->get_parameter("camera.pid_p").as_double();
  pid_i_ = node_->get_parameter("camera.pid_i").as_double();
  pid_d_ = node_->get_parameter("camera.pid_d").as_double();
  
  // Apply values
  reliability_policy_ = reliable_qos ? 
      rclcpp::ReliabilityPolicy::Reliable : 
      rclcpp::ReliabilityPolicy::BestEffort;
  
  // Validate parameter ranges
  if (min_fps_ < 1.0f) {
    RCLCPP_WARN(node_->get_logger(), 
                "Minimum FPS value cannot be less than 1.0. Setting to 1.0.");
    min_fps_ = 1.0f;
  }
  
  if (max_fps_ > 60.0f) {
    RCLCPP_WARN(node_->get_logger(), 
                "Maximum FPS value cannot exceed 60.0. Setting to 60.0.");
    max_fps_ = 60.0f;
  }
  
  if (qos_history_depth_ < 1) {
    RCLCPP_WARN(node_->get_logger(), 
                "QoS history depth must be at least 1. Setting to 1.");
    qos_history_depth_ = 1;
  }
  
  RCLCPP_INFO(node_->get_logger(), 
              "Camera configuration loaded: Model=%s, Resolution=%s, FPS range=[%.1f, %.1f]",
              camera_model_.c_str(), resolution_.c_str(), min_fps_, max_fps_);
}

}  // namespace camera
}  // namespace sensors
