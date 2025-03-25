#include "sensors/camera/zed_camera_node.h"
#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include "sensors/camera/camera_config.h"
#include "sensors/camera/camera_manager.h"

namespace sensors {
namespace camera {

ZedCameraNode::ZedCameraNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("zed_camera_node", options), // ROS2 remapping with __node:=name will override this
      camera_config_(nullptr),
      camera_manager_(nullptr) {
          
    // Log the actual node name that was applied (may be different if remapped)
    RCLCPP_INFO(get_logger(), "ZED camera node started with name: %s", get_name());

  // Initialize parameters
  initializeParameters();

  // Schedule initialization to happen after constructor finishes
  // This avoids the bad_weak_ptr issue with shared_from_this() in constructor
  auto init_timer = this->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&ZedCameraNode::initializeNode, this)
  );

  // Make it a one-shot timer
  init_timers_.push_back(init_timer);
}

void ZedCameraNode::initializeNode() {
  try {
    // Create camera configuration
    camera_config_ = std::make_shared<CameraConfig>(shared_from_this());

    // Create camera manager
    camera_manager_ = std::make_unique<CameraManager>(shared_from_this(), camera_config_);

    // Initialize camera manager
    if (!camera_manager_->initialize()) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize camera manager");
      return;
    }

    // Register parameter callback
    param_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&ZedCameraNode::onParametersChanged, this, std::placeholders::_1));

    // Start camera manager
    camera_manager_->start();

    RCLCPP_INFO(get_logger(), "ZED camera node initialized successfully");
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Exception during node initialization: %s", e.what());
  }
  
  // Stop the timer so this only runs once
  init_timers_.clear();
}

ZedCameraNode::~ZedCameraNode() {
  try {
    if (camera_manager_ && camera_manager_->isRunning()) {
      camera_manager_->stop();
    }
    RCLCPP_INFO(get_logger(), "ZED camera node shutdown complete");
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Exception during node shutdown: %s", e.what());
  }
}

void ZedCameraNode::initializeParameters() {
  // Declare node parameters
  declare_parameter<bool>("auto_start", true);
  declare_parameter<std::string>("camera_name", "zed_camera");
  declare_parameter<std::string>("base_frame", "zed_base_link");
  declare_parameter<bool>("use_tf", true);

  // Log parameter values
  RCLCPP_INFO(get_logger(), "Parameters initialized");
}

rcl_interfaces::msg::SetParametersResult ZedCameraNode::onParametersChanged(
    const std::vector<rclcpp::Parameter>& parameters) {

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  // Handle parameter changes
  for (const auto& param : parameters) {
    RCLCPP_INFO(get_logger(), "Parameter changed: %s", param.get_name().c_str());

    // Handle specific parameters that require action
    if (param.get_name() == "auto_start" && !param.as_bool() && camera_manager_->isRunning()) {
      camera_manager_->stop();
    }
    else if (param.get_name() == "auto_start" && param.as_bool() && !camera_manager_->isRunning()) {
      camera_manager_->start();
    }
  }

  return result;
}

}  // namespace camera
}  // namespace sensors

// ROS2 component registration
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sensors::camera::ZedCameraNode)

