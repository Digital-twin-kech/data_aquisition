#ifndef ZED_CAMERA_NODE_H
#define ZED_CAMERA_NODE_H

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "sensors/camera/camera_config.h"
#include "sensors/camera/camera_manager.h"

namespace sensors {
namespace camera {

/**
 * @brief ROS2 node for ZED camera
 * 
 * This class implements a ROS2 node for interfacing with ZED cameras,
 * handling parameter management, lifecycle transitions, and QoS settings.
 */
class ZedCameraNode : public rclcpp::Node {
public:
  /**
   * @brief Construct a new Zed Camera Node object
   * 
   * @param options ROS2 node options
   */
  explicit ZedCameraNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  
  /**
   * @brief Destructor
   */
  ~ZedCameraNode();

private:
  // Camera configuration
  std::shared_ptr<CameraConfig> camera_config_;
  
  // Camera manager
  std::unique_ptr<CameraManager> camera_manager_;
  
  // Node initialization
  void initializeParameters();
  void initializeNode();
  
  // Timers for deferred initialization
  std::vector<rclcpp::TimerBase::SharedPtr> init_timers_;
  
  // Parameter change callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rcl_interfaces::msg::SetParametersResult onParametersChanged(const std::vector<rclcpp::Parameter>& parameters);
};

}  // namespace camera
}  // namespace sensors

#endif  // ZED_CAMERA_NODE_H
