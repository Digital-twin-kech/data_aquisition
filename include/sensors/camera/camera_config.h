#ifndef CAMERA_CONFIG_H
#define CAMERA_CONFIG_H

#include <string>
#include <rclcpp/rclcpp.hpp>

namespace sensors {
namespace camera {

/**
 * @brief Configuration parameters for ZED cameras
 * 
 * This class holds configuration settings for ZED cameras,
 * including resolution, FPS settings, and QoS profiles.
 */
class CameraConfig {
public:
  /**
   * @brief Construct a new Camera Config object
   * 
   * @param node ROS2 node to read parameters from
   */
  explicit CameraConfig(rclcpp::Node::SharedPtr node);

  /**
   * @brief Destructor
   */
  ~CameraConfig() = default;

  /**
   * @brief Get the minimum FPS setting
   * 
   * @return float Minimum FPS value
   */
  float getMinFps() const { return min_fps_; }

  /**
   * @brief Get the maximum FPS setting
   * 
   * @return float Maximum FPS value
   */
  float getMaxFps() const { return max_fps_; }

  /**
   * @brief Get the camera serial number
   * 
   * @return int Camera serial number
   */
  int getSerialNumber() const { return serial_number_; }

  /**
   * @brief Get the resolution setting
   * 
   * @return std::string Resolution setting (e.g., "HD1080")
   */
  std::string getResolution() const { return resolution_; }

  /**
   * @brief Get the camera model name
   * 
   * @return std::string Camera model name (e.g., "ZED X" or "ZED2i")
   */
  std::string getCameraModel() const { return camera_model_; }

  /**
   * @brief Get the QoS reliability setting
   * 
   * @return rclcpp::ReliabilityPolicy QoS reliability policy
   */
  rclcpp::ReliabilityPolicy getReliabilityPolicy() const { return reliability_policy_; }

  /**
   * @brief Get the QoS history depth
   * 
   * @return int QoS history depth
   */
  int getQosHistoryDepth() const { return qos_history_depth_; }

  /**
   * @brief Get PID proportional gain
   * 
   * @return double PID proportional gain
   */
  double getPidP() const { return pid_p_; }

  /**
   * @brief Get PID integral gain
   * 
   * @return double PID integral gain
   */
  double getPidI() const { return pid_i_; }

  /**
   * @brief Get PID derivative gain
   * 
   * @return double PID derivative gain
   */
  double getPidD() const { return pid_d_; }
  
  /**
   * @brief Get whether depth processing should be enabled
   * 
   * @return bool True if depth processing should be enabled, false otherwise
   */
  bool getUseDepth() const { return use_depth_; }
  
  /**
   * @brief Get the frame ID for this camera
   * 
   * @return std::string Frame ID to use in message headers
   */
  std::string getFrameId() const { 
    // Use the namespace as the frame ID base
    std::string ns = node_->get_namespace();
    if (ns.empty() || ns == "/") {
      return "zed_camera_" + camera_model_;
    }
    
    // Remove leading slash if present
    if (ns[0] == '/') {
      ns = ns.substr(1);
    }
    
    return ns + "_frame";
  }

private:
  // ROS2 node
  rclcpp::Node::SharedPtr node_;

  // Camera settings
  std::string camera_model_;
  std::string resolution_;
  int serial_number_;
  bool use_depth_;
  
  // FPS settings
  float min_fps_;
  float max_fps_;
  
  // QoS settings
  rclcpp::ReliabilityPolicy reliability_policy_;
  int qos_history_depth_;
  
  // PID controller settings
  double pid_p_;
  double pid_i_;
  double pid_d_;
  
  /**
   * @brief Load configuration from ROS2 parameters
   */
  void loadParameters();
};

}  // namespace camera
}  // namespace sensors

#endif  // CAMERA_CONFIG_H
