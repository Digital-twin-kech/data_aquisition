#ifndef CAMERA_MANAGER_H
#define CAMERA_MANAGER_H

#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include "sensors/camera/camera_config.h"
#include "sensors/camera/zed_camera_driver.h"

namespace sensors {
namespace camera {

/**
 * @brief PID controller for camera FPS regulation
 */
class PidController {
public:
  /**
   * @brief Construct a new PID Controller object
   * 
   * @param p Proportional gain
   * @param i Integral gain
   * @param d Derivative gain
   * @param min_output Minimum output value
   * @param max_output Maximum output value
   */
  PidController(double p, double i, double d, double min_output, double max_output);

  /**
   * @brief Update the controller with a new error value
   * 
   * @param error Current error (setpoint - measured_value)
   * @param dt Time delta since last update in seconds
   * @return double Controller output
   */
  double update(double error, double dt);

  /**
   * @brief Reset the controller state
   */
  void reset();

private:
  double kp_;        // Proportional gain
  double ki_;        // Integral gain
  double kd_;        // Derivative gain
  double min_output_; // Minimum output value
  double max_output_; // Maximum output value
  
  double integral_;    // Integral term accumulator
  double previous_error_; // Previous error for derivative calculation
  bool first_update_;  // Flag for first update call
};

/**
 * @brief Manages camera operation and ROS2 integration
 * 
 * This class handles the camera lifecycle, data publishing, and dynamic
 * adjustment of camera settings based on vehicle speed using PID control.
 */
class CameraManager {
public:
  /**
   * @brief Construct a new Camera Manager object
   * 
   * @param node ROS2 node
   * @param config Camera configuration
   */
  CameraManager(rclcpp::Node::SharedPtr node, std::shared_ptr<CameraConfig> config);
  
  /**
   * @brief Destructor
   */
  ~CameraManager();
  
  /**
   * @brief Initialize the camera and publishers
   * 
   * @return true if initialization was successful
   * @return false if initialization failed
   */
  bool initialize();
  
  /**
   * @brief Start the camera operation
   */
  void start();
  
  /**
   * @brief Stop the camera operation
   */
  void stop();
  
  /**
   * @brief Check if the camera manager is running
   * 
   * @return true if running
   * @return false if not running
   */
  bool isRunning() const;

private:
  // ROS2 node
  rclcpp::Node::SharedPtr node_;
  
  // Configuration
  std::shared_ptr<CameraConfig> config_;
  
  // Camera driver
  std::unique_ptr<ZedCameraDriver> camera_driver_;
  
  // PID controller for FPS adjustment
  std::unique_ptr<PidController> fps_pid_controller_;
  
  // Vehicle speed (m/s)
  std::atomic<float> vehicle_speed_;
  
  // Running state
  std::atomic<bool> running_;
  
  // Camera operation thread
  std::thread camera_thread_;
  std::mutex camera_mutex_;
  
  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr svo2_pub_;
  
  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vehicle_speed_sub_;
  
  // Camera operation functions
  void cameraLoop();
  void publishCameraData();
  void updateCameraSettings();
  void vehicleSpeedCallback(const std_msgs::msg::Float32::SharedPtr msg);
  
  // Calculate optimal FPS based on vehicle speed
  float calculateTargetFps(float vehicle_speed);
};

}  // namespace camera
}  // namespace sensors

#endif  // CAMERA_MANAGER_H
