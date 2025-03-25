#include "sensors/camera/camera_manager.h"
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <chrono>
#include <functional>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
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

// PidController implementation
PidController::PidController(double p, double i, double d, double min_output, double max_output)
    : kp_(p),
      ki_(i),
      kd_(d),
      min_output_(min_output),
      max_output_(max_output),
      integral_(0.0),
      previous_error_(0.0),
      first_update_(true) {
}

double PidController::update(double error, double dt) {
  if (first_update_) {
    previous_error_ = error;
    first_update_ = false;
    return kp_ * error;
  }
  
  // Calculate integral term with anti-windup
  integral_ += error * dt;
  
  // Apply derivative term
  double derivative = (error - previous_error_) / dt;
  
  // Calculate PID output
  double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
  
  // Clamp output to limits
  output = std::max(min_output_, std::min(max_output_, output));
  
  // Anti-windup: if output is saturated, limit integral
  if ((output == max_output_ && error > 0) || (output == min_output_ && error < 0)) {
    integral_ -= error * dt;
  }
  
  // Save current error for next iteration
  previous_error_ = error;
  
  return output;
}

void PidController::reset() {
  integral_ = 0.0;
  previous_error_ = 0.0;
  first_update_ = true;
}

// CameraManager implementation
CameraManager::CameraManager(rclcpp::Node::SharedPtr node, std::shared_ptr<CameraConfig> config)
    : node_(node),
      config_(config),
      camera_driver_(nullptr),
      fps_pid_controller_(nullptr),
      vehicle_speed_(0.0f),
      running_(false) {
}

CameraManager::~CameraManager() {
  if (running_) {
    stop();
  }
}

bool CameraManager::initialize() {
  try {
    // Create camera driver
    camera_driver_ = std::make_unique<ZedCameraDriver>(config_);
    
    // Create PID controller for FPS regulation
    fps_pid_controller_ = std::make_unique<PidController>(
      config_->getPidP(),
      config_->getPidI(),
      config_->getPidD(),
      config_->getMinFps(),
      config_->getMaxFps()
    );
    
    // Setup QoS profile for camera topics that matches RViz and standard tools
    // Use reliability=reliable for compatibility with default RViz settings
    rclcpp::QoS camera_qos(config_->getQosHistoryDepth());
    // Override reliability to Reliable for better compatibility with visualization tools
    camera_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    camera_qos.history(rclcpp::HistoryPolicy::KeepLast);
    camera_qos.durability(rclcpp::DurabilityPolicy::Volatile);
    
    // Log QoS settings for debugging
    RCLCPP_INFO(node_->get_logger(), "Setting up camera publishers with QoS: reliability=%s, history=KeepLast(%d)",
                config_->getReliabilityPolicy() == rclcpp::ReliabilityPolicy::BestEffort ? "BestEffort" : "Reliable",
                config_->getQosHistoryDepth());
    
    // Get the ROS2 namespace - the ZED_CAMERA_X0, ZED_CAMERA_X1, etc.
    std::string ns = node_->get_namespace();
    if (ns.empty() || ns == "/") {
      // Fallback to using the node name if namespace isn't set
      ns = std::string("/") + node_->get_name();
      RCLCPP_WARN(node_->get_logger(), "No namespace set, publishing to '%s'", ns.c_str());
    }
    
    // Remove any trailing slash
    if (ns.back() == '/') {
      ns.pop_back();
    }
    
    RCLCPP_INFO(node_->get_logger(), "Publishing camera topics to namespace: '%s'", ns.c_str());
    
    // Create publishers using namespace and ROS2 standard naming conventions
    rgb_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
      ns + "/rgb/image_rect_color", camera_qos);
    
    depth_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
      ns + "/depth/depth_registered", camera_qos);
    
    point_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      ns + "/point_cloud/cloud_registered", camera_qos);
    
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
      ns + "/IMU", camera_qos);
    
    status_pub_ = node_->create_publisher<std_msgs::msg::String>(
      ns + "/status", camera_qos);
    
    svo2_pub_ = node_->create_publisher<std_msgs::msg::String>(
      ns + "/SVO2", camera_qos);
    
    // Create vehicle speed subscriber
    vehicle_speed_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
      "/vehicle/speed", 10,
      std::bind(&CameraManager::vehicleSpeedCallback, this, std::placeholders::_1));
    
    // Connect to camera
    if (!camera_driver_->connect()) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to connect to ZED camera");
      return false;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Camera manager initialized successfully");
    return true;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception during camera manager initialization: %s", e.what());
    return false;
  }
}

void CameraManager::start() {
  if (running_) {
    RCLCPP_WARN(node_->get_logger(), "Camera manager is already running");
    return;
  }
  
  if (!camera_driver_->isConnected()) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot start camera manager: Camera is not connected");
    return;
  }
  
  running_ = true;
  camera_thread_ = std::thread(&CameraManager::cameraLoop, this);
  RCLCPP_INFO(node_->get_logger(), "Camera manager started");
}

void CameraManager::stop() {
  if (!running_) {
    RCLCPP_WARN(node_->get_logger(), "Camera manager is already stopped");
    return;
  }
  
  running_ = false;
  
  if (camera_thread_.joinable()) {
    camera_thread_.join();
  }
  
  if (camera_driver_->isConnected()) {
    camera_driver_->disconnect();
  }
  
  RCLCPP_INFO(node_->get_logger(), "Camera manager stopped");
}

bool CameraManager::isRunning() const {
  return running_;
}

void CameraManager::cameraLoop() {
  using namespace std::chrono_literals;
  
  // Initialize loop timing variables
  auto last_time = std::chrono::steady_clock::now();
  auto last_update_time = last_time;
  
  while (running_) {
    auto current_time = std::chrono::steady_clock::now();
    last_time = current_time;
    
    try {
      // Update camera settings at regular intervals (100ms)
      auto time_since_update = std::chrono::duration<double>(current_time - last_update_time).count();
      if (time_since_update >= 0.1) {
        updateCameraSettings();
        last_update_time = current_time;
      }
      
      // Publish camera data
      publishCameraData();
      
      // Sleep based on current FPS
      float current_fps = camera_driver_->getCurrentFrameRate();
      if (current_fps > 0.0f) {
        auto sleep_duration = std::chrono::milliseconds(static_cast<int>(1000.0f / current_fps));
        std::this_thread::sleep_for(sleep_duration);
      } else {
        // Default sleep if FPS is unknown or zero
        std::this_thread::sleep_for(100ms);
      }
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(), "Exception in camera loop: %s", e.what());
      std::this_thread::sleep_for(1s);  // Sleep longer to avoid rapidly repeating errors
    }
  }
}

void CameraManager::publishCameraData() {
  if (!camera_driver_->isConnected()) {
    return;
  }
  
  // Get camera status
  auto status = camera_driver_->getStatus();
  
  // Publish status
  auto status_msg = std::make_unique<std_msgs::msg::String>();
  status_msg->data = status.status_message;
  status_pub_->publish(std::move(status_msg));
  
  // Publish RGB image
  sensor_msgs::msg::Image rgb_msg;
  if (camera_driver_->getRgbImage(rgb_msg)) {
    rgb_pub_->publish(rgb_msg);
  }
  
  // Publish depth image
  sensor_msgs::msg::Image depth_msg;
  if (camera_driver_->getDepthImage(depth_msg)) {
    depth_pub_->publish(depth_msg);
  }
  
  // Publish point cloud
  sensor_msgs::msg::PointCloud2 cloud_msg;
  if (camera_driver_->getPointCloud(cloud_msg)) {
    // Make sure we use a valid frame_id that works with RViz2
    std::string ns = node_->get_namespace();
    if (ns.empty() || ns == "/") {
      ns = "camera";
    } else if (ns[0] == '/') {
      ns = ns.substr(1); // Remove leading slash
    }
    
    // Use a simple frame_id that RViz will understand
    cloud_msg.header.frame_id = "map";
    
    RCLCPP_INFO(node_->get_logger(), "Publishing point cloud with %d points (%dx%d) on topic: %s",
               cloud_msg.width * cloud_msg.height, cloud_msg.width, cloud_msg.height,
               point_cloud_pub_->get_topic_name());
    
    point_cloud_pub_->publish(cloud_msg);
  } else {
    RCLCPP_DEBUG(node_->get_logger(), "Failed to get point cloud data from camera");
  }
  
  // Publish IMU data
  auto imu_data = camera_driver_->getImuData();
  auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
  
  // Fill IMU message
  imu_msg->header.stamp = node_->now();
  // Use the frame ID from the config
  imu_msg->header.frame_id = camera_driver_->getFrameId() + "_imu";
  
  // Set orientation
  imu_msg->orientation.x = imu_data.orientation[0];
  imu_msg->orientation.y = imu_data.orientation[1];
  imu_msg->orientation.z = imu_data.orientation[2];
  imu_msg->orientation.w = imu_data.orientation[3];
  
  // Set linear acceleration
  imu_msg->linear_acceleration.x = imu_data.linear_acceleration[0];
  imu_msg->linear_acceleration.y = imu_data.linear_acceleration[1];
  imu_msg->linear_acceleration.z = imu_data.linear_acceleration[2];
  
  // Set angular velocity
  imu_msg->angular_velocity.x = imu_data.angular_velocity[0];
  imu_msg->angular_velocity.y = imu_data.angular_velocity[1];
  imu_msg->angular_velocity.z = imu_data.angular_velocity[2];
  
  // Publish IMU message
  imu_pub_->publish(std::move(imu_msg));
}

void CameraManager::updateCameraSettings() {
  if (!camera_driver_->isConnected()) {
    return;
  }
  
  // Calculate target FPS based on vehicle speed
  float target_fps = calculateTargetFps(vehicle_speed_.load());
  
  // Get current FPS
  float current_fps = camera_driver_->getCurrentFrameRate();
  
  // Calculate error
  float error = target_fps - current_fps;
  
  // Update PID controller (using a fixed time step of 0.1s for simplicity)
  float output_fps = static_cast<float>(fps_pid_controller_->update(error, 0.1));
  
  // Set new FPS
  if (camera_driver_->setFrameRate(output_fps)) {
    RCLCPP_DEBUG(node_->get_logger(), "Updated camera FPS: target=%.1f, current=%.1f, new=%.1f", target_fps, current_fps, output_fps);
  } else {
    RCLCPP_WARN(node_->get_logger(), "Failed to update camera FPS");
  }
}

void CameraManager::vehicleSpeedCallback(const std_msgs::msg::Float32::SharedPtr msg) {
  vehicle_speed_.store(msg->data);
  RCLCPP_DEBUG(node_->get_logger(), "Received vehicle speed: %.2f m/s", msg->data);
}

float CameraManager::calculateTargetFps(float vehicle_speed) {
  // Direct mapping: FPS = vehicle speed (m/s)
  // Constrained to min/max FPS range
  float target_fps = vehicle_speed;
  return std::max(config_->getMinFps(), std::min(config_->getMaxFps(), target_fps));
}

}  // namespace camera
}  // namespace sensors

