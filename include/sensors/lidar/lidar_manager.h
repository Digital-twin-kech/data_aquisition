/**
 * @file lidar_manager.h
 * @brief Manager class for LiDAR sensors
 */

#ifndef LIDAR_MANAGER_H
#define LIDAR_MANAGER_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

#include "sensors/lidar/lidar_config.h"
#include "sensors/lidar/lidar_driver.h"

namespace sensors {
namespace lidar {

/**
 * @class LidarManager
 * @brief Manages LiDAR hardware and ROS2 interfaces
 */
class LidarManager {
public:
  /**
   * @brief Constructor
   * @param node ROS2 node
   * @param config LiDAR configuration
   */
  LidarManager(
    const rclcpp::Node::SharedPtr& node, 
    const LidarConfigPtr& config);
  
  /**
   * @brief Destructor
   */
  ~LidarManager();
  
  /**
   * @brief Initialize the manager
   * @return True if successful
   */
  bool initialize();
  
  /**
   * @brief Start the manager
   * @return True if successful
   */
  bool start();
  
  /**
   * @brief Stop the manager
   */
  void stop();
  
  /**
   * @brief Check if the manager is running
   * @return True if running
   */
  bool isRunning() const;

private:
  /**
   * @brief Handle point cloud data from the driver
   * @param msg Point cloud message
   */
  void handlePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  
  /**
   * @brief Handle IMU data from the driver
   * @param msg IMU message
   */
  void handleImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  
  /**
   * @brief Publish LiDAR status information
   */
  void publishStatus();
  
  /**
   * @brief Timer callback for status publication
   */
  void statusTimerCallback();

  rclcpp::Node::SharedPtr node_;
  LidarConfigPtr config_;
  LidarDriverPtr driver_;
  bool running_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  // Timer for status publishing
  rclcpp::TimerBase::SharedPtr status_timer_;
};

using LidarManagerPtr = std::shared_ptr<LidarManager>;

} // namespace lidar
} // namespace sensors

#endif // LIDAR_MANAGER_H