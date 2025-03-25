/**
 * @file lidar_driver.h
 * @brief Driver class for LiDAR sensors, primarily Livox
 */

#ifndef LIDAR_DRIVER_H
#define LIDAR_DRIVER_H

#include <string>
#include <memory>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include <functional>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensors/lidar/lidar_config.h"

namespace sensors {
namespace lidar {

/**
 * @class LidarDriver
 * @brief Interface to Livox LiDAR hardware
 */
class LidarDriver {
public:
  /**
   * @brief Constructor
   * @param config Configuration parameters
   */
  explicit LidarDriver(const LidarConfigPtr& config);
  
  /**
   * @brief Destructor
   */
  ~LidarDriver();
  
  /**
   * @brief Initialize the driver
   * @return True if initialization was successful
   */
  bool initialize();
  
  /**
   * @brief Connect to the LiDAR
   * @return True if connection was successful
   */
  bool connect();
  
  /**
   * @brief Disconnect from the LiDAR
   */
  void disconnect();
  
  /**
   * @brief Check if connected to the LiDAR
   * @return True if connected
   */
  bool isConnected() const;

  /**
   * @brief Start data acquisition
   * @return True if successful
   */
  bool start();
  
  /**
   * @brief Stop data acquisition
   */
  void stop();
  
  /**
   * @brief Check if acquisition is running
   * @return True if running
   */
  bool isRunning() const;

  /**
   * @brief Set callback for point cloud data
   * @param callback Function to call when point cloud data is available
   */
  void setPointCloudCallback(std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr)> callback);
  
  /**
   * @brief Set callback for IMU data
   * @param callback Function to call when IMU data is available
   */
  void setImuCallback(std::function<void(const sensor_msgs::msg::Imu::SharedPtr)> callback);

private:
  /**
   * @brief Thread function for handling callbacks
   */
  void callbackThread();

  /**
   * @brief Configure LiDAR parameters
   * @return True if successful
   */
  bool configureLidar();

  /**
   * @brief Create JSON configuration
   * @return True if successful
   */
  bool createJsonConfig();

  LidarConfigPtr config_;
  std::atomic<bool> connected_;
  std::atomic<bool> running_;
  std::mutex mutex_;
  
  // Callbacks
  std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr)> point_cloud_callback_;
  std::function<void(const sensor_msgs::msg::Imu::SharedPtr)> imu_callback_;
  
  // Threading
  std::thread callback_thread_;
  std::atomic<bool> exit_thread_;
  
  // For future use with direct Livox SDK integration
  std::string json_config_path_;
  void* sdk_handle_;
};

using LidarDriverPtr = std::shared_ptr<LidarDriver>;

} // namespace lidar
} // namespace sensors

#endif // LIDAR_DRIVER_H