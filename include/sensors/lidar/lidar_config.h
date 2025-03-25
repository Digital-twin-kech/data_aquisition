/**
 * @file lidar_config.h
 * @brief Configuration class for LiDAR sensors
 */

#ifndef LIDAR_CONFIG_H
#define LIDAR_CONFIG_H

#include <string>
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace sensors {
namespace lidar {

/**
 * @class LidarConfig
 * @brief Handles configuration parameters for LiDAR sensors
 */
class LidarConfig {
public:
  /**
   * @brief Constructor
   * @param node ROS2 node to load parameters from
   */
  explicit LidarConfig(const rclcpp::Node::SharedPtr& node);
  
  /**
   * @brief Destructor
   */
  ~LidarConfig() = default;
  
  /**
   * @brief Get the node namespace
   * @return The node namespace
   */
  std::string getNamespace() const;

  /**
   * @brief Get the frame ID for this lidar
   * @return The frame ID
   */
  std::string getFrameId() const;
  
  /**
   * @brief Get the IP address of the LiDAR
   * @return The IP address
   */
  std::string getIpAddress() const;
  
  /**
   * @brief Get the data port for LiDAR
   * @return The data port
   */
  int getDataPort() const;
  
  /**
   * @brief Get the command port for LiDAR
   * @return The command port
   */
  int getCommandPort() const;
  
  /**
   * @brief Get the IMU data port for LiDAR
   * @return The IMU data port
   */
  int getImuPort() const;

  /**
   * @brief Check if using DHCP
   * @return True if using DHCP, false for static IP
   */
  bool useDhcp() const;
  
  /**
   * @brief Get the host IP address
   * @return The host IP address
   */
  std::string getHostIpAddress() const;
  
  /**
   * @brief Get the publish frequency
   * @return The publish frequency in Hz
   */
  double getPublishFrequency() const;
  
  /**
   * @brief Get the point cloud type
   * @return The point cloud type
   */
  int getPointCloudType() const;

  /**
   * @brief Get the broadcast code
   * @return The broadcast code
   */
  std::string getBroadcastCode() const;

  /**
   * @brief Get QoS settings reliability policy
   * @return True for reliable, false for best effort
   */
  bool getQosReliability() const;

  /**
   * @brief Get QoS history depth
   * @return The history depth for QoS settings
   */
  int getQosHistoryDepth() const;

private:
  rclcpp::Node::SharedPtr node_;
  std::string frame_id_;
  std::string ip_address_;
  int data_port_;
  int command_port_;
  int imu_port_;
  bool use_dhcp_;
  std::string host_ip_address_;
  double publish_frequency_;
  int point_cloud_type_;
  std::string broadcast_code_;
  bool qos_reliability_;
  int qos_history_depth_;
};

using LidarConfigPtr = std::shared_ptr<LidarConfig>;

} // namespace lidar
} // namespace sensors

#endif // LIDAR_CONFIG_H