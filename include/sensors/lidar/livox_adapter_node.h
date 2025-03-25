/**
 * @file livox_adapter_node.h
 * @brief Adapter node for Livox ROS2 driver
 */

#ifndef LIVOX_ADAPTER_NODE_H
#define LIVOX_ADAPTER_NODE_H

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace sensors {
namespace lidar {

/**
 * @class LivoxAdapterNode
 * @brief Adapter node for connecting to the Livox ROS2 driver
 */
class LivoxAdapterNode : public rclcpp::Node {
public:
  /**
   * @brief Constructor
   * @param options Node options
   */
  explicit LivoxAdapterNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  
  /**
   * @brief Destructor
   */
  ~LivoxAdapterNode();

private:
  /**
   * @brief Initialize the node parameters
   */
  void initializeParameters();
  
  /**
   * @brief Initialize node subscribers and publishers
   */
  void initializeTopics();
  
  /**
   * @brief Handle point cloud data from Livox driver
   * @param msg Incoming point cloud message
   */
  void handlePointCloudData(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  
  /**
   * @brief Handle IMU data from Livox driver
   * @param msg Incoming IMU message
   */
  void handleImuData(const sensor_msgs::msg::Imu::SharedPtr msg);

  // ROS2 subscribers and publishers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  
  // Parameters
  std::string input_point_cloud_topic_;
  std::string input_imu_topic_;
  std::string output_point_cloud_topic_;
  std::string output_imu_topic_;
  std::string frame_id_;
  bool filter_points_;
  float min_distance_;
  float max_distance_;
  int downsample_factor_;
};

} // namespace lidar
} // namespace sensors

#endif // LIVOX_ADAPTER_NODE_H