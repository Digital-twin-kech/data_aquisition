/**
 * @file livox_lidar_node.h
 * @brief ROS2 node for Livox LiDAR
 */

#ifndef LIVOX_LIDAR_NODE_H
#define LIVOX_LIDAR_NODE_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include "sensors/lidar/lidar_config.h"
#include "sensors/lidar/lidar_manager.h"

namespace sensors {
namespace lidar {

/**
 * @class LivoxLidarNode
 * @brief ROS2 lifecycle node for Livox LiDAR
 */
class LivoxLidarNode : public rclcpp_lifecycle::LifecycleNode {
public:
  /**
   * @brief Constructor
   * @param options Node options
   */
  explicit LivoxLidarNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  
  /**
   * @brief Destructor
   */
  ~LivoxLidarNode();

  /**
   * @brief Configure callback for lifecycle node
   * @param state Previous state
   * @return CallbackReturn result
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_configure(const rclcpp_lifecycle::State& state);

  /**
   * @brief Activate callback for lifecycle node
   * @param state Previous state
   * @return CallbackReturn result
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_activate(const rclcpp_lifecycle::State& state);

  /**
   * @brief Deactivate callback for lifecycle node
   * @param state Previous state
   * @return CallbackReturn result
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_deactivate(const rclcpp_lifecycle::State& state);

  /**
   * @brief Cleanup callback for lifecycle node
   * @param state Previous state
   * @return CallbackReturn result
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_cleanup(const rclcpp_lifecycle::State& state);

  /**
   * @brief Shutdown callback for lifecycle node
   * @param state Previous state
   * @return CallbackReturn result
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_shutdown(const rclcpp_lifecycle::State& state);

private:
  rclcpp::Node::SharedPtr private_node_;
  LidarConfigPtr config_;
  LidarManagerPtr manager_;
};

} // namespace lidar
} // namespace sensors

#endif // LIVOX_LIDAR_NODE_H