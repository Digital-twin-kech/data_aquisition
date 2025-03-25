#ifndef GNSS_NODE_H
#define GNSS_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "sensors/gnss/gnss_config.h"
#include "sensors/gnss/gnss_manager.h"
#include <memory>

namespace sensors {
namespace gnss {

/**
 * @brief ROS2 node for u-blox F9P GNSS receiver
 * 
 * This class implements a ROS2 component node that manages a GNSS receiver,
 * publishes sensor data, and handles configuration.
 */
class GnssNode : public rclcpp::Node {
public:
  /**
   * @brief Construct a new GNSS Node object
   * 
   * @param options ROS2 node options
   */
  explicit GnssNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  
  /**
   * @brief Destructor
   */
  virtual ~GnssNode();

  /**
   * @brief Get the name of the node
   * 
   * @return std::string Node name
   */
  std::string getName() const;

private:
  /**
   * @brief Initialize the GNSS node
   * 
   * Creates configuration and GNSS manager, and starts data acquisition.
   * This is called after construction via a timer to ensure shared_from_this() works.
   */
  void initialize();

  // GNSS configuration
  std::shared_ptr<GnssConfig> config_;
  
  // GNSS manager
  std::unique_ptr<GnssManager> gnss_manager_;
  
  // Initialization timer
  rclcpp::TimerBase::SharedPtr initialize_timer_;
  
  // Parameter change callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
  
  // Node state
  bool initialized_;
  
  // CUDA/GPU validation
  bool validateJetsonEnvironment();
};

}  // namespace gnss
}  // namespace sensors

#endif  // GNSS_NODE_H