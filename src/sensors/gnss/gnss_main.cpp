#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "sensors/gnss/gnss_node.h"

int main(int argc, char** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  
  // Create our node options
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  
  // Create the GNSS node
  auto node = std::make_shared<sensors::gnss::GnssNode>(options);
  
  // Spin the node
  rclcpp::spin(node);
  
  // Shutdown ROS 2
  rclcpp::shutdown();
  
  return 0;
}