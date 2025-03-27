#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "sync/sensor_synchronizer.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions options;
  auto node = std::make_shared<data_aquisition::sync::SensorSynchronizer>(options);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}