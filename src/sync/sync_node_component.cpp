#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "sync/sensor_synchronizer.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(data_aquisition::sync::SensorSynchronizer)