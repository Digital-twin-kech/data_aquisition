#include "recording/rosbag_recorder.h"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>

int main(int argc, char** argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  
  // Set logging level
  auto logger = rclcpp::get_logger("rosbag_recorder_main");
  RCLCPP_INFO(logger, "Starting RosbagRecorder node");
  
  // Create node options
  rclcpp::NodeOptions options;
  
  // Create recorder node
  auto recorder_node = std::make_shared<data_aquisition::recording::RosbagRecorder>(options);
  
  // Create executor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(recorder_node->get_node_base_interface());
  
  // Start a separate thread to run the executor
  std::thread executor_thread([&executor]() {
    executor.spin();
  });
  
  // Wait for node to be ready
  std::this_thread::sleep_for(std::chrono::seconds(1));
  
  // Configure the node
  recorder_node->configure();
  
  // Auto-activate if requested via command line parameter
  if (argc > 1 && std::string(argv[1]) == "--auto-activate") {
    RCLCPP_INFO(logger, "Auto-activating recorder node");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    recorder_node->activate();
  } else {
    RCLCPP_INFO(logger, "Node in INACTIVE state. Activate with ROS2 lifecycle commands:");
    RCLCPP_INFO(logger, "ros2 lifecycle set /rosbag_recorder configure");
    RCLCPP_INFO(logger, "ros2 lifecycle set /rosbag_recorder activate");
  }
  
  // Wait for the executor thread (will never return unless interrupted)
  executor_thread.join();
  
  // Shutdown
  rclcpp::shutdown();
  return 0;
}