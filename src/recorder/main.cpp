#include "data_recorder_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::string output_dir = "/home/user/Desktop/data-aquisition-digital-twin/data_aquisition/records";
    auto recorder_node = std::make_shared<DataRecorderNode>(output_dir);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(recorder_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}