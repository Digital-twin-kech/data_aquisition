#ifndef ROSBAG_RECORDER_H
#define ROSBAG_RECORDER_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <filesystem>

namespace data_aquisition {
namespace recording {

struct RecorderConfig {
  std::string output_directory = "/home/user/rosbags";
  std::string bag_prefix = "sync_data";
  bool record_synchronized = true;
  bool record_raw = false;
  std::vector<std::string> camera_names = {"ZED_CAMERA_2i", "ZED_CAMERA_X0", "ZED_CAMERA_X1"};
  bool record_lidar = true;
  bool record_gnss = true;
  bool record_metrics = true;
  std::string compression_mode = "file";  // none, file, message
  size_t split_size = 0;  // in MB, 0 means no splitting
  std::chrono::seconds duration = std::chrono::seconds(0);  // 0 means record until stopped
  int max_bag_files = 0;  // maximum number of bag files to keep (0 means unlimited)
};

class RosbagRecorder : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit RosbagRecorder(const rclcpp::NodeOptions& options);
  virtual ~RosbagRecorder();

protected:
  // Lifecycle node overrides
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

private:
  // Configuration
  RecorderConfig config_;
  
  // Diagnostic updater
  std::unique_ptr<diagnostic_updater::Updater> diagnostics_;
  
  // Timer for statistics publishing
  rclcpp::TimerBase::SharedPtr stats_timer_;
  
  // ROS2 bag recording process management
  std::string recording_process_id_;
  std::string current_bag_path_;
  std::chrono::system_clock::time_point recording_start_time_;
  std::filesystem::space_info disk_space_;
  
  // Statistics
  size_t total_messages_recorded_ = 0;
  size_t total_bytes_recorded_ = 0;
  
  // Methods
  void loadParameters();
  bool startRecording();
  bool stopRecording();
  void checkDiskSpace();
  void manageOldBagFiles();
  void publishStatistics();
  void updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  std::string constructTopicList();
};

}  // namespace recording
}  // namespace data_aquisition

#endif  // ROSBAG_RECORDER_H