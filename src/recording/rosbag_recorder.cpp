#include "recording/rosbag_recorder.h"

#include <boost/process.hpp>
#include <boost/algorithm/string.hpp>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <algorithm>
#include <regex>

namespace bp = boost::process;
namespace fs = std::filesystem;

namespace data_aquisition {
namespace recording {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

RosbagRecorder::RosbagRecorder(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("rosbag_recorder", options)
{
  RCLCPP_INFO(get_logger(), "RosbagRecorder constructor");
}

RosbagRecorder::~RosbagRecorder() {
  if (!recording_process_id_.empty()) {
    RCLCPP_INFO(get_logger(), "Stopping active recording process before destruction");
    stopRecording();
  }
}

void RosbagRecorder::loadParameters() {
  // Load parameters with the same pattern used in the rest of the codebase
  config_.output_directory = declare_parameter("recorder.output_directory", config_.output_directory);
  config_.bag_prefix = declare_parameter("recorder.bag_prefix", config_.bag_prefix);
  config_.record_synchronized = declare_parameter("recorder.record_synchronized", config_.record_synchronized);
  config_.record_raw = declare_parameter("recorder.record_raw", config_.record_raw);
  config_.record_lidar = declare_parameter("recorder.record_lidar", config_.record_lidar);
  config_.record_gnss = declare_parameter("recorder.record_gnss", config_.record_gnss);
  config_.record_metrics = declare_parameter("recorder.record_metrics", config_.record_metrics);
  config_.compression_mode = declare_parameter("recorder.compression_mode", config_.compression_mode);
  config_.split_size = declare_parameter("recorder.split_size", static_cast<int>(config_.split_size));
  config_.duration = std::chrono::seconds(declare_parameter("recorder.duration", 0));
  config_.max_bag_files = declare_parameter("recorder.max_bag_files", config_.max_bag_files);
  
  // Get camera names parameter
  std::vector<std::string> camera_names = declare_parameter(
    "recorder.camera_names", 
    std::vector<std::string>{"ZED_CAMERA_2i", "ZED_CAMERA_X0", "ZED_CAMERA_X1"});
  config_.camera_names = camera_names;
  
  RCLCPP_INFO(get_logger(), "Configured recorder with output_directory: %s", config_.output_directory.c_str());
}

CallbackReturn RosbagRecorder::on_configure(const rclcpp_lifecycle::State& previous_state) {
  RCLCPP_INFO(get_logger(), "Configuring RosbagRecorder");
  
  // Load parameters
  loadParameters();
  
  // Create output directory if it doesn't exist
  try {
    if (!fs::exists(config_.output_directory)) {
      fs::create_directories(config_.output_directory);
      RCLCPP_INFO(get_logger(), "Created output directory: %s", config_.output_directory.c_str());
    }
  } catch (const fs::filesystem_error& e) {
    RCLCPP_ERROR(get_logger(), "Error creating output directory: %s", e.what());
    return CallbackReturn::FAILURE;
  }
  
  // Setup diagnostics
  diagnostics_ = std::make_unique<diagnostic_updater::Updater>(this);
  diagnostics_->setHardwareID("rosbag_recorder");
  diagnostics_->add("Recorder Status", std::bind(&RosbagRecorder::updateDiagnostics, this, std::placeholders::_1));
  
  // Setup statistics timer (runs at 1Hz)
  stats_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&RosbagRecorder::publishStatistics, this)
  );
  stats_timer_->cancel(); // Start deactivated
  
  RCLCPP_INFO(get_logger(), "RosbagRecorder configured successfully");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RosbagRecorder::on_activate(const rclcpp_lifecycle::State& previous_state) {
  RCLCPP_INFO(get_logger(), "Activating RosbagRecorder");
  
  // Check disk space before starting
  checkDiskSpace();
  if (disk_space_.available < 1024 * 1024 * 100) { // 100MB minimum
    RCLCPP_ERROR(get_logger(), "Insufficient disk space for recording");
    return CallbackReturn::FAILURE;
  }
  
  // Start the rosbag recording
  if (!startRecording()) {
    RCLCPP_ERROR(get_logger(), "Failed to start ROS2 bag recording");
    return CallbackReturn::FAILURE;
  }
  
  // Start the statistics timer
  stats_timer_->reset();
  
  // Clean up old bag files if configured
  if (config_.max_bag_files > 0) {
    manageOldBagFiles();
  }
  
  RCLCPP_INFO(get_logger(), "RosbagRecorder activated successfully");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RosbagRecorder::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
  RCLCPP_INFO(get_logger(), "Deactivating RosbagRecorder");
  
  // Stop the recording
  if (!stopRecording()) {
    RCLCPP_WARN(get_logger(), "Failed to stop ROS2 bag recording cleanly");
  }
  
  // Stop the statistics timer
  stats_timer_->cancel();
  
  RCLCPP_INFO(get_logger(), "RosbagRecorder deactivated successfully");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RosbagRecorder::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
  RCLCPP_INFO(get_logger(), "Cleaning up RosbagRecorder");
  
  // Ensure recording is stopped
  stopRecording();
  
  // Clear diagnostic updater
  diagnostics_.reset();
  
  RCLCPP_INFO(get_logger(), "RosbagRecorder cleaned up successfully");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RosbagRecorder::on_shutdown(const rclcpp_lifecycle::State& previous_state) {
  RCLCPP_INFO(get_logger(), "Shutting down RosbagRecorder");
  
  // Ensure recording is stopped
  stopRecording();
  
  RCLCPP_INFO(get_logger(), "RosbagRecorder shut down successfully");
  return CallbackReturn::SUCCESS;
}

void RosbagRecorder::checkDiskSpace() {
  try {
    disk_space_ = fs::space(config_.output_directory);
    RCLCPP_DEBUG(get_logger(), "Disk space: %lu bytes available", disk_space_.available);
  } catch (const fs::filesystem_error& e) {
    RCLCPP_ERROR(get_logger(), "Error checking disk space: %s", e.what());
  }
}

void RosbagRecorder::manageOldBagFiles() {
  try {
    // Get list of bag directories in the output directory
    std::vector<fs::path> bag_dirs;
    for (const auto& entry : fs::directory_iterator(config_.output_directory)) {
      if (entry.is_directory() && entry.path().filename().string().find(config_.bag_prefix) == 0) {
        bag_dirs.push_back(entry.path());
      }
    }
    
    // Sort by modification time (oldest first)
    std::sort(bag_dirs.begin(), bag_dirs.end(), 
      [](const fs::path& a, const fs::path& b) {
        return fs::last_write_time(a) < fs::last_write_time(b);
      });
    
    // Remove oldest bag directories if over the limit
    while (bag_dirs.size() > static_cast<size_t>(config_.max_bag_files)) {
      fs::path oldest = bag_dirs.front();
      RCLCPP_INFO(get_logger(), "Removing old bag directory: %s", oldest.string().c_str());
      fs::remove_all(oldest);
      bag_dirs.erase(bag_dirs.begin());
    }
  } catch (const fs::filesystem_error& e) {
    RCLCPP_ERROR(get_logger(), "Error managing old bag files: %s", e.what());
  }
}

bool RosbagRecorder::startRecording() {
  if (!recording_process_id_.empty()) {
    RCLCPP_WARN(get_logger(), "Recording already in progress");
    return false;
  }
  
  // Generate timestamp for bag name
  auto now = std::chrono::system_clock::now();
  auto now_time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << config_.bag_prefix << "_" 
     << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d-%H-%M-%S");
  std::string timestamp_str = ss.str();
  
  // Create full path for the bag
  current_bag_path_ = config_.output_directory + "/" + timestamp_str;
  
  // Construct command to start ROS2 bag recording
  std::string cmd = "ros2 bag record ";
  
  // Add options
  if (config_.compression_mode == "file") {
    cmd += "--storage sqlite --serialization-format cdr ";
  } else if (config_.compression_mode == "message") {
    cmd += "--compression-mode message ";
  } else if (config_.compression_mode == "none") {
    cmd += "--no-discovery ";
  }
  
  // Add split size if specified
  if (config_.split_size > 0) {
    cmd += "--max-bag-size " + std::to_string(config_.split_size) + " ";
  }
  
  // Add duration if specified
  if (config_.duration.count() > 0) {
    cmd += "--max-duration " + std::to_string(config_.duration.count()) + " ";
  }
  
  // Add output directory
  cmd += "-o " + current_bag_path_ + " ";
  
  // Add topics
  cmd += constructTopicList();
  
  // Execute the command using boost::process
  try {
    RCLCPP_INFO(get_logger(), "Starting ROS2 bag recording with command: %s", cmd.c_str());
    bp::child recorder_process(cmd, bp::std_out > bp::null, bp::std_err > bp::null, bp::std_in < bp::null);
    recording_process_id_ = std::to_string(recorder_process.id());
    recording_start_time_ = std::chrono::system_clock::now();
    RCLCPP_INFO(get_logger(), "ROS2 bag recording started with PID: %s", recording_process_id_.c_str());
    recorder_process.detach();
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Error starting ROS2 bag recording: %s", e.what());
    recording_process_id_ = "";
    current_bag_path_ = "";
    return false;
  }
}

bool RosbagRecorder::stopRecording() {
  if (recording_process_id_.empty()) {
    RCLCPP_WARN(get_logger(), "No recording in progress");
    return false;
  }
  
  try {
    // Use boost::process to send SIGINT to the recording process
    std::string kill_cmd = "kill -SIGINT " + recording_process_id_;
    bp::system(kill_cmd);
    RCLCPP_INFO(get_logger(), "Sent SIGINT to recording process %s", recording_process_id_.c_str());
    
    // Reset process ID and bag path
    recording_process_id_ = "";
    
    auto duration = std::chrono::system_clock::now() - recording_start_time_;
    RCLCPP_INFO(get_logger(), "Recording stopped. Duration: %.2f seconds. Bag saved to: %s", 
      std::chrono::duration<double>(duration).count(), current_bag_path_.c_str());
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Error stopping ROS2 bag recording: %s", e.what());
    recording_process_id_ = "";
    return false;
  }
}

void RosbagRecorder::publishStatistics() {
  // Update diagnostics - using force_update() which is public
  diagnostics_->force_update();
  
  // Check disk space periodically
  checkDiskSpace();
  
  // If disk space is low, automatically stop recording
  if (disk_space_.available < 1024 * 1024 * 50) { // 50MB threshold
    RCLCPP_WARN(get_logger(), "Low disk space! Stopping recording");
    stopRecording();
  }
}

void RosbagRecorder::updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  if (recording_process_id_.empty()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Recorder inactive");
  } else {
    auto duration = std::chrono::system_clock::now() - recording_start_time_;
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
    
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Recording in progress");
    stat.add("Recording PID", recording_process_id_);
    stat.add("Bag path", current_bag_path_);
    stat.add("Duration (seconds)", seconds);
    
    // Add disk space information
    try {
      disk_space_ = fs::space(config_.output_directory);
      stat.add("Disk space available (MB)", disk_space_.available / (1024 * 1024));
      stat.add("Disk space total (MB)", disk_space_.capacity / (1024 * 1024));
      
      // Change status level if disk space is low
      if (disk_space_.available < 1024 * 1024 * 100) { // 100MB threshold
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Low disk space");
      }
    } catch (const fs::filesystem_error& e) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Failed to check disk space");
    }
  }
}

std::string RosbagRecorder::constructTopicList() {
  std::vector<std::string> topics;
  
  if (config_.record_synchronized) {
    // Add synchronized camera topics for each configured camera
    for (const auto& camera_name : config_.camera_names) {
      topics.push_back("/synchronized/" + camera_name + "/rgb/image_rect_color");
      topics.push_back("/synchronized/" + camera_name + "/point_cloud/cloud_registered");
      topics.push_back("/synchronized/" + camera_name + "/IMU");
    }
    
    // Add synchronized LiDAR and GNSS topics
    if (config_.record_lidar) {
      topics.push_back("/synchronized/livox/lidar");
    }
    
    if (config_.record_gnss) {
      topics.push_back("/synchronized/gnss/fix");
    }
  }
  
  if (config_.record_raw) {
    // Add raw camera topics for each configured camera
    for (const auto& camera_name : config_.camera_names) {
      topics.push_back("/" + camera_name + "/rgb/image_rect_color");
      topics.push_back("/" + camera_name + "/point_cloud/cloud_registered");
      topics.push_back("/" + camera_name + "/IMU");
    }
    
    // Add raw LiDAR and GNSS topics
    if (config_.record_lidar) {
      topics.push_back("/livox/lidar");
    }
    
    if (config_.record_gnss) {
      topics.push_back("/gnss/fix");
    }
  }
  
  // Add sync metrics if configured
  if (config_.record_metrics) {
    topics.push_back("/sync/metrics");
  }
  
  // Add TF topics by default
  topics.push_back("/tf");
  topics.push_back("/tf_static");
  
  // Join topics with spaces
  std::string topic_list = "";
  for (const auto& topic : topics) {
    topic_list += topic + " ";
  }
  
  return topic_list;
}

}  // namespace recording
}  // namespace data_aquisition