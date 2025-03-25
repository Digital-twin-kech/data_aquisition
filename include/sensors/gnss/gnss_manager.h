#ifndef GNSS_MANAGER_H
#define GNSS_MANAGER_H

#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <vector>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
// #include <diagnostic_msgs/msg/diagnostic_status.hpp> // Removed to avoid conflicts
// #include <rtcm_msgs/msg/message.hpp> // Removed to avoid conflicts
#include "sensors/gnss/gnss_config.h"
#include "sensors/gnss/gnss_driver.h"
#include "sensors/gnss/ntrip_client.h"
#include "sensors/gnss/rtk_processor.h"

namespace sensors {
namespace gnss {

/**
 * @brief Manages GNSS operation and ROS2 integration
 * 
 * This class handles the GNSS lifecycle, data publishing, and dynamic
 * configuration of GNSS settings. It also manages RTCM corrections
 * if enabled.
 */
class GnssManager {
public:
  /**
   * @brief Construct a new GNSS Manager object
   * 
   * @param node ROS2 node
   * @param config GNSS configuration
   */
  GnssManager(rclcpp::Node::SharedPtr node, std::shared_ptr<GnssConfig> config);
  
  /**
   * @brief Destructor
   */
  ~GnssManager();
  
  /**
   * @brief Initialize the GNSS and publishers
   * 
   * @return true if initialization was successful
   * @return false if initialization failed
   */
  bool initialize();
  
  /**
   * @brief Start the GNSS operation
   */
  void start();
  
  /**
   * @brief Stop the GNSS operation
   */
  void stop();
  
  /**
   * @brief Check if the GNSS manager is running
   * 
   * @return true if running
   * @return false if not running
   */
  bool isRunning() const;

  /**
   * @brief Log GNSS status and metrics in JSON format
   * 
   * @param status GNSS status to log
   */
  void logStatusJson(const GnssDriver::GnssStatus& status);

private:
  // ROS2 node
  rclcpp::Node::SharedPtr node_;
  
  // Configuration
  std::shared_ptr<GnssConfig> config_;
  
  // GNSS driver
  std::unique_ptr<GnssDriver> gnss_driver_;
  
  // NTRIP client for RTK corrections (if enabled)
  std::unique_ptr<NtripClient> ntrip_client_;
  
  // RTK processor for correction processing
  std::unique_ptr<RtkProcessor> rtk_processor_;
  
  // Running state
  std::atomic<bool> running_;
  
  // GNSS operation thread
  std::thread gnss_thread_;
  std::mutex gnss_mutex_;
  
  // Watchdog timer
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  std::chrono::steady_clock::time_point last_data_time_;
  
  // Raw data buffer
  std::vector<uint8_t> raw_data_buffer_;
  
  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr time_ref_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_pub_;
  // rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnostic_pub_; // Removed to avoid conflicts
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  
  // Subscribers
  // rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr rtcm_sub_; // Removed to avoid conflicts
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_sub_;
  
  // Log file
  std::ofstream log_file_;
  
  // GNSS operation functions
  void gnssLoop();
  void publishGnssData();
  void publishOdometry();
  void publishDiagnostics();
  void handleRtcmCorrections();
  // void rtcmCallback(const rtcm_msgs::msg::Message::SharedPtr msg); // Removed to avoid conflicts
  void controlCallback(const std_msgs::msg::String::SharedPtr msg);
  void watchdogCallback();
  
  // Coordinate conversion functions
  bool convertNavSatToLocalPose(
    const sensor_msgs::msg::NavSatFix& nav_fix,
    geometry_msgs::msg::PoseWithCovarianceStamped& pose);
    
  bool initializeLocalOrigin(double lat, double lon, double alt);
  
  // GNSS data storage for local origin
  struct {
    bool initialized;
    double latitude;
    double longitude;
    double altitude;
    double north_offset;
    double east_offset;
  } local_origin_;
  
  // Utility functions
  std::string getTimeString();
};

}  // namespace gnss
}  // namespace sensors

#endif  // GNSS_MANAGER_H