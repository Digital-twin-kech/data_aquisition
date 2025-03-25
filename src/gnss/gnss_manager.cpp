#include "sensors/gnss/gnss_manager.h"
#include <chrono>
#include <iomanip>
#include <sstream>
// Remove dependency on nlohmann/json.hpp
#include <string>

// Define a simple JSON string builder class instead of nlohmann/json
class SimpleJson {
public:
  SimpleJson() : json_str_("{") {}
  ~SimpleJson() {}
  
  void add(const std::string& key, const std::string& value) {
    if (json_str_.length() > 1) json_str_ += ",";
    json_str_ += "\"" + key + "\":\"" + value + "\"";
  }
  
  void add(const std::string& key, int value) {
    if (json_str_.length() > 1) json_str_ += ",";
    json_str_ += "\"" + key + "\":" + std::to_string(value);
  }
  
  void add(const std::string& key, double value) {
    if (json_str_.length() > 1) json_str_ += ",";
    json_str_ += "\"" + key + "\":" + std::to_string(value);
  }
  
  void add(const std::string& key, bool value) {
    if (json_str_.length() > 1) json_str_ += ",";
    json_str_ += "\"" + key + "\":" + (value ? "true" : "false");
  }
  
  std::string dump() {
    return json_str_ + "}";
  }
  
private:
  std::string json_str_;
};
using namespace std::chrono_literals;

namespace sensors {
namespace gnss {

GnssManager::GnssManager(rclcpp::Node::SharedPtr node, std::shared_ptr<GnssConfig> config)
  : node_(node),
    config_(config),
    running_(false),
    local_origin_({false, 0.0, 0.0, 0.0, 0.0, 0.0})
{
  // Create the log file with timestamp in filename
  std::string timestamp = getTimeString();
  std::string log_filename = "/tmp/gnss_" + timestamp + ".jsonl";
  log_file_.open(log_filename, std::ios::out | std::ios::app);
  
  if (!log_file_.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open GNSS log file: %s", log_filename.c_str());
  } else {
    RCLCPP_INFO(node_->get_logger(), "GNSS logging to: %s", log_filename.c_str());
  }
}

GnssManager::~GnssManager()
{
  if (running_) {
    stop();
  }
  
  if (log_file_.is_open()) {
    log_file_.close();
  }
}

bool GnssManager::initialize()
{
  // Create QoS profile for reliable communication
  rclcpp::QoS qos_profile(config_->getQosHistoryDepth());
  qos_profile.reliability(config_->getReliabilityPolicy());
  qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
  qos_profile.history(rclcpp::HistoryPolicy::KeepLast); // Changed from KeepAll to KeepLast for intraprocess comm
  
  // Create publishers
  fix_pub_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>(
    "/gnss/fix", qos_profile);
  
  velocity_pub_ = node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "/gnss/vel", qos_profile);
  
  status_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/gnss/status", qos_profile);
  
  time_ref_pub_ = node_->create_publisher<sensor_msgs::msg::TimeReference>(
    "/gnss/time_reference", qos_profile);
  
  raw_pub_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>(
    "/gnss/raw", qos_profile);
  
  // // Create diagnostic publisher - Removed to avoid conflicts
  // diagnostic_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
  //   "/gnss/status", qos_profile);
    
  // Create pose and odometry publishers for robot_localization integration
  pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/gnss/pose", qos_profile);
    
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
    "/gnss/odom", qos_profile);
  
  // Create RTCM subscription if corrections are enabled - Removed to avoid conflicts
  // if (config_->useRtcmCorrections() && config_->getRtcmSource() == "ROS") {
  //   rtcm_sub_ = node_->create_subscription<rtcm_msgs::msg::Message>(
  //     "/gnss/rtcm", qos_profile,
  //     std::bind(&GnssManager::rtcmCallback, this, std::placeholders::_1));
  //   
  //   RCLCPP_INFO(node_->get_logger(), "Subscribed to RTCM corrections via ROS topic");
  // }
  
  // Create control subscription
  control_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/sync/control", qos_profile,
    std::bind(&GnssManager::controlCallback, this, std::placeholders::_1));
    
  RCLCPP_INFO(node_->get_logger(), "Subscribed to control messages");  
  
  // Create GNSS driver
  gnss_driver_ = std::make_unique<GnssDriver>(config_);
  
  // Connect to GNSS receiver
  if (!gnss_driver_->connect()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to connect to GNSS receiver");
    return false;
  }
  
  // Only create RTK processor if RTCM corrections are enabled
  if (config_->useRtcmCorrections()) {
    rtk_processor_ = std::make_unique<RtkProcessor>(node_->get_logger());
    
    // Initialize RTK processor with base position (if available)
    std::string rtk_config_file = "";  // Default to no config file
    
    // Base position for RTK processing (approximation)
    std::string base_position = "35.6895,139.6917,40.0";  // Default Tokyo coordinates
    
    if (rtk_processor_->initialize(base_position, rtk_config_file)) {
      RCLCPP_INFO(node_->get_logger(), "RTK processor initialized");
      
      // Set callback for processed RTK data
      rtk_processor_->setRtcmCallback(
        [this](const std::vector<uint8_t>& data) {
          if (this->gnss_driver_) {
            this->gnss_driver_->sendRtcmCorrection(data);
          }
        });
    } else {
      RCLCPP_WARN(node_->get_logger(), "Failed to initialize RTK processor");
    }
  } else {
    RCLCPP_INFO(node_->get_logger(), "RTK processor disabled - RTCM corrections not enabled");
  }
  
  // Create NTRIP client if configured and we have a valid server
  if (config_->useRtcmCorrections() && 
      config_->getRtcmSource() == "NTRIP" && 
      !config_->getNtripServer().empty()) {
    
    // Parse NTRIP server string (format: hostname:port/mountpoint)
    std::string server = config_->getNtripServer();
    std::string host, mountpoint;
    int port = 2101;  // Default NTRIP port
    
    size_t pos = server.find(":");
    if (pos != std::string::npos) {
      host = server.substr(0, pos);
      std::string remainder = server.substr(pos + 1);
      
      pos = remainder.find("/");
      if (pos != std::string::npos) {
        port = std::stoi(remainder.substr(0, pos));
        mountpoint = remainder.substr(pos + 1);
      }
    }
    
    if (!host.empty() && !mountpoint.empty()) {
      ntrip_client_ = std::make_unique<NtripClient>(
        host, port, mountpoint,
        "", "",  // Username and password placeholders
        node_->get_logger());
      
      // Set callback for RTCM data to go through RTK processor
      ntrip_client_->setRtcmCallback(
        [this](const std::vector<uint8_t>& data) {
          if (this->rtk_processor_) {
            this->rtk_processor_->processRtcmData(data);
          } else if (this->gnss_driver_) {
            // Fallback if RTK processor not initialized
            this->gnss_driver_->sendRtcmCorrection(data);
          }
        });
      
      RCLCPP_INFO(node_->get_logger(), "NTRIP client created for server: %s", server.c_str());
    } else {
      RCLCPP_WARN(node_->get_logger(), "Invalid NTRIP server format. Should be hostname:port/mountpoint");
    }
  } else if (config_->useRtcmCorrections() && config_->getRtcmSource() == "NTRIP") {
    RCLCPP_WARN(node_->get_logger(), "NTRIP corrections enabled but no server specified. NTRIP disabled.");
  }
  
  // Create watchdog timer
  watchdog_timer_ = node_->create_wall_timer(
    1s, std::bind(&GnssManager::watchdogCallback, this));
  
  last_data_time_ = std::chrono::steady_clock::now();
  
  RCLCPP_INFO(node_->get_logger(), "GNSS manager initialized");
  return true;
}

void GnssManager::start()
{
  if (running_) {
    return;
  }
  
  running_ = true;
  
  // Start NTRIP client if created
  if (ntrip_client_) {
    ntrip_client_->start();
  }
  
  // Start RTK processor if created
  if (rtk_processor_) {
    rtk_processor_->start();
  }
  
  // Start GNSS thread
  gnss_thread_ = std::thread(&GnssManager::gnssLoop, this);
  
  RCLCPP_INFO(node_->get_logger(), "GNSS manager started");
}

void GnssManager::stop()
{
  if (!running_) {
    return;
  }
  
  running_ = false;
  
  // Stop NTRIP client if created
  if (ntrip_client_) {
    ntrip_client_->stop();
  }
  
  // Stop RTK processor if created
  if (rtk_processor_) {
    rtk_processor_->stop();
  }
  
  // Wait for GNSS thread to finish
  if (gnss_thread_.joinable()) {
    gnss_thread_.join();
  }
  
  // Disconnect from GNSS receiver
  if (gnss_driver_) {
    gnss_driver_->disconnect();
  }
  
  RCLCPP_INFO(node_->get_logger(), "GNSS manager stopped");
}

bool GnssManager::isRunning() const
{
  return running_;
}

void GnssManager::gnssLoop()
{
  // Set thread priority if possible
  // Note: This requires appropriate permissions or running as root
  // pthread_setschedprio(pthread_self(), ...)
  
  // Update interval based on configured frequency
  const auto update_interval = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(1.0 / config_->getFrequency()));
  
  while (running_) {
    auto cycle_start = std::chrono::steady_clock::now();
    
    // Publish GNSS data
    publishGnssData();
    
    // Publish odometry data for robot_localization
    publishOdometry();
    
    // Publish diagnostics
    publishDiagnostics();
    
    // Handle RTCM corrections if needed
    if (config_->useRtcmCorrections()) {
      handleRtcmCorrections();
    }
    
    // Update the last data time
    last_data_time_ = std::chrono::steady_clock::now();
    
    // Sleep to maintain desired frequency
    auto cycle_end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(cycle_end - cycle_start);
    
    if (elapsed < update_interval) {
      std::this_thread::sleep_for(update_interval - elapsed);
    } else if (elapsed > update_interval * 1.5) {
      // Log warning if cycle takes too long
      RCLCPP_WARN(node_->get_logger(), "GNSS cycle took %ld ms, exceeding target of %ld ms",
                 elapsed.count(), update_interval.count());
    }
  }
}

void GnssManager::publishGnssData()
{
  if (!gnss_driver_ || !gnss_driver_->isConnected()) {
    return;
  }
  
  // Get current time for message timestamps
  rclcpp::Time current_time = node_->now();
  
  // Get GNSS status
  GnssDriver::GnssStatus status = gnss_driver_->getStatus();
  
  // Log status in JSON format
  logStatusJson(status);
  
  // Publish NavSatFix
  sensor_msgs::msg::NavSatFix fix_msg;
  if (gnss_driver_->getNavSatFix(fix_msg)) {
    fix_msg.header.stamp = current_time;
    fix_msg.header.frame_id = config_->getFrameId();
    fix_pub_->publish(fix_msg);
  }
  
  // Publish TwistWithCovarianceStamped
  geometry_msgs::msg::TwistWithCovarianceStamped velocity_msg;
  if (gnss_driver_->getVelocity(velocity_msg)) {
    velocity_msg.header.stamp = current_time;
    velocity_msg.header.frame_id = config_->getFrameId();
    velocity_pub_->publish(velocity_msg);
  }
  
  // Publish TimeReference
  sensor_msgs::msg::TimeReference time_ref_msg;
  if (gnss_driver_->getTimeReference(time_ref_msg)) {
    time_ref_msg.header.stamp = current_time;
    time_ref_msg.header.frame_id = config_->getFrameId();
    time_ref_pub_->publish(time_ref_msg);
  }
  
  // Publish raw data
  std::vector<uint8_t> raw_data;
  if (gnss_driver_->getRawData(raw_data)) {
    std_msgs::msg::UInt8MultiArray raw_msg;
    raw_msg.data = raw_data;
    raw_pub_->publish(raw_msg);
  }
  
  // Publish status
  std_msgs::msg::String status_msg;
  status_msg.data = status.status_message;
  status_pub_->publish(status_msg);
}

void GnssManager::publishDiagnostics()
{
  // Diagnostics publishing removed to avoid conflicts
  if (!gnss_driver_ || !gnss_driver_->isConnected()) {
    return;
  }
  
  // Get current GNSS status for logging
  GnssDriver::GnssStatus status = gnss_driver_->getStatus();
  
  // Log the status instead of publishing diagnostics
  RCLCPP_DEBUG(node_->get_logger(), "GNSS Status: %s, Fix: %d, Satellites: %d",
              status.status_message.c_str(), status.fix_type, status.satellites_visible);
}

void GnssManager::handleRtcmCorrections()
{
  // Implementation depends on RTCM source
  // NTRIP client handling is already set up in the initialize method
  // Additional handling would be needed for other sources
}

// RTCM callback removed to avoid conflicts
// void GnssManager::rtcmCallback(const rtcm_msgs::msg::Message::SharedPtr msg)
// {
//   if (rtk_processor_) {
//     // Process through RTK processor
//     rtk_processor_->processRtcmData(msg->message);
//   } else if (gnss_driver_ && gnss_driver_->isConnected()) {
//     // Send directly to GNSS driver if no RTK processor
//     gnss_driver_->sendRtcmCorrection(msg->message);
//   }
// }

void GnssManager::controlCallback(const std_msgs::msg::String::SharedPtr msg)
{
  // Handle control commands
  const std::string& command = msg->data;
  
  if (command == "reset_origin") {
    // Reset local origin
    local_origin_.initialized = false;
    RCLCPP_INFO(node_->get_logger(), "Local origin reset");
  } else if (command == "enable_rtk") {
    if (rtk_processor_) {
      rtk_processor_->start();
      RCLCPP_INFO(node_->get_logger(), "RTK processing enabled");
    }
  } else if (command == "disable_rtk") {
    if (rtk_processor_) {
      rtk_processor_->stop();
      RCLCPP_INFO(node_->get_logger(), "RTK processing disabled");
    }
  } else {
    RCLCPP_WARN(node_->get_logger(), "Unknown control command: %s", command.c_str());
  }
}

void GnssManager::watchdogCallback()
{
  if (!running_ || !gnss_driver_) {
    return;
  }
  
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_data_time_);
  
  // Log watchdog status
  SimpleJson watchdog_json;
  watchdog_json.add("timestamp", getTimeString());
  watchdog_json.add("type", "watchdog");
  watchdog_json.add("elapsed_sec", static_cast<int>(elapsed.count()));
  watchdog_json.add("status", "ok");
  
  // Check if data is stale
  if (elapsed > std::chrono::seconds(5)) {
    RCLCPP_WARN(node_->get_logger(), "GNSS data is stale: %ld seconds since last update", 
               elapsed.count());
    
    watchdog_json = SimpleJson(); // Reset JSON
    watchdog_json.add("timestamp", getTimeString());
    watchdog_json.add("type", "watchdog");
    watchdog_json.add("elapsed_sec", static_cast<int>(elapsed.count()));
    watchdog_json.add("status", "stale");
    
    // Check connection and attempt to reconnect if needed
    if (!gnss_driver_->isConnected()) {
      RCLCPP_WARN(node_->get_logger(), "GNSS connection lost, attempting to reconnect");
      gnss_driver_->connect();
      
      watchdog_json = SimpleJson(); // Reset JSON
      watchdog_json.add("timestamp", getTimeString());
      watchdog_json.add("type", "watchdog");
      watchdog_json.add("elapsed_sec", static_cast<int>(elapsed.count()));
      watchdog_json.add("status", "reconnecting");
    }
  }
  
  // Write watchdog log
  if (log_file_.is_open()) {
    log_file_ << watchdog_json.dump() << std::endl;
  }
}

void GnssManager::logStatusJson(const GnssDriver::GnssStatus& status)
{
  if (!log_file_.is_open()) {
    return;
  }
  
  // Create JSON object for logging
  SimpleJson status_json;
  status_json.add("timestamp", getTimeString());
  status_json.add("type", "status");
  status_json.add("connected", status.connected);
  status_json.add("satellites", status.satellites_visible);
  status_json.add("fix_type", status.fix_type);
  status_json.add("hdop", status.hdop);
  status_json.add("pdop", status.pdop);
  status_json.add("message", status.status_message);
  
  // Add rtk_status field if available
  if (status.fix_type >= 5) {
    SimpleJson rtk_json;
    rtk_json.add("timestamp", getTimeString());
    rtk_json.add("type", "status");
    rtk_json.add("connected", status.connected);
    rtk_json.add("satellites", status.satellites_visible);
    rtk_json.add("fix_type", status.fix_type);
    rtk_json.add("hdop", status.hdop);
    rtk_json.add("pdop", status.pdop);
    rtk_json.add("message", status.status_message);
    rtk_json.add("rtk_status", (status.fix_type == 6) ? "fixed" : "float");
    
    // Write enhanced RTK status to log file
    log_file_ << rtk_json.dump() << std::endl;
    return;
  }
  
  // Write to log file
  log_file_ << status_json.dump() << std::endl;
}

void GnssManager::publishOdometry()
{
  if (!gnss_driver_ || !gnss_driver_->isConnected()) {
    return;
  }
  
  // Get current time
  rclcpp::Time current_time = node_->now();
  
  // Get NavSatFix message
  sensor_msgs::msg::NavSatFix fix_msg;
  if (!gnss_driver_->getNavSatFix(fix_msg)) {
    return;
  }
  
  // Only proceed if we have a valid fix
  if (fix_msg.status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
    return;
  }
  
  // Initialize local origin if not already done
  if (!local_origin_.initialized) {
    initializeLocalOrigin(fix_msg.latitude, fix_msg.longitude, fix_msg.altitude);
  }
  
  // Convert to local pose
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  if (convertNavSatToLocalPose(fix_msg, pose_msg)) {
    // Set header
    pose_msg.header.stamp = current_time;
    pose_msg.header.frame_id = "map";
    
    // Publish pose
    pose_pub_->publish(pose_msg);
    
    // Create odometry message for robot_localization
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header = pose_msg.header;
    odom_msg.child_frame_id = config_->getFrameId();
    
    // Copy position and covariance from pose
    odom_msg.pose = pose_msg.pose;
    
    // Get velocity if available
    geometry_msgs::msg::TwistWithCovarianceStamped vel_msg;
    if (gnss_driver_->getVelocity(vel_msg)) {
      odom_msg.twist = vel_msg.twist;
    }
    
    // Publish odometry
    odom_pub_->publish(odom_msg);
  }
}

bool GnssManager::convertNavSatToLocalPose(const sensor_msgs::msg::NavSatFix& nav_fix,
                                         geometry_msgs::msg::PoseWithCovarianceStamped& pose)
{
  if (!local_origin_.initialized) {
    return false;
  }
  
  // Constants
  constexpr double EARTH_RADIUS_EQUATOR = 6378137.0;  // in meters
  constexpr double EARTH_FLATTENING = 1.0/298.257223563;
  constexpr double EARTH_RADIUS_POLAR = EARTH_RADIUS_EQUATOR * (1.0 - EARTH_FLATTENING);
  
  // Convert latitude to radians
  double lat_rad = nav_fix.latitude * M_PI / 180.0;
  double lon_rad = nav_fix.longitude * M_PI / 180.0;
  double orig_lat_rad = local_origin_.latitude * M_PI / 180.0;
  double orig_lon_rad = local_origin_.longitude * M_PI / 180.0;
  
  // Calculate Earth radius at current latitude
  double cos_lat = std::cos(lat_rad);
  double sin_lat = std::sin(lat_rad);
  double radius_at_lat = std::sqrt(
    std::pow(EARTH_RADIUS_EQUATOR * cos_lat, 2) +
    std::pow(EARTH_RADIUS_POLAR * sin_lat, 2)
  );
  
  // Calculate local position (ENU - East-North-Up)
  double east = radius_at_lat * std::cos(lat_rad) * (lon_rad - orig_lon_rad);
  double north = radius_at_lat * (lat_rad - orig_lat_rad);
  double up = nav_fix.altitude - local_origin_.altitude;
  
  // Apply any offsets if needed
  east += local_origin_.east_offset;
  north += local_origin_.north_offset;
  
  // Fill pose message
  pose.pose.pose.position.x = north;  // ROS convention: x = north
  pose.pose.pose.position.y = east;   // ROS convention: y = east
  pose.pose.pose.position.z = up;     // ROS convention: z = up
  
  // Set orientation to identity (no heading information from GNSS)
  pose.pose.pose.orientation.x = 0.0;
  pose.pose.pose.orientation.y = 0.0;
  pose.pose.pose.orientation.z = 0.0;
  pose.pose.pose.orientation.w = 1.0;
  
  // Copy covariance (first 3x3 block for position)
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      pose.pose.covariance[i*6 + j] = nav_fix.position_covariance[i*3 + j];
    }
  }
  
  // Set high covariance for orientation (since we don't have heading)
  for (int i = 3; i < 6; i++) {
    pose.pose.covariance[i*6 + i] = 99999.0;
  }
  
  return true;
}

bool GnssManager::initializeLocalOrigin(double lat, double lon, double alt)
{
  if (lat == 0.0 && lon == 0.0) {
    RCLCPP_WARN(node_->get_logger(), "Invalid origin position (0,0)");
    return false;
  }
  
  // Store local origin
  local_origin_.initialized = true;
  local_origin_.latitude = lat;
  local_origin_.longitude = lon;
  local_origin_.altitude = alt;
  local_origin_.north_offset = 0.0;
  local_origin_.east_offset = 0.0;
  
  RCLCPP_INFO(node_->get_logger(), "Local origin initialized at %.6f, %.6f, %.2f",
             lat, lon, alt);
  
  return true;
}

std::string GnssManager::getTimeString()
{
  auto now = std::chrono::system_clock::now();
  auto now_time_t = std::chrono::system_clock::to_time_t(now);
  auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()) % 1000;
  
  std::stringstream ss;
  ss << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S") 
     << '.' << std::setfill('0') << std::setw(3) << now_ms.count();
  
  return ss.str();
}

}  // namespace gnss
}  // namespace sensors