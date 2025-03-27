#include "sensors/gnss/rtk_processor.h"
#include <chrono>
#include <thread>
#include <algorithm>
#include <sstream>
#include <functional>

namespace sensors {
namespace gnss {

RtkProcessor::RtkProcessor(rclcpp::Logger logger)
  : logger_(logger),
    initialized_(false),
    running_(false),
    fix_quality_(0)
{
  // Initialize with default settings
}

RtkProcessor::~RtkProcessor()
{
  if (running_) {
    stop();
  }
}

bool RtkProcessor::initialize(const std::string& base_position, const std::string& config_file)
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  base_position_ = base_position;
  config_file_ = config_file;
  
  // Parse base position (lat,lon,alt format)
  if (!base_position.empty()) {
    // For now, just store the position string
    // In a production system, we'd validate this and convert to a usable format
    RCLCPP_INFO(logger_, "RTK base position set to: %s", base_position.c_str());
  }
  
  // In a full implementation, we would initialize RTKLIB (or similar) here
  // For this implementation, we'll simulate initialization success
  
  initialized_ = true;
  RCLCPP_INFO(logger_, "RTK processor initialized successfully");
  
  return true;
}

bool RtkProcessor::processRtcmData(const std::vector<uint8_t>& rtcm_data)
{
  if (!initialized_ || !running_) {
    return false;
  }
  
  std::lock_guard<std::mutex> lock(mutex_);
  
  // Add the data to the queue for processing
  rtcm_queue_.push_back(rtcm_data);
  
  // Limit the queue size to prevent memory issues
  constexpr size_t MAX_QUEUE_SIZE = 100;
  while (rtcm_queue_.size() > MAX_QUEUE_SIZE) {
    rtcm_queue_.pop_front();
    RCLCPP_WARN(logger_, "RTK correction queue overflow, dropping oldest data");
  }
  
  return true;
}

void RtkProcessor::setRtcmCallback(RtcmCallback callback)
{
  std::lock_guard<std::mutex> lock(mutex_);
  rtcm_callback_ = callback;
}

void RtkProcessor::start()
{
  if (!initialized_ || running_) {
    return;
  }
  
  running_ = true;
  
  // Start the processing thread
  processing_thread_ = std::thread(&RtkProcessor::processingLoop, this);
  
  RCLCPP_INFO(logger_, "RTK processor started");
}

void RtkProcessor::stop()
{
  if (!running_) {
    return;
  }
  
  running_ = false;
  
  // Wait for the processing thread to finish
  if (processing_thread_.joinable()) {
    processing_thread_.join();
  }
  
  RCLCPP_INFO(logger_, "RTK processor stopped");
}

bool RtkProcessor::isRunning() const
{
  return running_;
}

std::string RtkProcessor::getStatusString() const
{
  switch (fix_quality_) {
    case 0: return "No RTK";
    case 1: return "RTK Initializing";
    case 2: return "RTK Float";
    case 3: return "RTK Fixed";
    default: return "Unknown";
  }
}

int RtkProcessor::getFixQuality() const
{
  return fix_quality_;
}

void RtkProcessor::processingLoop()
{
  // Set thread properties (optional)
  // pthread_setschedprio(pthread_self(), ...);
  
  auto last_status_time = std::chrono::steady_clock::now();
  
  while (running_) {
    std::vector<uint8_t> data;
    
    // Process any data in the queue
    {
      std::lock_guard<std::mutex> lock(mutex_);
      
      if (!rtcm_queue_.empty()) {
        data = rtcm_queue_.front();
        rtcm_queue_.pop_front();
      }
    }
    
    if (!data.empty()) {
      // Process the RTCM data
      bool success = processRtklib(data);
      
      if (!success) {
        RCLCPP_WARN(logger_, "Failed to process RTCM data");
      }
    }
    
    // Update status periodically (every 5 seconds)
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_status_time).count() >= 5) {
      last_status_time = now;
      
      // Simulate different RTK fix states
      static int counter = 0;
      counter = (counter + 1) % 30;
      
      if (counter < 10) {
        fix_quality_ = 1;  // Initializing
      } else if (counter < 20) {
        fix_quality_ = 2;  // Float
      } else {
        fix_quality_ = 3;  // Fixed
      }
      
      RCLCPP_INFO(logger_, "RTK status: %s", getStatusString().c_str());
    }
    
    // Sleep to reduce CPU usage
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

bool RtkProcessor::processRtklib(const std::vector<uint8_t>& rtcm_data)
{
  // In a full implementation, this would use RTKLIB to process the RTCM data
  // For now, we'll just simulate processing and forward the data
  
  // Log information about the RTCM message
  std::stringstream ss;
  ss << "RTCM message type: ";
  
  if (rtcm_data.size() >= 3) {
    // Extract RTCM message type from bytes 1-2
    int message_type = ((rtcm_data[1] & 0x03) << 8) | rtcm_data[2];
    ss << message_type;
    
    // Interpret common RTCM3 message types
    switch (message_type) {
      case 1001:
      case 1002:
      case 1003:
      case 1004:
        ss << " (GPS RTK L1 data)";
        break;
      case 1005:
      case 1006:
        ss << " (Station coordinates)";
        break;
      case 1007:
      case 1008:
        ss << " (Antenna descriptor)";
        break;
      case 1009:
      case 1010:
      case 1011:
      case 1012:
        ss << " (GLONASS RTK data)";
        break;
      case 1019:
        ss << " (GPS ephemeris)";
        break;
      case 1020:
        ss << " (GLONASS ephemeris)";
        break;
      case 1071:
      case 1072:
      case 1073:
      case 1074:
      case 1075:
      case 1076:
      case 1077:
        ss << " (GPS MSM)";
        break;
      case 1081:
      case 1082:
      case 1083:
      case 1084:
      case 1085:
      case 1086:
      case 1087:
        ss << " (GLONASS MSM)";
        break;
      case 1091:
      case 1092:
      case 1093:
      case 1094:
      case 1095:
      case 1096:
      case 1097:
        ss << " (Galileo MSM)";
        break;
      default:
        ss << " (Unknown)";
    }
  } else {
    ss << "Unknown (too short)";
  }
  
  RCLCPP_DEBUG(logger_, "Processing RTCM data: %s, %zu bytes", ss.str().c_str(), rtcm_data.size());
  
  // In a real implementation, we'd process the RTCM data here
  // For now, just forward the data to any registered callback
  if (rtcm_callback_) {
    rtcm_callback_(rtcm_data);
  }
  
  return true;
}

}  // namespace gnss
}  // namespace sensors