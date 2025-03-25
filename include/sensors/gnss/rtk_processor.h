#ifndef RTK_PROCESSOR_H
#define RTK_PROCESSOR_H

#include <memory>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>
#include <deque>
#include "rclcpp/rclcpp.hpp"

namespace sensors {
namespace gnss {

/**
 * @brief RTK correction processor for GNSS data
 * 
 * This class processes RTCM3 messages for RTK correction
 * and interfaces with RTKLIB (or similar libraries) for detailed processing.
 */
class RtkProcessor {
public:
  /**
   * @brief Callback function type for RTCM data
   */
  using RtcmCallback = std::function<void(const std::vector<uint8_t>&)>;
  
  /**
   * @brief Construct a new RTK Processor object
   * 
   * @param logger ROS logger for status messages
   */
  explicit RtkProcessor(rclcpp::Logger logger);
  
  /**
   * @brief Destructor
   */
  ~RtkProcessor();
  
  /**
   * @brief Initialize RTK processing
   * 
   * @param base_position Reference position for RTK base
   * @param config_file Path to RTK configuration file
   * @return true if initialization succeeded
   * @return false if initialization failed
   */
  bool initialize(const std::string& base_position, const std::string& config_file);
  
  /**
   * @brief Process RTCM3 data
   * 
   * @param rtcm_data RTCM3 data to process
   * @return true if processing succeeded
   * @return false if processing failed
   */
  bool processRtcmData(const std::vector<uint8_t>& rtcm_data);
  
  /**
   * @brief Set callback for processed correction data
   * 
   * @param callback Function to call with processed data
   */
  void setRtcmCallback(RtcmCallback callback);
  
  /**
   * @brief Start RTK processing
   */
  void start();
  
  /**
   * @brief Stop RTK processing
   */
  void stop();
  
  /**
   * @brief Check if RTK processor is running
   * 
   * @return true if running
   * @return false if not running
   */
  bool isRunning() const;
  
  /**
   * @brief Get RTK status as a string
   * 
   * @return std::string Status description
   */
  std::string getStatusString() const;
  
  /**
   * @brief Get RTK fix quality (0-6)
   * 
   * @return int Fix quality
   */
  int getFixQuality() const;
  
private:
  // ROS logger
  rclcpp::Logger logger_;
  
  // RTK state
  std::atomic<bool> initialized_;
  std::atomic<bool> running_;
  std::atomic<int> fix_quality_;
  
  // Processing thread
  std::thread processing_thread_;
  std::mutex mutex_;
  
  // RTCM data queue
  std::deque<std::vector<uint8_t>> rtcm_queue_;
  
  // RTCM callback
  RtcmCallback rtcm_callback_;
  
  // Configuration
  std::string base_position_;
  std::string config_file_;
  
  // Processing loop
  void processingLoop();
  
  // Internal functions for RTKLIB integration
  bool processRtklib(const std::vector<uint8_t>& rtcm_data);
};

}  // namespace gnss
}  // namespace sensors

#endif  // RTK_PROCESSOR_H