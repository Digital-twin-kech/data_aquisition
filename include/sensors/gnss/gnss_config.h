#ifndef GNSS_CONFIG_H
#define GNSS_CONFIG_H

#include <string>
#include "rclcpp/rclcpp.hpp"

namespace sensors {
namespace gnss {

/**
 * @brief Configuration parameters for GNSS receiver
 * 
 * This class holds configuration settings for GNSS (such as u-blox F9P),
 * including communication settings and QoS profiles.
 */
class GnssConfig {
public:
  /**
   * @brief Construct a new GNSS Config object
   * 
   * @param node ROS2 node to read parameters from
   */
  explicit GnssConfig(rclcpp::Node::SharedPtr node);

  /**
   * @brief Destructor
   */
  ~GnssConfig() = default;

  /**
   * @brief Get the serial port for GNSS receiver
   * 
   * @return std::string Serial port (e.g., "/dev/ttyACM0")
   */
  std::string getSerialPort() const { return serial_port_; }

  /**
   * @brief Get the baud rate for GNSS communication
   * 
   * @return int Baud rate (e.g., 115200)
   */
  int getBaudRate() const { return baud_rate_; }

  /**
   * @brief Get the update frequency
   * 
   * @return double Update frequency in Hz
   */
  double getFrequency() const { return frequency_; }

  /**
   * @brief Get the GNSS receiver model
   * 
   * @return std::string GNSS model name
   */
  std::string getGnssModel() const { return gnss_model_; }

  /**
   * @brief Get the QoS reliability setting
   * 
   * @return rclcpp::ReliabilityPolicy QoS reliability policy
   */
  rclcpp::ReliabilityPolicy getReliabilityPolicy() const { return reliability_policy_; }

  /**
   * @brief Get the QoS history depth
   * 
   * @return int QoS history depth
   */
  int getQosHistoryDepth() const { return qos_history_depth_; }
  
  /**
   * @brief Get the GNSS frame ID
   * 
   * @return std::string GNSS frame ID
   */
  std::string getFrameId() const { return frame_id_; }
  
  /**
   * @brief Get whether to use RTCM corrections
   * 
   * @return bool True if RTCM corrections are enabled
   */
  bool useRtcmCorrections() const { return use_rtcm_corrections_; }
  
  /**
   * @brief Get RTCM correction source
   * 
   * @return std::string RTCM source (e.g., "NTRIP" or "Serial")
   */
  std::string getRtcmSource() const { return rtcm_source_; }
  
  /**
   * @brief Get NTRIP server settings (if applicable)
   * 
   * @return std::string NTRIP server URL
   */
  std::string getNtripServer() const { return ntrip_server_; }
  
  /**
   * @brief Get whether to enable dynamic model
   * 
   * @return bool True if dynamic model is enabled
   */
  bool useDynamicModel() const { return use_dynamic_model_; }
  
  /**
   * @brief Get the dynamic model type
   * 
   * @return std::string Dynamic model (e.g., "automotive", "airborne", etc.)
   */
  std::string getDynamicModel() const { return dynamic_model_; }

private:
  // ROS2 node
  rclcpp::Node::SharedPtr node_;

  // GNSS settings
  std::string gnss_model_;
  std::string serial_port_;
  int baud_rate_;
  double frequency_;
  std::string frame_id_ = "gnss_frame";
  
  // RTK/RTCM settings
  bool use_rtcm_corrections_;
  std::string rtcm_source_;
  std::string ntrip_server_;
  std::string ntrip_username_;
  std::string ntrip_password_;
  std::string rtcm_serial_port_;
  
  // Dynamic model settings
  bool use_dynamic_model_;
  std::string dynamic_model_;
  
  // QoS settings
  rclcpp::ReliabilityPolicy reliability_policy_;
  int qos_history_depth_;
  
  /**
   * @brief Load configuration from ROS2 parameters
   */
  void loadParameters();
};

}  // namespace gnss
}  // namespace sensors

#endif  // GNSS_CONFIG_H