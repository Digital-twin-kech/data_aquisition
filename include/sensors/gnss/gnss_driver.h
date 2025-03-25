#ifndef GNSS_DRIVER_H
#define GNSS_DRIVER_H

#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
// #include <diagnostic_msgs/msg/diagnostic_status.hpp> // Removed to avoid conflicts
#include "sensors/gnss/gnss_config.h"
#include "sensors/gnss/nmea_parser.h"

namespace sensors {
namespace gnss {

/**
 * @brief Class for interfacing with GNSS receiver (e.g., u-blox F9P)
 * 
 * This class handles direct communication with the GNSS receiver via serial port.
 * It provides methods to configure the receiver and retrieve data.
 */
class GnssDriver {
public:
  /**
   * @brief Struct to hold GNSS status information
   */
  struct GnssStatus {
    bool connected;
    int satellites_visible;
    int fix_type;  // 0=no fix, 1=time only, 2=2D, 3=3D, 4=GNSS+dead reckoning, 5=RTK float, 6=RTK fixed
    float hdop;
    float pdop;
    std::string status_message;
  };

  /**
   * @brief Construct a new GNSS Driver object
   * 
   * @param config GNSS configuration
   */
  explicit GnssDriver(std::shared_ptr<GnssConfig> config);

  /**
   * @brief Destructor
   */
  ~GnssDriver();

  /**
   * @brief Connect to the GNSS receiver
   * 
   * @return true if connection successful
   * @return false if connection failed
   */
  bool connect();

  /**
   * @brief Disconnect from the GNSS receiver
   */
  void disconnect();

  /**
   * @brief Check if GNSS receiver is connected
   * 
   * @return true if GNSS is connected
   * @return false if GNSS is not connected
   */
  bool isConnected() const;

  /**
   * @brief Configure the GNSS receiver with optimal settings
   * 
   * @return true if configuration successful
   * @return false if configuration failed
   */
  bool configureReceiver();

  /**
   * @brief Get GNSS fix data
   * 
   * @param fix Reference to NavSatFix message to be filled
   * @return true if fix was successfully retrieved
   * @return false if failed to retrieve fix
   */
  bool getNavSatFix(sensor_msgs::msg::NavSatFix& fix);

  /**
   * @brief Get GNSS velocity data
   * 
   * @param velocity Reference to TwistWithCovarianceStamped message to be filled
   * @return true if velocity was successfully retrieved
   * @return false if failed to retrieve velocity
   */
  bool getVelocity(geometry_msgs::msg::TwistWithCovarianceStamped& velocity);

  /**
   * @brief Get GNSS time reference
   * 
   * @param time_ref Reference to TimeReference message to be filled
   * @return true if time reference was successfully retrieved
   * @return false if failed to retrieve time reference
   */
  bool getTimeReference(sensor_msgs::msg::TimeReference& time_ref);

  /**
   * @brief Get raw GNSS data
   * 
   * @param data Vector reference to store raw data
   * @return true if data was successfully retrieved
   * @return false if failed to retrieve data
   */
  bool getRawData(std::vector<uint8_t>& data);

  /**
   * @brief Get GNSS status
   * 
   * @return GnssStatus Struct containing GNSS status information
   */
  GnssStatus getStatus();

  /**
   * @brief Send RTCM correction data to the receiver
   * 
   * @param rtcm_data RTCM correction data
   * @return true if successfully sent
   * @return false if failed to send
   */
  bool sendRtcmCorrection(const std::vector<uint8_t>& rtcm_data);

private:
  // GNSS configuration
  std::shared_ptr<GnssConfig> config_;
  
  // Serial port handle
  int serial_port_fd_;
  
  // GNSS state
  bool connected_;
  std::mutex mutex_;
  
  // Status information
  GnssStatus current_status_;
  
  // Logger for messages
  rclcpp::Logger logger_;
  
  // Last received data
  sensor_msgs::msg::NavSatFix last_fix_;
  geometry_msgs::msg::TwistWithCovarianceStamped last_velocity_;
  sensor_msgs::msg::TimeReference last_time_ref_;
  std::vector<uint8_t> last_raw_data_;
  
  // Internal functions
  bool openSerialPort();
  void closeSerialPort();
  bool readData();
  bool processNmeaData(const std::vector<uint8_t>& buffer);
  bool generateSimulatedData();
  bool readFromGpsd();
  bool processGpsdJson(const std::string& json_msg);
  bool parseUBXMessage(const std::vector<uint8_t>& message);
  bool parseNMEAMessage(const std::string& message);
  void updateGnssStatus(const NmeaParser::GnssData& gnss_data = NmeaParser::GnssData());
};

}  // namespace gnss
}  // namespace sensors

#endif  // GNSS_DRIVER_H