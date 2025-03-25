#ifndef NTRIP_CLIENT_H
#define NTRIP_CLIENT_H

#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>
#include "rclcpp/rclcpp.hpp"

namespace sensors {
namespace gnss {

/**
 * @brief Client for NTRIP (Networked Transport of RTCM via Internet Protocol)
 * 
 * This class handles connecting to an NTRIP server and retrieving
 * RTCM correction data for RTK positioning.
 */
class NtripClient {
public:
  /**
   * @brief Callback function type for receiving RTCM data
   */
  using RtcmCallback = std::function<void(const std::vector<uint8_t>&)>;

  /**
   * @brief Construct a new Ntrip Client object
   * 
   * @param host NTRIP server hostname or IP address
   * @param port NTRIP server port
   * @param mountpoint NTRIP mountpoint
   * @param username NTRIP username (if authentication required)
   * @param password NTRIP password (if authentication required)
   * @param logger ROS logger to use for status messages
   */
  NtripClient(
    const std::string& host,
    int port,
    const std::string& mountpoint,
    const std::string& username,
    const std::string& password,
    rclcpp::Logger logger
  );
  
  /**
   * @brief Destructor
   */
  ~NtripClient();
  
  /**
   * @brief Connect to the NTRIP server
   * 
   * @return true if connection successful
   * @return false if connection failed
   */
  bool connect();
  
  /**
   * @brief Disconnect from the NTRIP server
   */
  void disconnect();
  
  /**
   * @brief Check if connected to the NTRIP server
   * 
   * @return true if connected
   * @return false if not connected
   */
  bool isConnected() const;
  
  /**
   * @brief Set the callback for received RTCM data
   * 
   * @param callback Function to call when RTCM data is received
   */
  void setRtcmCallback(RtcmCallback callback);
  
  /**
   * @brief Start receiving RTCM data
   */
  void start();
  
  /**
   * @brief Stop receiving RTCM data
   */
  void stop();

private:
  // NTRIP server settings
  std::string host_;
  int port_;
  std::string mountpoint_;
  std::string username_;
  std::string password_;
  
  // Socket and connection state
  int socket_fd_;
  std::atomic<bool> connected_;
  std::atomic<bool> running_;
  
  // Thread for receiving data
  std::thread receive_thread_;
  std::mutex mutex_;
  
  // Callback for RTCM data
  RtcmCallback rtcm_callback_;
  
  // Buffer for received data
  std::vector<uint8_t> receive_buffer_;
  
  // Logger
  rclcpp::Logger logger_;
  
  // Internal functions
  bool sendRequest();
  void receiveLoop();
  bool parseRtcmData(const std::vector<uint8_t>& data);
};

}  // namespace gnss
}  // namespace sensors

#endif  // NTRIP_CLIENT_H