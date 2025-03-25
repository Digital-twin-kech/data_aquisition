#include "sensors/gnss/ntrip_client.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <cstring>
#include <sstream>
#include <algorithm>
#include "sensors/gnss/base64.h"

namespace sensors {
namespace gnss {

NtripClient::NtripClient(
  const std::string& host,
  int port,
  const std::string& mountpoint,
  const std::string& username,
  const std::string& password,
  rclcpp::Logger logger
)
  : host_(host),
    port_(port),
    mountpoint_(mountpoint),
    username_(username),
    password_(password),
    socket_fd_(-1),
    connected_(false),
    running_(false),
    logger_(logger)
{
  receive_buffer_.reserve(4096);
}

NtripClient::~NtripClient()
{
  if (running_) {
    stop();
  }
  
  if (connected_) {
    disconnect();
  }
}

bool NtripClient::connect()
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (connected_) {
    return true;
  }
  
  // Resolve hostname
  struct hostent* host = gethostbyname(host_.c_str());
  if (!host) {
    RCLCPP_ERROR(logger_, "Failed to resolve NTRIP server hostname: %s", host_.c_str());
    return false;
  }
  
  // Create socket
  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ < 0) {
    RCLCPP_ERROR(logger_, "Failed to create socket for NTRIP client");
    return false;
  }
  
  // Set socket options
  int flag = 1;
  if (setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag)) < 0) {
    close(socket_fd_);
    socket_fd_ = -1;
    RCLCPP_ERROR(logger_, "Failed to set socket options for NTRIP client");
    return false;
  }
  
  // Set up server address
  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port_);
  memcpy(&server_addr.sin_addr, host->h_addr, host->h_length);
  
  // Connect to server
  if (::connect(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    close(socket_fd_);
    socket_fd_ = -1;
    RCLCPP_ERROR(logger_, "Failed to connect to NTRIP server %s:%d", host_.c_str(), port_);
    return false;
  }
  
  // Send NTRIP request
  if (!sendRequest()) {
    close(socket_fd_);
    socket_fd_ = -1;
    RCLCPP_ERROR(logger_, "Failed to send NTRIP request");
    return false;
  }
  
  connected_ = true;
  RCLCPP_INFO(logger_, "Connected to NTRIP server %s:%d, mountpoint: %s", 
              host_.c_str(), port_, mountpoint_.c_str());
  
  return true;
}

void NtripClient::disconnect()
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (!connected_) {
    return;
  }
  
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
  
  connected_ = false;
  RCLCPP_INFO(logger_, "Disconnected from NTRIP server");
}

bool NtripClient::isConnected() const
{
  return connected_;
}

void NtripClient::setRtcmCallback(RtcmCallback callback)
{
  std::lock_guard<std::mutex> lock(mutex_);
  rtcm_callback_ = callback;
}

void NtripClient::start()
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (running_) {
    return;
  }
  
  if (!connected_ && !connect()) {
    RCLCPP_ERROR(logger_, "Cannot start NTRIP client: not connected");
    return;
  }
  
  running_ = true;
  receive_thread_ = std::thread(&NtripClient::receiveLoop, this);
  
  RCLCPP_INFO(logger_, "NTRIP client started");
}

void NtripClient::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!running_) {
      return;
    }
    
    running_ = false;
  }
  
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
  
  RCLCPP_INFO(logger_, "NTRIP client stopped");
}

bool NtripClient::sendRequest()
{
  // Create authorization string if credentials provided
  std::string auth_string;
  if (!username_.empty()) {
    std::string credentials = username_ + ":" + password_;
    auth_string = "Authorization: Basic " + 
                 base64_encode(reinterpret_cast<const unsigned char*>(credentials.c_str()), 
                              credentials.length()) + 
                 "\r\n";
  }
  
  // Format NTRIP request
  std::ostringstream request;
  request << "GET /" << mountpoint_ << " HTTP/1.0\r\n"
          << "User-Agent: NTRIP ROS2Client/1.0\r\n"
          << "Accept: */*\r\n"
          << auth_string
          << "Connection: close\r\n"
          << "\r\n";
  
  std::string request_str = request.str();
  
  // Send request
  ssize_t bytes_sent = send(socket_fd_, request_str.c_str(), request_str.length(), 0);
  
  return bytes_sent == static_cast<ssize_t>(request_str.length());
}

void NtripClient::receiveLoop()
{
  constexpr size_t buffer_size = 4096;
  char buffer[buffer_size];
  
  while (running_ && connected_) {
    // Read data from socket
    ssize_t bytes_read = recv(socket_fd_, buffer, buffer_size, 0);
    
    if (bytes_read <= 0) {
      // Connection closed or error
      RCLCPP_WARN(logger_, "NTRIP connection closed or error");
      
      // Attempt to reconnect
      std::lock_guard<std::mutex> lock(mutex_);
      connected_ = false;
      
      if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
      }
      
      if (running_ && connect()) {
        RCLCPP_INFO(logger_, "Reconnected to NTRIP server");
      } else {
        break;
      }
      
      continue;
    }
    
    // Process received data
    std::vector<uint8_t> data(buffer, buffer + bytes_read);
    
    // Parse RTCM data
    if (parseRtcmData(data) && rtcm_callback_) {
      rtcm_callback_(data);
    }
    
    // Small sleep to prevent 100% CPU usage
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

bool NtripClient::parseRtcmData(const std::vector<uint8_t>& data)
{
  // In a real implementation, this would validate RTCM data
  // and extract RTCM messages from the HTTP response
  
  // For simplicity, we assume all received data after connection
  // is valid RTCM data
  
  return !data.empty();
}

// Use the base64.h implementation instead

}  // namespace gnss
}  // namespace sensors