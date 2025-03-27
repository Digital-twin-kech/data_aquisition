#include "sensors/gnss/gnss_driver.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <cstring>
#include <chrono>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <cmath>

namespace sensors {
namespace gnss {

GnssDriver::GnssDriver(std::shared_ptr<GnssConfig> config)
  : config_(config),
    serial_port_fd_(-1),
    connected_(false),
    logger_(rclcpp::get_logger("gnss_driver"))
{
  // Initialize status
  current_status_.connected = false;
  current_status_.satellites_visible = 0;
  current_status_.fix_type = 0;
  current_status_.hdop = 99.9;
  current_status_.pdop = 99.9;
  current_status_.status_message = "Not connected";
}

GnssDriver::~GnssDriver()
{
  if (connected_) {
    disconnect();
  }
}

bool GnssDriver::connect()
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (connected_) {
    return true;
  }
  
  if (!openSerialPort()) {
    current_status_.status_message = "Failed to open serial port";
    return false;
  }
  
  connected_ = true;
  current_status_.connected = true;
  current_status_.status_message = "Connected to GNSS receiver";
  
  // Configure the receiver with optimal settings
  if (!configureReceiver()) {
    current_status_.status_message = "Connected but failed to configure receiver";
  }
  
  return true;
}

void GnssDriver::disconnect()
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (!connected_) {
    return;
  }
  
  closeSerialPort();
  
  connected_ = false;
  current_status_.connected = false;
  current_status_.status_message = "Disconnected from GNSS receiver";
}

bool GnssDriver::isConnected() const
{
  return connected_;
}

bool GnssDriver::configureReceiver()
{
  if (!connected_) {
    return false;
  }
  
  // Implementation depends on specific GNSS receiver model (u-blox F9P in this case)
  // This would contain UBX protocol configuration commands
  
  // Example configuration for u-blox F9P:
  // 1. Set update rate
  // 2. Enable NMEA/UBX messages we want
  // 3. Configure dynamic model
  // 4. Configure RTCM input
  
  // For brevity, actual configuration bytes are not included
  // In a real implementation, you would send UBX protocol messages
  
  return true;
}

bool GnssDriver::getNavSatFix(sensor_msgs::msg::NavSatFix& fix)
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (!connected_) {
    return false;
  }
  
  // Read data if needed
  if (!readData()) {
    return false;
  }
  
  // Copy the last received fix data
  fix = last_fix_;
  
  return true;
}

bool GnssDriver::getVelocity(geometry_msgs::msg::TwistWithCovarianceStamped& velocity)
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (!connected_) {
    return false;
  }
  
  // Read data if needed
  if (!readData()) {
    return false;
  }
  
  // Copy the last received velocity data
  velocity = last_velocity_;
  
  return true;
}

bool GnssDriver::getTimeReference(sensor_msgs::msg::TimeReference& time_ref)
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (!connected_) {
    return false;
  }
  
  // Read data if needed
  if (!readData()) {
    return false;
  }
  
  // Copy the last received time reference
  time_ref = last_time_ref_;
  
  return true;
}

bool GnssDriver::getRawData(std::vector<uint8_t>& data)
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (!connected_) {
    return false;
  }
  
  // Read data if needed
  if (!readData()) {
    return false;
  }
  
  // Copy the last received raw data
  data = last_raw_data_;
  
  return true;
}

GnssDriver::GnssStatus GnssDriver::getStatus()
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (connected_) {
    updateGnssStatus();
  }
  
  return current_status_;
}

bool GnssDriver::sendRtcmCorrection(const std::vector<uint8_t>& rtcm_data)
{
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (!connected_) {
    return false;
  }
  
  // Send RTCM correction data to the receiver via serial port
  if (!rtcm_data.empty()) {
    ssize_t bytes_written = write(serial_port_fd_, rtcm_data.data(), rtcm_data.size());
    return bytes_written == static_cast<ssize_t>(rtcm_data.size());
  }
  
  return false;
}

bool GnssDriver::openSerialPort()
{
  RCLCPP_INFO(logger_, "Opening serial port: %s at %d baud", 
              config_->getSerialPort().c_str(), config_->getBaudRate());
              
  // Check if the serial port path exists
  if (access(config_->getSerialPort().c_str(), F_OK) != 0) {
    RCLCPP_ERROR(logger_, "Serial port %s does not exist", config_->getSerialPort().c_str());
    return false;
  }
  
  // Check if GPSD is using the port
  std::string cmd = "ps aux | grep 'gpsd.*" + config_->getSerialPort() + "' | grep -v grep";
  FILE* pipe = popen(cmd.c_str(), "r");
  if (!pipe) {
    RCLCPP_ERROR(logger_, "Failed to execute command to check GPSD");
  } else {
    char buffer[128];
    if (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
      RCLCPP_INFO(logger_, "Detected GPSD is already using %s", config_->getSerialPort().c_str());
      RCLCPP_INFO(logger_, "Using GPSD data source instead of direct port access");
      // Use a special value to indicate GPSD mode
      serial_port_fd_ = -3; // Special value to indicate GPSD mode
      pclose(pipe);
      return true;
    }
    pclose(pipe);
  }

  // Open serial port
  serial_port_fd_ = open(config_->getSerialPort().c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  
  if (serial_port_fd_ < 0) {
    RCLCPP_ERROR(logger_, "Failed to open serial port: %s (errno: %d: %s)", 
                config_->getSerialPort().c_str(), errno, strerror(errno));
    
    // Don't use simulation mode by default anymore - the user should explicitly enable it
    RCLCPP_WARN(logger_, "Hardware GNSS not available. Returning connection failure.");
    RCLCPP_INFO(logger_, "Set use_simulation=true in config to enable simulation mode for testing.");
    return false;
  }
  
  // Configure serial port
  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  
  if (tcgetattr(serial_port_fd_, &tty) != 0) {
    RCLCPP_ERROR(logger_, "Failed to get serial port attributes (errno: %d: %s)",
                errno, strerror(errno));
    close(serial_port_fd_);
    serial_port_fd_ = -1;
    return false;
  }
  
  // Set baud rate
  speed_t baud;
  switch (config_->getBaudRate()) {
    case 9600:   baud = B9600;   break;
    case 19200:  baud = B19200;  break;
    case 38400:  baud = B38400;  break;
    case 57600:  baud = B57600;  break;
    case 115200: baud = B115200; break;
    case 230400: baud = B230400; break;
    case 460800: baud = B460800; break;
    case 921600: baud = B921600; break;
    default:     
      RCLCPP_WARN(logger_, "Unsupported baud rate %d, using 115200", config_->getBaudRate());
      baud = B115200; 
      break;
  }
  
  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);
  
  tty.c_cflag |= (CLOCAL | CREAD);    // Ignore modem controls, enable reading
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;                 // 8-bit characters
  tty.c_cflag &= ~PARENB;             // No parity bit
  tty.c_cflag &= ~CSTOPB;             // One stop bit
  tty.c_cflag &= ~CRTSCTS;            // No hardware flow control
  
  // Setup for non-canonical mode
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_oflag &= ~OPOST;
  
  // Fetch bytes as they become available
  tty.c_cc[VMIN] = 0;                 // Non-blocking read
  tty.c_cc[VTIME] = 0;                // No timeout
  
  if (tcsetattr(serial_port_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(logger_, "Failed to set serial port attributes (errno: %d: %s)",
                errno, strerror(errno));
    close(serial_port_fd_);
    serial_port_fd_ = -1;
    return false;
  }
  
  RCLCPP_INFO(logger_, "Serial port opened successfully");
  return true;
}

void GnssDriver::closeSerialPort()
{
  if (serial_port_fd_ >= 0) {
    close(serial_port_fd_);
    serial_port_fd_ = -1;
  }
}

bool GnssDriver::readData()
{
  // Handle simulation mode special case
  if (serial_port_fd_ == -2) {
    // Generate simulated GNSS data
    return generateSimulatedData();
  }
  
  // Handle GPSD mode special case
  if (serial_port_fd_ == -3) {
    // Read from GPSD using libgps or gpspipe
    RCLCPP_DEBUG(logger_, "Reading from GPSD");
    return readFromGpsd();
  }

  if (!connected_ || serial_port_fd_ < 0) {
    return false;
  }
  
  // Check if there's data available
  int bytes_available;
  if (ioctl(serial_port_fd_, FIONREAD, &bytes_available) < 0) {
    RCLCPP_ERROR(logger_, "Failed to check for available data (errno: %d: %s)",
                errno, strerror(errno));
    return false;
  }
  
  if (bytes_available == 0) {
    // No data available right now
    return false;
  }
  
  // Read available data
  std::vector<uint8_t> buffer(bytes_available);
  ssize_t bytes_read = read(serial_port_fd_, buffer.data(), bytes_available);
  
  if (bytes_read <= 0) {
    RCLCPP_ERROR(logger_, "Failed to read data (bytes_read: %ld, errno: %d: %s)",
                bytes_read, errno, strerror(errno));
    return false;
  }
  
  // Resize buffer to actual bytes read
  buffer.resize(bytes_read);
  
  // Store raw data
  last_raw_data_ = buffer;
  
  // Process received data
  return processNmeaData(buffer);
}

bool GnssDriver::processNmeaData(const std::vector<uint8_t>& buffer)
{
  // Convert buffer to string for NMEA parsing
  std::string nmea_data(buffer.begin(), buffer.end());
  
  // Create NMEA parser if not already created
  static NmeaParser nmea_parser(logger_);
  
  // Split by newlines and parse each NMEA sentence
  size_t pos = 0;
  size_t prev_pos = 0;
  bool valid_data_received = false;
  
  while ((pos = nmea_data.find('\n', prev_pos)) != std::string::npos) {
    std::string sentence = nmea_data.substr(prev_pos, pos - prev_pos);
    
    // Remove carriage return if present
    if (!sentence.empty() && sentence.back() == '\r') {
      sentence.pop_back();
    }
    
    // Parse NMEA sentence
    if (!sentence.empty() && sentence[0] == '$') {
      RCLCPP_DEBUG(logger_, "Parsing NMEA sentence: %s", sentence.c_str());
      if (nmea_parser.parseSentence(sentence)) {
        // Fill the last_fix_, last_velocity_, last_time_ref_ from parsed data
        nmea_parser.fillNavSatFix(last_fix_);
        nmea_parser.fillVelocity(last_velocity_);
        nmea_parser.fillTimeReference(last_time_ref_);
        valid_data_received = true;
      }
    }
    
    prev_pos = pos + 1;
  }
  
  // Update GNSS status based on received data
  if (valid_data_received) {
    updateGnssStatus(nmea_parser.getData());
  }
  
  return valid_data_received;
}

bool GnssDriver::generateSimulatedData()
{
  // Static variables to track simulation state
  static auto last_sim_time = std::chrono::steady_clock::now();
  static double latitude = 48.8566;   // Paris latitude - using a more reasonable local position
  static double longitude = 2.3522;   // Paris longitude
  static double altitude = 35.0;      // Altitude in meters
  static double velocity_north = 0.1; // Meters per second - slow movement
  static double velocity_east = 0.2;  // Meters per second
  
  // Only generate new data every 100ms
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_sim_time);
  
  if (elapsed.count() < 100) {
    return true; // Return true but don't update data yet
  }
  
  last_sim_time = now;
  
  // Update simulated position (about 1 degree is ~111km at equator)
  double time_delta_sec = elapsed.count() / 1000.0;
  latitude += (velocity_north / 111000.0) * time_delta_sec;  
  longitude += (velocity_east / (111000.0 * cos(latitude * M_PI / 180.0))) * time_delta_sec;
  
  // Add some noise to make it realistic
  latitude += (rand() % 100 - 50) * 0.000001;   // ~10cm noise
  longitude += (rand() % 100 - 50) * 0.000001;  // ~10cm noise
  altitude += (rand() % 20 - 10) * 0.01;        // 10cm noise
  
  // Update timestamp
  auto unix_time = std::chrono::system_clock::now();
  auto unix_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    unix_time.time_since_epoch()).count();
    
  // Fill NavSatFix message
  last_fix_.header.stamp.sec = unix_time_ns / 1000000000;
  last_fix_.header.stamp.nanosec = unix_time_ns % 1000000000;
  last_fix_.header.frame_id = config_->getFrameId();
  last_fix_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  last_fix_.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS | 
                            sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
  last_fix_.latitude = latitude;
  last_fix_.longitude = longitude;
  last_fix_.altitude = altitude;
  
  // Set covariance (low for simulation)
  last_fix_.position_covariance[0] = 0.01; // x variance
  last_fix_.position_covariance[4] = 0.01; // y variance
  last_fix_.position_covariance[8] = 0.04; // z variance
  last_fix_.position_covariance_type = 
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    
  // Fill velocity message
  last_velocity_.header = last_fix_.header;
  last_velocity_.twist.twist.linear.x = velocity_north;
  last_velocity_.twist.twist.linear.y = velocity_east;
  last_velocity_.twist.twist.linear.z = 0.0;
  
  // Set velocity covariance
  for (int i = 0; i < 36; i++) {
    last_velocity_.twist.covariance[i] = 0.0;
  }
  last_velocity_.twist.covariance[0] = 0.01; // vx variance
  last_velocity_.twist.covariance[7] = 0.01; // vy variance
  last_velocity_.twist.covariance[14] = 0.01; // vz variance
  
  // Fill time reference
  last_time_ref_.header = last_fix_.header;
  last_time_ref_.time_ref.sec = unix_time_ns / 1000000000;
  last_time_ref_.time_ref.nanosec = unix_time_ns % 1000000000;
  last_time_ref_.source = "sim";
  
  // Generate simulated raw data (simple NMEA-like format)
  std::stringstream raw_ss;
  raw_ss << "$GPGGA," 
         << std::setfill('0') << std::setw(2) << (rand() % 24) << std::setw(2) << (rand() % 60) << std::setw(2) << (rand() % 60) << ".000,"
         << std::fixed << std::setprecision(6) << std::abs(latitude) << (latitude >= 0 ? ",N," : ",S,")
         << std::fixed << std::setprecision(6) << std::abs(longitude) << (longitude >= 0 ? ",E," : ",W,")
         << "1,08,1.0," << std::fixed << std::setprecision(1) << altitude << ",M,0.0,M,,*";
  
  std::string raw_nmea = raw_ss.str();
  last_raw_data_.assign(raw_nmea.begin(), raw_nmea.end());
  
  // Update GNSS status for simulated data
  NmeaParser::GnssData sim_data;
  sim_data.fix_quality = 1;
  sim_data.satellites_used = 8;
  // No satellites_visible field in GnssData struct, use satellites_used instead
  sim_data.hdop = 1.0;
  
  updateGnssStatus(sim_data);
  
  RCLCPP_DEBUG(logger_, "Generated simulated GNSS data: lat=%.6f, lon=%.6f, alt=%.1f",
              latitude, longitude, altitude);
  
  return true;
}

bool GnssDriver::parseUBXMessage(const std::vector<uint8_t>& /* message */)
{
  // Implementation of UBX protocol parser
  // This would handle different UBX message classes and IDs
  
  // In a real implementation, this would parse messages like:
  // - NAV-PVT for position, velocity and time
  // - NAV-SAT for satellite information
  // - NAV-DOP for dilution of precision values
  // - etc.
  
  // For brevity, detailed implementation not included
  
  return true;
}

bool GnssDriver::parseNMEAMessage(const std::string& /* message */)
{
  // Implementation of NMEA message parser
  // This would handle different NMEA sentences
  
  // In a real implementation, this would parse sentences like:
  // - GNGGA for fix data
  // - GNRMC for recommended minimum data
  // - GNVTG for course over ground and speed
  // - etc.
  
  // For brevity, detailed implementation not included
  
  return true;
}

void GnssDriver::updateGnssStatus(const NmeaParser::GnssData& gnss_data)
{
  // Update GNSS status based on the parsed NMEA data
  if (connected_) {
    current_status_.connected = true;
    
    // Only update from NMEA if we're not in GPSD mode
    if (serial_port_fd_ >= 0 || serial_port_fd_ == -2) {
      // Use satellites_used from NMEA parser for satellites_visible in status
      current_status_.satellites_visible = gnss_data.satellites_used;
      current_status_.fix_type = gnss_data.fix_quality;
      current_status_.hdop = gnss_data.hdop;
      current_status_.pdop = 1.5;  // PDOP not directly available from all NMEA sentences
    }
    
    // Create status message based on fix quality
    std::ostringstream status_msg;
    status_msg << "GNSS operational";
    
    // Add data source
    if (serial_port_fd_ == -2) {
      status_msg << " (SIMULATED)";
    } else if (serial_port_fd_ == -3) {
      status_msg << " via GPSD";
    }
    
    status_msg << ": ";
    
    switch (current_status_.fix_type) {
      case 0:
        status_msg << "No fix";
        break;
      case 1:
        status_msg << "GPS fix";
        break;
      case 2:
        status_msg << "DGPS fix";
        break;
      case 4:
        status_msg << "RTK fixed";
        break;
      case 5:
        status_msg << "RTK float";
        break;
      default:
        status_msg << "Fix quality " << current_status_.fix_type;
    }
    
    status_msg << " (satellites: " << current_status_.satellites_visible 
              << ", HDOP: " << current_status_.hdop << ")";
    current_status_.status_message = status_msg.str();
  } else {
    current_status_.connected = false;
    current_status_.satellites_visible = 0;
    current_status_.fix_type = 0;
    current_status_.hdop = 99.9;
    current_status_.pdop = 99.9;
    current_status_.status_message = "Not connected";
  }
}

bool GnssDriver::readFromGpsd()
{
  // Implementation of GPSD client
  // Uses direct socket communication with GPSD on localhost:2947
  static std::string gpsd_host = "localhost";
  static int gpsd_port = 2947;
  static int gpsd_socket = -1;
  static bool gpsd_connected = false;
  static int gpsd_reconnect_attempts = 0;
  static auto last_reconnect_time = std::chrono::steady_clock::now();
  
  // Connect to GPSD if not already connected
  if (!gpsd_connected) {
    // Check if we should attempt reconnection (exponential backoff)
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_reconnect_time).count();
    int backoff_time = std::min(30, (1 << gpsd_reconnect_attempts)); // Max 30 seconds
    
    if (elapsed < backoff_time) {
      return false; // Wait before next reconnect attempt
    }
    
    // Try to connect to GPSD
    RCLCPP_INFO(logger_, "Connecting to GPSD at %s:%d", gpsd_host.c_str(), gpsd_port);
    
    // Create socket
    gpsd_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (gpsd_socket < 0) {
      RCLCPP_ERROR(logger_, "Failed to create socket for GPSD: %s", strerror(errno));
      last_reconnect_time = now;
      gpsd_reconnect_attempts++;
      return false;
    }
    
    // Set up non-blocking socket
    int flags = fcntl(gpsd_socket, F_GETFL, 0);
    fcntl(gpsd_socket, F_SETFL, flags | O_NONBLOCK);
    
    // Set up address
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(gpsd_port);
    
    // Convert hostname to address
    struct hostent* server = gethostbyname(gpsd_host.c_str());
    if (server == nullptr) {
      RCLCPP_ERROR(logger_, "Failed to resolve GPSD host: %s", gpsd_host.c_str());
      close(gpsd_socket);
      gpsd_socket = -1;
      last_reconnect_time = now;
      gpsd_reconnect_attempts++;
      return false;
    }
    
    memcpy(&server_addr.sin_addr.s_addr, server->h_addr, server->h_length);
    
    // Connect to GPSD
    int connect_result = ::connect(gpsd_socket, (struct sockaddr*)&server_addr, sizeof(server_addr));
    if (connect_result < 0 && errno != EINPROGRESS) {
      RCLCPP_ERROR(logger_, "Failed to connect to GPSD: %s", strerror(errno));
      close(gpsd_socket);
      gpsd_socket = -1;
      last_reconnect_time = now;
      gpsd_reconnect_attempts++;
      return false;
    }
    
    // Non-blocking connect can return EINPROGRESS, need to check if it completed
    if (connect_result < 0 && errno == EINPROGRESS) {
      fd_set write_fds;
      struct timeval tv;
      
      FD_ZERO(&write_fds);
      FD_SET(gpsd_socket, &write_fds);
      tv.tv_sec = 2;
      tv.tv_usec = 0;
      
      int select_result = select(gpsd_socket + 1, NULL, &write_fds, NULL, &tv);
      if (select_result <= 0) {
        RCLCPP_ERROR(logger_, "Connection to GPSD timed out");
        close(gpsd_socket);
        gpsd_socket = -1;
        last_reconnect_time = now;
        gpsd_reconnect_attempts++;
        return false;
      }
      
      // Check if connection was successful
      int error = 0;
      socklen_t error_len = sizeof(error);
      getsockopt(gpsd_socket, SOL_SOCKET, SO_ERROR, &error, &error_len);
      if (error != 0) {
        RCLCPP_ERROR(logger_, "Failed to connect to GPSD: %s", strerror(error));
        close(gpsd_socket);
        gpsd_socket = -1;
        last_reconnect_time = now;
        gpsd_reconnect_attempts++;
        return false;
      }
    }
    
    // Connection successful
    RCLCPP_INFO(logger_, "Connected to GPSD");
    gpsd_connected = true;
    gpsd_reconnect_attempts = 0;
    
    // Send commands to enable reporting
    const char* watch_cmd = "?WATCH={\"enable\":true,\"json\":true}\n";
    send(gpsd_socket, watch_cmd, strlen(watch_cmd), 0);
    
    // Small delay to allow GPSD to start sending data
    usleep(100000);  // 100ms
  }
  
  // Read data from GPSD
  char buffer[4096];
  std::string received_data;
  
  // Read available data
  while (true) {
    int bytes_read = recv(gpsd_socket, buffer, sizeof(buffer) - 1, 0);
    
    if (bytes_read > 0) {
      buffer[bytes_read] = '\0';
      received_data += buffer;
    } else if (bytes_read == 0) {
      // Connection closed
      RCLCPP_WARN(logger_, "GPSD connection closed");
      close(gpsd_socket);
      gpsd_socket = -1;
      gpsd_connected = false;
      last_reconnect_time = std::chrono::steady_clock::now();
      return false;
    } else {
      // No more data or error
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_ERROR(logger_, "Error reading from GPSD: %s", strerror(errno));
        close(gpsd_socket);
        gpsd_socket = -1;
        gpsd_connected = false;
        last_reconnect_time = std::chrono::steady_clock::now();
        return false;
      }
      break;  // No more data available
    }
  }
  
  // Process received data
  if (received_data.empty()) {
    // No data received yet
    return false;
  }
  
  // Split into individual JSON messages
  size_t start_pos = 0;
  size_t end_pos = 0;
  bool data_updated = false;
  
  while ((end_pos = received_data.find('\n', start_pos)) != std::string::npos) {
    std::string json_msg = received_data.substr(start_pos, end_pos - start_pos);
    
    // Parse JSON message
    if (!json_msg.empty()) {
      if (processGpsdJson(json_msg)) {
        data_updated = true;
      }
    }
    
    start_pos = end_pos + 1;
  }
  
  return data_updated;
}

bool GnssDriver::processGpsdJson(const std::string& json_msg)
{
  // Simple JSON parser for GPSD messages
  // This is a basic implementation that extracts only what we need
  
  // Check for TPV (Time-Position-Velocity) reports which contain most of the data we need
  if (json_msg.find("\"class\":\"TPV\"") != std::string::npos) {
    // Parse TPV message
    double lat = 0.0, lon = 0.0, alt = 0.0;
    double speed = 0.0, track = 0.0;
    int fix_mode = 0;
    int64_t time_sec = 0, time_nsec = 0;
    
    // Extract latitude
    size_t lat_pos = json_msg.find("\"lat\":");
    if (lat_pos != std::string::npos) {
      lat_pos += 6;  // Skip "\"lat\":"
      size_t lat_end = json_msg.find_first_of(",}", lat_pos);
      if (lat_end != std::string::npos) {
        std::string lat_str = json_msg.substr(lat_pos, lat_end - lat_pos);
        try {
          lat = std::stod(lat_str);
        } catch (...) {
          // Parsing error, keep default value
        }
      }
    }
    
    // Extract longitude
    size_t lon_pos = json_msg.find("\"lon\":");
    if (lon_pos != std::string::npos) {
      lon_pos += 6;  // Skip "\"lon\":"
      size_t lon_end = json_msg.find_first_of(",}", lon_pos);
      if (lon_end != std::string::npos) {
        std::string lon_str = json_msg.substr(lon_pos, lon_end - lon_pos);
        try {
          lon = std::stod(lon_str);
        } catch (...) {
          // Parsing error, keep default value
        }
      }
    }
    
    // Extract altitude
    size_t alt_pos = json_msg.find("\"alt\":");
    if (alt_pos != std::string::npos) {
      alt_pos += 6;  // Skip "\"alt\":"
      size_t alt_end = json_msg.find_first_of(",}", alt_pos);
      if (alt_end != std::string::npos) {
        std::string alt_str = json_msg.substr(alt_pos, alt_end - alt_pos);
        try {
          alt = std::stod(alt_str);
        } catch (...) {
          // Parsing error, keep default value
        }
      }
    }
    
    // Extract speed
    size_t speed_pos = json_msg.find("\"speed\":");
    if (speed_pos != std::string::npos) {
      speed_pos += 8;  // Skip "\"speed\":"
      size_t speed_end = json_msg.find_first_of(",}", speed_pos);
      if (speed_end != std::string::npos) {
        std::string speed_str = json_msg.substr(speed_pos, speed_end - speed_pos);
        try {
          speed = std::stod(speed_str);
        } catch (...) {
          // Parsing error, keep default value
        }
      }
    }
    
    // Extract track angle
    size_t track_pos = json_msg.find("\"track\":");
    if (track_pos != std::string::npos) {
      track_pos += 8;  // Skip "\"track\":"
      size_t track_end = json_msg.find_first_of(",}", track_pos);
      if (track_end != std::string::npos) {
        std::string track_str = json_msg.substr(track_pos, track_end - track_pos);
        try {
          track = std::stod(track_str);
        } catch (...) {
          // Parsing error, keep default value
        }
      }
    }
    
    // Extract fix mode
    size_t mode_pos = json_msg.find("\"mode\":");
    if (mode_pos != std::string::npos) {
      mode_pos += 7;  // Skip "\"mode\":"
      size_t mode_end = json_msg.find_first_of(",}", mode_pos);
      if (mode_end != std::string::npos) {
        std::string mode_str = json_msg.substr(mode_pos, mode_end - mode_pos);
        try {
          fix_mode = std::stoi(mode_str);
        } catch (...) {
          // Parsing error, keep default value
        }
      }
    }
    
    // Extract time if available (ISO8601 format: "2025-03-22T15:45:01.000Z")
    size_t time_pos = json_msg.find("\"time\":");
    if (time_pos != std::string::npos) {
      time_pos = json_msg.find("\"", time_pos + 7) + 1;  // Skip "\"time\":\"" to find the actual time string
      size_t time_end = json_msg.find("\"", time_pos);
      if (time_end != std::string::npos) {
        std::string time_str = json_msg.substr(time_pos, time_end - time_pos);
        
        // Basic ISO8601 parsing (more robust parsing would use a library)
        struct tm tm = {};
        char ns_str[10] = {0};
        
        // Parse date, time and fractions of second
        if (sscanf(time_str.c_str(), "%d-%d-%dT%d:%d:%d.%9s", 
                  &tm.tm_year, &tm.tm_mon, &tm.tm_mday, 
                  &tm.tm_hour, &tm.tm_min, &tm.tm_sec, ns_str) >= 6) {
          
          // Adjust for tm struct expectations
          tm.tm_year -= 1900;  // Years since 1900
          tm.tm_mon -= 1;      // Months are 0-11
          
          // Convert to epoch time
          time_t epoch = timegm(&tm);  // Use timegm instead of mktime to preserve UTC
          time_sec = static_cast<int64_t>(epoch);
          
          // Parse fractional seconds
          int ns_len = strlen(ns_str);
          if (ns_len > 0) {
            // Remove trailing 'Z' if present
            if (ns_str[ns_len-1] == 'Z') {
              ns_str[ns_len-1] = '\0';
              ns_len--;
            }
            
            // Parse nanoseconds
            int ns_val = 0;
            if (sscanf(ns_str, "%d", &ns_val) == 1) {
              // Convert to nanoseconds based on number of digits
              time_nsec = static_cast<int64_t>(ns_val);
              for (int i = ns_len; i < 9; i++) {
                time_nsec *= 10;
              }
            }
          }
        }
      }
    }
    
    // Update NavSatFix message
    last_fix_.header.stamp.sec = time_sec;
    last_fix_.header.stamp.nanosec = time_nsec;
    last_fix_.header.frame_id = config_->getFrameId();
    
    // Set fix status based on mode
    // mode: 0=no mode, 1=no fix, 2=2D fix, 3=3D fix
    if (fix_mode <= 1) {
      last_fix_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    } else if (fix_mode == 2) {
      last_fix_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    } else {
      last_fix_.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
    }
    
    last_fix_.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    last_fix_.latitude = lat;
    last_fix_.longitude = lon;
    last_fix_.altitude = alt;
    
    // Set position covariance (approximated)
    double hdop = 1.0;  // Default HDOP when not available
    size_t hdop_pos = json_msg.find("\"hdop\":");
    if (hdop_pos != std::string::npos) {
      hdop_pos += 7;  // Skip "\"hdop\":"
      size_t hdop_end = json_msg.find_first_of(",}", hdop_pos);
      if (hdop_end != std::string::npos) {
        std::string hdop_str = json_msg.substr(hdop_pos, hdop_end - hdop_pos);
        try {
          hdop = std::stod(hdop_str);
        } catch (...) {
          // Parsing error, keep default value
        }
      }
    }
    
    const double hdop_factor = 2.5;  // Typical conversion factor
    const double min_covariance = 1.0;  // Minimum covariance value
    
    double position_variance = std::max(std::pow(hdop * hdop_factor, 2), min_covariance);
    double altitude_variance = std::max(std::pow(hdop * hdop_factor * 2.0, 2), min_covariance * 2.0);
    
    last_fix_.position_covariance[0] = position_variance;  // East-west variance
    last_fix_.position_covariance[4] = position_variance;  // North-south variance
    last_fix_.position_covariance[8] = altitude_variance;  // Up-down variance
    last_fix_.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    
    // Update velocity message
    last_velocity_.header = last_fix_.header;
    
    // Calculate north and east components from speed and track
    double speed_north = speed * std::cos(track * M_PI / 180.0);
    double speed_east = speed * std::sin(track * M_PI / 180.0);
    
    last_velocity_.twist.twist.linear.x = speed_north;
    last_velocity_.twist.twist.linear.y = speed_east;
    last_velocity_.twist.twist.linear.z = 0.0;
    
    // Set velocity covariance (approximated)
    double velocity_variance = position_variance * 0.1;  // Scale down for velocity
    
    for (int i = 0; i < 36; i++) {
      last_velocity_.twist.covariance[i] = 0.0;
    }
    
    last_velocity_.twist.covariance[0] = velocity_variance;  // vx variance
    last_velocity_.twist.covariance[7] = velocity_variance;  // vy variance
    last_velocity_.twist.covariance[14] = velocity_variance;  // vz variance
    
    // Update time reference
    last_time_ref_.header = last_fix_.header;
    last_time_ref_.time_ref.sec = time_sec;
    last_time_ref_.time_ref.nanosec = time_nsec;
    last_time_ref_.source = "gpsd";
    
    // Update status information
    current_status_.fix_type = fix_mode;
    current_status_.hdop = hdop;
    
    // For consistency with the rest of the code, fix_mode values are mapped to fix_quality
    if (fix_mode == 2) {
      current_status_.fix_type = 1;  // 2D fix -> GPS fix
    } else if (fix_mode == 3) {
      current_status_.fix_type = 2;  // 3D fix -> DGPS fix
    }
    
    return true;
  }
  // Check for SKY reports which contain satellite information
  else if (json_msg.find("\"class\":\"SKY\"") != std::string::npos) {
    // Extract the number of satellites
    int satellites_visible = 0;
    int satellites_used = 0;
    
    // Extract total satellites in view (nSat)
    size_t nsat_pos = json_msg.find("\"nSat\":");
    if (nsat_pos != std::string::npos) {
      nsat_pos += 7;  // Skip "\"nSat\":"
      size_t nsat_end = json_msg.find_first_of(",}", nsat_pos);
      if (nsat_end != std::string::npos) {
        std::string nsat_str = json_msg.substr(nsat_pos, nsat_end - nsat_pos);
        try {
          satellites_visible = std::stoi(nsat_str);
          RCLCPP_DEBUG(logger_, "Parsed satellites visible: %d", satellites_visible);
        } catch (const std::exception& e) {
          RCLCPP_WARN(logger_, "Error parsing satellite count: %s", e.what());
        }
      }
    }
    
    // Extract satellites used in solution (uSat)
    size_t usat_pos = json_msg.find("\"uSat\":");
    if (usat_pos != std::string::npos) {
      usat_pos += 7;  // Skip "\"uSat\":"
      size_t usat_end = json_msg.find_first_of(",}", usat_pos);
      if (usat_end != std::string::npos) {
        std::string usat_str = json_msg.substr(usat_pos, usat_end - usat_pos);
        try {
          satellites_used = std::stoi(usat_str);
          RCLCPP_DEBUG(logger_, "Parsed satellites used: %d", satellites_used);
        } catch (const std::exception& e) {
          RCLCPP_WARN(logger_, "Error parsing satellites used: %s", e.what());
        }
      }
    }
    
    // Update status information with visible satellite count
    if (satellites_visible > 0) {
      RCLCPP_INFO(logger_, "Updating satellite count: %d visible, %d used", 
                 satellites_visible, satellites_used);
      current_status_.satellites_visible = satellites_visible;
    }
    
    // Update status message
    std::ostringstream status_msg;
    status_msg << "GNSS operational via GPSD: ";
    
    switch (current_status_.fix_type) {
      case 0:
        status_msg << "No fix";
        break;
      case 1:
        status_msg << "GPS fix";
        break;
      case 2:
        status_msg << "DGPS fix";
        break;
      case 4:
        status_msg << "RTK fixed";
        break;
      case 5:
        status_msg << "RTK float";
        break;
      default:
        status_msg << "Fix quality " << current_status_.fix_type;
    }
    
    status_msg << " (satellites visible: " << satellites_visible 
              << ", used: " << satellites_used 
              << ", HDOP: " << current_status_.hdop << ")";
    current_status_.status_message = status_msg.str();
    
    RCLCPP_DEBUG(logger_, "GPSD satellite data: visible=%d, used=%d", 
                satellites_visible, satellites_used);
    
    return true;
  }
  
  return false;
}

}  // namespace gnss
}  // namespace sensors