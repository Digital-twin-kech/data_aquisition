#include "sensors/gnss/nmea_parser.h"
#include <cmath>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <chrono>

namespace sensors {
namespace gnss {

NmeaParser::NmeaParser(rclcpp::Logger logger)
  : logger_(logger)
{
  // Initialize NMEA sentence handlers
  handlers_["GGA"] = std::bind(&NmeaParser::handleGGA, this, std::placeholders::_1);
  handlers_["RMC"] = std::bind(&NmeaParser::handleRMC, this, std::placeholders::_1);
  handlers_["VTG"] = std::bind(&NmeaParser::handleVTG, this, std::placeholders::_1);
  handlers_["GSA"] = std::bind(&NmeaParser::handleGSA, this, std::placeholders::_1);
  handlers_["GSV"] = std::bind(&NmeaParser::handleGSV, this, std::placeholders::_1);
}

bool NmeaParser::parseSentence(const std::string& sentence)
{
  // Check if this is a valid NMEA sentence
  if (sentence.empty() || sentence[0] != '$') {
    RCLCPP_DEBUG(logger_, "Invalid NMEA sentence (doesn't start with $): %s", sentence.c_str());
    return false;
  }
  
  // Validate checksum
  if (!validateChecksum(sentence)) {
    RCLCPP_DEBUG(logger_, "Invalid NMEA checksum: %s", sentence.c_str());
    return false;
  }
  
  // Split the sentence into fields
  std::vector<std::string> fields = splitSentence(sentence);
  
  if (fields.empty()) {
    RCLCPP_DEBUG(logger_, "Empty NMEA sentence: %s", sentence.c_str());
    return false;
  }
  
  // Extract sentence type (remove the "$GP" or "$GN" prefix)
  std::string type;
  if (fields[0].length() > 2) {
    type = fields[0].substr(2);
  }
  
  // Find the appropriate handler for this sentence type
  auto it = handlers_.find(type);
  if (it != handlers_.end()) {
    // Call the handler
    bool result = it->second(fields);
    
    if (!result) {
      RCLCPP_DEBUG(logger_, "Failed to parse NMEA sentence: %s", sentence.c_str());
    }
    
    return result;
  }
  
  // No handler found for this sentence type
  RCLCPP_DEBUG(logger_, "Unhandled NMEA sentence type: %s", type.c_str());
  return false;
}

const NmeaParser::GnssData& NmeaParser::getData() const
{
  return data_;
}

bool NmeaParser::fillNavSatFix(sensor_msgs::msg::NavSatFix& msg)
{
  if (!data_.valid_position) {
    return false;
  }
  
  // Set the status based on fix quality
  if (data_.fix_quality == 0) {
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  } else if (data_.fix_quality == 1) {
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  } else if (data_.fix_quality >= 2) {
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
  }
  
  // Set the service to use GPS constellation
  msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  
  // Set position data
  msg.latitude = data_.latitude;
  msg.longitude = data_.longitude;
  msg.altitude = data_.altitude;
  
  // Set position covariance based on HDOP
  // This is a rough approximation, but works for most applications
  const double hdop_factor = 2.5;  // Typical conversion factor
  const double min_covariance = 1.0;  // Minimum covariance value
  
  double position_variance = std::max(std::pow(data_.hdop * hdop_factor, 2), min_covariance);
  double altitude_variance = std::max(std::pow(data_.hdop * hdop_factor * 2.0, 2), min_covariance * 2.0);
  
  msg.position_covariance[0] = position_variance;  // East-west variance
  msg.position_covariance[4] = position_variance;  // North-south variance
  msg.position_covariance[8] = altitude_variance;  // Up-down variance
  
  msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
  
  return true;
}

bool NmeaParser::fillVelocity(geometry_msgs::msg::TwistWithCovarianceStamped& msg)
{
  if (!data_.valid_velocity) {
    return false;
  }
  
  // Calculate x (north) and y (east) components from speed and track angle
  double speed_north = data_.speed * std::cos(data_.track_angle * M_PI / 180.0);
  double speed_east = data_.speed * std::sin(data_.track_angle * M_PI / 180.0);
  
  // Set linear velocity
  msg.twist.twist.linear.x = speed_north;
  msg.twist.twist.linear.y = speed_east;
  msg.twist.twist.linear.z = 0.0;  // No vertical velocity in NMEA
  
  // Set angular velocity to zero (not provided by NMEA)
  msg.twist.twist.angular.x = 0.0;
  msg.twist.twist.angular.y = 0.0;
  msg.twist.twist.angular.z = 0.0;
  
  // Set covariance (diagonal elements only)
  // These are rough approximations based on fix quality
  double linear_covariance;
  if (data_.fix_quality <= 1) {
    linear_covariance = 1.0;  // Standard fix
  } else if (data_.fix_quality <= 3) {
    linear_covariance = 0.5;  // DGPS or better
  } else {
    linear_covariance = 0.1;  // RTK
  }
  
  // Zero out covariance matrix
  for (int i = 0; i < 36; i++) {
    msg.twist.covariance[i] = 0.0;
  }
  
  // Set diagonal elements
  msg.twist.covariance[0] = linear_covariance;   // x linear
  msg.twist.covariance[7] = linear_covariance;   // y linear
  msg.twist.covariance[14] = linear_covariance;  // z linear
  msg.twist.covariance[21] = 99999.0;  // x angular - not available
  msg.twist.covariance[28] = 99999.0;  // y angular - not available
  msg.twist.covariance[35] = 99999.0;  // z angular - not available
  
  return true;
}

bool NmeaParser::fillTimeReference(sensor_msgs::msg::TimeReference& msg)
{
  if (!data_.valid_time) {
    return false;
  }
  
  // Create a tm structure for the current UTC time from GNSS
  std::tm timeinfo = {};
  
  if (data_.year > 0) {
    // We have complete date information
    timeinfo.tm_year = data_.year - 1900;  // years since 1900
    timeinfo.tm_mon = data_.month - 1;     // months since January
    timeinfo.tm_mday = data_.day;          // day of the month
  } else {
    // Only time information, use current date
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::tm* now_tm = std::gmtime(&now_c);
    
    timeinfo.tm_year = now_tm->tm_year;
    timeinfo.tm_mon = now_tm->tm_mon;
    timeinfo.tm_mday = now_tm->tm_mday;
  }
  
  // Set time information
  timeinfo.tm_hour = data_.utc_hours;
  timeinfo.tm_min = data_.utc_minutes;
  timeinfo.tm_sec = static_cast<int>(data_.utc_seconds);
  
  // Convert to epoch time
  std::time_t epoch = std::mktime(&timeinfo);
  
  // Record the fractional seconds
  double fractional_seconds = data_.utc_seconds - static_cast<int>(data_.utc_seconds);
  
  // Convert to nanoseconds
  int32_t nanoseconds = static_cast<int32_t>(fractional_seconds * 1e9);
  
  // Set the time_ref message
  builtin_interfaces::msg::Time time;
  time.sec = static_cast<int32_t>(epoch);
  time.nanosec = nanoseconds;
  
  msg.time_ref = time;
  msg.source = "gnss";
  
  return true;
}

bool NmeaParser::validateChecksum(const std::string& sentence)
{
  // Find the checksum separator "*"
  size_t asterisk_pos = sentence.find_last_of('*');
  if (asterisk_pos == std::string::npos || asterisk_pos + 3 > sentence.length()) {
    // No valid checksum format
    return false;
  }
  
  // Extract the checksum
  std::string checksum_str = sentence.substr(asterisk_pos + 1, 2);
  
  // Compute the checksum
  unsigned char computed_checksum = 0;
  for (size_t i = 1; i < asterisk_pos; i++) {
    computed_checksum ^= sentence[i];
  }
  
  // Convert hex string to value
  unsigned int provided_checksum;
  std::stringstream ss;
  ss << std::hex << checksum_str;
  ss >> provided_checksum;
  
  return (computed_checksum == provided_checksum);
}

std::vector<std::string> NmeaParser::splitSentence(const std::string& sentence)
{
  std::vector<std::string> fields;
  std::stringstream ss(sentence);
  std::string field;
  
  // Skip the $
  ss.get();
  
  // Get each field delimited by commas
  while (std::getline(ss, field, ',')) {
    fields.push_back(field);
  }
  
  // The last field might contain a checksum, remove it
  if (!fields.empty()) {
    size_t asterisk_pos = fields.back().find('*');
    if (asterisk_pos != std::string::npos) {
      fields.back() = fields.back().substr(0, asterisk_pos);
    }
  }
  
  return fields;
}

bool NmeaParser::handleGGA(const std::vector<std::string>& fields)
{
  // GGA format:
  // GGA,time,lat,N/S,lon,E/W,quality,numSats,hdop,alt,M,geoid,M,ageDGPS,refID
  
  if (fields.size() < 15) {
    RCLCPP_DEBUG(logger_, "Invalid GGA sentence: insufficient fields");
    return false;
  }
  
  // Parse time
  parseTime(fields[1]);
  
  // Parse latitude and longitude
  data_.latitude = parseLatitude(fields[2], fields[3]);
  data_.longitude = parseLongitude(fields[4], fields[5]);
  
  // Parse fix quality
  if (!fields[6].empty()) {
    data_.fix_quality = std::stoi(fields[6]);
  } else {
    data_.fix_quality = 0;  // Invalid
  }
  
  // Parse number of satellites
  if (!fields[7].empty()) {
    data_.satellites_used = std::stoi(fields[7]);
  }
  
  // Parse HDOP
  if (!fields[8].empty()) {
    data_.hdop = std::stod(fields[8]);
  }
  
  // Parse altitude
  if (!fields[9].empty()) {
    data_.altitude = std::stod(fields[9]);
  }
  
  // Set valid flags
  data_.valid_position = (data_.fix_quality > 0);
  
  return true;
}

bool NmeaParser::handleRMC(const std::vector<std::string>& fields)
{
  // RMC format:
  // RMC,time,status,lat,N/S,lon,E/W,speed,track,date,magvar,E/W,mode
  
  if (fields.size() < 12) {
    RCLCPP_DEBUG(logger_, "Invalid RMC sentence: insufficient fields");
    return false;
  }
  
  // Parse time
  parseTime(fields[1]);
  
  // Parse status
  bool status_valid = (fields[2] == "A");
  
  // Parse latitude and longitude
  data_.latitude = parseLatitude(fields[3], fields[4]);
  data_.longitude = parseLongitude(fields[5], fields[6]);
  
  // Parse speed and track
  if (!fields[7].empty()) {
    // Convert from knots to m/s
    data_.speed = std::stod(fields[7]) * 0.514444;
  }
  
  if (!fields[8].empty()) {
    data_.track_angle = std::stod(fields[8]);
  }
  
  // Parse date
  parseDate(fields[9]);
  
  // Set valid flags
  data_.valid_position = status_valid;
  data_.valid_velocity = status_valid;
  
  return true;
}

bool NmeaParser::handleVTG(const std::vector<std::string>& fields)
{
  // VTG format:
  // VTG,trackTrue,T,trackMag,M,speedKnots,N,speedKm,K,mode
  
  if (fields.size() < 9) {
    RCLCPP_DEBUG(logger_, "Invalid VTG sentence: insufficient fields");
    return false;
  }
  
  // Parse track angle
  if (!fields[1].empty()) {
    data_.track_angle = std::stod(fields[1]);
  }
  
  // Parse speed
  if (!fields[7].empty()) {
    // Convert from km/h to m/s
    data_.speed = std::stod(fields[7]) / 3.6;
  }
  
  // Mode indicator
  char mode = (fields.size() > 9 && !fields[9].empty()) ? fields[9][0] : 'N';
  
  // Set valid flags based on mode
  // A=Autonomous, D=Differential, E=Estimated, M=Manual, S=Simulator, N=Not valid
  data_.valid_velocity = (mode != 'N');
  
  return true;
}

bool NmeaParser::handleGSA(const std::vector<std::string>& fields)
{
  // GSA format:
  // GSA,mode,fixType,satellites...,pdop,hdop,vdop
  
  if (fields.size() < 18) {
    RCLCPP_DEBUG(logger_, "Invalid GSA sentence: insufficient fields");
    return false;
  }
  
  // Parse fix type
  if (!fields[2].empty()) {
    int fix_type = std::stoi(fields[2]);
    if (fix_type == 2) {
      data_.fix_quality = std::max(data_.fix_quality, 1);  // 2D fix
    } else if (fix_type == 3) {
      data_.fix_quality = std::max(data_.fix_quality, 2);  // 3D fix
    }
  }
  
  // Parse PDOP and HDOP
  if (!fields[15].empty()) {
    data_.pdop = std::stod(fields[15]);
  }
  
  if (!fields[16].empty()) {
    data_.hdop = std::stod(fields[16]);
  }
  
  return true;
}

bool NmeaParser::handleGSV(const std::vector<std::string>& fields)
{
  // GSV format:
  // GSV,numMsg,msgNum,numSats,satID,elevation,azimuth,snr,...
  
  if (fields.size() < 4) {
    RCLCPP_DEBUG(logger_, "Invalid GSV sentence: insufficient fields");
    return false;
  }
  
  // Extract total number of satellites in view
  if (!fields[3].empty()) {
    try {
      int total_sats = std::stoi(fields[3]);
      
      // Only update satellites_used if it's larger than current value
      // as GSV messages come in multiple parts
      if (total_sats > data_.satellites_used) {
        data_.satellites_used = total_sats;
        RCLCPP_DEBUG(logger_, "Updated satellites in view: %d", data_.satellites_used);
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(logger_, "Error parsing satellite count: %s", e.what());
    }
  }
  
  return true;
}

double NmeaParser::parseLatitude(const std::string& lat, const std::string& ns)
{
  if (lat.empty()) {
    return 0.0;
  }
  
  // Convert DDMM.MMMM to decimal degrees
  double value = std::stod(lat);
  
  // Extract the degrees
  int degrees = static_cast<int>(value / 100);
  
  // Extract the minutes
  double minutes = value - (degrees * 100);
  
  // Convert to decimal degrees
  double decimal_degrees = degrees + (minutes / 60.0);
  
  // Apply sign based on N/S
  if (ns == "S") {
    decimal_degrees = -decimal_degrees;
  }
  
  return decimal_degrees;
}

double NmeaParser::parseLongitude(const std::string& lon, const std::string& ew)
{
  if (lon.empty()) {
    return 0.0;
  }
  
  // Convert DDDMM.MMMM to decimal degrees
  double value = std::stod(lon);
  
  // Extract the degrees
  int degrees = static_cast<int>(value / 100);
  
  // Extract the minutes
  double minutes = value - (degrees * 100);
  
  // Convert to decimal degrees
  double decimal_degrees = degrees + (minutes / 60.0);
  
  // Apply sign based on E/W
  if (ew == "W") {
    decimal_degrees = -decimal_degrees;
  }
  
  return decimal_degrees;
}

bool NmeaParser::parseTime(const std::string& time)
{
  if (time.length() < 6) {
    return false;
  }
  
  try {
    // Format: HHMMSS.SSS
    double value = std::stod(time);
    
    // Extract hours, minutes, seconds
    int hms = static_cast<int>(value);
    data_.utc_hours = hms / 10000;
    data_.utc_minutes = (hms % 10000) / 100;
    data_.utc_seconds = (hms % 100) + (value - hms);
    
    data_.valid_time = true;
    return true;
  } catch (const std::exception& e) {
    RCLCPP_WARN(logger_, "Failed to parse NMEA time: %s (%s)", time.c_str(), e.what());
    return false;
  }
}

bool NmeaParser::parseDate(const std::string& date)
{
  if (date.length() != 6) {
    return false;
  }
  
  try {
    // Format: DDMMYY
    int value = std::stoi(date);
    
    // Extract day, month, year
    data_.day = value / 10000;
    data_.month = (value % 10000) / 100;
    data_.year = 2000 + (value % 100);  // Assume 21st century
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_WARN(logger_, "Failed to parse NMEA date: %s (%s)", date.c_str(), e.what());
    return false;
  }
}

}  // namespace gnss
}  // namespace sensors