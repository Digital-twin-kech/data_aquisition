#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <functional>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include "rclcpp/rclcpp.hpp"

namespace sensors {
namespace gnss {

/**
 * @brief NMEA sentence parser for GNSS data
 * 
 * This class parses NMEA sentences from GNSS receivers
 * and provides structured data access.
 */
class NmeaParser {
public:
  /**
   * @brief Structure to hold parsed GNSS data
   */
  struct GnssData {
    // Fix data
    double latitude;
    double longitude;
    double altitude;
    int fix_quality;  // 0=invalid, 1=GPS fix, 2=DGPS fix, 4=RTK fix, 5=Float RTK
    int satellites_used;
    double hdop;
    double pdop;
    
    // Velocity data
    double speed;       // m/s
    double track_angle; // degrees from true north
    
    // Time data
    uint32_t utc_seconds;  // seconds since midnight
    uint8_t utc_hours;
    uint8_t utc_minutes;
    
    // Date data (if available)
    uint8_t day;
    uint8_t month;
    uint16_t year;
    
    // Status flags
    bool valid_position;
    bool valid_velocity;
    bool valid_time;
    
    // Initialize with default values
    GnssData()
      : latitude(0.0),
        longitude(0.0),
        altitude(0.0),
        fix_quality(0),
        satellites_used(0),
        hdop(99.9),
        pdop(99.9),
        speed(0.0),
        track_angle(0.0),
        utc_seconds(0),
        utc_hours(0),
        utc_minutes(0),
        day(0),
        month(0),
        year(0),
        valid_position(false),
        valid_velocity(false),
        valid_time(false)
    {}
  };
  
  /**
   * @brief Constructor
   * 
   * @param logger ROS logger for status messages
   */
  explicit NmeaParser(rclcpp::Logger logger);
  
  /**
   * @brief Parse a NMEA sentence
   * 
   * @param sentence The NMEA sentence to parse
   * @return true if parsing successful
   * @return false if parsing failed
   */
  bool parseSentence(const std::string& sentence);
  
  /**
   * @brief Get the parsed GNSS data
   * 
   * @return const GnssData& Reference to the parsed data
   */
  const GnssData& getData() const;
  
  /**
   * @brief Fill a NavSatFix message with parsed data
   * 
   * @param msg NavSatFix message to fill
   * @return true if data is valid and message was filled
   * @return false if data is invalid
   */
  bool fillNavSatFix(sensor_msgs::msg::NavSatFix& msg);
  
  /**
   * @brief Fill a TwistWithCovarianceStamped message with parsed data
   * 
   * @param msg TwistWithCovarianceStamped message to fill
   * @return true if data is valid and message was filled
   * @return false if data is invalid
   */
  bool fillVelocity(geometry_msgs::msg::TwistWithCovarianceStamped& msg);
  
  /**
   * @brief Fill a TimeReference message with parsed data
   * 
   * @param msg TimeReference message to fill
   * @return true if data is valid and message was filled
   * @return false if data is invalid
   */
  bool fillTimeReference(sensor_msgs::msg::TimeReference& msg);
  
private:
  // GNSS data structure
  GnssData data_;
  
  // ROS logger
  rclcpp::Logger logger_;
  
  // NMEA sentence parsers
  using NmeaHandler = std::function<bool(const std::vector<std::string>&)>;
  std::map<std::string, NmeaHandler> handlers_;
  
  // Parse NMEA checksum
  bool validateChecksum(const std::string& sentence);
  
  // Split NMEA sentence into fields
  std::vector<std::string> splitSentence(const std::string& sentence);
  
  // Individual NMEA sentence handlers
  bool handleGGA(const std::vector<std::string>& fields);
  bool handleRMC(const std::vector<std::string>& fields);
  bool handleVTG(const std::vector<std::string>& fields);
  bool handleGSA(const std::vector<std::string>& fields);
  bool handleGSV(const std::vector<std::string>& fields);
  
  // Utility functions
  double parseLatitude(const std::string& lat, const std::string& ns);
  double parseLongitude(const std::string& lon, const std::string& ew);
  bool parseTime(const std::string& time);
  bool parseDate(const std::string& date);
};

}  // namespace gnss
}  // namespace sensors

#endif  // NMEA_PARSER_H