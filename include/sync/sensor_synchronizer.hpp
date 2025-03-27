#ifndef SENSOR_SYNCHRONIZER_HPP
#define SENSOR_SYNCHRONIZER_HPP

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace data_aquisition {
namespace sync {

class SensorSynchronizer : public rclcpp::Node {
public:
  explicit SensorSynchronizer(const rclcpp::NodeOptions & options);
  virtual ~SensorSynchronizer();

private:
  // Parameters
  std::vector<std::string> camera_names_;
  bool sync_lidar_;
  bool sync_gnss_;
  
  // Publishers
  std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> sync_camera_rgb_pubs_;
  std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> sync_camera_pc_pubs_;
  std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> sync_camera_imu_pubs_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sync_lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr sync_gnss_pub_;
  
  // Message filter subscribers
  std::unordered_map<std::string, message_filters::Subscriber<sensor_msgs::msg::Image>> camera_rgb_subs_;
  std::unordered_map<std::string, message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> camera_pc_subs_;
  std::unordered_map<std::string, message_filters::Subscriber<sensor_msgs::msg::Imu>> camera_imu_subs_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> lidar_sub_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::NavSatFix>> gnss_sub_;
  
  // Synchronizers for different sensor combinations
  using CameraSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Imu>;
  std::unordered_map<std::string, std::shared_ptr<message_filters::Synchronizer<CameraSyncPolicy>>> camera_syncs_;
  
  using LidarGnssSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, sensor_msgs::msg::NavSatFix>;
  std::shared_ptr<message_filters::Synchronizer<LidarGnssSyncPolicy>> lidar_gnss_sync_;
  
  // Initialize subscribers and publishers
  void initializeSubscribers();
  void initializePublishers();
  void initializeSynchronizers();
  void loadParameters();
  
  // Sync callbacks
  void cameraSyncCallback(const std::string& camera_name,
                         const sensor_msgs::msg::Image::ConstSharedPtr& img,
                         const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc,
                         const sensor_msgs::msg::Imu::ConstSharedPtr& imu);
  
  void lidarGnssSyncCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& lidar_msg,
                           const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gnss_msg);
};

}  // namespace sync
}  // namespace data_aquisition

#endif  // SENSOR_SYNCHRONIZER_HPP