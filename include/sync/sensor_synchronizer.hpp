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
  std::string sync_policy_;
  double time_tolerance_;
  int cache_size_;
  double max_delay_;
  bool pass_through_; // Whether to directly forward messages without waiting for synchronization
  
  // Synchronization statistics
  struct SyncStats {
    std::string name;
    int messages_received{0};
    int messages_dropped{0};
    int sync_success_count{0};
    double max_time_diff{0.0};
    double total_time_diff{0.0};
    rclcpp::Time last_message_time;
    
    // Keep track of rates
    std::deque<rclcpp::Time> message_timestamps;
    
    void add_message() {
      messages_received++;
      message_timestamps.push_back(rclcpp::Clock().now());
      // Keep only last 100 timestamps for rate calculation
      if (message_timestamps.size() > 100) {
        message_timestamps.pop_front();
      }
    }
    
    double get_rate() const {
      if (message_timestamps.size() < 2) {
        return 0.0;
      }
      
      auto duration = message_timestamps.back() - message_timestamps.front();
      double seconds = duration.seconds();
      return seconds > 0.0 ? message_timestamps.size() / seconds : 0.0;
    }
  };
  
  std::unordered_map<std::string, SyncStats> camera_stats_;
  SyncStats lidar_stats_;
  SyncStats gnss_stats_;
  
  // Timer for reporting statistics
  rclcpp::TimerBase::SharedPtr stats_timer_;
  
  // Publishers
  std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> sync_camera_rgb_pubs_;
  std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> sync_camera_pc_pubs_;
  std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> sync_camera_imu_pubs_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sync_lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr sync_gnss_pub_;
  
  // Message filter subscribers (for synchronized timestamps)
  std::unordered_map<std::string, message_filters::Subscriber<sensor_msgs::msg::Image>> camera_rgb_subs_;
  std::unordered_map<std::string, message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> camera_pc_subs_;
  std::unordered_map<std::string, message_filters::Subscriber<sensor_msgs::msg::Imu>> camera_imu_subs_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> lidar_sub_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::NavSatFix>> gnss_sub_;
  
  // Regular subscribers (for pass-through mode)
  std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> camera_rgb_direct_subs_;
  std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> camera_pc_direct_subs_;
  std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr> camera_imu_direct_subs_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_direct_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_direct_sub_;
  
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
  void initializeDirectSubscribers();
  void loadParameters();
  void initializeStatistics();
  void reportSyncStatistics();
  
  // Sync callbacks
  void cameraSyncCallback(const std::string& camera_name,
                         const sensor_msgs::msg::Image::ConstSharedPtr& img,
                         const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc,
                         const sensor_msgs::msg::Imu::ConstSharedPtr& imu);
  
  void lidarGnssSyncCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& lidar_msg,
                           const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gnss_msg);
                           
  // Direct pass-through callbacks - using standard ROS callback signature
  void cameraRgbCallback(const std::string& camera_name,
                       const sensor_msgs::msg::Image::ConstSharedPtr msg);
  
  void cameraPcCallback(const std::string& camera_name,
                      const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  
  void cameraImuCallback(const std::string& camera_name,
                       const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  
  void lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  
  void gnssCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
};

}  // namespace sync
}  // namespace data_aquisition

#endif  // SENSOR_SYNCHRONIZER_HPP