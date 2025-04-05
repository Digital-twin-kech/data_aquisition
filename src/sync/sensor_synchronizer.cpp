#include "sync/sensor_synchronizer.hpp"

namespace data_aquisition {
namespace sync {

SensorSynchronizer::SensorSynchronizer(const rclcpp::NodeOptions & options)
: rclcpp::Node("sensor_synchronizer", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing Sensor Synchronizer Node");
  
  // Load parameters
  loadParameters();
  
  // Initialize publishers and subscribers
  initializePublishers();
  initializeSubscribers();
  initializeSynchronizers();
  
  // Initialize direct pass-through subscribers if enabled
  if (pass_through_) {
    RCLCPP_INFO(this->get_logger(), "Initializing direct pass-through subscribers");
    initializeDirectSubscribers();
  }
  
  // Initialize statistics
  initializeStatistics();
  
  // Create a timer for periodic statistics reporting
  stats_timer_ = this->create_wall_timer(
    std::chrono::seconds(10),
    std::bind(&SensorSynchronizer::reportSyncStatistics, this));
  
  RCLCPP_INFO(this->get_logger(), "Sensor Synchronizer initialized");
}

SensorSynchronizer::~SensorSynchronizer()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down Sensor Synchronizer");
}

void SensorSynchronizer::loadParameters()
{
  // Declare and load parameters with defaults
  this->declare_parameter<std::vector<std::string>>("camera_names", 
    std::vector<std::string>{"ZED_CAMERA_2i", "ZED_CAMERA_X0", "ZED_CAMERA_X1"});
  this->declare_parameter<bool>("sync_lidar", true);
  this->declare_parameter<bool>("sync_gnss", true);
  this->declare_parameter<std::string>("sync_policy", "ApproximateTime");
  this->declare_parameter<double>("time_tolerance", 0.10);  // Increased from 0.02 to 0.10s (100ms) default
  this->declare_parameter<int>("cache_size", 100);
  this->declare_parameter<double>("max_delay", 0.5);  // 500ms default
  this->declare_parameter<bool>("pass_through", true); // Enable direct pass-through by default
  
  // Get parameters
  this->get_parameter("camera_names", camera_names_);
  this->get_parameter("sync_lidar", sync_lidar_);
  this->get_parameter("sync_gnss", sync_gnss_);
  this->get_parameter("sync_policy", sync_policy_);
  this->get_parameter("time_tolerance", time_tolerance_);
  this->get_parameter("cache_size", cache_size_);
  this->get_parameter("max_delay", max_delay_);
  this->get_parameter("pass_through", pass_through_);
  
  RCLCPP_INFO(this->get_logger(), "Loaded parameters:");
  RCLCPP_INFO(this->get_logger(), "  Sync Policy: %s", sync_policy_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Time Tolerance: %.3f s", time_tolerance_);
  RCLCPP_INFO(this->get_logger(), "  Cache Size: %d", cache_size_);
  RCLCPP_INFO(this->get_logger(), "  Max Delay: %.3f s", max_delay_);
  RCLCPP_INFO(this->get_logger(), "  Pass Through: %s", pass_through_ ? "enabled" : "disabled");
}

void SensorSynchronizer::initializePublishers()
{
  // Use a reliable QoS with history depth based on cache_size parameter
  auto reliable_qos = rclcpp::QoS(cache_size_)
    .reliable()
    .durability_volatile();
  
  RCLCPP_INFO(this->get_logger(), "Publisher QoS: Reliable, History: %d", cache_size_);
  
  // Create camera publishers
  for (const auto & camera_name : camera_names_) {
    // RGB Image publisher
    sync_camera_rgb_pubs_[camera_name] = this->create_publisher<sensor_msgs::msg::Image>(
      "/synchronized/" + camera_name + "/rgb/image_rect_color", reliable_qos);
    
    // Point Cloud publisher
    sync_camera_pc_pubs_[camera_name] = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/synchronized/" + camera_name + "/point_cloud/cloud_registered", reliable_qos);
    
    // IMU publisher
    sync_camera_imu_pubs_[camera_name] = this->create_publisher<sensor_msgs::msg::Imu>(
      "/synchronized/" + camera_name + "/IMU", reliable_qos);
    
    RCLCPP_INFO(this->get_logger(), "Created publishers for camera: %s", camera_name.c_str());
  }
  
  // LiDAR publisher
  if (sync_lidar_) {
    sync_lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/synchronized/livox/lidar", reliable_qos);
    RCLCPP_INFO(this->get_logger(), "Created LiDAR publisher");
  }
  
  // GNSS publisher
  if (sync_gnss_) {
    sync_gnss_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/synchronized/gnss/fix", reliable_qos);
    RCLCPP_INFO(this->get_logger(), "Created GNSS publisher");
  }
}

void SensorSynchronizer::initializeSubscribers()
{
  // Configure sensor data QoS profile with best effort reliability and appropriate history
  auto sensor_qos = rclcpp::SensorDataQoS()
    .keep_last(cache_size_)
    .best_effort()
    .durability_volatile()
    .get_rmw_qos_profile();
  
  RCLCPP_INFO(this->get_logger(), "Subscriber QoS: Best Effort, History: %d", cache_size_);
  
  // Create message filter subscribers for each camera
  for (const auto & camera_name : camera_names_) {
    // RGB Image subscriber
    std::string rgb_topic = "/" + camera_name + "/rgb/image_rect_color";
    camera_rgb_subs_[camera_name].subscribe(this, rgb_topic, sensor_qos);
    RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", rgb_topic.c_str());
    
    // Point Cloud subscriber
    std::string pc_topic = "/" + camera_name + "/point_cloud/cloud_registered";
    camera_pc_subs_[camera_name].subscribe(this, pc_topic, sensor_qos);
    RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", pc_topic.c_str());
    
    // IMU subscriber
    std::string imu_topic = "/" + camera_name + "/IMU";
    camera_imu_subs_[camera_name].subscribe(this, imu_topic, sensor_qos);
    RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", imu_topic.c_str());
  }
  
  // LiDAR subscriber
  if (sync_lidar_) {
    lidar_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>();
    lidar_sub_->subscribe(this, "/livox/lidar", sensor_qos);
    RCLCPP_INFO(this->get_logger(), "Subscribed to: /livox/lidar");
  }
  
  // GNSS subscriber
  if (sync_gnss_) {
    gnss_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::NavSatFix>>();
    gnss_sub_->subscribe(this, "/gnss/fix", sensor_qos);
    RCLCPP_INFO(this->get_logger(), "Subscribed to: /gnss/fix");
  }
}

void SensorSynchronizer::initializeSynchronizers()
{
  // Create camera synchronizers
  for (const auto & camera_name : camera_names_) {
    if (camera_rgb_subs_.count(camera_name) > 0 && 
        camera_pc_subs_.count(camera_name) > 0 && 
        camera_imu_subs_.count(camera_name) > 0) {
      
      // Create synchronizer with the configured cache size      
      camera_syncs_[camera_name] = std::make_shared<message_filters::Synchronizer<CameraSyncPolicy>>(
        CameraSyncPolicy(cache_size_),
        camera_rgb_subs_[camera_name],
        camera_pc_subs_[camera_name],
        camera_imu_subs_[camera_name]);
      
      // Set time tolerance if using ApproximateTime
      if (sync_policy_.compare("ApproximateTime") == 0) {
        camera_syncs_[camera_name]->setMaxIntervalDuration(rclcpp::Duration::from_seconds(time_tolerance_));
      }
      
      // Register callback using std::bind with placeholders
      using std::placeholders::_1;
      using std::placeholders::_2;
      using std::placeholders::_3;
      
      camera_syncs_[camera_name]->registerCallback(
        std::bind(&SensorSynchronizer::cameraSyncCallback, this, camera_name, _1, _2, _3));
      
      RCLCPP_INFO(this->get_logger(), "Created synchronizer for camera: %s with %s policy", 
                 camera_name.c_str(), sync_policy_.c_str());
    }
  }
  
  // Create LiDAR-GNSS synchronizer if both are enabled
  if (sync_lidar_ && sync_gnss_ && lidar_sub_ && gnss_sub_) {
    // Create synchronizer with queue size
    lidar_gnss_sync_ = std::make_shared<message_filters::Synchronizer<LidarGnssSyncPolicy>>(
      LidarGnssSyncPolicy(cache_size_), *lidar_sub_, *gnss_sub_);
    
    // Set time tolerance if using ApproximateTime
    if (sync_policy_.compare("ApproximateTime") == 0) {
      lidar_gnss_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(time_tolerance_));
    }
    
    using std::placeholders::_1;
    using std::placeholders::_2;
    
    lidar_gnss_sync_->registerCallback(
      std::bind(&SensorSynchronizer::lidarGnssSyncCallback, this, _1, _2));
    
    RCLCPP_INFO(this->get_logger(), "Created synchronizer for LiDAR and GNSS with %s policy", 
               sync_policy_.c_str());
  }
}

void SensorSynchronizer::cameraSyncCallback(
  const std::string& camera_name,
  const sensor_msgs::msg::Image::ConstSharedPtr& img,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc,
  const sensor_msgs::msg::Imu::ConstSharedPtr& imu)
{
  // Calculate timestamp differences for diagnostics
  rclcpp::Time img_time(img->header.stamp);
  rclcpp::Time pc_time(pc->header.stamp);
  rclcpp::Time imu_time(imu->header.stamp);
  
  double img_pc_diff = std::abs((img_time - pc_time).seconds());
  double img_imu_diff = std::abs((img_time - imu_time).seconds());
  double pc_imu_diff = std::abs((pc_time - imu_time).seconds());
  double max_diff = std::max({img_pc_diff, img_imu_diff, pc_imu_diff});
  
  // Update synchronization statistics
  if (camera_stats_.count(camera_name) > 0) {
    camera_stats_[camera_name].sync_success_count++;
    camera_stats_[camera_name].total_time_diff += max_diff;
    camera_stats_[camera_name].max_time_diff = std::max(camera_stats_[camera_name].max_time_diff, max_diff);
    camera_stats_[camera_name].last_message_time = this->now();
    camera_stats_[camera_name].add_message();
  }
  
  if (max_diff > time_tolerance_) {
    RCLCPP_WARN(this->get_logger(), 
               "Camera %s: Large timestamp difference %.3f s (> tolerance %.3f s)", 
               camera_name.c_str(), max_diff, time_tolerance_);
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Camera %s: Max timestamp diff %.3f s", 
                camera_name.c_str(), max_diff);
  }
  
  // Publish synchronized messages
  if (sync_camera_rgb_pubs_.count(camera_name) > 0) {
    sync_camera_rgb_pubs_[camera_name]->publish(*img);
  }
  
  if (sync_camera_pc_pubs_.count(camera_name) > 0) {
    sync_camera_pc_pubs_[camera_name]->publish(*pc);
  }
  
  if (sync_camera_imu_pubs_.count(camera_name) > 0) {
    sync_camera_imu_pubs_[camera_name]->publish(*imu);
  }
  
  // Log at debug level to avoid flooding logs
  RCLCPP_DEBUG(this->get_logger(), "Published synchronized data for camera %s", camera_name.c_str());
}

void SensorSynchronizer::lidarGnssSyncCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& lidar_msg,
  const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gnss_msg)
{
  // Calculate timestamp difference
  rclcpp::Time lidar_time(lidar_msg->header.stamp);
  rclcpp::Time gnss_time(gnss_msg->header.stamp);
  double time_diff = std::abs((lidar_time - gnss_time).seconds());
  
  // Update statistics
  lidar_stats_.sync_success_count++;
  lidar_stats_.total_time_diff += time_diff;
  lidar_stats_.max_time_diff = std::max(lidar_stats_.max_time_diff, time_diff);
  lidar_stats_.last_message_time = this->now();
  lidar_stats_.add_message();
  
  gnss_stats_.sync_success_count++;
  gnss_stats_.total_time_diff += time_diff;
  gnss_stats_.max_time_diff = std::max(gnss_stats_.max_time_diff, time_diff);
  gnss_stats_.last_message_time = this->now();
  gnss_stats_.add_message();
  
  if (time_diff > time_tolerance_) {
    RCLCPP_WARN(this->get_logger(), 
               "LiDAR-GNSS: Large timestamp difference %.3f s (> tolerance %.3f s)", 
               time_diff, time_tolerance_);
  } else {
    RCLCPP_DEBUG(this->get_logger(), "LiDAR-GNSS: Timestamp diff %.3f s", time_diff);
  }
  
  // Publish synchronized messages
  if (sync_lidar_pub_) {
    sync_lidar_pub_->publish(*lidar_msg);
  }
  
  if (sync_gnss_pub_) {
    sync_gnss_pub_->publish(*gnss_msg);
  }
  
  // Log at debug level to avoid flooding logs
  RCLCPP_DEBUG(this->get_logger(), "Published synchronized LiDAR and GNSS data");
}

void SensorSynchronizer::initializeDirectSubscribers()
{
  // Configure sensor data QoS profile with best effort reliability for direct subscriptions
  auto sensor_qos = rclcpp::SensorDataQoS()
    .keep_last(cache_size_)
    .best_effort()
    .durability_volatile();
  
  // Create direct subscribers for each camera
  for (const auto & camera_name : camera_names_) {
    // RGB Image direct subscriber - using lambda for callback
    std::string rgb_topic = "/" + camera_name + "/rgb/image_rect_color";
    camera_rgb_direct_subs_[camera_name] = this->create_subscription<sensor_msgs::msg::Image>(
      rgb_topic, 
      sensor_qos,
      [this, camera_name](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        this->cameraRgbCallback(camera_name, msg);
      });
    
    // Point Cloud direct subscriber - using lambda for callback
    std::string pc_topic = "/" + camera_name + "/point_cloud/cloud_registered";
    camera_pc_direct_subs_[camera_name] = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pc_topic, 
      sensor_qos,
      [this, camera_name](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        this->cameraPcCallback(camera_name, msg);
      });
    
    // IMU direct subscriber - using lambda for callback
    std::string imu_topic = "/" + camera_name + "/IMU";
    camera_imu_direct_subs_[camera_name] = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, 
      sensor_qos,
      [this, camera_name](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        this->cameraImuCallback(camera_name, msg);
      });
    
    RCLCPP_INFO(this->get_logger(), "Created direct pass-through subscribers for camera: %s", camera_name.c_str());
  }
  
  // LiDAR direct subscriber
  if (sync_lidar_) {
    lidar_direct_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar",
      sensor_qos,
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        this->lidarCallback(msg);
      });
    RCLCPP_INFO(this->get_logger(), "Created direct pass-through subscriber for LiDAR");
  }
  
  // GNSS direct subscriber
  if (sync_gnss_) {
    gnss_direct_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gnss/fix",
      sensor_qos,
      [this](const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
        this->gnssCallback(msg);
      });
    RCLCPP_INFO(this->get_logger(), "Created direct pass-through subscriber for GNSS");
  }
}

// Direct pass-through callbacks
void SensorSynchronizer::cameraRgbCallback(
  const std::string& camera_name,
  const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  if (sync_camera_rgb_pubs_.count(camera_name) > 0) {
    RCLCPP_DEBUG(this->get_logger(), "Direct pass-through for camera %s RGB image", camera_name.c_str());
    sync_camera_rgb_pubs_[camera_name]->publish(*msg);
    
    // Update statistics
    if (camera_stats_.count(camera_name) > 0) {
      camera_stats_[camera_name].messages_received++;
      camera_stats_[camera_name].add_message();
    }
  }
}

void SensorSynchronizer::cameraPcCallback(
  const std::string& camera_name,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (sync_camera_pc_pubs_.count(camera_name) > 0) {
    RCLCPP_DEBUG(this->get_logger(), "Direct pass-through for camera %s point cloud", camera_name.c_str());
    sync_camera_pc_pubs_[camera_name]->publish(*msg);
    
    // Update statistics
    if (camera_stats_.count(camera_name) > 0) {
      camera_stats_[camera_name].messages_received++;
    }
  }
}

void SensorSynchronizer::cameraImuCallback(
  const std::string& camera_name,
  const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (sync_camera_imu_pubs_.count(camera_name) > 0) {
    RCLCPP_DEBUG(this->get_logger(), "Direct pass-through for camera %s IMU", camera_name.c_str());
    sync_camera_imu_pubs_[camera_name]->publish(*msg);
    
    // Update statistics
    if (camera_stats_.count(camera_name) > 0) {
      camera_stats_[camera_name].messages_received++;
    }
  }
}

void SensorSynchronizer::lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (sync_lidar_pub_) {
    RCLCPP_DEBUG(this->get_logger(), "Direct pass-through for LiDAR");
    sync_lidar_pub_->publish(*msg);
    
    // Update statistics
    lidar_stats_.messages_received++;
    lidar_stats_.add_message();
  }
}

void SensorSynchronizer::gnssCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  if (sync_gnss_pub_) {
    RCLCPP_DEBUG(this->get_logger(), "Direct pass-through for GNSS");
    sync_gnss_pub_->publish(*msg);
  }
  
  // Update statistics
  gnss_stats_.messages_received++;
}

void SensorSynchronizer::initializeStatistics() 
{
  RCLCPP_INFO(this->get_logger(), "Initializing synchronization statistics tracking");
  
  // Initialize camera statistics
  for (const auto& camera_name : camera_names_) {
    camera_stats_[camera_name].name = camera_name;
    camera_stats_[camera_name].messages_received = 0;
    camera_stats_[camera_name].messages_dropped = 0;
    camera_stats_[camera_name].sync_success_count = 0;
    camera_stats_[camera_name].max_time_diff = 0.0;
    camera_stats_[camera_name].total_time_diff = 0.0;
  }
  
  // Initialize LiDAR statistics
  lidar_stats_.name = "LiDAR";
  lidar_stats_.messages_received = 0;
  lidar_stats_.messages_dropped = 0;
  lidar_stats_.sync_success_count = 0;
  lidar_stats_.max_time_diff = 0.0;
  lidar_stats_.total_time_diff = 0.0;
  
  // Initialize GNSS statistics
  gnss_stats_.name = "GNSS";
  gnss_stats_.messages_received = 0;
  gnss_stats_.messages_dropped = 0;
  gnss_stats_.sync_success_count = 0;
  gnss_stats_.max_time_diff = 0.0;
  gnss_stats_.total_time_diff = 0.0;
}

void SensorSynchronizer::reportSyncStatistics()
{
  RCLCPP_INFO(this->get_logger(), "==== Synchronization Statistics ====");
  RCLCPP_INFO(this->get_logger(), "Time tolerance: %.3f seconds", time_tolerance_);
  RCLCPP_INFO(this->get_logger(), "Cache size: %d", cache_size_);
  RCLCPP_INFO(this->get_logger(), "Sync policy: %s", sync_policy_.c_str());
  RCLCPP_INFO(this->get_logger(), "Pass-through mode: %s", pass_through_ ? "enabled" : "disabled");
  RCLCPP_INFO(this->get_logger(), " ");
  
  // Report camera stats
  for (const auto& [name, stats] : camera_stats_) {
    double avg_diff = stats.sync_success_count > 0 ? stats.total_time_diff / stats.sync_success_count : 0.0;
    
    RCLCPP_INFO(this->get_logger(), "Camera %s:", name.c_str());
    RCLCPP_INFO(this->get_logger(), "  Message rate: %.2f Hz", stats.get_rate());
    RCLCPP_INFO(this->get_logger(), "  Sync success count: %d", stats.sync_success_count);
    RCLCPP_INFO(this->get_logger(), "  Avg timestamp diff: %.3f s", avg_diff);
    RCLCPP_INFO(this->get_logger(), "  Max timestamp diff: %.3f s", stats.max_time_diff);
  }
  
  // Report LiDAR stats
  double lidar_avg_diff = lidar_stats_.sync_success_count > 0 ? 
    lidar_stats_.total_time_diff / lidar_stats_.sync_success_count : 0.0;
  
  RCLCPP_INFO(this->get_logger(), "LiDAR:");
  RCLCPP_INFO(this->get_logger(), "  Message rate: %.2f Hz", lidar_stats_.get_rate());
  RCLCPP_INFO(this->get_logger(), "  Sync success count: %d", lidar_stats_.sync_success_count);
  RCLCPP_INFO(this->get_logger(), "  Avg timestamp diff: %.3f s", lidar_avg_diff);
  RCLCPP_INFO(this->get_logger(), "  Max timestamp diff: %.3f s", lidar_stats_.max_time_diff);
  
  // Report GNSS stats
  double gnss_avg_diff = gnss_stats_.sync_success_count > 0 ? 
    gnss_stats_.total_time_diff / gnss_stats_.sync_success_count : 0.0;
  
  RCLCPP_INFO(this->get_logger(), "GNSS:");
  RCLCPP_INFO(this->get_logger(), "  Message rate: %.2f Hz", gnss_stats_.get_rate());
  RCLCPP_INFO(this->get_logger(), "  Sync success count: %d", gnss_stats_.sync_success_count);
  RCLCPP_INFO(this->get_logger(), "  Avg timestamp diff: %.3f s", gnss_avg_diff);
  RCLCPP_INFO(this->get_logger(), "  Max timestamp diff: %.3f s", gnss_stats_.max_time_diff);
  
  RCLCPP_INFO(this->get_logger(), "====================================");
}

}  // namespace sync
}  // namespace data_aquisition