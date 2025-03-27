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
  
  // Get parameters
  this->get_parameter("camera_names", camera_names_);
  this->get_parameter("sync_lidar", sync_lidar_);
  this->get_parameter("sync_gnss", sync_gnss_);
  
  RCLCPP_INFO(this->get_logger(), "Loaded parameters");
}

void SensorSynchronizer::initializePublishers()
{
  auto reliable_qos = rclcpp::QoS(10).reliable();
  
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
  auto sensor_qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();
  
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
      
      // Create synchronizer with a queue size of 10
      camera_syncs_[camera_name] = std::make_shared<message_filters::Synchronizer<CameraSyncPolicy>>(
        CameraSyncPolicy(10),
        camera_rgb_subs_[camera_name],
        camera_pc_subs_[camera_name],
        camera_imu_subs_[camera_name]);
      
      // Register callback using std::bind with placeholders
      using std::placeholders::_1;
      using std::placeholders::_2;
      using std::placeholders::_3;
      
      camera_syncs_[camera_name]->registerCallback(
        std::bind(&SensorSynchronizer::cameraSyncCallback, this, camera_name, _1, _2, _3));
      
      RCLCPP_INFO(this->get_logger(), "Created synchronizer for camera: %s", camera_name.c_str());
    }
  }
  
  // Create LiDAR-GNSS synchronizer if both are enabled
  if (sync_lidar_ && sync_gnss_ && lidar_sub_ && gnss_sub_) {
    lidar_gnss_sync_ = std::make_shared<message_filters::Synchronizer<LidarGnssSyncPolicy>>(
      LidarGnssSyncPolicy(10), *lidar_sub_, *gnss_sub_);
    
    using std::placeholders::_1;
    using std::placeholders::_2;
    
    lidar_gnss_sync_->registerCallback(
      std::bind(&SensorSynchronizer::lidarGnssSyncCallback, this, _1, _2));
    
    RCLCPP_INFO(this->get_logger(), "Created synchronizer for LiDAR and GNSS");
  }
}

void SensorSynchronizer::cameraSyncCallback(
  const std::string& camera_name,
  const sensor_msgs::msg::Image::ConstSharedPtr& img,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc,
  const sensor_msgs::msg::Imu::ConstSharedPtr& imu)
{
  RCLCPP_INFO(this->get_logger(), "Received synchronized data for camera %s", camera_name.c_str());
  
  if (sync_camera_rgb_pubs_.count(camera_name) > 0) {
    sync_camera_rgb_pubs_[camera_name]->publish(*img);
  }
  
  if (sync_camera_pc_pubs_.count(camera_name) > 0) {
    sync_camera_pc_pubs_[camera_name]->publish(*pc);
  }
  
  if (sync_camera_imu_pubs_.count(camera_name) > 0) {
    sync_camera_imu_pubs_[camera_name]->publish(*imu);
  }
  
  RCLCPP_INFO(this->get_logger(), "Published synchronized data for camera %s", camera_name.c_str());
}

void SensorSynchronizer::lidarGnssSyncCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& lidar_msg,
  const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gnss_msg)
{
  RCLCPP_INFO(this->get_logger(), "Received synchronized LiDAR and GNSS data");
  
  if (sync_lidar_pub_) {
    sync_lidar_pub_->publish(*lidar_msg);
  }
  
  if (sync_gnss_pub_) {
    sync_gnss_pub_->publish(*gnss_msg);
  }
  
  RCLCPP_INFO(this->get_logger(), "Published synchronized LiDAR and GNSS data");
}

}  // namespace sync
}  // namespace data_aquisition