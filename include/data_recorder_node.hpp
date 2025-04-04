#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <fstream>
#include <string>

class DataRecorderNode : public rclcpp::Node {
public:
    DataRecorderNode(const std::string & base_output_dir);
    ~DataRecorderNode();

private:
    // Helper methods
    std::string getTimestampStr(const builtin_interfaces::msg::Time & stamp);
    std::string currentDateTimeString();
    
    // Data saving methods
    void saveImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg, const std::string &folder);
    void savePointCloudRaw(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, const std::string &folder);
    void savePointCloudPLY(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, const std::string &folder);
    void saveImu(const sensor_msgs::msg::Imu::ConstSharedPtr &msg, std::ofstream &outfile);
    void saveNavSatFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &msg, std::ofstream &outfile);
    
    // Member variables
    std::string base_path_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_2i_, image_sub_X0_, image_sub_X1_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr camera_pc_sub_2i_, camera_pc_sub_X0_, camera_pc_sub_X1_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_2i_, imu_sub_X0_, imu_sub_X1_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
    
    // File streams for CSV logs
    std::ofstream imu_file_2i_, imu_file_X0_, imu_file_X1_, gnss_file_;
};