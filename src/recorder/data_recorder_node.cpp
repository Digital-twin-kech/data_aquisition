#include "data_recorder_node.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <sstream>

DataRecorderNode::DataRecorderNode(const std::string &base_output_dir)
  : Node("data_recorder")
{
    // Declare parameters
    this->declare_parameter("output_dir", base_output_dir);
    
    // Get parameter value
    std::string output_dir = this->get_parameter("output_dir").as_string();
    
    // Determine session directory name using current time
    std::string session_dir = output_dir + "/" + currentDateTimeString();
    base_path_ = session_dir;
    
    // Create directories for each topic
    RCLCPP_INFO(this->get_logger(), "Creating directories in %s", session_dir.c_str());
    std::filesystem::create_directories(session_dir + "/ZED_CAMERA_2i_IMU");
    std::filesystem::create_directories(session_dir + "/ZED_CAMERA_2i_rgb_image_rect_color");
    std::filesystem::create_directories(base_path_ + "/ZED_CAMERA_X0_IMU");
    std::filesystem::create_directories(base_path_ + "/ZED_CAMERA_X0_rgb_image_rect_color");
    std::filesystem::create_directories(base_path_ + "/ZED_CAMERA_X1_IMU");
    std::filesystem::create_directories(base_path_ + "/ZED_CAMERA_X1_rgb_image_rect_color");
    std::filesystem::create_directories(base_path_ + "/gnss_fix");
    std::filesystem::create_directories(base_path_ + "/point_clouds");
    
    std::filesystem::create_directories(base_path_ + "/ZED_CAMERA_2i_point_clouds");
    std::filesystem::create_directories(base_path_ + "/ZED_CAMERA_X0_point_clouds");
    std::filesystem::create_directories(base_path_ + "/ZED_CAMERA_X1_point_clouds");
    

    // Open CSV files for IMU and GNSS topics
    imu_file_2i_.open(session_dir + "/ZED_CAMERA_2i_IMU/data.csv");
    imu_file_X0_.open(session_dir + "/ZED_CAMERA_X0_IMU/data.csv");
    imu_file_X1_.open(session_dir + "/ZED_CAMERA_X1_IMU/data.csv");
    gnss_file_.open(session_dir + "/gnss_fix/data.csv");
    
    // Write headers to CSV files
    imu_file_2i_ << "timestamp,qw,qx,qy,qz,ang_vel_x,ang_vel_y,ang_vel_z,lin_acc_x,lin_acc_y,lin_acc_z\n";
    imu_file_X0_ << "timestamp,qw,qx,qy,qz,ang_vel_x,ang_vel_y,ang_vel_z,lin_acc_x,lin_acc_y,lin_acc_z\n";
    imu_file_X1_ << "timestamp,qw,qx,qy,qz,ang_vel_x,ang_vel_y,ang_vel_z,lin_acc_x,lin_acc_y,lin_acc_z\n";
    gnss_file_ << "timestamp,latitude,longitude,altitude\n";
    
    // Set up subscriptions with lambda callbacks
    using std::placeholders::_1;
    
    // ZED Camera 2i subscriptions
    image_sub_2i_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/synchronized/ZED_CAMERA_2i/rgb/image_rect_color", 10,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
            this->saveImage(msg, "ZED_CAMERA_2i_rgb_image_rect_color");
        });
    
    imu_sub_2i_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/synchronized/ZED_CAMERA_2i/IMU", 100,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
            this->saveImu(msg, imu_file_2i_);
        });
    
    // ZED Camera X0 subscriptions
    image_sub_X0_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/synchronized/ZED_CAMERA_X0/rgb/image_rect_color", 10,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
            this->saveImage(msg, "ZED_CAMERA_X0_rgb_image_rect_color");
        });
    
    imu_sub_X0_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/synchronized/ZED_CAMERA_X0/IMU", 100,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
            this->saveImu(msg, imu_file_X0_);
        });
    
    // ZED Camera X1 subscriptions
    image_sub_X1_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/synchronized/ZED_CAMERA_X1/rgb/image_rect_color", 10,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
            this->saveImage(msg, "ZED_CAMERA_X1_rgb_image_rect_color");
        });
    
    imu_sub_X1_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/synchronized/ZED_CAMERA_X1/IMU", 100,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
            this->saveImu(msg, imu_file_X1_);
        });
    

    camera_pc_sub_2i_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/synchronized/ZED_CAMERA_2i/point_cloud/cloud_registered", 10,
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                this->savePointCloudPLY(msg, "ZED_CAMERA_2i_point_clouds");
            });
        
    camera_pc_sub_X0_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/synchronized/ZED_CAMERA_X0/point_cloud/cloud_registered", 10,
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                this->savePointCloudPLY(msg, "ZED_CAMERA_X0_point_clouds");
            });
        
    camera_pc_sub_X1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/synchronized/ZED_CAMERA_X1/point_cloud/cloud_registered", 10,
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                this->savePointCloudPLY(msg, "ZED_CAMERA_X1_point_clouds");
            });
        

    // GNSS subscription
    gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/synchronized/gnss/fix", 100,
        [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
            this->saveNavSatFix(msg, gnss_file_);
        });
    
    // Point cloud subscriptions - just save as binary files for now
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/synchronized/livox/lidar", 10,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            this->savePointCloudPLY(msg, "point_clouds");
        });
    
    RCLCPP_INFO(this->get_logger(), "DataRecorderNode initialized. Saving to %s", session_dir.c_str());
}

DataRecorderNode::~DataRecorderNode() {
    // Close files on destruction
    imu_file_2i_.close();
    imu_file_X0_.close();
    imu_file_X1_.close();
    gnss_file_.close();
    
    RCLCPP_INFO(this->get_logger(), "DataRecorderNode shut down");
}

// Helper to get timestamp string from msg header
std::string DataRecorderNode::getTimestampStr(const builtin_interfaces::msg::Time & stamp) {
    // Combine sec and nanosec to a single string
    std::ostringstream oss;
    oss << stamp.sec << std::setw(9) << std::setfill('0') << stamp.nanosec;
    return oss.str();
}
void DataRecorderNode::savePointCloudPLY(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, const std::string &folder) {
    std::string stamp_str = getTimestampStr(msg->header.stamp);
    std::string filename = base_path_ + "/" + folder + "/" + stamp_str + ".ply";

    try {
        // Convert ROS message to PCL cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty point cloud received, skipping: %s", filename.c_str());
            return;
        }

        // Save to PLY file
        if (pcl::io::savePLYFileBinary(filename, *cloud) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write point cloud to: %s", filename.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Successfully saved point cloud with %zu points to %s", 
                cloud->size(), filename.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception saving point cloud: %s", e.what());
    }
}

// Save image as PNG file
void DataRecorderNode::saveImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg, const std::string &folder) {
    // Convert ROS Image to OpenCV
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    
    // Compose file path
    std::string stamp_str = getTimestampStr(msg->header.stamp);
    std::string filename = base_path_ + "/" + folder + "/" + stamp_str + ".png";
    
    // Save image to file
    cv::imwrite(filename, cv_ptr->image);
}

// Save PointCloud2 message as raw binary file
void DataRecorderNode::savePointCloudRaw(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, const std::string &folder) {
    std::string stamp_str = getTimestampStr(msg->header.stamp);
    std::string filename = base_path_ + "/" + folder + "/" + stamp_str + ".cloud";
    
    // Open file in binary mode
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing: %s", filename.c_str());
        return;
    }
    
    // Write header information (useful for custom deserializer later)
    file.write("CLOUD", 5);  // Magic bytes
    uint32_t height = msg->height;
    uint32_t width = msg->width;
    file.write(reinterpret_cast<char*>(&height), sizeof(height));
    file.write(reinterpret_cast<char*>(&width), sizeof(width));
    
    // Write raw point cloud data
    file.write(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());
    file.close();
}

// Save IMU data to CSV
void DataRecorderNode::saveImu(const sensor_msgs::msg::Imu::ConstSharedPtr &msg, std::ofstream &outfile) {
    // Write a line to CSV
    const auto & stamp = msg->header.stamp;
    outfile << stamp.sec << "." << std::setw(9) << std::setfill('0') << stamp.nanosec << ","
            << msg->orientation.w << "," << msg->orientation.x << "," 
            << msg->orientation.y << "," << msg->orientation.z << ","
            << msg->angular_velocity.x << "," << msg->angular_velocity.y << "," << msg->angular_velocity.z << ","
            << msg->linear_acceleration.x << "," << msg->linear_acceleration.y << "," << msg->linear_acceleration.z 
            << "\n";
    outfile.flush();
}

// Save NavSatFix data to CSV
void DataRecorderNode::saveNavSatFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &msg, std::ofstream &outfile) {
    const auto & stamp = msg->header.stamp;
    outfile << stamp.sec << "." << std::setw(9) << std::setfill('0') << stamp.nanosec << ","
            << msg->latitude << "," << msg->longitude << "," << msg->altitude << "\n";
    outfile.flush();
}

// Utility to get current datetime as string for session folder
std::string DataRecorderNode::currentDateTimeString() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
    return ss.str();
}