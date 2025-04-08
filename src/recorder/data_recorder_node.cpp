#include "data_recorder_node.hpp"

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
    // Camera 2i directories
    std::filesystem::create_directories(session_dir + "/ZED_CAMERA_2i_IMU");
    std::filesystem::create_directories(session_dir + "/ZED_CAMERA_2i_rgb_image_rect_color");
    std::filesystem::create_directories(session_dir + "/ZED_CAMERA_2i_point_cloud_ply");
    
    // Camera X0 directories
    std::filesystem::create_directories(base_path_ + "/ZED_CAMERA_X0_IMU");
    std::filesystem::create_directories(base_path_ + "/ZED_CAMERA_X0_rgb_image_rect_color");
    std::filesystem::create_directories(base_path_ + "/ZED_CAMERA_X0_point_cloud_ply");
    
    // Camera X1 directories
    std::filesystem::create_directories(base_path_ + "/ZED_CAMERA_X1_IMU");
    std::filesystem::create_directories(base_path_ + "/ZED_CAMERA_X1_rgb_image_rect_color");
    std::filesystem::create_directories(base_path_ + "/ZED_CAMERA_X1_point_cloud_ply");
    
    // Other sensor directories  
    std::filesystem::create_directories(base_path_ + "/gnss_fix");
    std::filesystem::create_directories(base_path_ + "/point_clouds");
    std::filesystem::create_directories(base_path_ + "/point_clouds_ply");
    
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
    
    // ZED Camera 2i subscriptions - using direct topics instead of synchronized ones
    image_sub_2i_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/ZED_CAMERA_2i/rgb/image_rect_color", 10,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
            this->saveImage(msg, "ZED_CAMERA_2i_rgb_image_rect_color");
        });
    
    imu_sub_2i_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/ZED_CAMERA_2i/IMU", 100,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
            this->saveImu(msg, imu_file_2i_);
        });
    
    // Camera 2i point cloud subscription - increased queue size and added diagnostics
    camera_pc_sub_2i_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/ZED_CAMERA_2i/point_cloud/cloud_registered", 30,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            // Save only in PLY format for camera point clouds
            this->savePointCloudPLY(msg, "ZED_CAMERA_2i_point_cloud_ply");
            RCLCPP_DEBUG(this->get_logger(), "Received and saved ZED_CAMERA_2i point cloud with timestamp: %u.%u, size: %dx%d points",
                msg->header.stamp.sec, msg->header.stamp.nanosec, msg->width, msg->height);
            camera_pc_2i_count_++;
        });
    
    // ZED Camera X0 subscriptions
    image_sub_X0_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/ZED_CAMERA_X0/rgb/image_rect_color", 10,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
            this->saveImage(msg, "ZED_CAMERA_X0_rgb_image_rect_color");
        });
    
    imu_sub_X0_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/ZED_CAMERA_X0/IMU", 100,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
            this->saveImu(msg, imu_file_X0_);
        });
    
    // Camera X0 point cloud subscription - increased queue size and added diagnostics
    camera_pc_sub_X0_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/ZED_CAMERA_X0/point_cloud/cloud_registered", 30,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            // Save only in PLY format for camera point clouds
            this->savePointCloudPLY(msg, "ZED_CAMERA_X0_point_cloud_ply");
            RCLCPP_DEBUG(this->get_logger(), "Received and saved ZED_CAMERA_X0 point cloud with timestamp: %u.%u, size: %dx%d points",
                msg->header.stamp.sec, msg->header.stamp.nanosec, msg->width, msg->height);
            camera_pc_X0_count_++;
        });
    
    // ZED Camera X1 subscriptions
    image_sub_X1_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/ZED_CAMERA_X1/rgb/image_rect_color", 10,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
            this->saveImage(msg, "ZED_CAMERA_X1_rgb_image_rect_color");
        });
    
    imu_sub_X1_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/ZED_CAMERA_X1/IMU", 100,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
            this->saveImu(msg, imu_file_X1_);
        });
    
    // Camera X1 point cloud subscription - increased queue size and added diagnostics
    camera_pc_sub_X1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/ZED_CAMERA_X1/point_cloud/cloud_registered", 30,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            // Save only in PLY format for camera point clouds
            this->savePointCloudPLY(msg, "ZED_CAMERA_X1_point_cloud_ply");
            RCLCPP_DEBUG(this->get_logger(), "Received and saved ZED_CAMERA_X1 point cloud with timestamp: %u.%u, size: %dx%d points",
                msg->header.stamp.sec, msg->header.stamp.nanosec, msg->width, msg->height);
            camera_pc_X1_count_++;
        });
    
    // GNSS subscription
    gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gnss/fix", 100,
        [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
            this->saveNavSatFix(msg, gnss_file_);
        });
    
    // LiDAR point cloud subscription - use direct topic from Livox driver with larger queue
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar", 30,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            // Save LiDAR points in both formats
            this->savePointCloudRaw(msg, "point_clouds"); 
            this->savePointCloudPLY(msg, "point_clouds_ply");
            RCLCPP_DEBUG(this->get_logger(), "Received and saved LiDAR point cloud with timestamp: %u.%u, size: %dx%d points",
                msg->header.stamp.sec, msg->header.stamp.nanosec, msg->width, msg->height);
            lidar_pc_count_++;
        });
    
    RCLCPP_INFO(this->get_logger(), "DataRecorderNode initialized. Saving to %s", session_dir.c_str());
    
    // Log subscription topics to help with debugging
    RCLCPP_INFO(this->get_logger(), "Subscribed to the following topics:");
    RCLCPP_INFO(this->get_logger(), "  Camera images: /ZED_CAMERA_*/rgb/image_rect_color");
    RCLCPP_INFO(this->get_logger(), "  Camera IMU: /ZED_CAMERA_*/IMU");
    RCLCPP_INFO(this->get_logger(), "  Camera point clouds: /ZED_CAMERA_*/point_cloud/cloud_registered");
    RCLCPP_INFO(this->get_logger(), "  LiDAR: /livox/lidar");
    RCLCPP_INFO(this->get_logger(), "  GNSS: /gnss/fix");
    
    RCLCPP_INFO(this->get_logger(), "Storage formats:");
    RCLCPP_INFO(this->get_logger(), "  Camera point clouds: PLY format only (with RGB data)");
    RCLCPP_INFO(this->get_logger(), "  LiDAR point clouds: Both raw binary (.cloud) and PLY format");
    
    // Start a timer to periodically report recording progress
    using namespace std::chrono_literals;
    progress_timer_ = this->create_wall_timer(
        10s, // Report every 10 seconds
        std::bind(&DataRecorderNode::reportProgress, this));
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

// Save PointCloud2 message as PLY format (widely supported by visualization tools)
void DataRecorderNode::savePointCloudPLY(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, const std::string &folder) {
    std::string stamp_str = getTimestampStr(msg->header.stamp);
    std::string filename = base_path_ + "/" + folder + "/" + stamp_str + ".ply";
    
    // Open file in binary mode for better performance
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing: %s", filename.c_str());
        return;
    }
    
    // Get point cloud information
    size_t point_count = msg->width * msg->height;
    bool has_color = false;
    bool has_intensity = false;
    
    // Check if the point cloud has color channels
    for (const auto& field : msg->fields) {
        if (field.name == "rgb" || field.name == "rgba") {
            has_color = true;
        } else if (field.name == "intensity") {
            has_intensity = true;
        }
    }

    // Set up field offsets map for faster lookups
    std::unordered_map<std::string, size_t> field_offsets;
    for (const auto& field : msg->fields) {
        field_offsets[field.name] = field.offset;
    }

    // Prepare header as a string first for better performance
    std::stringstream header;
    header << "ply\n";
    // Use binary format for faster writing and smaller files (camera point clouds can be large)
    header << "format binary_little_endian 1.0\n";
    header << "comment Generated by data_recorder_node\n";
    header << "comment Frame ID: " << msg->header.frame_id << "\n";
    header << "comment Timestamp: " << msg->header.stamp.sec << "." << msg->header.stamp.nanosec << "\n";
    header << "element vertex " << point_count << "\n";
    header << "property float x\n";
    header << "property float y\n";
    header << "property float z\n";
    if (has_color) {
        header << "property uchar red\n";
        header << "property uchar green\n";
        header << "property uchar blue\n";
    }
    if (has_intensity) {
        header << "property float intensity\n";
    }
    header << "end_header\n";
    
    // Write the header
    std::string header_str = header.str();
    file.write(header_str.c_str(), header_str.size());
    
    // Process and write points - optimize for camera point clouds
    const uint8_t* data_ptr = msg->data.data();
    
    // Pre-check if this is a camera point cloud (has color but no intensity) 
    bool is_camera_pc = has_color && !has_intensity;
    
    // For camera point clouds, we can process faster by assuming a standard format
    if (is_camera_pc && field_offsets.count("x") && field_offsets.count("y") && 
        field_offsets.count("z") && field_offsets.count("rgb")) {
        
        size_t x_offset = field_offsets["x"];
        size_t y_offset = field_offsets["y"];
        size_t z_offset = field_offsets["z"];
        size_t rgb_offset = field_offsets["rgb"];
        
        // Struct for batch processing
        struct PointXYZRGB {
            float x, y, z;
            uint8_t r, g, b;
        };
        
        // Process points in batches for camera point clouds
        for (size_t i = 0; i < point_count; ++i) {
            PointXYZRGB point;
            
            // Copy XYZ coordinates
            memcpy(&point.x, data_ptr + (i * msg->point_step) + x_offset, sizeof(float));
            memcpy(&point.y, data_ptr + (i * msg->point_step) + y_offset, sizeof(float));
            memcpy(&point.z, data_ptr + (i * msg->point_step) + z_offset, sizeof(float));
            
            // Extract RGB from the packed color
            uint32_t rgb_val;
            memcpy(&rgb_val, data_ptr + (i * msg->point_step) + rgb_offset, sizeof(uint32_t));
            point.r = (rgb_val >> 16) & 0xFF;
            point.g = (rgb_val >> 8) & 0xFF;
            point.b = rgb_val & 0xFF;
            
            // Write the formatted point directly
            file.write(reinterpret_cast<const char*>(&point), sizeof(float) * 3);  // XYZ
            file.write(reinterpret_cast<const char*>(&point.r), 3);  // RGB
        }
    } 
    // For LiDAR point clouds (no color, but has intensity)
    else if (!has_color && has_intensity && field_offsets.count("x") && field_offsets.count("y") && 
             field_offsets.count("z") && field_offsets.count("intensity")) {
        
        size_t x_offset = field_offsets["x"];
        size_t y_offset = field_offsets["y"];
        size_t z_offset = field_offsets["z"];
        size_t intensity_offset = field_offsets["intensity"];
        
        // Struct for batch processing
        struct PointXYZI {
            float x, y, z, intensity;
        };
        
        // Process points in batches for LiDAR point clouds
        for (size_t i = 0; i < point_count; ++i) {
            PointXYZI point;
            
            // Copy XYZI values
            memcpy(&point.x, data_ptr + (i * msg->point_step) + x_offset, sizeof(float));
            memcpy(&point.y, data_ptr + (i * msg->point_step) + y_offset, sizeof(float));
            memcpy(&point.z, data_ptr + (i * msg->point_step) + z_offset, sizeof(float));
            memcpy(&point.intensity, data_ptr + (i * msg->point_step) + intensity_offset, sizeof(float));
            
            // Write the formatted point directly
            file.write(reinterpret_cast<const char*>(&point), sizeof(float) * 4);
        }
    }
    // Fallback for any other point cloud format
    else {
        for (size_t i = 0; i < point_count; ++i) {
            float x = 0.0f, y = 0.0f, z = 0.0f, intensity = 0.0f;
            uint8_t r = 255, g = 255, b = 255;  // Default white
            
            // Access point data based on offset map
            if (field_offsets.count("x")) {
                memcpy(&x, data_ptr + (i * msg->point_step) + field_offsets["x"], sizeof(float));
            }
            if (field_offsets.count("y")) {
                memcpy(&y, data_ptr + (i * msg->point_step) + field_offsets["y"], sizeof(float));
            }
            if (field_offsets.count("z")) {
                memcpy(&z, data_ptr + (i * msg->point_step) + field_offsets["z"], sizeof(float));
            }
            if (field_offsets.count("intensity")) {
                memcpy(&intensity, data_ptr + (i * msg->point_step) + field_offsets["intensity"], sizeof(float));
            }
            if (has_color && field_offsets.count("rgb")) {
                uint32_t rgb_val;
                memcpy(&rgb_val, data_ptr + (i * msg->point_step) + field_offsets["rgb"], sizeof(uint32_t));
                r = (rgb_val >> 16) & 0xFF;
                g = (rgb_val >> 8) & 0xFF;
                b = rgb_val & 0xFF;
            }
            
            // Write point data to PLY file
            file.write(reinterpret_cast<const char*>(&x), sizeof(float));
            file.write(reinterpret_cast<const char*>(&y), sizeof(float));
            file.write(reinterpret_cast<const char*>(&z), sizeof(float));
            
            if (has_color) {
                file.write(reinterpret_cast<const char*>(&r), 1);
                file.write(reinterpret_cast<const char*>(&g), 1);
                file.write(reinterpret_cast<const char*>(&b), 1);
            }
            
            if (has_intensity) {
                file.write(reinterpret_cast<const char*>(&intensity), sizeof(float));
            }
        }
    }
    
    file.close();
    RCLCPP_DEBUG(this->get_logger(), "Saved point cloud in binary PLY format: %s", filename.c_str());
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

// Report recording progress for all sensors
void DataRecorderNode::reportProgress() {
    // Log counts of received sensor data
    RCLCPP_INFO(this->get_logger(), "Recording progress:");
    RCLCPP_INFO(this->get_logger(), "  ZED_CAMERA_2i point clouds: %lu files", camera_pc_2i_count_.load());
    RCLCPP_INFO(this->get_logger(), "  ZED_CAMERA_X0 point clouds: %lu files", camera_pc_X0_count_.load());
    RCLCPP_INFO(this->get_logger(), "  ZED_CAMERA_X1 point clouds: %lu files", camera_pc_X1_count_.load());
    RCLCPP_INFO(this->get_logger(), "  LiDAR point clouds: %lu files", lidar_pc_count_.load());
    
    // Calculate rates (files per second) based on elapsed time
    auto now = std::chrono::steady_clock::now();
    static auto start_time = now;
    std::chrono::duration<double> elapsed = now - start_time;
    double seconds = elapsed.count();
    
    if (seconds > 0) {
        double camera_2i_rate = static_cast<double>(camera_pc_2i_count_.load()) / seconds;
        double camera_X0_rate = static_cast<double>(camera_pc_X0_count_.load()) / seconds;
        double camera_X1_rate = static_cast<double>(camera_pc_X1_count_.load()) / seconds;
        double lidar_rate = static_cast<double>(lidar_pc_count_.load()) / seconds;
        
        RCLCPP_INFO(this->get_logger(), "Recording rates (files/second):");
        RCLCPP_INFO(this->get_logger(), "  ZED_CAMERA_2i point clouds: %.2f", camera_2i_rate);
        RCLCPP_INFO(this->get_logger(), "  ZED_CAMERA_X0 point clouds: %.2f", camera_X0_rate);
        RCLCPP_INFO(this->get_logger(), "  ZED_CAMERA_X1 point clouds: %.2f", camera_X1_rate);
        RCLCPP_INFO(this->get_logger(), "  LiDAR point clouds: %.2f", lidar_rate);
    }
}