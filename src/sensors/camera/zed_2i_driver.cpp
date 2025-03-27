#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sl/Camera.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

/**
 * @brief Specialized ZED 2i USB camera driver to fix connection issues
 * 
 * This implementation specifically targets the ZED 2i USB camera with serial number 37503998
 * and includes several workarounds to attempt to connect successfully.
 */
class ZED2iDriver : public rclcpp::Node 
{
public:
    ZED2iDriver() : Node("zed_2i_driver_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing ZED 2i driver node...");
        
        // Declare parameters
        this->declare_parameter<bool>("use_depth", false);
        this->declare_parameter<int>("camera_fps", 30);
        this->declare_parameter<std::string>("camera_resolution", "HD720");
        this->declare_parameter<int>("reconnect_attempts", 5);
        
        // Get parameters
        bool use_depth = this->get_parameter("use_depth").as_bool();
        int camera_fps = this->get_parameter("camera_fps").as_int();
        std::string resolution_str = this->get_parameter("camera_resolution").as_string();
        int reconnect_attempts = this->get_parameter("reconnect_attempts").as_int();
        
        // Map resolution string to enum
        sl::RESOLUTION resolution = sl::RESOLUTION::HD720;
        if (resolution_str == "HD2K") resolution = sl::RESOLUTION::HD2K;
        else if (resolution_str == "HD1080") resolution = sl::RESOLUTION::HD1080;
        else if (resolution_str == "HD720") resolution = sl::RESOLUTION::HD720;
        else if (resolution_str == "VGA") resolution = sl::RESOLUTION::VGA;
        
        // Create publishers
        rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>("zed2i/rgb/image_rect_color", 10);
        if (use_depth) {
            depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("zed2i/depth/depth_registered", 10);
        }
        
        // Initialize camera
        RCLCPP_INFO(this->get_logger(), "Connecting to ZED 2i camera (SN: 37503998)...");
        RCLCPP_INFO(this->get_logger(), "Using resolution: %s, FPS: %d, Depth: %s", 
                    resolution_str.c_str(), camera_fps, use_depth ? "ON" : "OFF");
        
        // Try multiple approaches to connect to the camera
        bool connected = false;
        
        for (int attempt = 0; attempt < reconnect_attempts && !connected; attempt++) {
            RCLCPP_INFO(this->get_logger(), "Connection attempt %d/%d", attempt+1, reconnect_attempts);
            
            // Initialize with different parameter sets based on attempt number
            sl::InitParameters init_params;
            init_params.sdk_verbose = true;
            init_params.input.setFromSerialNumber(37503998);
            
            switch (attempt) {
                case 0: // Default approach
                    init_params.camera_resolution = resolution;
                    init_params.camera_fps = camera_fps;
                    if (!use_depth) init_params.depth_mode = sl::DEPTH_MODE::NONE;
                    break;
                    
                case 1: // Try minimal parameters
                    init_params.camera_resolution = sl::RESOLUTION::VGA; // Lower resolution
                    init_params.camera_fps = 15; // Lower framerate
                    init_params.depth_mode = sl::DEPTH_MODE::NONE; // No depth
                    break;
                    
                case 2: // Try with default params (no serial number)
                    init_params = sl::InitParameters();
                    init_params.sdk_verbose = true;
                    init_params.depth_mode = sl::DEPTH_MODE::NONE;
                    break;
                    
                case 3: // Try with different coordinate system
                    init_params.camera_resolution = resolution;
                    init_params.camera_fps = camera_fps;
                    init_params.depth_mode = sl::DEPTH_MODE::NONE;
                    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
                    break;
                    
                case 4: // Try with different sensing mode
                    init_params.camera_resolution = resolution;
                    init_params.camera_fps = camera_fps;
                    init_params.depth_mode = sl::DEPTH_MODE::NONE;
                    init_params.sensing_mode = sl::SENSING_MODE::FILL;
                    break;
            }
            
            // Attempt to open camera with current parameters
            sl::ERROR_CODE err = zed_.open(init_params);
            
            if (err == sl::ERROR_CODE::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Successfully connected to ZED 2i camera!");
                connected = true;
                
                // Get and display camera information
                sl::CameraInformation cam_info = zed_.getCameraInformation();
                RCLCPP_INFO(this->get_logger(), "Camera model: %s", sl::toString(cam_info.camera_model).c_str());
                RCLCPP_INFO(this->get_logger(), "Serial number: %d", cam_info.serial_number);
                RCLCPP_INFO(this->get_logger(), "Firmware version: %d", cam_info.camera_configuration.firmware_version);
                RCLCPP_INFO(this->get_logger(), "Resolution: %dx%d", 
                            cam_info.camera_configuration.resolution.width,
                            cam_info.camera_configuration.resolution.height);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Connection attempt %d failed: %s", 
                             attempt+1, sl::toString(err).c_str());
                RCLCPP_ERROR(this->get_logger(), "Details: %s", sl::toVerbose(err).c_str());
                
                // Short delay before next attempt
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
        }
        
        if (!connected) {
            RCLCPP_FATAL(this->get_logger(), "Failed to connect to ZED 2i camera after %d attempts!", reconnect_attempts);
            return;
        }
        
        // Create timer to publish images
        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(
            33ms, std::bind(&ZED2iDriver::publish_images, this));
        
        RCLCPP_INFO(this->get_logger(), "ZED 2i driver initialized successfully. Publishing data...");
    }
    
    ~ZED2iDriver()
    {
        if (zed_.isOpened()) {
            RCLCPP_INFO(this->get_logger(), "Closing ZED 2i camera...");
            zed_.close();
        }
    }
    
private:
    void publish_images()
    {
        if (!zed_.isOpened()) {
            RCLCPP_WARN(this->get_logger(), "Camera is not open. Cannot publish images.");
            return;
        }
        
        // Grab a new frame
        auto grab_status = zed_.grab();
        if (grab_status != sl::ERROR_CODE::SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "Failed to grab frame: %s", sl::toString(grab_status).c_str());
            return;
        }
        
        // Retrieve left RGB image
        sl::Mat zed_image;
        zed_.retrieveImage(zed_image, sl::VIEW::LEFT);
        
        // Convert to OpenCV format
        cv::Mat cv_image(zed_image.getHeight(), zed_image.getWidth(), CV_8UC4, zed_image.getPtr<sl::uchar1>(sl::MEM::CPU));
        cv::Mat cv_image_rgb;
        cv::cvtColor(cv_image, cv_image_rgb, cv::COLOR_RGBA2RGB);
        
        // Convert to ROS message
        auto ros_image = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", cv_image_rgb).toImageMsg();
        ros_image->header.stamp = this->now();
        ros_image->header.frame_id = "zed2i_camera_center";
        
        // Publish RGB image
        rgb_pub_->publish(*ros_image);
        
        // Retrieve and publish depth if enabled
        if (depth_pub_ && zed_.getRuntimeParameters().enable_depth) {
            sl::Mat zed_depth;
            zed_.retrieveImage(zed_depth, sl::VIEW::DEPTH);
            
            // Convert to OpenCV format (32-bit float)
            cv::Mat cv_depth(zed_depth.getHeight(), zed_depth.getWidth(), CV_32FC1, zed_depth.getPtr<sl::float1>(sl::MEM::CPU));
            
            // Convert to ROS message
            auto ros_depth = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", cv_depth).toImageMsg();
            ros_depth->header.stamp = this->now();
            ros_depth->header.frame_id = "zed2i_camera_center";
            
            // Publish depth image
            depth_pub_->publish(*ros_depth);
        }
    }
    
    sl::Camera zed_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZED2iDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}