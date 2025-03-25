#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>
#include <map>
#include <sl/Camera.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_srvs/srv/trigger.hpp>

/**
 * @brief Unified Camera Controller for managing multiple ZED cameras
 * 
 * This node manages multiple ZED cameras (ZED 2i, ZED X0, ZED X1) and publishes
 * their data to ROS topics. It provides services for starting and stopping cameras
 * and handles synchronization between them.
 */
class UnifiedCameraController : public rclcpp::Node
{
public:
    UnifiedCameraController() : Node("unified_camera_controller")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Unified Camera Controller...");
        
        // Declare parameters
        this->declare_parameter<bool>("use_depth", true);
        this->declare_parameter<bool>("use_pointcloud", true);
        this->declare_parameter<std::vector<int>>("camera_serials", std::vector<int>{37503998, 40894785, 41050786});
        this->declare_parameter<std::vector<std::string>>("camera_models", std::vector<std::string>{"ZED2i", "ZEDX", "ZEDX"});
        this->declare_parameter<bool>("auto_start", true);
        
        // Get parameters
        use_depth_ = this->get_parameter("use_depth").as_bool();
        use_pointcloud_ = this->get_parameter("use_pointcloud").as_bool();
        auto_start_ = this->get_parameter("auto_start").as_bool();
        
        // Get camera information
        std::vector<int> camera_serials = this->get_parameter("camera_serials").as_integer_array();
        std::vector<std::string> camera_models = this->get_parameter("camera_models").as_string_array();
        
        // Check that camera_serials and camera_models have the same size
        if (camera_serials.size() != camera_models.size()) {
            RCLCPP_ERROR(this->get_logger(), "camera_serials and camera_models must have the same size");
            return;
        }
        
        // Initialize cameras
        for (size_t i = 0; i < camera_serials.size(); ++i) {
            int serial = camera_serials[i];
            std::string model = camera_models[i];
            
            // Create camera entry
            CameraEntry entry;
            entry.serial = serial;
            entry.model = model;
            entry.camera = std::make_unique<sl::Camera>();
            entry.connected = false;
            
            // Create publishers for this camera
            std::string camera_name = model + "_" + std::to_string(serial);
            entry.rgb_pub = this->create_publisher<sensor_msgs::msg::Image>(
                camera_name + "/rgb/image_rect_color", 10);
            
            if (use_depth_) {
                entry.depth_pub = this->create_publisher<sensor_msgs::msg::Image>(
                    camera_name + "/depth/depth_registered", 10);
            }
            
            if (use_pointcloud_) {
                entry.pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                    camera_name + "/point_cloud/cloud", 10);
            }
            
            cameras_.push_back(std::move(entry));
            
            RCLCPP_INFO(this->get_logger(), "Added camera: %s (SN: %d)", model.c_str(), serial);
        }
        
        // Create services
        start_all_service_ = this->create_service<std_srvs::srv::Trigger>(
            "start_all_cameras",
            std::bind(&UnifiedCameraController::startAllCamerasCallback, this, 
                      std::placeholders::_1, std::placeholders::_2));
        
        stop_all_service_ = this->create_service<std_srvs::srv::Trigger>(
            "stop_all_cameras",
            std::bind(&UnifiedCameraController::stopAllCamerasCallback, this, 
                      std::placeholders::_1, std::placeholders::_2));
        
        // Auto-start cameras if requested
        if (auto_start_) {
            RCLCPP_INFO(this->get_logger(), "Auto-starting cameras...");
            startAllCameras();
        }
        
        // Create timer for publishing data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), // ~30fps
            std::bind(&UnifiedCameraController::publishData, this));
        
        RCLCPP_INFO(this->get_logger(), "Unified Camera Controller initialized.");
    }
    
    ~UnifiedCameraController()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down Unified Camera Controller...");
        stopAllCameras();
    }
    
private:
    struct CameraEntry {
        int serial;
        std::string model;
        std::unique_ptr<sl::Camera> camera;
        bool connected;
        
        // Publishers
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;
    };
    
    bool startCamera(CameraEntry& entry)
    {
        if (entry.connected) {
            RCLCPP_INFO(this->get_logger(), "Camera %s (SN: %d) already connected", 
                        entry.model.c_str(), entry.serial);
            return true;
        }
        
        RCLCPP_INFO(this->get_logger(), "Connecting to camera %s (SN: %d)...", 
                    entry.model.c_str(), entry.serial);
        
        // Initialize camera parameters
        sl::InitParameters init_params;
        init_params.sdk_verbose = true;
        init_params.input.setFromSerialNumber(entry.serial);
        
        // Set specific parameters based on model
        if (entry.model == "ZED2i") {
            init_params.camera_resolution = sl::RESOLUTION::HD720;
            init_params.camera_fps = 30;
        } else if (entry.model == "ZEDX") {
            init_params.camera_resolution = sl::RESOLUTION::AUTO;
            init_params.camera_fps = 0; // Auto FPS
        } else {
            init_params.camera_resolution = sl::RESOLUTION::HD720;
            init_params.camera_fps = 30;
        }
        
        // Set depth mode based on parameters
        if (!use_depth_) {
            init_params.depth_mode = sl::DEPTH_MODE::NONE;
        } else if (entry.model == "ZEDX") {
            init_params.depth_mode = sl::DEPTH_MODE::NEURAL;
        } else {
            init_params.depth_mode = sl::DEPTH_MODE::ULTRA;
        }
        
        // Try to open the camera
        sl::ERROR_CODE err = entry.camera->open(init_params);
        if (err != sl::ERROR_CODE::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Error opening camera %s (SN: %d): %s", 
                         entry.model.c_str(), entry.serial, sl::toString(err).c_str());
            RCLCPP_ERROR(this->get_logger(), "Details: %s", sl::toVerbose(err).c_str());
            return false;
        }
        
        // Get camera information
        sl::CameraInformation info = entry.camera->getCameraInformation();
        RCLCPP_INFO(this->get_logger(), "Connected to camera: %s (SN: %d)", 
                    sl::toString(info.camera_model).c_str(), info.serial_number);
        RCLCPP_INFO(this->get_logger(), "  Firmware: %d", info.camera_configuration.firmware_version);
        RCLCPP_INFO(this->get_logger(), "  Resolution: %dx%d @ %d FPS", 
                    info.camera_configuration.resolution.width,
                    info.camera_configuration.resolution.height,
                    info.camera_configuration.fps);
        
        entry.connected = true;
        return true;
    }
    
    void stopCamera(CameraEntry& entry)
    {
        if (!entry.connected) {
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Disconnecting from camera %s (SN: %d)...", 
                    entry.model.c_str(), entry.serial);
        
        entry.camera->close();
        entry.connected = false;
    }
    
    bool startAllCameras()
    {
        bool all_success = true;
        for (auto& entry : cameras_) {
            if (!startCamera(entry)) {
                all_success = false;
            }
        }
        return all_success;
    }
    
    void stopAllCameras()
    {
        for (auto& entry : cameras_) {
            stopCamera(entry);
        }
    }
    
    void publishData()
    {
        for (auto& entry : cameras_) {
            if (!entry.connected) {
                continue;
            }
            
            // Grab a new frame
            sl::ERROR_CODE grab_status = entry.camera->grab();
            if (grab_status != sl::ERROR_CODE::SUCCESS) {
                RCLCPP_WARN(this->get_logger(), "Error grabbing frame from camera %s (SN: %d): %s", 
                            entry.model.c_str(), entry.serial, sl::toString(grab_status).c_str());
                continue;
            }
            
            // Create ROS timestamp
            auto stamp = this->now();
            std::string frame_id = entry.model + "_" + std::to_string(entry.serial) + "_camera_center";
            
            // Retrieve and publish RGB image
            sl::Mat zed_image;
            entry.camera->retrieveImage(zed_image, sl::VIEW::LEFT);
            
            // Convert to OpenCV format
            cv::Mat cv_image(zed_image.getHeight(), zed_image.getWidth(), CV_8UC4, zed_image.getPtr<sl::uchar1>(sl::MEM::CPU));
            cv::Mat cv_image_rgb;
            cv::cvtColor(cv_image, cv_image_rgb, cv::COLOR_RGBA2RGB);
            
            // Convert to ROS message
            auto rgb_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", cv_image_rgb).toImageMsg();
            rgb_msg->header.stamp = stamp;
            rgb_msg->header.frame_id = frame_id;
            
            // Publish RGB image
            entry.rgb_pub->publish(*rgb_msg);
            
            // Retrieve and publish depth if enabled
            if (use_depth_ && entry.depth_pub) {
                sl::Mat zed_depth;
                entry.camera->retrieveImage(zed_depth, sl::VIEW::DEPTH);
                
                // Convert to OpenCV format (32-bit float)
                cv::Mat cv_depth(zed_depth.getHeight(), zed_depth.getWidth(), CV_32FC1, zed_depth.getPtr<sl::float1>(sl::MEM::CPU));
                
                // Convert to ROS message
                auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", cv_depth).toImageMsg();
                depth_msg->header.stamp = stamp;
                depth_msg->header.frame_id = frame_id;
                
                // Publish depth image
                entry.depth_pub->publish(*depth_msg);
            }
            
            // Retrieve and publish point cloud if enabled
            if (use_pointcloud_ && entry.pointcloud_pub) {
                sl::Mat point_cloud;
                entry.camera->retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA);
                
                // Convert to PCL point cloud
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                float* p_point_cloud = point_cloud.getPtr<float>();
                int width = point_cloud.getWidth();
                int height = point_cloud.getHeight();
                
                pcl_cloud->width = width;
                pcl_cloud->height = height;
                pcl_cloud->is_dense = false;
                pcl_cloud->points.resize(width * height);
                
                for (int i = 0; i < height; i++) {
                    for (int j = 0; j < width; j++) {
                        int index = i * width + j;
                        float x = p_point_cloud[index * 4];
                        float y = p_point_cloud[index * 4 + 1];
                        float z = p_point_cloud[index * 4 + 2];
                        
                        // Skip invalid points
                        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                            continue;
                        }
                        
                        // Get color
                        sl::uchar4 color = point_cloud.getPixel<sl::uchar4>(j, i);
                        
                        // Add point to cloud
                        pcl_cloud->points[index].x = x;
                        pcl_cloud->points[index].y = y;
                        pcl_cloud->points[index].z = z;
                        pcl_cloud->points[index].r = color.r;
                        pcl_cloud->points[index].g = color.g;
                        pcl_cloud->points[index].b = color.b;
                    }
                }
                
                // Convert to ROS message
                sensor_msgs::msg::PointCloud2 cloud_msg;
                pcl::toROSMsg(*pcl_cloud, cloud_msg);
                cloud_msg.header.stamp = stamp;
                cloud_msg.header.frame_id = frame_id;
                
                // Publish point cloud
                entry.pointcloud_pub->publish(cloud_msg);
            }
        }
    }
    
    // Service callbacks
    void startAllCamerasCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        bool success = startAllCameras();
        response->success = success;
        response->message = success ? "All cameras started successfully" : "Some cameras failed to start";
    }
    
    void stopAllCamerasCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        stopAllCameras();
        response->success = true;
        response->message = "All cameras stopped successfully";
    }
    
    // Member variables
    std::vector<CameraEntry> cameras_;
    bool use_depth_;
    bool use_pointcloud_;
    bool auto_start_;
    
    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_all_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_all_service_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UnifiedCameraController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}