#include "sensors/camera/zed_camera_driver.h"
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/string.hpp>
#include "sensors/camera/camera_config.h"
#include <iostream>
#include <map>

namespace sensors {
namespace camera {

// Serial number mapping for easier identification - for debugging purposes
const std::map<int, std::string> CAMERA_SERIALS = {
  {37503998, "USB-0"},   // ZED 2i (USB)
  {40894785, "GMSL-0"},  // ZED X0 (GMSL-0)
  {41050786, "GMSL-1"}   // ZED X1 (GMSL-1)
};

ZedCameraDriver::ZedCameraDriver(std::shared_ptr<CameraConfig> config)
    : config_(config),
      connected_(false),
      recording_(false),
      current_fps_(15.0f) { // Default FPS
  
  // Create ZED camera instance
  zed_ = std::make_unique<sl::Camera>();
  std::cout << "ZED Camera driver initialized with real ZED SDK" << std::endl;
  
  // Log SDK version and details for debugging
  #if defined(ZED_SDK_MAJOR_VERSION) && defined(ZED_SDK_MINOR_VERSION)
    std::cout << "ZED SDK Version: " << ZED_SDK_MAJOR_VERSION << "." << ZED_SDK_MINOR_VERSION << std::endl;
  #else
    std::cout << "ZED SDK Version: Unknown (SDK macros not defined)" << std::endl;
  #endif
}

ZedCameraDriver::~ZedCameraDriver() {
  if (connected_) {
    disconnect();
  }
}

bool ZedCameraDriver::connect() {
  // Check if already connected
  if (connected_) {
    return true;
  }
  
  try {
    // Get the serial number from configuration
    int serial_number = config_->getSerialNumber();
    
    // Log which camera we're connecting to
    std::string camera_id = "Unknown";
    if (serial_number > 0 && CAMERA_SERIALS.find(serial_number) != CAMERA_SERIALS.end()) {
      camera_id = CAMERA_SERIALS.at(serial_number);
      std::cout << "Connecting to ZED camera: " << camera_id << " (SN: " << serial_number << ")" << std::endl;
    } else {
      std::cout << "No serial number specified or unknown serial, connecting to first available camera" << std::endl;
    }
    
    // List available cameras for debugging
    std::cout << "Searching for available ZED cameras..." << std::endl;
    #if defined(ZED_SDK_MAJOR_VERSION) && (ZED_SDK_MAJOR_VERSION > 3 || (ZED_SDK_MAJOR_VERSION == 3 && ZED_SDK_MINOR_VERSION >= 5))
    // Use newer SDK device list function if available
    auto devList = sl::Camera::getDeviceList();
    if (devList.size() == 0) {
      std::cerr << "No ZED cameras detected on the system!" << std::endl;
    } else {
      std::cout << "Found " << devList.size() << " ZED camera(s):" << std::endl;
      for (auto& device : devList) {
        std::cout << " - Camera SN: " << device.serial_number 
                  << ", Model: " << sl::toString(device.camera_model) 
                  << ", State: " << (device.camera_state == sl::CAMERA_STATE::AVAILABLE ? "Available" : "Already in use")
                  << std::endl;
      }
    }
    #else
    // Older SDK doesn't have getDeviceList method
    std::cout << "ZED SDK version doesn't support device listing" << std::endl;
    #endif
    
    // Initialize ZED camera parameters
    sl::InitParameters init_params;
    
    // First try with minimal settings to test connection
    // Try with HD1080 resolution for ZED X cameras
    init_params.camera_resolution = sl::RESOLUTION::HD1080;
    init_params.camera_fps = 15;
    init_params.depth_mode = sl::DEPTH_MODE::NONE;
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.sdk_verbose = true;
    init_params.sensors_required = false;
    
    // Set specific settings for different camera models based on serial number
    if (serial_number == 37503998) { // ZED 2i - USB - Not available in this system
      std::cout << "Setting ZED 2i specific parameters" << std::endl;
      init_params.camera_resolution = sl::RESOLUTION::HD720;
      init_params.camera_fps = 15;
    } else if (serial_number == 40894785 || serial_number == 41050786) { // ZED X - GMSL
      std::cout << "Setting ZED X specific parameters for GMSL camera" << std::endl;
      // Try HD1080 for ZED X cameras
      init_params.camera_resolution = sl::RESOLUTION::HD1080;
      init_params.camera_fps = 15;
    }
    
    // Use specific serial number if provided
    if (serial_number > 0) {
      std::cout << "Setting ZED initialization for specific camera SN: " << serial_number << std::endl;
      init_params.input.setFromSerialNumber(serial_number);
    } else {
      std::cout << "Attempting to connect to first available camera" << std::endl;
    }
    
    // Try opening the camera with minimal settings first to test connection
    std::cout << "Attempting initial connection to camera..." << std::endl;
    sl::ERROR_CODE err = zed_->open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
      std::cerr << "Error opening ZED camera: " << sl::toString(err) << std::endl;
      std::cerr << "Detailed error: " << sl::toVerbose(err) << std::endl;
      
      if (err == sl::ERROR_CODE::CAMERA_NOT_DETECTED) {
        std::cerr << "No ZED camera detected with serial " << serial_number << std::endl;
        std::cerr << "Please check USB/GMSL connections and permissions" << std::endl;
      }
      
      return false;
    }
    
    // If successful, close camera and reopen with desired settings
    std::cout << "Initial connection successful, reconfiguring with full settings..." << std::endl;
    zed_->close();
    
    // Model-specific settings for reopening the camera
    if (serial_number == 37503998) { // ZED 2i - not available in this system
        std::cout << "Using model-specific settings for ZED 2i (USB)" << std::endl;
        init_params.camera_resolution = sl::RESOLUTION::HD720;
        init_params.camera_fps = 15;
    } else if (serial_number == 40894785 || serial_number == 41050786) { // ZED X
        std::cout << "Using model-specific settings for ZED X (GMSL)" << std::endl;
        // ZED X cameras using HD1080 resolution
        init_params.camera_resolution = sl::RESOLUTION::HD1080;
        init_params.camera_fps = 15;
    } else {
        // Generic settings based on model in config - use HD1080 for ZED X cameras
        std::cout << "Using generic HD1080 settings for unknown camera" << std::endl;
        init_params.camera_resolution = sl::RESOLUTION::HD1080;
        init_params.camera_fps = 15;
    }
    
    // Configure depth mode - need QUALITY mode for better point clouds
    // and only if requested in the config
    bool use_depth = config_->getUseDepth();
    if (!use_depth) {
      init_params.depth_mode = sl::DEPTH_MODE::NONE;
      std::cout << "Depth processing disabled" << std::endl;
    } else {
      // Use QUALITY mode for better point clouds with real hardware
      init_params.depth_mode = sl::DEPTH_MODE::ULTRA;
      init_params.coordinate_units = sl::UNIT::METER;
      init_params.depth_minimum_distance = 0.3f; // 30cm minimum depth
      init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // ROS convention
      init_params.depth_stabilization = true; // Enable depth stabilization for better quality
      std::cout << "Using ULTRA depth mode with coordinate system: RIGHT_HANDED_Y_UP" << std::endl;
    }
    
    // Additional settings
    init_params.async_grab_camera_recovery = true; // Keep auto-recovery enabled
    init_params.sensors_required = false; // Don't require any sensors to be present
    
    // Set serial number again for the reopening
    if (serial_number > 0) {
      init_params.input.setFromSerialNumber(serial_number);
    }
    
    // Reopen with full settings
    err = zed_->open(init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
      std::cerr << "Error reopening ZED camera with full settings: " << sl::toString(err) << std::endl;
      std::cerr << "Trying with simpler settings (no depth)..." << std::endl;
      
      // Try again with simpler settings
      init_params.depth_mode = sl::DEPTH_MODE::NONE;
      init_params.sensors_required = false;
      
      err = zed_->open(init_params);
      if (err != sl::ERROR_CODE::SUCCESS) {
        std::cerr << "Still failed to open camera: " << sl::toString(err) << std::endl;
        return false;
      }
    }
    
    // Get camera information
    sl::CameraInformation zed_info = zed_->getCameraInformation();
    
    // Print camera info for debugging
    std::cout << "ZED Camera Information:" << std::endl;
    std::cout << " - Serial Number: " << zed_info.serial_number << std::endl;
    std::cout << " - Camera Model: " << sl::toString(zed_info.camera_model) << std::endl;
    std::cout << " - Input Type: USB" << std::endl; // Simplified for mock SDK
    std::cout << " - Firmware: " << zed_info.camera_configuration.firmware_version << std::endl;
    std::cout << " - Resolution: " << zed_info.camera_configuration.resolution.width << "x" << 
                                     zed_info.camera_configuration.resolution.height << std::endl;
    
    // Mark camera as connected
    connected_ = true;
    
    // Initialize camera state
    current_fps_ = zed_->getCurrentFPS();
    recording_ = false;
    
    // Create runtime parameters for grab with optimized values for point cloud
    runtime_params_ = sl::RuntimeParameters();
    runtime_params_.measure3D_reference_frame = sl::REFERENCE_FRAME::CAMERA;
    runtime_params_.confidence_threshold = 50; // Higher confidence threshold for more accurate points
    runtime_params_.texture_confidence_threshold = 100; // Maximum texture confidence for better points
    
    // Print success message with camera model and serial
    std::cout << "ZED camera " << sl::toString(zed_info.camera_model) 
              << " (SN: " << zed_info.serial_number << ") connected successfully" << std::endl;
    
    // Print SDK version for debugging
    std::cout << "Using ZED SDK version: " << ZED_SDK_MAJOR_VERSION << "." << ZED_SDK_MINOR_VERSION << "." << ZED_SDK_PATCH_VERSION << std::endl;
    
    // Test image capture to verify functionality
    std::cout << "Testing image capture..." << std::endl;
    
    // Try to grab a frame to ensure the camera is working
    sl::Mat test_image;
    sl::ERROR_CODE test_grab_err = zed_->grab(runtime_params_);
    if (test_grab_err != sl::ERROR_CODE::SUCCESS) {
        std::cerr << "Test grab failed: " << sl::toString(test_grab_err) << std::endl;
    } else {
        sl::ERROR_CODE test_retrieve_err = zed_->retrieveImage(test_image, sl::VIEW::LEFT, sl::MEM::CPU);
        if (test_retrieve_err != sl::ERROR_CODE::SUCCESS) {
            std::cerr << "Test image retrieve failed: " << sl::toString(test_retrieve_err) << std::endl;
        } else {
            // Check if the test image has data (not all zeros)
            bool has_data = false;
            unsigned char* data_ptr = test_image.getPtr<sl::uchar1>();
            if (data_ptr) {
                // Sample a few points in the image
                int width = test_image.getWidth();
                int height = test_image.getHeight();
                int channels = test_image.getChannels();
                size_t stride = test_image.getStepBytes();
                
                // Check center point and corners
                size_t center_idx = (height/2) * stride + (width/2) * channels;
                size_t tl_idx = 0;  // Top-left
                size_t tr_idx = (width-1) * channels;  // Top-right
                size_t bl_idx = (height-1) * stride;  // Bottom-left
                size_t br_idx = (height-1) * stride + (width-1) * channels;  // Bottom-right
                
                int num_checked_pixels = 0;
                int num_nonzero_pixels = 0;
                
                // Check center
                num_checked_pixels += 3;
                if (data_ptr[center_idx] > 0) num_nonzero_pixels++;
                if (data_ptr[center_idx+1] > 0) num_nonzero_pixels++;
                if (data_ptr[center_idx+2] > 0) num_nonzero_pixels++;
                
                // Check top-left
                num_checked_pixels += 3;
                if (data_ptr[tl_idx] > 0) num_nonzero_pixels++;
                if (data_ptr[tl_idx+1] > 0) num_nonzero_pixels++;
                if (data_ptr[tl_idx+2] > 0) num_nonzero_pixels++;
                
                // Check top-right
                num_checked_pixels += 3;
                if (data_ptr[tr_idx] > 0) num_nonzero_pixels++;
                if (data_ptr[tr_idx+1] > 0) num_nonzero_pixels++;
                if (data_ptr[tr_idx+2] > 0) num_nonzero_pixels++;
                
                // Check bottom-left
                num_checked_pixels += 3;
                if (data_ptr[bl_idx] > 0) num_nonzero_pixels++;
                if (data_ptr[bl_idx+1] > 0) num_nonzero_pixels++;
                if (data_ptr[bl_idx+2] > 0) num_nonzero_pixels++;
                
                // Check bottom-right
                num_checked_pixels += 3;
                if (data_ptr[br_idx] > 0) num_nonzero_pixels++;
                if (data_ptr[br_idx+1] > 0) num_nonzero_pixels++;
                if (data_ptr[br_idx+2] > 0) num_nonzero_pixels++;
                
                has_data = (num_nonzero_pixels > 0);
                
                std::cout << "Test image check: " << num_nonzero_pixels << "/" << num_checked_pixels 
                          << " non-zero pixels at sample points" << std::endl;
                
                if (!has_data) {
                    std::cerr << "WARNING: Test image contains all zeros at sample points!" << std::endl;
                }
            } else {
                std::cerr << "Test image data pointer is NULL!" << std::endl;
            }
            
            std::cout << "Image size: " << test_image.getWidth() << "x" << test_image.getHeight() 
                      << ", channels: " << test_image.getChannels() 
                      << ", data type: " << static_cast<int>(test_image.getDataType()) << std::endl;
        }
    }

    return true;
  }
  catch (const std::exception& e) {
    std::cerr << "Exception during camera connection: " << e.what() << std::endl;
    connected_ = false;
    return false;
  }
}

void ZedCameraDriver::disconnect() {
  if (!connected_) {
    return;
  }
  
  try {
    // Stop any ongoing recording
    if (recording_) {
      stopRecording();
    }
    
    // Close the camera
    if (zed_) {
      zed_->close();
    }
    
    // Set as disconnected
    connected_ = false;
    std::cout << "ZED camera disconnected" << std::endl;
  }
  catch (const std::exception& e) {
    std::cerr << "Exception during camera disconnection: " << e.what() << std::endl;
    connected_ = false;
  }
}

bool ZedCameraDriver::isConnected() const {
  return connected_;
}

bool ZedCameraDriver::setFrameRate(float fps) {
  if (!connected_ || !zed_) {
    return false;
  }
  
  try {
    // Clamp FPS to valid range
    float clamped_fps = std::max(config_->getMinFps(), std::min(config_->getMaxFps(), fps));
    
    // Update the current FPS
    current_fps_ = clamped_fps;
    
    // Note: ZED SDK doesn't allow changing FPS after camera is opened
    // We'll just use this value for our grab timing
    std::cout << "Camera FPS set to " << current_fps_ << std::endl;
    
    return true;
  }
  catch (const std::exception& e) {
    std::cerr << "Exception setting camera FPS: " << e.what() << std::endl;
    return false;
  }
}

float ZedCameraDriver::getCurrentFrameRate() const {
  if (connected_ && zed_) {
    return zed_->getCurrentFPS();
  }
  return current_fps_;
}

bool ZedCameraDriver::getRgbImage(sensor_msgs::msg::Image& image) {
  if (!connected_ || !zed_) {
    return false;
  }
  
  try {
    // Grab a new frame from camera with timeout (150ms)
    sl::ERROR_CODE err = zed_->grab(runtime_params_);
    if (err != sl::ERROR_CODE::SUCCESS) {
      if (err != sl::ERROR_CODE::CAMERA_NOT_DETECTED) {
        std::cerr << "Error grabbing frame: " << sl::toString(err) << std::endl;
      }
      return false;
    }
    
    // Retrieve the left RGB image
    sl::Mat zed_image;
    err = zed_->retrieveImage(zed_image, sl::VIEW::LEFT, sl::MEM::CPU);
    if (err != sl::ERROR_CODE::SUCCESS) {
      std::cerr << "Error retrieving RGB image: " << sl::toString(err) << std::endl;
      return false;
    }
    
    // Sanity check: make sure the image isn't all zeros
    // Sample a few points to verify we have actual image data
    bool hasValidData = false;
    unsigned char* data_ptr = zed_image.getPtr<sl::uchar1>();
    if (data_ptr) {
      // Check a few sample points in the image
      const int width = zed_image.getWidth();
      const int height = zed_image.getHeight();
      const int channels = zed_image.getChannels();
      const int sample_points = 5;
      
      // Sample the center and 4 corners of the image
      const size_t stride = zed_image.getStepBytes();
      
      // Check center point
      size_t center_idx = (height/2) * stride + (width/2) * channels;
      if (data_ptr[center_idx] > 0 || data_ptr[center_idx+1] > 0 || data_ptr[center_idx+2] > 0) {
        hasValidData = true;
      }
      
      // If center is zero, check a few more points
      if (!hasValidData) {
        // Top-left
        size_t tl_idx = 0;
        // Top-right
        size_t tr_idx = (width-1) * channels;
        // Bottom-left
        size_t bl_idx = (height-1) * stride;
        // Bottom-right
        size_t br_idx = (height-1) * stride + (width-1) * channels;
        
        if (data_ptr[tl_idx] > 0 || data_ptr[tr_idx] > 0 || 
            data_ptr[bl_idx] > 0 || data_ptr[br_idx] > 0) {
          hasValidData = true;
        }
      }
      
      if (!hasValidData) {
        std::cerr << "Warning: Image data appears to be all zeros!" << std::endl;
        
        // Print some debug info about the camera
        sl::CameraInformation info = zed_->getCameraInformation();
        std::cerr << "Camera info: SN=" << info.serial_number 
                  << ", Model=" << sl::toString(info.camera_model) 
                  << ", Resolution=" << width << "x" << height
                  << ", FPS=" << zed_->getCurrentFPS() << std::endl;
      }
    }
    
    // Fill the ROS2 image message with the namespace frame ID
    fillImageMsg(image, zed_image, config_->getFrameId());
    
    return true;
  }
  catch (const std::exception& e) {
    std::cerr << "Exception getting RGB image: " << e.what() << std::endl;
    return false;
  }
}

bool ZedCameraDriver::getDepthImage(sensor_msgs::msg::Image& image) {
  if (!connected_ || !zed_) {
    return false;
  }
  
  try {
    // Grab has already been done in getRgbImage, no need to grab again
    
    // Retrieve the depth image - In ZED SDK 4.x, it's just DEPTH instead of VIEW_DEPTH
    sl::Mat zed_depth;
    sl::ERROR_CODE err = zed_->retrieveImage(zed_depth, sl::VIEW::DEPTH, sl::MEM::CPU);
    if (err != sl::ERROR_CODE::SUCCESS) {
      std::cerr << "Error retrieving depth image: " << sl::toString(err) << std::endl;
      return false;
    }
    
    // Fill ROS2 image message with the namespace frame ID
    fillImageMsg(image, zed_depth, config_->getFrameId());
    
    return true;
  }
  catch (const std::exception& e) {
    std::cerr << "Exception getting depth image: " << e.what() << std::endl;
    return false;
  }
}

bool ZedCameraDriver::getPointCloud(sensor_msgs::msg::PointCloud2& cloud) {
  if (!connected_ || !zed_) {
    return false;
  }
  
  try {
    // Note: No need to grab again - the camera manager already ensures fresh data by calling grab
    // Using the current frame data for better efficiency
    
    // Retrieve the point cloud with ULTRA depth resolution
    sl::Mat zed_cloud;
    runtime_params_.confidence_threshold = 50; // Adjust confidence threshold for better point cloud
    
    // Try getting a filtered point cloud
    sl::ERROR_CODE err = zed_->retrieveMeasure(zed_cloud, sl::MEASURE::XYZRGBA, sl::MEM::CPU);
    if (err != sl::ERROR_CODE::SUCCESS) {
      std::cerr << "Error retrieving point cloud: " << sl::toString(err) << std::endl;
      
      // Try with a lower resolution as fallback
      runtime_params_.confidence_threshold = 30;
      err = zed_->retrieveMeasure(zed_cloud, sl::MEASURE::XYZRGBA, sl::MEM::CPU);
      if (err != sl::ERROR_CODE::SUCCESS) {
        std::cerr << "Error retrieving point cloud even with lower confidence: " << sl::toString(err) << std::endl;
        return false;
      }
    }
    
    // Sanity check on point cloud data
    if (zed_cloud.getWidth() == 0 || zed_cloud.getHeight() == 0) {
      std::cerr << "Warning: Retrieved point cloud has zero dimensions!" << std::endl;
      return false;
    }
    
    // Fill the ROS2 point cloud message
    cloud.header.stamp = rclcpp::Clock().now();
    
    // Use camera-specific frame IDs for better compatibility
    std::string ns = config_->getNamespace();
    if (ns == "ZED_CAMERA_X0") {
        cloud.header.frame_id = "zed_camera_left"; // Default name for the main camera
    } else if (ns == "ZED_CAMERA_X1") {
        cloud.header.frame_id = "zed_camera_left_x1"; // Custom name for X1
    } else if (ns == "ZED_CAMERA_2i") {
        cloud.header.frame_id = "zed_camera_left_2i"; // Custom name for 2i
    } else {
        // Fallback to standard name
        cloud.header.frame_id = "zed_camera_left";
    }
    
    // For better visualization, downsample the point cloud but keep enough detail for proper visualization
    int downsample_factor = 2; // Use 2 for half resolution for better quality while keeping message size manageable
    cloud.height = zed_cloud.getHeight() / downsample_factor;
    cloud.width = zed_cloud.getWidth() / downsample_factor;
    cloud.is_dense = true; // Set to true for better compatibility with RViz
    cloud.is_bigendian = false;
    
    // Set point cloud format
    const size_t point_step = 16; // XYZRGBA - 4 bytes per field (x,y,z,rgba)
    cloud.point_step = point_step;
    cloud.row_step = cloud.width * point_step;
    
    // Define fields
    sensor_msgs::msg::PointField field_x, field_y, field_z, field_rgb;
    
    field_x.name = "x";
    field_x.offset = 0;
    field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_x.count = 1;
    
    field_y.name = "y";
    field_y.offset = 4;
    field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_y.count = 1;
    
    field_z.name = "z";
    field_z.offset = 8;
    field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_z.count = 1;
    
    // CRITICAL: The field name must be "rgb" for PCL/RViz compatibility
    field_rgb.name = "rgb";
    field_rgb.offset = 12;
    field_rgb.datatype = sensor_msgs::msg::PointField::UINT32; // Must be UINT32 for RGB
    field_rgb.count = 1;
    
    cloud.fields = {field_x, field_y, field_z, field_rgb};
    
    // Resize the data buffer for the downsampled point cloud
    const size_t data_size = cloud.height * cloud.width * point_step;
    cloud.data.resize(data_size);
    
    // Safety check to ensure we have valid data to copy
    if (!zed_cloud.getPtr<sl::float4>()) {
      std::cerr << "Error: Point cloud data pointer is null!" << std::endl;
      return false;
    }
    
    // Create a simplified and RViz-friendly point cloud
    sl::float4* src_ptr = zed_cloud.getPtr<sl::float4>();
    
    // Clear the data buffer first
    cloud.data.clear();
    cloud.data.resize(cloud.height * cloud.width * point_step, 0);
    uint8_t* dst_ptr = cloud.data.data();
    
    // Simplified point cloud with only valid points
    int valid_points = 0;
    for (unsigned int y = 0; y < cloud.height; y++) {
      for (unsigned int x = 0; x < cloud.width; x++) {
        // Source index in the original cloud
        size_t src_idx = (y * downsample_factor) * zed_cloud.getWidth() + (x * downsample_factor);
        // Destination index in the downsampled cloud
        size_t dst_idx = y * cloud.width + x;
        
        // Skip boundary points which often have errors
        if (y < 2 || y >= cloud.height - 2 || x < 2 || x >= cloud.width - 2) {
          continue;
        }
        
        // Get the point from the original cloud
        sl::float4 point = src_ptr[src_idx];
        
        // Check if the point is valid (not NaN) - relaxed filtering to include more points
        if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z) && 
            std::abs(point.z) < 20.0f) { // Include points up to 20 meters for better visualization
          
          // PCL expects a different memory layout for RGB
          // Swap from RGBA to BGRA (PCL expects a certain memory layout)
          uint8_t* rgba = (uint8_t*)&point.w;
          uint8_t r = rgba[0];
          uint8_t g = rgba[1];
          uint8_t b = rgba[2];
          uint8_t a = rgba[3];
          
          // Create RGB in PCL's expected format (BGR format where B is in the highest bits)
          // PCL expects BGR ordering, not RGB
          uint32_t rgb_val = ((uint32_t)b << 16 | (uint32_t)g << 8 | (uint32_t)r);
          
          // Create a temporary buffer for this point
          uint8_t point_data[point_step];
          
          // Copy XYZ coordinates (12 bytes)
          memcpy(point_data, &point, 12);
          
          // Copy RGB value (4 bytes)
          memcpy(point_data + 12, &rgb_val, 4);
          
          // Copy the assembled point to the destination buffer
          memcpy(dst_ptr + (dst_idx * point_step), point_data, point_step);
          valid_points++;
        }
      }
    }
    
    // Print cloud statistics
    float valid_percentage = (valid_points * 100.0f) / (cloud.width * cloud.height);
    std::cout << "Publishing point cloud: " << cloud.width << "x" << cloud.height 
              << " with " << valid_points << " valid points (" << valid_percentage << "% valid)"
              << " (" << (data_size / 1024) << " KB)" << std::endl;
    
    if (valid_points < 100) {
      std::cerr << "Warning: Very few valid points in cloud! Check camera positioning and depth settings." << std::endl;
      // Still return true to publish what we have
    }
    
    return true;
  }
  catch (const std::exception& e) {
    std::cerr << "Exception getting point cloud: " << e.what() << std::endl;
    return false;
  }
}

ZedCameraDriver::ImuData ZedCameraDriver::getImuData() {
  ImuData imu_data = {};
  
  if (!connected_ || !zed_) {
    return imu_data;
  }
  
  try {
    // Get sensor data from ZED camera
    sl::SensorsData sensor_data;
    if (zed_->getSensorsData(sensor_data, sl::TIME_REFERENCE::CURRENT) != sl::ERROR_CODE::SUCCESS) {
      std::cerr << "Error getting sensor data" << std::endl;
      return imu_data;
    }
    
    // Get IMU data
    sl::SensorsData::IMUData zed_imu_data = sensor_data.imu;
    
    // Set timestamp (milliseconds since epoch)
    imu_data.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    // Get linear acceleration (m/s²)
    imu_data.linear_acceleration[0] = zed_imu_data.linear_acceleration.x;
    imu_data.linear_acceleration[1] = zed_imu_data.linear_acceleration.y;
    imu_data.linear_acceleration[2] = zed_imu_data.linear_acceleration.z;
    
    // Get angular velocity (rad/s)
    imu_data.angular_velocity[0] = zed_imu_data.angular_velocity.x;
    imu_data.angular_velocity[1] = zed_imu_data.angular_velocity.y;
    imu_data.angular_velocity[2] = zed_imu_data.angular_velocity.z;
    
    // Get orientation as quaternion (x, y, z, w)
    imu_data.orientation[0] = zed_imu_data.pose.getOrientation()[0];
    imu_data.orientation[1] = zed_imu_data.pose.getOrientation()[1];
    imu_data.orientation[2] = zed_imu_data.pose.getOrientation()[2];
    imu_data.orientation[3] = zed_imu_data.pose.getOrientation()[3];
    
    return imu_data;
  }
  catch (const std::exception& e) {
    std::cerr << "Exception getting IMU data: " << e.what() << std::endl;
    return imu_data;
  }
}

ZedCameraDriver::CameraStatus ZedCameraDriver::getStatus() {
  CameraStatus status = {};
  
  status.connected = connected_;
  
  if (!connected_ || !zed_) {
    status.status_message = "Camera not connected";
    return status;
  }
  
  try {
    // Get current FPS
    status.current_fps = zed_->getCurrentFPS();
    
    // Set a default temperature value
    status.temperature = 25.0f;
    
    // Get camera information
    sl::CameraInformation cam_info = zed_->getCameraInformation();
    std::string serial_str = std::to_string(cam_info.serial_number);
    std::string model_str = std::string(sl::toString(cam_info.camera_model).c_str());
    
    // Add camera identification in status message
    std::string camera_id = "Unknown";
    if (CAMERA_SERIALS.find(cam_info.serial_number) != CAMERA_SERIALS.end()) {
      camera_id = CAMERA_SERIALS.at(cam_info.serial_number);
    }
    
    status.status_message = "Camera " + camera_id + " (" + model_str + " SN:" + serial_str + ") operating normally";
    
    return status;
  }
  catch (const std::exception& e) {
    std::cerr << "Exception getting camera status: " << e.what() << std::endl;
    status.status_message = "Error getting camera status: " + std::string(e.what());
    return status;
  }
}

bool ZedCameraDriver::startRecording(const std::string& filename) {
  if (!connected_ || !zed_) {
    return false;
  }
  
  if (recording_) {
    // Already recording
    return true;
  }
  
  try {
    // Configure SVO recording parameters
    sl::RecordingParameters params;
    params.video_filename = filename.c_str();
    params.compression_mode = sl::SVO_COMPRESSION_MODE::H264;
    
    // Start recording
    sl::ERROR_CODE err = zed_->enableRecording(params);
    if (err != sl::ERROR_CODE::SUCCESS) {
      std::cerr << "Error starting SVO recording: " << sl::toString(err) << std::endl;
      return false;
    }
    
    svo_filename_ = filename;
    recording_ = true;
    std::cout << "Started recording to " << filename << std::endl;
    
    return true;
  }
  catch (const std::exception& e) {
    std::cerr << "Exception starting recording: " << e.what() << std::endl;
    recording_ = false;
    return false;
  }
}

bool ZedCameraDriver::stopRecording() {
  if (!connected_ || !zed_ || !recording_) {
    return false;
  }
  
  try {
    // Stop recording
    zed_->disableRecording();
    recording_ = false;
    std::cout << "Stopped recording SVO file" << std::endl;
    
    return true;
  }
  catch (const std::exception& e) {
    std::cerr << "Exception stopping recording: " << e.what() << std::endl;
    recording_ = false;
    return false;
  }
}

bool ZedCameraDriver::isRecording() const {
  return recording_;
}

void ZedCameraDriver::updateCameraStatus() {
  if (!connected_ || !zed_) {
    return;
  }
  
  try {
    // Get camera information
    sl::CameraInformation cam_info = zed_->getCameraInformation();
    std::cout << "Camera " << cam_info.serial_number << " status updated" << std::endl;
  }
  catch (const std::exception& e) {
    std::cerr << "Exception updating camera status: " << e.what() << std::endl;
  }
}

// Helper method to fill a ROS image message from a ZED image
void ZedCameraDriver::fillImageMsg(sensor_msgs::msg::Image& ros_image, const sl::Mat& zed_image, const std::string& frame_id) {
  ros_image.header.stamp = rclcpp::Clock().now();
  ros_image.header.frame_id = frame_id;
  ros_image.height = zed_image.getHeight();
  ros_image.width = zed_image.getWidth();
  
  // Set encoding based on the image format
  switch (zed_image.getDataType()) {
    case sl::MAT_TYPE::U8_C1:
      ros_image.encoding = sensor_msgs::image_encodings::MONO8;
      break;
    case sl::MAT_TYPE::U8_C3:
      ros_image.encoding = sensor_msgs::image_encodings::RGB8;
      break;
    case sl::MAT_TYPE::U8_C4:
      ros_image.encoding = sensor_msgs::image_encodings::RGBA8;
      break;
    case sl::MAT_TYPE::F32_C1:
      ros_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      break;
    default:
      std::cerr << "Unsupported ZED image format: " << static_cast<int>(zed_image.getDataType()) << std::endl;
      return;
  }
  
  ros_image.is_bigendian = false;
  ros_image.step = zed_image.getStepBytes();
  
  // Copy the image data directly from the ZED SDK
  const size_t data_size = zed_image.getHeight() * zed_image.getStepBytes();
  ros_image.data.resize(data_size);
  memcpy(ros_image.data.data(), zed_image.getPtr<sl::uchar1>(), data_size);
}

}  // namespace camera
}  // namespace sensors