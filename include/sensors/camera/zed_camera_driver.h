#ifndef ZED_CAMERA_DRIVER_H
#define ZED_CAMERA_DRIVER_H

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include "sensors/camera/camera_config.h"

// Check if ZED SDK is available
#if __has_include(<sl/Camera.hpp>)
  #define ZED_SDK_AVAILABLE 1
  #include <sl/Camera.hpp>
#else
  #define ZED_SDK_AVAILABLE 0
  // Mock ZED SDK definitions
  namespace sl {
    // Camera model enum
    enum MODEL { 
      ZED, 
      ZED_M, 
      ZED2, 
      ZED2i, 
      ZED_X, 
      ZED_XM 
    };

    // Resolution enum
    enum RESOLUTION { 
      HD2K, 
      HD1080, 
      HD720, 
      VGA, 
      AUTO 
    };

    // Depth mode enum
    enum DEPTH_MODE { 
      NONE, 
      PERFORMANCE, 
      QUALITY, 
      ULTRA, 
      NEURAL 
    };

    // View type enum for retrieving images
    enum VIEW {
      LEFT,
      RIGHT,
      LEFT_UNRECTIFIED,
      RIGHT_UNRECTIFIED,
      LEFT_GRAY,
      RIGHT_GRAY,
      LEFT_UNRECTIFIED_GRAY,
      RIGHT_UNRECTIFIED_GRAY,
      SIDE_BY_SIDE,
      VIEW_DEPTH,           // Renamed to avoid conflict with MEASURE::DEPTH
      VIEW_CONFIDENCE,      // Renamed to avoid conflict with MEASURE::CONFIDENCE
      VIEW_NORMALS,         // Renamed to avoid conflict with MEASURE::NORMALS
      VIEW_DEPTH_RIGHT,     // Renamed to avoid conflict with MEASURE::DEPTH_RIGHT
      VIEW_NORMALS_RIGHT    // Renamed to avoid conflict with MEASURE::NORMALS_RIGHT
    };

    // Measure type enum for retrieving measures
    enum MEASURE {
      MEASURE_DEPTH,         // Renamed to avoid conflict with VIEW::DEPTH
      MEASURE_CONFIDENCE,    // Renamed to avoid conflict with VIEW::CONFIDENCE
      XYZ,
      XYZRGBA,
      XYZBGRA,
      XYZARGB,
      XYZABGR,
      MEASURE_NORMALS,       // Renamed to avoid conflict with VIEW::NORMALS
      DISPARITY,
      MEASURE_DEPTH_RIGHT,   // Renamed to avoid conflict with VIEW::DEPTH_RIGHT
      MEASURE_NORMALS_RIGHT  // Renamed to avoid conflict with VIEW::NORMALS_RIGHT
    };

    // Reference frame enum
    enum REFERENCE_FRAME {
      WORLD,
      CAMERA
    };

    // Unit enum
    enum UNIT {
      MILLIMETER,
      CENTIMETER,
      METER,
      INCH,
      FOOT
    };

    // SVO compression modes
    enum SVO_COMPRESSION_MODE {
      LOSSLESS,
      H264,
      H265,
      H264_LOSSLESS,
      H265_LOSSLESS
    };

    // Time reference enum
    enum TIME_REFERENCE {
      IMAGE,
      CURRENT
    };

    // Memory type enum
    enum MEM {
      CPU,
      GPU
    };

    // Mat type enum
    enum MAT_TYPE {
      F32_C1,
      F32_C2,
      F32_C3,
      F32_C4,
      U8_C1,
      U8_C2,
      U8_C3,
      U8_C4,
      U16_C1
    };

    // Error code enum
    enum ERROR_CODE {
      SUCCESS,
      FAILURE,
      NO_GPU_COMPATIBLE,
      NOT_ENOUGH_GPU_MEMORY,
      CAMERA_NOT_DETECTED,
      SENSORS_NOT_AVAILABLE,
      INVALID_RESOLUTION,
      LOW_USB_BANDWIDTH,
      CALIBRATION_FILE_NOT_AVAILABLE,
      INVALID_SVO_FILE,
      SVO_RECORDING_ERROR,
      SVO_UNSUPPORTED_COMPRESSION,
      END_OF_SVOFILE_REACHED,
      INVALID_COORDINATE_SYSTEM,
      INVALID_FIRMWARE,
      INVALID_FUNCTION_PARAMETERS,
      CUDA_ERROR,
      CAMERA_NOT_INITIALIZED,
      NVIDIA_DRIVER_OUT_OF_DATE,
      INVALID_FUNCTION_CALL,
      CORRUPTED_SDK_INSTALLATION,
      INCOMPATIBLE_SDK_VERSION,
      INVALID_AREA_FILE,
      INCOMPATIBLE_AREA_FILE,
      CAMERA_FAILED_TO_SETUP,
      CAMERA_DETECTION_ISSUE,
      CANNOT_START_CAMERA_STREAM,
      NO_GPU_DETECTED,
      PLANE_NOT_FOUND,
      MODULE_NOT_COMPATIBLE_WITH_CAMERA,
      MOTION_SENSORS_REQUIRED,
      MODULE_NOT_COMPATIBLE_WITH_CUDA_VERSION
    };

    // Input parameters for camera
    struct InputType {
      enum INPUT_TYPE {
        USB,
        SVO,
        STREAM,
        GMSL
      };
      
      INPUT_TYPE input_type = INPUT_TYPE::USB;
      void setFromSerialNumber(int serial) {}
    };

    // Basic vector types
    struct Vector2 {
      float x, y;
    };

    struct Vector3 {
      float x, y, z;
    };

    struct Vector4 {
      float x, y, z, w;
    };

    // Matrix/Vector definitions
    struct float4 {
      float x, y, z, w;
    };

    struct uchar1 {
      unsigned char x;
    };

    struct uchar3 {
      unsigned char x, y, z;
    };

    struct uchar4 {
      unsigned char x, y, z, w;
    };

    // Rotation and translation structures
    struct Rotation {
      float r00 = 1.0f, r01 = 0.0f, r02 = 0.0f;
      float r10 = 0.0f, r11 = 1.0f, r12 = 0.0f;
      float r20 = 0.0f, r21 = 0.0f, r22 = 1.0f;
    };

    struct Translation {
      float tx = 0.0f, ty = 0.0f, tz = 0.0f;
    };

    // Quaternion structure
    struct Quaternion {
      float x = 0.0f, y = 0.0f, z = 0.0f, w = 1.0f;
      
      // Allow array-like access for compatibility
      float operator[](int index) const {
        switch(index) {
          case 0: return x;
          case 1: return y;
          case 2: return z;
          case 3: return w;
          default: return 0.0f;
        }
      }
    };

    // Rect structure
    struct Rect {
      size_t width = 1280;
      size_t height = 720;
    };

    // Transform structure
    struct Transform {
      Rotation rotation;
      Translation translation;
    };

    // Pose structure for IMU data
    struct Pose {
      Translation translation;
      Rotation rotation;
      Quaternion getOrientation() const { 
        return Quaternion{0, 0, 0, 1}; 
      }
    };

    // Camera settings
    struct CameraConfiguration {
      int firmware_version = 1000;
      std::string serial_number = "0";
      Rect resolution;
    };

    // Runtime parameters
    struct RuntimeParameters {
      bool enable_depth = true;
      DEPTH_MODE depth_mode = DEPTH_MODE::PERFORMANCE;
      float depth_minimum_distance = 0.3f;
      float depth_maximum_distance = 40.0f;
      REFERENCE_FRAME measure3D_reference_frame = REFERENCE_FRAME::CAMERA;
    };

    // Init parameters
    struct InitParameters {
      InputType input;
      RESOLUTION camera_resolution = RESOLUTION::HD720;
      int camera_fps = 30;
      int camera_image_flip = 0;
      bool enable_right_side_measure = false;
      bool sdk_verbose = false;
      bool sensors_required = false;
      bool async_grab_camera_recovery = false;
      bool depth_stabilization = false;
      UNIT coordinate_units = UNIT::METER;
      DEPTH_MODE depth_mode = DEPTH_MODE::PERFORMANCE;
    };

    // Positional tracking parameters
    struct PositionalTrackingParameters {
      bool enable_area_memory = true;
    };

    // Camera information
    struct CameraInformation {
      MODEL camera_model = ZED2i;
      unsigned int serial_number = 0;
      InputType::INPUT_TYPE input_type = InputType::INPUT_TYPE::USB;
      CameraConfiguration camera_configuration;
    };

    // Additional IMU data structures
    struct SensorsData {
      struct IMUData {
        Vector3 linear_acceleration;
        Vector3 angular_velocity;
        Pose pose;
        unsigned long long timestamp = 0;
      };
      
      IMUData imu;
      
      ERROR_CODE getMagnetometerData(Vector3& mag_data, unsigned long long* timestamp = nullptr) { 
        mag_data = Vector3{0, 0, 0};
        if (timestamp) *timestamp = 0;
        return ERROR_CODE::SUCCESS;
      }
    };

    // Recording parameters
    struct RecordingParameters {
      const char* video_filename = nullptr;
      SVO_COMPRESSION_MODE compression_mode = SVO_COMPRESSION_MODE::H264;
    };

    // Main mat class for images and depth
    class Mat {
    public:
      Mat() : width_(0), height_(0), type_(MAT_TYPE::F32_C4), step_bytes_(0) {}
      
      void alloc(size_t width, size_t height, MAT_TYPE type = MAT_TYPE::F32_C4) {
        width_ = width;
        height_ = height;
        type_ = type;
        
        // Calculate step bytes based on the type
        switch(type_) {
          case MAT_TYPE::F32_C1: step_bytes_ = width_ * 4; break;
          case MAT_TYPE::F32_C2: step_bytes_ = width_ * 4 * 2; break;
          case MAT_TYPE::F32_C3: step_bytes_ = width_ * 4 * 3; break;
          case MAT_TYPE::F32_C4: step_bytes_ = width_ * 4 * 4; break;
          case MAT_TYPE::U8_C1: step_bytes_ = width_ * 1; break;
          case MAT_TYPE::U8_C2: step_bytes_ = width_ * 1 * 2; break;
          case MAT_TYPE::U8_C3: step_bytes_ = width_ * 1 * 3; break;
          case MAT_TYPE::U8_C4: step_bytes_ = width_ * 1 * 4; break;
          case MAT_TYPE::U16_C1: step_bytes_ = width_ * 2; break;
          default: step_bytes_ = width_ * 4; break;
        }
        
        // Allocate the data
        data_.resize(step_bytes_ * height_);
      }
      
      void write(const std::string& filename) {}
      
      size_t getWidth() const { return width_; }
      size_t getHeight() const { return height_; }
      size_t getStepBytes() const { return step_bytes_; }
      MAT_TYPE getDataType() const { return type_; }
      
      template<typename T>
      T* getPtr() { return reinterpret_cast<T*>(data_.data()); }
      
      template<typename T>
      const T* getPtr() const { return reinterpret_cast<const T*>(data_.data()); }
      
    private:
      size_t width_;
      size_t height_;
      MAT_TYPE type_;
      size_t step_bytes_;
      std::vector<unsigned char> data_;
    };

    // String conversion utilities
    inline std::string toString(const ERROR_CODE& error) {
      switch(error) {
        case SUCCESS: return "SUCCESS";
        case CAMERA_NOT_DETECTED: return "CAMERA_NOT_DETECTED";
        default: return "UNKNOWN_ERROR";
      }
    }

    inline std::string toVerbose(const ERROR_CODE& error) {
      switch(error) {
        case SUCCESS: return "Operation succeeded";
        case CAMERA_NOT_DETECTED: return "Camera not detected or already in use";
        default: return "Unknown error occurred";
      }
    }

    inline std::string toString(const MODEL& model) {
      switch(model) {
        case ZED: return "ZED";
        case ZED_M: return "ZED Mini";
        case ZED2: return "ZED 2";
        case ZED2i: return "ZED 2i";
        case ZED_X: return "ZED X";
        case ZED_XM: return "ZED XM";
        default: return "Unknown";
      }
    }

    inline std::string toString(const InputType::INPUT_TYPE& input_type) {
      return "USB";
    }

    // Main camera class
    class Camera {
    public:
      // Constructor/Destructor
      Camera() {}
      ~Camera() {}
      
      // Core methods
      ERROR_CODE open(InitParameters& parameters) { 
        return ERROR_CODE::SUCCESS; 
      }
      
      void close() {}
      
      ERROR_CODE grab(RuntimeParameters& parameters) {
        return ERROR_CODE::SUCCESS;
      }
      
      CameraInformation getCameraInformation() { 
        CameraInformation info;
        info.camera_model = ZED2i;
        info.serial_number = 0;
        info.camera_configuration.resolution.width = 1280;
        info.camera_configuration.resolution.height = 720;
        return info;
      }
      
      bool enablePositionalTracking(PositionalTrackingParameters& params) { 
        return false; 
      }
      
      ERROR_CODE retrieveImage(Mat& image, VIEW view, MEM memory = MEM::CPU) {
        // Create a mock image of appropriate size
        image.alloc(1280, 720, view == VIEW_DEPTH ? MAT_TYPE::F32_C1 : MAT_TYPE::U8_C4);
        return ERROR_CODE::SUCCESS;
      }
      
      ERROR_CODE retrieveMeasure(Mat& measure, MEASURE measure_type, MEM memory = MEM::CPU) {
        // Create a mock measure of appropriate size and type
        MAT_TYPE type = MAT_TYPE::F32_C4;
        
        switch(measure_type) {
          case MEASURE_DEPTH:
          case MEASURE_CONFIDENCE:
          case DISPARITY:
            type = MAT_TYPE::F32_C1;
            break;
          
          case XYZ:
            type = MAT_TYPE::F32_C3;
            break;
            
          case XYZRGBA:
          case XYZBGRA:
          case XYZARGB:
          case XYZABGR:
            type = MAT_TYPE::F32_C4;
            break;
            
          default:
            type = MAT_TYPE::F32_C4;
            break;
        }
        
        measure.alloc(1280, 720, type);
        return ERROR_CODE::SUCCESS;
      }
      
      ERROR_CODE enableRecording(const RecordingParameters& params) {
        return ERROR_CODE::SUCCESS;
      }
      
      void disableRecording() {}
      
      float getCurrentFPS() const {
        return 30.0f;
      }
      
      ERROR_CODE getSensorsData(SensorsData& data, TIME_REFERENCE time_ref = TIME_REFERENCE::CURRENT) {
        data.imu.linear_acceleration = Vector3{0, 0, 9.81f};
        data.imu.angular_velocity = Vector3{0, 0, 0};
        data.imu.timestamp = 0;
        return ERROR_CODE::SUCCESS;
      }
    };
  }
#endif

namespace sensors {
namespace camera {

/**
 * @brief Class for interfacing with ZED cameras
 * 
 * This class handles direct communication with ZED cameras via the ZED SDK.
 * It provides methods to control the camera and retrieve data.
 */
class ZedCameraDriver {
public:
  /**
   * @brief Struct to hold IMU data
   */
  struct ImuData {
    double timestamp;
    double linear_acceleration[3];
    double angular_velocity[3];
    double orientation[4];  // Quaternion (x, y, z, w)
  };

  /**
   * @brief Struct to hold camera status information
   */
  struct CameraStatus {
    bool connected;
    float temperature;
    float current_fps;
    std::string status_message;
  };

  /**
   * @brief Construct a new Zed Camera Driver object
   * 
   * @param config Camera configuration
   */
  explicit ZedCameraDriver(std::shared_ptr<CameraConfig> config);

  /**
   * @brief Destructor
   */
  ~ZedCameraDriver();

  /**
   * @brief Connect to the camera
   * 
   * @return true if connection successful
   * @return false if connection failed
   */
  bool connect();

  /**
   * @brief Disconnect from the camera
   */
  void disconnect();

  /**
   * @brief Check if camera is connected
   * 
   * @return true if camera is connected
   * @return false if camera is not connected
   */
  bool isConnected() const;

  /**
   * @brief Set frame rate of the camera
   * 
   * @param fps Target frames per second
   * @return true if successful
   * @return false if failed
   */
  bool setFrameRate(float fps);

  /**
   * @brief Get current frame rate of the camera
   * 
   * @return float Current frame rate
   */
  float getCurrentFrameRate() const;

  /**
   * @brief Get RGB image from the camera
   * 
   * @param image Reference to image message to be filled
   * @return true if image was successfully retrieved
   * @return false if failed to retrieve image
   */
  bool getRgbImage(sensor_msgs::msg::Image& image);

  /**
   * @brief Get depth image from the camera
   * 
   * @param image Reference to image message to be filled
   * @return true if image was successfully retrieved
   * @return false if failed to retrieve image
   */
  bool getDepthImage(sensor_msgs::msg::Image& image);

  /**
   * @brief Get point cloud from the camera
   * 
   * @param cloud Reference to point cloud message to be filled
   * @return true if point cloud was successfully retrieved
   * @return false if failed to retrieve point cloud
   */
  bool getPointCloud(sensor_msgs::msg::PointCloud2& cloud);

  /**
   * @brief Get IMU data from the camera
   * 
   * @return ImuData Struct containing IMU data
   */
  ImuData getImuData();

  /**
   * @brief Get camera status
   * 
   * @return CameraStatus Struct containing camera status information
   */
  CameraStatus getStatus();

  /**
   * @brief Start recording to SVO file
   * 
   * @param filename Filename to save the SVO file
   * @return true if recording started successfully
   * @return false if failed to start recording
   */
  bool startRecording(const std::string& filename);

  /**
   * @brief Stop recording to SVO file
   * 
   * @return true if recording stopped successfully
   * @return false if failed to stop recording
   */
  bool stopRecording();

  /**
   * @brief Check if recording is active
   * 
   * @return true if recording
   * @return false if not recording
   */
  bool isRecording() const;
  
  /**
   * @brief Get the frame ID for this camera
   * 
   * @return std::string Frame ID from camera configuration
   */
  std::string getFrameId() const {
    return config_->getFrameId();
  }

private:
  // Camera configuration
  std::shared_ptr<CameraConfig> config_;
  
  // ZED SDK objects
  std::unique_ptr<sl::Camera> zed_;
  sl::RuntimeParameters runtime_params_;
  
  // Camera state
  bool connected_;
  bool recording_;
  float current_fps_;
  std::string svo_filename_;

  // Internal functions
  void updateCameraStatus();
  
  // Helper function to fill ROS2 image message from ZED image
  void fillImageMsg(sensor_msgs::msg::Image& ros_image, const sl::Mat& zed_image, const std::string& frame_id);
};

}  // namespace camera
}  // namespace sensors

#endif  // ZED_CAMERA_DRIVER_H