#include "sensors/gnss/gnss_node.h"
#include "sensors/gnss/cuda_runtime.h"

namespace sensors {
namespace gnss {

GnssNode::GnssNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("gnss_node", options),
  initialized_(false)
{
  // Check NVIDIA Jetson environment
  if (!validateJetsonEnvironment()) {
    RCLCPP_WARN(get_logger(), "Not running on a supported NVIDIA Jetson platform");
  }
  
  // Set up parameter callback
  param_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&GnssNode::parametersCallback, this, std::placeholders::_1));

  // Schedule initialization to be performed after construction is complete
  auto initialize_timer = this->create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() {
      this->initialize();
      // This is a one-shot timer, so cancel it after use
      this->initialize_timer_->cancel();
    });
  initialize_timer_ = initialize_timer;
}

void GnssNode::initialize()
{
  if (initialized_) {
    RCLCPP_WARN(get_logger(), "GNSS node already initialized");
    return;
  }

  RCLCPP_INFO(get_logger(), "Initializing GNSS node...");
  
  // Creating a shared_ptr to this node is now safe because the object is fully constructed
  auto node_shared = shared_from_this();
  
  // Create configuration
  config_ = std::make_shared<GnssConfig>(node_shared);
  
  // Create GNSS manager
  gnss_manager_ = std::make_unique<GnssManager>(node_shared, config_);
  
  // Initialize GNSS manager
  if (gnss_manager_->initialize()) {
    // Start GNSS manager
    gnss_manager_->start();
    initialized_ = true;
    RCLCPP_INFO(get_logger(), "GNSS node initialized and started");
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to initialize GNSS node");
  }
}

GnssNode::~GnssNode()
{
  if (initialized_ && gnss_manager_) {
    gnss_manager_->stop();
  }
  
  RCLCPP_INFO(get_logger(), "GNSS node shutting down");
}

std::string GnssNode::getName() const
{
  return get_name();
}

rcl_interfaces::msg::SetParametersResult GnssNode::parametersCallback(
  const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "Success";
  
  // Handle parameter changes
  for (const auto &param : parameters) {
    RCLCPP_INFO(get_logger(), "Parameter change: %s", param.get_name().c_str());
    
    // Parameters that require reinitialization should be handled here
    // For the current implementation, we don't support dynamic reconfiguration
    // of critical parameters that would require restarting the GNSS manager
  }
  
  return result;
}

bool GnssNode::validateJetsonEnvironment()
{
  // Check CUDA availability (indicates NVIDIA GPU)
  int deviceCount = 0;
  cudaError_t error = cudaGetDeviceCount(&deviceCount);
  
  if (error != cudaSuccess || deviceCount == 0) {
    RCLCPP_WARN(get_logger(), "CUDA not available or no CUDA devices found");
    return false;
  }
  
  // Get CUDA device properties
  cudaDeviceProp deviceProp;
  cudaGetDeviceProperties(&deviceProp, 0);
  
  RCLCPP_INFO(get_logger(), "Running on CUDA device: %s", deviceProp.name);
  
  // For development: Mock JetPack environment
  RCLCPP_INFO(get_logger(), "Mock JetPack environment for development");
  RCLCPP_INFO(get_logger(), "Simulating JetPack 5.1.2");
  return true;
  
  // In production environment, uncomment this code:
  /*
  // Check for Jetson-specific environment variables
  const char* jetpack_version = std::getenv("JETPACK_VERSION");
  if (!jetpack_version) {
    RCLCPP_WARN(get_logger(), "JETPACK_VERSION environment variable not found");
    return false;
  }
  
  RCLCPP_INFO(get_logger(), "Detected JetPack version: %s", jetpack_version);
  
  // Check if JetPack version is 5.x
  std::string version(jetpack_version);
  if (version.length() >= 1 && version[0] == '5') {
    RCLCPP_INFO(get_logger(), "Running on JetPack 5.x as required");
    return true;
  }
  
  RCLCPP_WARN(get_logger(), "JetPack version requirement not met. Required: 5.x, Found: %s", 
             jetpack_version);
  */
  return false;
}

}  // namespace gnss
}  // namespace sensors

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sensors::gnss::GnssNode)