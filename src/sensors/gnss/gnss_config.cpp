#include "sensors/gnss/gnss_config.h"

namespace sensors {
namespace gnss {

GnssConfig::GnssConfig(rclcpp::Node::SharedPtr node)
: node_(node),
  gnss_model_("ublox-f9p"),
  serial_port_("/dev/ttyACM0"),
  baud_rate_(115200),
  frequency_(10.0),
  frame_id_("gnss_frame"),
  use_rtcm_corrections_(false),  // Explicitly disable RTK corrections
  rtcm_source_("none"),         // Set to none instead of NTRIP
  ntrip_server_(""),
  ntrip_username_(""),
  ntrip_password_(""),
  rtcm_serial_port_(""),
  use_dynamic_model_(true),
  dynamic_model_("automotive"),
  reliability_policy_(rclcpp::ReliabilityPolicy::Reliable),
  qos_history_depth_(10)
{
  loadParameters();
}

void GnssConfig::loadParameters()
{
  // GNSS receiver settings
  node_->declare_parameter<std::string>("gnss.model", gnss_model_);
  node_->declare_parameter<std::string>("gnss.serial_port", serial_port_);
  node_->declare_parameter<int>("gnss.baud_rate", baud_rate_);
  node_->declare_parameter<double>("gnss.frequency", frequency_);
  node_->declare_parameter<std::string>("gnss.frame_id", frame_id_);
  
  // RTK/RTCM settings
  node_->declare_parameter<bool>("gnss.use_rtcm_corrections", use_rtcm_corrections_);
  node_->declare_parameter<std::string>("gnss.rtcm_source", rtcm_source_);
  node_->declare_parameter<std::string>("gnss.ntrip_server", ntrip_server_);
  node_->declare_parameter<std::string>("gnss.ntrip_username", ntrip_username_);
  node_->declare_parameter<std::string>("gnss.ntrip_password", ntrip_password_);
  node_->declare_parameter<std::string>("gnss.rtcm_serial_port", rtcm_serial_port_);
  
  // Dynamic model settings
  node_->declare_parameter<bool>("gnss.use_dynamic_model", use_dynamic_model_);
  node_->declare_parameter<std::string>("gnss.dynamic_model", dynamic_model_);
  
  // QoS settings
  std::string reliability;
  node_->declare_parameter<std::string>("gnss.qos.reliability", "reliable");
  node_->declare_parameter<int>("gnss.qos.history_depth", qos_history_depth_);
  
  // Load parameters
  gnss_model_ = node_->get_parameter("gnss.model").as_string();
  serial_port_ = node_->get_parameter("gnss.serial_port").as_string();
  baud_rate_ = node_->get_parameter("gnss.baud_rate").as_int();
  frequency_ = node_->get_parameter("gnss.frequency").as_double();
  frame_id_ = node_->get_parameter("gnss.frame_id").as_string();
  
  use_rtcm_corrections_ = node_->get_parameter("gnss.use_rtcm_corrections").as_bool();
  rtcm_source_ = node_->get_parameter("gnss.rtcm_source").as_string();
  ntrip_server_ = node_->get_parameter("gnss.ntrip_server").as_string();
  ntrip_username_ = node_->get_parameter("gnss.ntrip_username").as_string();
  ntrip_password_ = node_->get_parameter("gnss.ntrip_password").as_string();
  rtcm_serial_port_ = node_->get_parameter("gnss.rtcm_serial_port").as_string();
  
  use_dynamic_model_ = node_->get_parameter("gnss.use_dynamic_model").as_bool();
  dynamic_model_ = node_->get_parameter("gnss.dynamic_model").as_string();
  
  reliability = node_->get_parameter("gnss.qos.reliability").as_string();
  qos_history_depth_ = node_->get_parameter("gnss.qos.history_depth").as_int();
  
  // Set QoS reliability policy
  if (reliability == "reliable") {
    reliability_policy_ = rclcpp::ReliabilityPolicy::Reliable;
  } else if (reliability == "best_effort") {
    reliability_policy_ = rclcpp::ReliabilityPolicy::BestEffort;
  }
  
  RCLCPP_INFO(node_->get_logger(), "GNSS configuration loaded: model=%s, port=%s", 
              gnss_model_.c_str(), serial_port_.c_str());
}

}  // namespace gnss
}  // namespace sensors