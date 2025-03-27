#ifndef SYNC_METRICS_MSG_HPP
#define SYNC_METRICS_MSG_HPP

#include <string>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>

namespace data_aquisition {
namespace sync {

// Since we don't have custom message definitions, 
// we'll use standard ROS messages to publish our metrics
struct SyncMetricsData {
  double time_tolerance;
  int cache_size;
  double avg_offset;
  double max_offset;
  double min_offset;
  double std_dev;
  int dropped_messages;
  double max_delay;
  double avg_processing_time;
  std::string sync_policy;
};

}  // namespace sync
}  // namespace data_aquisition

#endif  // SYNC_METRICS_MSG_HPP