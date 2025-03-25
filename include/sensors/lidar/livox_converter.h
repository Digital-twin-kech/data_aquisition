/**
 * @file livox_converter.h
 * @brief Converter utilities for Livox point clouds
 */

#ifndef LIVOX_CONVERTER_H
#define LIVOX_CONVERTER_H

#include <memory>
#include <vector>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace sensors {
namespace lidar {

/**
 * @class LivoxConverter
 * @brief Utility class for converting Livox data formats
 */
class LivoxConverter {
public:
  /**
   * @brief Constructor
   */
  LivoxConverter() = default;
  
  /**
   * @brief Destructor
   */
  ~LivoxConverter() = default;

  /**
   * @brief Convert raw Livox point cloud to standard ROS PointCloud2
   * @param raw_data Raw Livox point data
   * @param frame_id Frame ID for the point cloud
   * @param timestamp Timestamp for the point cloud
   * @return Converted PointCloud2 message
   */
  static sensor_msgs::msg::PointCloud2::SharedPtr 
  convertToPointCloud2(const std::vector<uint8_t>& raw_data, 
                      const std::string& frame_id);

  /**
   * @brief Downsample a point cloud
   * @param input Input point cloud
   * @param downsample_factor Downsampling factor (e.g., 2 = half resolution)
   * @return Downsampled point cloud
   */
  static sensor_msgs::msg::PointCloud2::SharedPtr 
  downsamplePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& input, 
                      int downsample_factor);

  /**
   * @brief Filter invalid points from point cloud
   * @param input Input point cloud
   * @param min_distance Minimum valid distance
   * @param max_distance Maximum valid distance
   * @return Filtered point cloud
   */
  static sensor_msgs::msg::PointCloud2::SharedPtr 
  filterPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& input,
                 float min_distance, 
                 float max_distance);

private:
  // Add any private helper methods or data members here
};

} // namespace lidar
} // namespace sensors

#endif // LIVOX_CONVERTER_H