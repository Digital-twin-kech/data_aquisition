/**
 * @file livox_converter.cpp
 * @brief Implementation of converter utilities for Livox point clouds
 */

#include "sensors/lidar/livox_converter.h"
#include <vector>
#include <cmath>
#include <cstring> // For memcpy

namespace sensors {
namespace lidar {

sensor_msgs::msg::PointCloud2::SharedPtr 
LivoxConverter::convertToPointCloud2(
    const std::vector<uint8_t>& raw_data, 
    const std::string& frame_id) {
  
  // This is a placeholder implementation
  // In a real implementation, we would convert raw Livox data to PointCloud2
  
  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  
  // Set header
  cloud->header.frame_id = frame_id;
  cloud->header.stamp = {};  // Use default timestamp
  
  // Set basic fields for XYZRGB point cloud
  cloud->fields.resize(4);
  cloud->fields[0].name = "x";
  cloud->fields[0].offset = 0;
  cloud->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud->fields[0].count = 1;
  
  cloud->fields[1].name = "y";
  cloud->fields[1].offset = 4;
  cloud->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud->fields[1].count = 1;
  
  cloud->fields[2].name = "z";
  cloud->fields[2].offset = 8;
  cloud->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud->fields[2].count = 1;
  
  // CRITICAL: The field name must be "rgb" for PCL/RViz compatibility
  cloud->fields[3].name = "rgb";
  cloud->fields[3].offset = 12;
  cloud->fields[3].datatype = sensor_msgs::msg::PointField::UINT32;
  cloud->fields[3].count = 1;
  
  // Set point cloud properties
  cloud->point_step = 16;  // Size of each point (including padding)
  cloud->height = 1;       // Unorganized point cloud
  
  // In a real implementation, we would parse raw_data to populate points
  // For now, just create a minimal cloud
  cloud->width = 10;  // Just 10 points for placeholder
  cloud->row_step = cloud->width * cloud->point_step;
  cloud->is_dense = true;
  
  // Allocate data
  cloud->data.resize(cloud->row_step);
  
  // Fill with sample data
  for (size_t i = 0; i < cloud->width; ++i) {
    size_t idx = i * cloud->point_step;
    
    // x, y, z coordinates
    float x = static_cast<float>(i) * 0.1f;
    float y = static_cast<float>(i) * 0.05f;
    float z = static_cast<float>(i) * 0.01f;
    
    std::memcpy(&cloud->data[idx + 0], &x, sizeof(float));
    std::memcpy(&cloud->data[idx + 4], &y, sizeof(float));
    std::memcpy(&cloud->data[idx + 8], &z, sizeof(float));
    
    // Create RGB values with proper PCL RGB format
    uint8_t r = 100;  // Adjust these values for better visualization
    uint8_t g = 180;
    uint8_t b = 220;
    
    // Create RGB in PCL's expected format (BGR format where B is in the highest bits)
    uint32_t rgb_val = ((uint32_t)b << 16 | (uint32_t)g << 8 | (uint32_t)r);
    
    // Copy RGB value as a single UINT32
    std::memcpy(&cloud->data[idx + 12], &rgb_val, sizeof(uint32_t));
  }
  
  return cloud;
}

sensor_msgs::msg::PointCloud2::SharedPtr 
LivoxConverter::downsamplePointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr& input, 
    int downsample_factor) {
  
  if (downsample_factor <= 1) {
    // No downsampling needed
    return input;
  }
  
  // Create a new point cloud with same format but fewer points
  auto output = std::make_shared<sensor_msgs::msg::PointCloud2>();
  
  // Copy header and field definitions
  output->header = input->header;
  output->fields = input->fields;
  output->point_step = input->point_step;
  output->is_dense = input->is_dense;
  
  // Calculate new dimensions
  if (input->height > 1) {
    // Organized point cloud (like depth camera)
    output->height = input->height / downsample_factor;
    output->width = input->width / downsample_factor;
  } else {
    // Unorganized point cloud (like lidar)
    output->height = 1;
    output->width = input->width / downsample_factor;
  }
  
  output->row_step = output->width * output->point_step;
  output->data.resize(output->row_step * output->height);
  
  // Copy every Nth point
  for (size_t y = 0; y < output->height; ++y) {
    for (size_t x = 0; x < output->width; ++x) {
      size_t src_idx = ((y * downsample_factor) * input->width + (x * downsample_factor)) * input->point_step;
      size_t dst_idx = (y * output->width + x) * output->point_step;
      
      // Copy the point data
      memcpy(&output->data[dst_idx], &input->data[src_idx], output->point_step);
    }
  }
  
  return output;
}

sensor_msgs::msg::PointCloud2::SharedPtr 
LivoxConverter::filterPointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr& input,
    float min_distance, 
    float max_distance) {
  
  // Create a new point cloud with same format
  auto output = std::make_shared<sensor_msgs::msg::PointCloud2>();
  
  // Copy header and field definitions
  output->header = input->header;
  output->fields = input->fields;
  output->point_step = input->point_step;
  output->is_dense = true;  // We'll make it dense by filtering out invalid points
  
  // Temporary storage for filtered points
  std::vector<uint8_t> filtered_data;
  filtered_data.reserve(input->data.size());  // Reserve max capacity
  
  // Find offsets for x, y, z fields
  size_t x_offset = 0, y_offset = 0, z_offset = 0;
  for (const auto& field : input->fields) {
    if (field.name == "x") x_offset = field.offset;
    if (field.name == "y") y_offset = field.offset;
    if (field.name == "z") z_offset = field.offset;
  }
  
  // Filter points
  size_t valid_points = 0;
  for (size_t i = 0; i < input->width * input->height; ++i) {
    size_t idx = i * input->point_step;
    
    // Get x, y, z coordinates
    float x, y, z;
    memcpy(&x, &input->data[idx + x_offset], sizeof(float));
    memcpy(&y, &input->data[idx + y_offset], sizeof(float));
    memcpy(&z, &input->data[idx + z_offset], sizeof(float));
    
    // Calculate distance
    float distance = std::sqrt(x*x + y*y + z*z);
    
    // Check if point is within valid range
    if (std::isfinite(distance) && distance >= min_distance && distance <= max_distance) {
      // Copy the point data
      filtered_data.insert(filtered_data.end(), 
                          input->data.begin() + idx, 
                          input->data.begin() + idx + input->point_step);
      ++valid_points;
    }
  }
  
  // Set dimensions for filtered cloud
  output->height = 1;
  output->width = valid_points;
  output->row_step = output->width * output->point_step;
  output->data = std::move(filtered_data);
  
  return output;
}

} // namespace lidar
} // namespace sensors