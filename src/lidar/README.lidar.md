# Livox LiDAR Subsystem Documentation

## Overview

The LiDAR subsystem provides integration with Livox HAP LiDAR sensors. It offers a flexible approach that can use either a reference implementation or our custom integration. This document provides comprehensive information about the LiDAR subsystem's configuration, usage, and troubleshooting.

## Hardware Specifications

| Model | Type | Interface | Range | Points/s | IP (Default) |
|-------|------|-----------|-------|----------|--------------|
| Livox HAP | Mid-range | Ethernet | 0.1-450m | 240,000 | 192.168.1.100 |

## Network Configuration

The LiDAR uses the following default network configuration:

- **LiDAR IP**: 192.168.1.100
- **Host IP**: 192.168.1.50
- **Command Port**: 56000
- **Data Port**: 57000
- **IMU Port**: 58000

## Installation Requirements

1. **Livox SDK**: Required for communication with the LiDAR hardware
2. **ROS2 Humble**: For integration with the rest of the system
3. **Reference Implementation**: `/home/user/Desktop/instll_liv/ws_livox/`

## Integration Approaches

### 1. Reference Implementation (Recommended for Testing)

The reference Livox ROS2 driver is used directly. This is the most reliable approach for testing with hardware.

### 2. Custom Implementation (Recommended for Full System Integration)

Our custom implementation follows the Driver-Manager-Node pattern used throughout the project. This provides better integration with the system but may be less stable for direct hardware testing.

## Code Structure

### Core Components

1. **LidarDriver** (`lidar_driver.h`/`lidar_driver.cpp`)
   - Handles low-level communication with the LiDAR
   - Manages connection state and data parsing
   - Provides callbacks for point cloud and IMU data

2. **LidarManager** (`lidar_manager.h`/`lidar_manager.cpp`)
   - Manages the lifecycle of the LiDAR subsystem
   - Handles ROS2 publishers and subscriptions
   - Processes raw data into ROS2 messages

3. **LivoxLidarNode** (`livox_lidar_node.h`/`livox_lidar_node.cpp`)
   - Implements ROS2 lifecycle node for the LiDAR
   - Handles parameter loading and configuration
   - Manages node state transitions

4. **LivoxAdapter** (`livox_adapter_node.h`/`livox_adapter_node.cpp`)
   - Provides a bridge between the reference implementation and our system
   - Adapts data formats and coordinates from the reference driver

### Key Classes and Methods

#### LidarDriver

```cpp
// Initialize the driver
bool initialize();

// Connect to the LiDAR
bool connect();

// Start data acquisition
bool start();

// Stop data acquisition
void stop();

// Set callback for point cloud data
void setPointCloudCallback(std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr)> callback);
```

#### LidarManager

```cpp
// Initialize the manager
bool initialize(const rclcpp::Node::SharedPtr& node);

// Start data processing
bool start();

// Stop data processing
void stop();

// Process point cloud data
void handlePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
```

## Configuration

### Configuration Files

1. **YAML Configuration** (`config/lidar/livox_params.yaml`)
   - Used by our custom implementation
   - Contains parameters like IP addresses, ports, frame ID
   
   Example configuration:
   ```yaml
   lidar:
     frame_id: "livox_frame"
     ip_address: "192.168.1.100"
     host_ip_address: "192.168.1.50"
     use_dhcp: false
     data_port: 57000
     command_port: 56000
     imu_port: 58000
     broadcast_code: "livox0000000001"
     publish_frequency: 10.0
     point_cloud_type: 0  # 0=XYZ+Intensity, 1=XYZ+RGB
   ```

2. **JSON Configuration** (`config/lidar/HAP_config.json`)
   - Used by the reference implementation
   - Defines network configuration and device parameters

### Modifying Configuration

#### Changing IP Address

To change the LiDAR IP address:

1. Update `livox_params.yaml`:
   ```yaml
   lidar:
     ip_address: "192.168.1.101"  # New IP
     host_ip_address: "192.168.1.50"  # Host IP
   ```

2. If using the run script, use the IP parameter:
   ```bash
   ./scripts/run_livox_lidar.sh --ip 192.168.1.101
   ```

#### Changing Frame ID

To change the coordinate frame ID:

1. Update `livox_params.yaml`:
   ```yaml
   lidar:
     frame_id: "my_livox_frame"  # New frame ID
   ```

2. If using the reference implementation, specify it as a parameter:
   ```bash
   ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args -p frame_id:=my_livox_frame
   ```

## Running the LiDAR

### Using the Script (Recommended)

The simplest way to run the LiDAR is using the provided script:

```bash
# Run the LiDAR with default settings
./scripts/run_livox_lidar.sh

# Run with custom IP address
./scripts/run_livox_lidar.sh --ip 192.168.1.101
```

### Manual Execution

For more control, you can run the LiDAR components manually:

```bash
# Source ROS2 and reference implementation
source /opt/ros/humble/setup.bash
source /home/user/Desktop/instll_liv/ws_livox/install/setup.bash

# Run the LiDAR driver node
ros2 run livox_ros_driver2 livox_ros_driver2_node \
  --ros-args \
  -p xfer_format:=0 \
  -p multi_topic:=0 \
  -p data_src:=0 \
  -p publish_freq:=10.0 \
  -p output_data_type:=0 \
  -p frame_id:="livox_frame" \
  -p user_config_path:="path/to/HAP_config.json" \
  -p cmdline_input_bd_code:="livox0000000001"
```

## Published Topics

The LiDAR subsystem publishes to the following topics:

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | Raw point cloud data from the LiDAR |
| `/livox/status` | `std_msgs/String` | Status information from the LiDAR |
| `/livox/imu` | `sensor_msgs/Imu` | IMU data (if available) |
| `/livox_lidar/point_cloud` | `sensor_msgs/PointCloud2` | Processed point cloud (with adapter) |

## Troubleshooting

### Common Issues

#### Cannot Connect to LiDAR

**Symptoms**:
- "Cannot ping LiDAR" message
- No data received from LiDAR

**Solutions**:
1. Check physical connection (Ethernet cable)
2. Verify LiDAR power
3. Check network configuration:
   ```bash
   # Set static IP for host
   sudo ip addr add 192.168.1.50/24 dev eth0
   sudo ip link set eth0 up
   ```
4. Check firewall settings:
   ```bash
   # Allow UDP ports for LiDAR data
   sudo ufw allow 57000/udp
   sudo ufw allow 58000/udp
   ```

#### No Point Cloud Data

**Symptoms**:
- LiDAR is connected but no point cloud data is published
- `/livox/lidar` topic exists but has no subscribers or publishers

**Solutions**:
1. Check if LiDAR is in the correct work mode:
   ```bash
   # Check topic status
   ros2 topic info /livox/status
   ```
2. Verify JSON configuration:
   ```bash
   # Copy correct configuration
   cp config/lidar/HAP_config.json /home/user/Desktop/instll_liv/ws_livox/src/livox_ros_driver2/config/
   ```
3. Restart the LiDAR driver:
   ```bash
   # Restart using the script
   ./scripts/run_livox_lidar.sh
   ```

#### Poor Point Cloud Quality

**Symptoms**:
- Sparse or noisy point cloud
- Missing regions in the point cloud

**Solutions**:
1. Check LiDAR positioning and orientation
2. Clean the LiDAR lens
3. Adjust point cloud filtering parameters:
   ```yaml
   # In adapter configuration
   filter_points: true
   min_distance: 0.1  # Filter out points closer than 0.1m
   max_distance: 100.0  # Filter out points farther than 100m
   downsample_factor: 1  # No downsampling
   ```

## Advanced Topics

### Integrating with Custom Applications

To integrate the LiDAR subsystem with custom applications:

```cpp
// Include necessary headers
#include "sensors/lidar/lidar_manager.h"

// Create a LiDAR manager
auto lidar_manager = std::make_shared<sensors::lidar::LidarManager>(config);

// Initialize with ROS node
lidar_manager->initialize(node);

// Start data processing
lidar_manager->start();

// Subscribe to point cloud topic
auto subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
  "/livox/lidar",
  10,
  [](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Process point cloud data
  }
);
```

### Custom Data Processing

For custom point cloud processing:

```cpp
// Define a processor function
void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
  // Get point cloud dimensions
  int width = msg->width;
  int height = msg->height;
  int point_step = msg->point_step;
  int row_step = msg->row_step;
  
  // Access point cloud data
  const uint8_t* data = msg->data.data();
  
  // Process each point
  for (size_t i = 0; i < width * height; ++i) {
    // Get point offset
    size_t offset = i * point_step;
    
    // Extract x, y, z coordinates
    float x, y, z;
    std::memcpy(&x, &data[offset + 0], sizeof(float));
    std::memcpy(&y, &data[offset + 4], sizeof(float));
    std::memcpy(&z, &data[offset + 8], sizeof(float));
    
    // Process point
    // ...
  }
}
```

## Related Documentation

- [Project README](../../README.md): Overview of the entire system
- [Livox SDK Documentation](https://github.com/Livox-SDK/Livox-SDK/wiki): Official SDK documentation
- [ROS2 Point Cloud Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-a-PCL-Consumer-Node.html): Information about ROS2 point cloud handling