# LiDAR Subsystem Documentation

This document provides comprehensive information about the LiDAR subsystem of the Data Acquisition Digital Twin project. It covers hardware details, software architecture, configuration options, and troubleshooting guidance.

## Supported LiDAR Hardware

The system supports the following LiDAR models:

| LiDAR Model | Connection Type | Status      | Point Cloud Rate | Range         |
|-------------|-----------------|-------------|------------------|---------------|
| Livox Mid-40| Ethernet        | Supported   | Up to 100 Hz     | Up to 260m    |
| Livox HAP   | Ethernet        | Supported   | Up to 20 Hz      | Up to 450m    |

### Hardware Requirements

- **Livox Mid-40/HAP**: Requires Ethernet connection (100Mbps or better)
- **Computer**: Ethernet port configured with appropriate IP settings
- **Power**: Livox HAP requires external power, Mid-40 can be powered via PoE

## LiDAR System Architecture

The LiDAR system follows the modular driver-manager-node architecture used throughout the project:

1. **LidarDriver** (`lidar_driver.h/cpp`):
   - Core interface with LiDAR hardware
   - Handles network communication and data retrieval
   - Manages hardware configuration and status monitoring

2. **LidarManager** (`lidar_manager.h/cpp`):
   - Manages the LiDAR lifecycle
   - Handles ROS2 publishers and subscriptions
   - Processes and filters LiDAR data

3. **LivoxLidarNode** (`livox_lidar_node.h/cpp`):
   - Implements the ROS2 Lifecycle Node interface
   - Handles parameter management and node configuration
   - Manages component lifecycle

4. **LidarConfig** (`lidar_config.h/cpp`):
   - Manages LiDAR configuration parameters
   - Handles parameter validation and defaults
   - Provides consistent parameter access

5. **LivoxConverter** (`livox_converter.h/cpp`):
   - Utility functions for converting between data formats
   - Implements point cloud downsampling and filtering
   - Manages coordinate system transformations

6. **LivoxAdapterNode** (`livox_adapter_node.h/cpp`):
   - Provides an adapter layer to the Livox ROS2 driver
   - Transforms data to match our system's conventions
   - Implements additional filtering and processing

## Integration Approaches

The system offers two approaches for LiDAR integration:

### 1. Direct Integration

The direct integration approach uses our custom driver to communicate directly with the Livox LiDAR. This approach:
- Provides tight integration with our system
- Offers full control over the communication and data flow
- Can be extended to support additional LiDAR models

### 2. Adapter Integration

The adapter integration approach uses the official Livox ROS2 driver with our adapter node. This approach:
- Leverages the official Livox SDK features
- Offers compatibility with Livox firmware updates
- Provides a migration path for existing Livox users

## Running the LiDAR Subsystem

### Basic Operation

To run the LiDAR node with default settings:

```bash
# Source the ROS2 workspace
source /path/to/data-aquisition-digital-twin/install/setup.bash

# Run LiDAR node
ros2 launch data_aquisition lidar_launch.py
```

This will:
1. Start the LiDAR node with default parameters
2. Launch RViz for visualization
3. Use the adapter approach (if enabled)

### Direct Integration Mode

To run with direct integration (no adapter):

```bash
# Run without the adapter
ros2 launch data_aquisition lidar_launch.py use_adapter:=false
```

### Running Without Visualization

To run without launching RViz:

```bash
# Run without RViz
ros2 launch data_aquisition lidar_launch.py use_rviz:=false
```

## LiDAR Topics

The LiDAR node publishes to the following topics:

| Topic                     | Message Type             | Description                |
|---------------------------|--------------------------|----------------------------|
| `/livox_lidar/point_cloud`| sensor_msgs/PointCloud2  | 3D point cloud data        |
| `/livox_lidar/imu`        | sensor_msgs/Imu          | IMU data (if available)    |
| `/livox_lidar/status`     | std_msgs/String          | LiDAR status information   |

## ROS2 Parameters

The LiDAR node accepts the following parameters:

### Network Parameters

| Parameter              | Type   | Default          | Description                         |
|------------------------|--------|------------------|-------------------------------------|
| `lidar.ip_address`     | string | "192.168.1.100"  | LiDAR IP address                    |
| `lidar.host_ip_address`| string | "192.168.1.5"    | Host computer IP address            |
| `lidar.use_dhcp`       | bool   | false            | Use DHCP instead of static IP       |
| `lidar.data_port`      | int    | 57000            | UDP port for point cloud data       |
| `lidar.command_port`   | int    | 56000            | TCP port for commands               |
| `lidar.imu_port`       | int    | 58000            | UDP port for IMU data               |

### LiDAR Configuration

| Parameter                | Type   | Default           | Description                      |
|--------------------------|--------|-------------------|----------------------------------|
| `lidar.frame_id`         | string | "livox_frame"     | TF frame ID for LiDAR messages   |
| `lidar.broadcast_code`   | string | "livox0000000001" | Broadcast code for device ID     |
| `lidar.publish_frequency`| double | 10.0              | Publishing frequency in Hz       |
| `lidar.point_cloud_type` | int    | 0                 | Point type (0=XYZ+I, 1=XYZ+RGB)  |

### QoS Settings

| Parameter                | Type   | Default       | Description                       |
|--------------------------|--------|---------------|-----------------------------------|
| `lidar.qos_reliability`  | bool   | true          | Use reliable QoS (vs. best effort)|
| `lidar.qos_history_depth`| int    | 5             | QoS history depth for publishers  |

## Network Configuration

### Static IP Setup (Default)

For static IP configuration:

1. **Configure the host computer**:

```bash
# Set static IP address for your Ethernet interface
sudo ip addr add 192.168.1.5/24 dev eth0
sudo ip link set eth0 up
```

2. **Run with static IP parameters**:

```bash
ros2 run data_aquisition livox_lidar_node --ros-args \
  -p lidar.ip_address:=192.168.1.100 \
  -p lidar.host_ip_address:=192.168.1.5 \
  -p lidar.use_dhcp:=false
```

### DHCP Setup

For DHCP configuration:

1. **Configure the host computer**:

```bash
# Ensure DHCP is enabled on your network interface
sudo dhclient eth0
```

2. **Run with DHCP parameters**:

```bash
ros2 run data_aquisition livox_lidar_node --ros-args -p lidar.use_dhcp:=true
```

## Point Cloud Filtering and Processing

The system provides several options for filtering and processing point clouds:

### Downsampling

Reduce point cloud density for better performance:

```bash
# Run with downsampling (half resolution)
ros2 run data_aquisition livox_adapter_node --ros-args -p downsample_factor:=2
```

### Distance Filtering

Filter points based on distance:

```bash
# Filter points between 0.5m and 50m
ros2 run data_aquisition livox_adapter_node --ros-args \
  -p filter_points:=true \
  -p min_distance:=0.5 \
  -p max_distance:=50.0
```

## Visualization

To visualize the LiDAR point cloud:

```bash
# Using the launch file with RViz enabled
ros2 launch data_aquisition lidar_launch.py use_rviz:=true

# Alternative: Run RViz separately
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix data_aquisition)/share/data_aquisition/config/pointcloud_viewer.rviz
```

## JSON Configuration

The LiDAR driver creates a JSON configuration file based on the ROS2 parameters. The default format is:

```json
{
  "lidar_summary_info": {
    "lidar_type": 8
  },
  "HAP": {
    "lidar_net_info": {
      "cmd_data_port": 56000,
      "push_msg_port": 0,
      "point_data_port": 57000,
      "imu_data_port": 58000,
      "log_data_port": 59000
    },
    "host_net_info": {
      "cmd_data_ip": "192.168.1.5",
      "cmd_data_port": 56000,
      "push_msg_ip": "",
      "push_msg_port": 0,
      "point_data_ip": "192.168.1.5",
      "point_data_port": 57000,
      "imu_data_ip": "192.168.1.5",
      "imu_data_port": 58000,
      "log_data_ip": "",
      "log_data_port": 59000
    }
  },
  "lidar_configs": [
    {
      "ip": "192.168.1.100",
      "pcl_data_type": 0,
      "pattern_mode": 0,
      "extrinsic_parameter": {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}
```

## Common Issues and Solutions

### Network Issues

| Issue | Possible Causes | Solutions |
|-------|----------------|-----------|
| Cannot connect to LiDAR | IP configuration mismatch | Verify IP settings, check network interface is up |
| "Device not found" error | Network or power issue | Check physical connections, verify LiDAR power |
| Intermittent connection | Network congestion or cable issues | Use dedicated network, check cable quality |

### Point Cloud Issues

| Issue | Possible Causes | Solutions |
|-------|----------------|-----------|
| No point cloud in RViz | Frame ID issues, RViz configuration | Check frame ID matches RViz fixed frame |
| Sparse point cloud | Filtering too aggressive | Adjust min/max distance, reduce downsample factor |
| High CPU usage | Point cloud rate too high | Reduce publish frequency, increase downsample factor |
| Incorrect point colors | Point cloud format issue | Verify point_cloud_type parameter matches hardware |

## Testing Your Setup

To verify your LiDAR setup:

```bash
# Run the minimal test script
./scripts/minimal_lidar_test.sh

# Run the comprehensive test
./scripts/test_livox_lidar.sh -v
```

These scripts will:
1. Verify network connectivity to the LiDAR
2. Check if the LiDAR is responding to commands
3. Validate point cloud data is being received
4. Test IMU data (if applicable)

## Integration with Other Sensors

The LiDAR subsystem can be integrated with other sensors:

1. **Time Synchronization**: All sensor timestamps use the system clock for alignment
2. **Spatial Registration**: Define transformations between LiDAR and other sensors
3. **Combined Launch**: Use `all_sensors_launch.py` to run cameras, GNSS, and LiDAR together

## References

- [Livox SDK Documentation](https://github.com/Livox-SDK/Livox-SDK)
- [Livox ROS2 Driver](https://github.com/Livox-SDK/livox_ros_driver2)
- [ROS2 PointCloud2 Message](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html)
- [ROS2 REP 103 - Standard Units](https://www.ros.org/reps/rep-0103.html)