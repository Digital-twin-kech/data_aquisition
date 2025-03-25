# Livox LiDAR Integration

This document provides detailed information about the Livox LiDAR integration in the Data Acquisition Digital Twin project. It covers both the reference implementation and our custom implementation.

## Hardware Information

This project supports the following Livox LiDAR models:

| Model | Type | Interface | Range | Points/s |
|-------|------|-----------|-------|----------|
| Livox HAP | Mid-range | Ethernet | 0.1-450m | 240,000 |

## Network Configuration

The LiDAR uses the following network configuration:

- **LiDAR IP**: 192.168.1.100
- **Host IP**: 192.168.1.50
- **Command Port**: 56000
- **Data Port**: 57000
- **IMU Port**: 58000

## Integration Approaches

We provide two methods for integrating the Livox LiDAR:

1. **Direct Integration**: Uses the reference Livox ROS2 driver
2. **Adapter Integration**: Uses our custom adapter with the reference driver

## Quick Start Guide

### Method 1: Direct Integration (Recommended for Hardware Testing)

The simplest way to test with actual hardware:

```bash
# Run the direct launch script (uses reference implementation)
./scripts/run_livox_direct.sh
```

This script:
- Sources the reference implementation
- Copies our configuration to the reference path
- Launches the reference driver with RViz visualization

### Method 2: Adapter Integration (Recommended for Full System Integration)

For integration with our system:

```bash
# Build the system
cd ..
colcon build --packages-select data_aquisition

# Source the setup
source install/setup.bash

# Run the adapter
./scripts/run_livox_adapter.sh
```

## Configuration Files

The main configuration file is located at `config/lidar/HAP_config.json`. This file defines:

- Network ports and IP addresses
- LiDAR device information
- Point cloud data format

For a custom configuration:

1. Edit the configuration file
2. Update the host IP address to match your network interface
3. Run with the modified configuration

## Troubleshooting

### Common Issues

1. **"Cannot ping LiDAR"**
   - Ensure physical connection (Ethernet cable)
   - Verify LiDAR power
   - Check host computer network settings

2. **"Cannot access command port"**
   - Ensure no firewall is blocking ports
   - Verify correct IP settings
   - Try rebooting the LiDAR

3. **"No point cloud data"**
   - Check configuration file format
   - Verify broadcast code matches your device
   - Check ROS2 topic list for active publishers

### Diagnostic Tools

```bash
# Test connectivity
./scripts/test_livox_connection.sh -v

# View point cloud data
ros2 topic echo /livox/lidar --field-match x --max-msgs 1

# Check active topics
ros2 topic list | grep livox
```

## Reference Implementation

The reference implementation is located at:
`/home/user/Desktop/instll_liv/ws_livox/`

Key components:
- Launch files: `launch_ROS2/rviz_HAP_launch.py`
- Configuration: `config/HAP_config.json`

## Custom Implementation

Our custom implementation provides:

1. A fully integrated LiDAR subsystem following our Driver-Manager-Node pattern
2. An adapter to use the reference implementation with our system
3. Custom configuration and launch files

## Building from Source

```bash
# In the project root
colcon build --packages-select data_aquisition

# Source the setup
source install/setup.bash

# Test the built executable
ros2 run data_aquisition livox_adapter_node --help
```

## ROS2 Topics

The LiDAR publishes to the following topics:

- `/livox/lidar`: Raw point cloud data (sensor_msgs/PointCloud2)
- `/livox/imu`: IMU data (sensor_msgs/Imu)
- `/livox_lidar/point_cloud`: Processed point cloud (with our adapter)
- `/livox_lidar/imu`: Processed IMU data (with our adapter)