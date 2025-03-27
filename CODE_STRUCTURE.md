# Data Acquisition System - Code Structure Guide

## Project Overview

This project implements a comprehensive data acquisition system for autonomous vehicles and robotics applications. It focuses on integrating multiple sensors (cameras, GNSS, LiDAR) into a unified ROS2-based framework.

## Directory Structure

```
data_aquisition/
├── include/                      # Header files
│   └── sensors/                  # Sensor-specific headers
│       ├── camera/              # Camera interface headers
│       ├── gnss/                # GNSS interface headers
│       └── lidar/               # LiDAR interface headers
├── src/                          # Source code
│   └── sensors/                 # Sensor implementations
│       ├── camera/              # Camera implementation
│       │   ├── README.md       # Camera subsystem documentation
│       │   ├── ZED_CAMERA_GUIDE.md # ZED Camera specific guide
│       │   ├── camera_config.cpp # Camera configuration
│       │   ├── camera_manager.cpp # Camera management
│       │   ├── zed_camera_driver.cpp # ZED camera driver
│       │   └── zed_camera_node.cpp # ROS2 node for ZED cameras
│       ├── gnss/                # GNSS implementation
│       └── lidar/               # LiDAR implementation
├── scripts/                      # Utility scripts
│   ├── camera/                  # Camera-specific scripts
│   │   └── launch_all_cameras.sh # Script to launch multiple cameras
│   └── test_zed_cameras.sh      # ZED camera test script
├── launch/                       # ROS2 launch files
│   ├── all_sensors_launch.py    # Launch all sensors
│   ├── cameras_launch.py        # Launch only cameras
│   └── zed_x_cameras_launch.py  # Launch ZED X cameras
└── config/                       # Configuration files
    └── camera/                  # Camera configurations
```

## Code Architecture

### Core Subsystems

1. **Camera Subsystem**
   - **Architecture**: Modular design with separation between driver, configuration, and ROS2 interface
   - **Key Classes**:
     - `ZedCameraDriver`: Handles communication with ZED SDK
     - `CameraConfig`: Manages configuration parameters
     - `CameraManager`: Coordinates camera operations and ROS publishing
     - `ZedCameraNode`: ROS2 lifecycle node implementation

2. **GNSS Subsystem**
   - **Architecture**: Layered design with NMEA parsing and RTK processing
   - **Key Classes**:
     - `GnssDriver`: Interface to GNSS hardware
     - `NmeaParser`: Parses NMEA sentences
     - `RtkProcessor`: Handles RTK corrections
     - `GnssManager`: Coordinates GNSS operations
     - `GnssNode`: ROS2 node implementation

3. **LiDAR Subsystem**
   - **Architecture**: Adapter-based approach for different LiDAR models
   - **Key Classes**:
     - `LidarDriver`: Interface to LiDAR hardware
     - `LivoxConverter`: Converts Livox data to standard formats
     - `LidarManager`: Coordinates LiDAR operations
     - `LivoxLidarNode`: ROS2 node for Livox LiDAR

### Common Patterns

1. **Configuration Management**
   - Each subsystem has a dedicated Config class (`CameraConfig`, `GnssConfig`, etc.)
   - Config classes parse ROS parameters and provide typed access to settings
   - Default values ensure functionality even with minimal configuration

2. **Driver-Manager-Node Pattern**
   - **Driver**: Hardware-specific code, platform-independent, minimal ROS dependencies
   - **Manager**: Coordinates driver, handles complex operations, interfaces with ROS
   - **Node**: ROS2 lifecycle node, parameter handling, topic management

3. **Error Handling**
   - Comprehensive error detection and reporting
   - Graceful degradation when sensors are unavailable
   - Automatic retry with exponential backoff for transient errors

## Camera Module in Detail

The camera module is the most sophisticated part of the system, supporting multiple stereo cameras from Stereolabs.

### Multi-Camera Support

The system supports multiple ZED cameras (ZED 2i, ZED X) with two approaches:

1. **Single Process Approach** (via launch file):
   - Limited by ZED SDK restrictions
   - May lead to "CAMERA NOT DETECTED" errors when running multiple cameras

2. **Multi-Process Approach** (via script):
   - Each camera runs in a separate process
   - Output is consolidated with color-coding in a single terminal
   - Script provides automatic camera detection and clean termination

### Integration with ZED SDK

The system integrates with ZED SDK 4.2.5 and includes:
- Camera detection and initialization
- Image capture with format conversion
- RGB, depth, and point cloud processing
- IMU data acquisition
- Camera health monitoring

### ROS2 Integration

The camera subsystem publishes to the following ROS2 topics:
- `/<camera_name>/rgb/image_rect_color`: RGB images
- `/<camera_name>/depth/depth_registered`: Depth images
- `/<camera_name>/point_cloud/cloud_registered`: 3D point clouds
- `/<camera_name>/imu/data`: IMU data
- `/<camera_name>/status`: Camera status information

## Extending the System

### Adding a New Sensor Type

To add a new sensor type:

1. Create a directory structure:
   ```
   include/sensors/new_sensor/
   src/new_sensor/
   ```

2. Implement the required classes:
   - `NewSensorConfig.h/cpp` for configuration
   - `NewSensorDriver.h/cpp` for hardware interface
   - `NewSensorManager.h/cpp` for operational logic
   - `NewSensorNode.h/cpp` for ROS2 interface

3. Update CMakeLists.txt to build the new components

4. Create launch files for the new sensor

### Modifying Camera Support

To add support for new camera models:

1. Extend `CameraConfig` with new model parameters
2. Create a new driver class (e.g., `NewCameraDriver.h/cpp`)
3. Update `CameraManager` to support the new driver
4. Add the new driver to CMakeLists.txt
5. Update launch files and scripts for the new camera type

## Testing

Each subsystem includes test scripts in the `scripts/` directory:

1. **Camera Testing**:
   - `test_zed_cameras.sh`: Tests basic camera connectivity
   - `launch_all_cameras.sh`: Runs all cameras simultaneously

2. **GNSS Testing** (coming soon):
   - `test_gnss_connection.sh`: Tests GNSS receiver connectivity
   - `test_ntrip_connection.sh`: Tests NTRIP server connection

3. **LiDAR Testing** (coming soon):
   - `test_livox_connection.sh`: Tests Livox LiDAR connectivity

## Build System

The project uses colcon build system for ROS2 packages:

```bash
# Build entire project
colcon build --symlink-install

# Build specific package
colcon build --packages-select data_aquisition

# Clean build
rm -rf build/ install/ log/
colcon build
```

## More Information

For more detailed information about specific subsystems, refer to the README files in each subdirectory:

- `src/sensors/camera/README.md`: Camera subsystem documentation
- `src/sensors/camera/ZED_CAMERA_GUIDE.md`: ZED Camera specific guide
- `src/sensors/lidar/README.md`: LiDAR subsystem documentation