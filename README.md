# Data Acquisition Digital Twin

A comprehensive data acquisition system for sensor integration, designed for autonomous vehicle and robotics applications. This package provides a flexible, modular framework for integrating multiple sensors including cameras, GNSS receivers, and LiDAR systems.

## Features

- **Multi-Sensor Integration**: Seamless integration of ZED cameras, GNSS receivers, and more
- **Multi-Camera Support**: Run multiple ZED cameras (ZED 2i, ZED X) simultaneously
- **Point Cloud Processing**: Optimized point cloud generation and visualization
- **RTK-Enhanced GNSS**: High-precision positioning with RTK corrections
- **Flexible Configuration**: Easily configurable through YAML files and launch parameters
- **Visualization Tools**: Pre-configured RViz profiles for sensor data visualization
- **ROS2 Compliance**: Fully compatible with ROS2 standards and best practices

## System Requirements

- **Operating System**: Ubuntu 22.04 or newer
- **ROS2**: Humble or newer
- **ZED SDK**: 4.0 or newer (tested with 4.2.5)
- **CUDA**: 11.0 or newer (for ZED SDK)
- **Hardware**: Compatible with ZED cameras (USB and GMSL), u-blox F9P GNSS receivers

## Project Structure

```
data-aquisition-digital-twin/
├── data_aquisition/             # Main ROS2 package
│   ├── include/                 # Header files
│   │   └── sensors/             # Sensor-specific headers
│   │       ├── camera/          # ZED camera interface
│   │       ├── gnss/            # GNSS interface
│   │       └── lidar/           # LiDAR interface
│   ├── src/                     # Source code
│   │   ├── camera/              # ZED camera implementation
│   │   ├── gnss/                # GNSS implementation
│   │   └── lidar/               # LiDAR implementation
│   ├── launch/                  # Launch files
│   │   ├── all_sensors_launch.py      # Launch all sensors
│   │   ├── cameras_launch.py          # Launch cameras only
│   │   ├── gnss_launch.py             # Launch GNSS only
│   │   └── multi_camera_launch.py     # Launch multiple cameras
│   ├── scripts/                 # Utility scripts
│   │   ├── camera/              # Camera-specific scripts
│   │   ├── gnss/                # GNSS-specific scripts
│   │   └── visualize_zed_pointcloud.sh # Point cloud visualization
│   ├── config/                  # Configuration files
│   │   ├── camera/              # Camera configuration
│   │   └── gnss/                # GNSS configuration
│   └── CMakeLists.txt           # Build configuration
└── CMakeLists.txt               # Main build configuration
```

## Building the Project

### Prerequisites

1. Install ROS2 Humble or newer: [ROS2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)
2. Install ZED SDK: [ZED SDK Installation](https://www.stereolabs.com/docs/installation/)
3. Install required dependencies:

```bash
sudo apt update
sudo apt install -y build-essential cmake libusb-1.0-0-dev
```

### Build Steps

```bash
# Navigate to the project root directory
cd /path/to/data-aquisition-digital-twin

# Build the project
colcon build --symlink-install

# Source the setup files
source install/setup.bash
```

## Running the System

### Running All Sensors

To launch the complete system with all sensors:

```bash
ros2 launch data_aquisition all_sensors_launch.py
```

This will start:
- All available ZED cameras
- GNSS receiver
- Data processing nodes

### Running Specific Sensors

#### Cameras Only

For just the camera subsystem:

```bash
# Launch with default parameters
ros2 launch data_aquisition cameras_launch.py

# Launch with specific parameters
ros2 launch data_aquisition cameras_launch.py camera_model:=ZED_X
```

#### Multiple Cameras

For running multiple cameras with proper namespace separation:

```bash
# Using the multi-camera launch system (recommended)
ros2 launch data_aquisition multi_camera_launch.py

# Using the dedicated script (alternative)
./scripts/camera/launch_all_cameras.sh
```

#### GNSS Only

For just the GNSS subsystem:

```bash
# Launch with default parameters
ros2 launch data_aquisition gnss_launch.py

# Launch with RTK enabled
ros2 launch data_aquisition gnss_launch.py use_rtk:=true
```

### Running Individual Nodes

#### ZED Camera Node

```bash
# Run ZED X0 (GMSL-0) camera
ros2 run data_aquisition zed_camera_node --ros-args -r __ns:=/ZED_CAMERA_X0 -r __node:=ZED_CAMERA_X0 -p camera.serial_number:=40894785 -p camera.model:=ZED_X

# Run ZED X1 (GMSL-1) camera
ros2 run data_aquisition zed_camera_node --ros-args -r __ns:=/ZED_CAMERA_X1 -r __node:=ZED_CAMERA_X1 -p camera.serial_number:=41050786 -p camera.model:=ZED_X

# Run ZED 2i (USB) camera
ros2 run data_aquisition zed_camera_node --ros-args -r __ns:=/ZED_CAMERA_2i -r __node:=ZED_CAMERA_2i -p camera.serial_number:=37503998 -p camera.model:=ZED2i
```

#### GNSS Node

```bash
# Run GNSS node
ros2 run data_aquisition gnss_node

# Run with RTK enabled
ros2 run data_aquisition gnss_node --ros-args -p gnss.use_rtcm_corrections:=true
```

## Visualization

### Point Cloud Visualization

The system includes a dedicated script for visualizing point clouds in RViz2:

```bash
# Setup RViz configuration for point cloud viewing
./scripts/setup_rviz_for_pointcloud.sh

# Launch visualization with a running camera
./scripts/visualize_zed_pointcloud.sh
```

### Camera Image Visualization

To view camera RGB and depth images:

```bash
# Using RViz
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix data_aquisition)/share/data_aquisition/config/camera_view.rviz

# Using image_view (alternative)
ros2 run image_view image_view --ros-args -r image:=/ZED_CAMERA_X0/rgb/image_rect_color
```

### GNSS Data Visualization

For visualizing GNSS data:

```bash
# View fix data
ros2 topic echo /gnss/fix

# Using RViz (if TF is published)
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix data_aquisition)/share/data_aquisition/config/gnss_view.rviz
```

## Common Issues and Troubleshooting

### Camera Issues

- **No cameras detected**: Ensure ZED SDK is properly installed and cameras are connected
- **Permission issues**: Check USB device permissions (try `sudo chmod 666 /dev/video*`)
- **Multiple camera conflicts**: Use separate processes as described in documentation

### GNSS Issues

- **No GNSS connection**: Check serial port (`/dev/ttyACM0` by default) and permissions
- **RTK not working**: Verify NTRIP server settings and internet connectivity
- **Poor position accuracy**: Ensure antenna has clear sky view and RTK corrections are received

## Additional Documentation

For more detailed information, refer to these specialized guides:

- [Camera System Documentation](README.camera.md): Detailed guide for camera configuration and usage
- [GNSS System Documentation](README.gnss.md): Guide for GNSS configuration and RTK setup

## License

This software is licensed under the MIT License. See the LICENSE file for details.