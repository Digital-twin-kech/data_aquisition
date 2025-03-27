# ZED Camera System Documentation

This document provides comprehensive information about the ZED camera subsystem of the Data Acquisition Digital Twin project. It covers hardware details, software architecture, configuration options, and troubleshooting.

## Supported Camera Hardware

The system supports the following ZED camera models:

| Camera Model | Connection Type | Serial Number | Status           | Resolution      | FPS      |
|--------------|-----------------|---------------|------------------|-----------------|----------|
| ZED X0       | GMSL            | 40894785      | Supported        | Up to HD1080    | Up to 30 |
| ZED X1       | GMSL            | 41050786      | Supported        | Up to HD1080    | Up to 30 |
| ZED 2i       | USB 3.0         | 37503998      | Supported        | Up to HD1080    | Up to 30 |

### Hardware Requirements

- **ZED X (GMSL)**: Requires a compatible GMSL interface (e.g., NVIDIA Xavier AGX with GMSL adapter)
- **ZED 2i (USB)**: Requires USB 3.0/3.1 port for full functionality
- **All Models**: Require ZED SDK version 4.0 or newer (tested with 4.2.5)

## Software Architecture

The ZED camera system follows a modular driver-manager-node architecture:

1. **ZedCameraDriver** (`zed_camera_driver.h/cpp`):
   - Core interface with ZED SDK
   - Handles camera initialization, configuration, and data acquisition
   - Converts ZED data formats to ROS2 messages

2. **CameraManager** (`camera_manager.h/cpp`):
   - Manages the camera lifecycle
   - Handles ROS2 publishers and subscription
   - Controls camera parameters based on external input

3. **ZedCameraNode** (`zed_camera_node.h/cpp`):
   - Implements the ROS2 Lifecycle Node interface
   - Handles parameter management and node configuration
   - Manages component activation/deactivation sequences

4. **CameraConfig** (`camera_config.h/cpp`):
   - Manages camera configuration parameters
   - Handles parameter validation and defaults
   - Provides consistent parameter access

## Running ZED Cameras

### Single Camera

To run a single ZED camera:

```bash
# Source the ROS2 workspace
source /path/to/data-aquisition-digital-twin/install/setup.bash

# Run ZED X0 camera
ros2 run data_aquisition zed_camera_node --ros-args \
  -r __ns:=/ZED_CAMERA_X0 \
  -r __node:=ZED_CAMERA_X0 \
  -p camera.serial_number:=40894785 \
  -p camera.model:=ZED_X \
  -p camera.resolution:=HD1080 \
  -p camera.min_fps:=15.0 \
  -p camera.max_fps:=15.0
```

### Multiple Cameras

For multiple cameras, use the provided script or launch file:

#### Using the Script (Recommended for Development)

```bash
# Make script executable
chmod +x scripts/camera/launch_all_cameras.sh

# Run the script
./scripts/camera/launch_all_cameras.sh
```

This script:
- Launches each camera in a separate process
- Uses color-coded output for easier monitoring
- Automatically detects available cameras
- Handles proper cleanup on exit

#### Using the Launch File (Recommended for Production)

```bash
# Using the launch file
ros2 launch data_aquisition multi_camera_launch.py
```

This launch file:
- Creates separate ROS2 nodes with proper namespacing
- Sets appropriate parameters for each camera
- Integrates properly with ROS2 launch system

## Camera Topics

Each camera publishes to its own namespace. For a camera with namespace `/ZED_CAMERA_X0`, the topics are:

| Topic                                      | Message Type                | Description                |
|--------------------------------------------|----------------------------|----------------------------|
| `/ZED_CAMERA_X0/rgb/image_rect_color`      | sensor_msgs/Image          | Rectified RGB image        |
| `/ZED_CAMERA_X0/depth/depth_registered`    | sensor_msgs/Image          | Registered depth image     |
| `/ZED_CAMERA_X0/point_cloud/cloud_registered` | sensor_msgs/PointCloud2 | 3D point cloud             |
| `/ZED_CAMERA_X0/imu/data`                  | sensor_msgs/Imu            | IMU measurements           |
| `/ZED_CAMERA_X0/status`                    | std_msgs/String            | Camera status information  |
| `/ZED_CAMERA_X0/IMU`                       | sensor_msgs/Imu            | Alternative IMU topic      |

## ROS2 Parameters

The ZED camera nodes accept the following parameters:

### Basic Parameters

| Parameter                 | Type   | Default          | Description                             |
|---------------------------|--------|------------------|-----------------------------------------|
| `camera.model`            | string | "ZED_X"          | Camera model (ZED_X or ZED2i)           |
| `camera.serial_number`    | int    | 0                | Camera serial number (0 = first available) |
| `camera.resolution`       | string | "HD1080"         | Image resolution (HD1080, HD720, VGA)   |
| `camera.min_fps`          | float  | 15.0             | Minimum frame rate                      |
| `camera.max_fps`          | float  | 15.0             | Maximum frame rate                      |

### Advanced Parameters

| Parameter                 | Type   | Default          | Description                             |
|---------------------------|--------|------------------|-----------------------------------------|
| `camera.reliable_qos`     | bool   | true             | Use reliable QoS (vs. best effort)      |
| `camera.qos_history_depth`| int    | 5                | QoS history depth for publishers        |
| `camera.pid_p`            | double | 0.8              | PID P gain for FPS control              |
| `camera.pid_i`            | double | 0.2              | PID I gain for FPS control              |
| `camera.pid_d`            | double | 0.05             | PID D gain for FPS control              |
| `depth.enabled`           | bool   | true             | Enable depth measurement                |
| `point_cloud.enabled`     | bool   | true             | Enable point cloud generation           |

## Modifying Camera Code

### Changing RGB Image Resolution

To change the image resolution, modify the initialization parameters:

```cpp
// In zed_camera_driver.cpp in the connect() method
init_params.camera_resolution = sl::RESOLUTION::HD720;  // Change to HD720, HD1080, etc.
```

Alternatively, pass the parameter at runtime:

```bash
ros2 run data_aquisition zed_camera_node --ros-args -p camera.resolution:=HD720
```

### Changing Point Cloud Generation

The point cloud generation can be customized:

```cpp
// In zed_camera_driver.cpp in getPointCloud() method
// Modify downsample_factor to change point cloud resolution
int downsample_factor = 4;  // 4 = quarter resolution, 2 = half resolution
```

To disable point clouds entirely:

```bash
ros2 run data_aquisition zed_camera_node --ros-args -p point_cloud.enabled:=false
```

### Changing Depth Mode

The depth mode affects quality and computational requirements:

```cpp
// In zed_camera_driver.cpp in connect() method
init_params.depth_mode = sl::DEPTH_MODE::ULTRA;  // Options: NONE, PERFORMANCE, QUALITY, ULTRA
```

## Point Cloud Visualization

The system includes a specialized script for visualizing point clouds:

```bash
# Run the setup script (once)
./scripts/setup_rviz_for_pointcloud.sh

# Then run the visualization script
./scripts/visualize_zed_pointcloud.sh
```

This will:
1. Start a camera node with optimal point cloud settings
2. Set up proper TF transforms for visualization
3. Launch RViz2 with a preconfigured view for point clouds

## Common Issues and Solutions

### General Issues

| Issue | Possible Causes | Solutions |
|-------|----------------|-----------|
| Camera not detected | USB/GMSL connection issue, permissions | Check connections, verify with `lsusb`, check ZED Explorer |
| "CAMERA NOT DETECTED" error | SDK conflict, multiple access | Run cameras in separate processes, check if another application is using the camera |
| All-zero images | Initialization issue | Verify camera is working with ZED Explorer, check image validation logs |

### Multiple Camera Issues

| Issue | Possible Causes | Solutions |
|-------|----------------|-----------|
| "CAMERA NOT DETECTED" with multiple cameras | ZED SDK limitation | Use separate processes with launch_all_cameras.sh or multi_camera_launch.py |
| Namespace conflicts | Improper ROS2 configuration | Use proper node naming and namespacing with `-r __ns:=/CAMERA_NAME` |
| Performance impact | Resource limitations | Reduce resolution/FPS, disable depth on some cameras |

### Point Cloud Issues

| Issue | Possible Causes | Solutions |
|-------|----------------|-----------|
| No point cloud in RViz | Frame ID issues, RViz configuration | Use visualize_zed_pointcloud.sh, check frame_id setting |
| "0 points" in RViz | Filtering too aggressive | Modify point cloud filtering in zed_camera_driver.cpp |
| Point cloud too dense/slow | Resolution too high | Increase downsample_factor in zed_camera_driver.cpp |
| All-zero point cloud | Depth processing disabled | Enable depth with `-p depth.enabled:=true` |

## Creating Custom Camera Configurations

You can create custom YAML configuration files for different camera setups:

1. Create a config file in `config/camera/custom_config.yaml`:
   ```yaml
   /**:
     ros__parameters:
       camera:
         model: 'ZED_X'
         serial_number: 40894785
         resolution: 'HD720'
         min_fps: 15.0
         max_fps: 30.0
         reliable_qos: true
         qos_history_depth: 5
       depth:
         enabled: true
       point_cloud:
         enabled: true
   ```

2. Run with this configuration:
   ```bash
   ros2 run data_aquisition zed_camera_node --ros-args --params-file $(pwd)/config/camera/custom_config.yaml
   ```

## Performance Optimization

For optimal performance:

1. **Reduce Resolution**: Use HD720 instead of HD1080 for higher frame rates
2. **Reduce Point Cloud Density**: Increase the downsampling factor (4 or 8)
3. **Use Best Effort QoS**: Set `reliable_qos:=false` for lower latency
4. **Disable Depth When Not Needed**: Set `depth.enabled:=false` if not required
5. **Optimize FPS**: Set min_fps and max_fps to appropriate values based on use case

## ZED Camera Code Explanation

### How the Point Cloud Generation Works

The point cloud generation process:
1. Retrieves XYZRGBA data from ZED SDK using `retrieveMeasure()`
2. Downsamples to reduce data size and processing requirements
3. Filters invalid or distant points to improve visualization
4. Packs data into ROS2 PointCloud2 format with proper field definitions
5. Publishes the data with appropriate frame_id and timestamps

```cpp
// Key steps in getPointCloud():
sl::Mat zed_cloud;
zed_->retrieveMeasure(zed_cloud, sl::MEASURE::XYZRGBA, sl::MEM::CPU);

// Downsample and filter
int downsample_factor = 4;
cloud.height = zed_cloud.getHeight() / downsample_factor;
cloud.width = zed_cloud.getWidth() / downsample_factor;

// Define fields for x, y, z, rgb
sensor_msgs::msg::PointField field_x, field_y, field_z, field_rgb;
// ... field setup ...

// Filter and copy valid points only
for (unsigned int y = 0; y < cloud.height; y++) {
  for (unsigned int x = 0; x < cloud.width; x++) {
    // ... filtering logic ...
    if (valid_point) {
      // ... copy point data ...
    }
  }
}
```

### How Camera Namespacing Works

The namespacing system:
1. Uses ROS2 remapping to set node name and namespace
2. Retrieves the namespace in the node's code with `node_->get_namespace()`
3. Uses the namespace as a prefix for all topic publications
4. Sets frame_ids to match the namespace for proper TF integration

```cpp
// In the launch script:
ros2 run data_aquisition zed_camera_node \
  --ros-args \
  -r __ns:=/ZED_CAMERA_X0 \
  -r __node:=ZED_CAMERA_X0

// In the camera_manager.cpp code:
std::string ns = node_->get_namespace();
rgb_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
  ns + "/rgb/image_rect_color", camera_qos);
```

## Integration with Other Sensors

The ZED camera system can be integrated with other sensors:

1. **Time Synchronization**: ZED cameras use the system clock for timestamps, which aligns with GNSS time
2. **Spatial Calibration**: For precise integration, calibrate the transformation between cameras and other sensors
3. **Launch Integration**: Use the `all_sensors_launch.py` file to launch cameras alongside GNSS and other sensors

## Advanced Features

### Recording to Disk

The ZED SDK supports recording to SVO files:

```bash
# Enable recording with parameter
ros2 run data_aquisition zed_camera_node --ros-args -p camera.record_svo:=true -p camera.svo_filename:="/path/to/recording.svo"
```

### Camera Calibration

The ZED cameras come factory-calibrated, but fine-tuning is possible:

```bash
# Get current calibration parameters
ros2 topic echo /ZED_CAMERA_X0/camera_info
```

## References

- [ZED SDK API Documentation](https://www.stereolabs.com/docs/api/index.html)
- [ZED ROS2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper)
- [ROS2 Camera Interfaces](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)