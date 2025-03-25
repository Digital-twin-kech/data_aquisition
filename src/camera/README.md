# Camera Module Documentation

This directory contains the implementation of the camera subsystem for the data acquisition platform. The current implementation focuses on ZED stereo cameras, specifically the ZED X models connected via GMSL.

## Code Structure

The camera module is structured as follows:

### Core Components

1. **Camera Configuration (`camera_config.cpp`, `camera_config.h`)**: 
   - Handles camera parameters like resolution, FPS, and other settings
   - Manages parameter parsing and validation from ROS parameters

2. **Camera Manager (`camera_manager.cpp`, `camera_manager.h`)**:
   - Manages camera lifecycle and communication with ROS
   - Handles publishing of images, point clouds, and camera status
   - Provides interfaces for camera control

3. **ZED Camera Driver (`zed_camera_driver.cpp`, `zed_camera_driver.h`)**:
   - Core implementation of the ZED SDK integration
   - Handles camera connection, configuration, and image acquisition
   - Converts between ZED SDK and ROS formats

4. **ZED Camera Node (`zed_camera_node.cpp`, `zed_camera_node.h`)**:
   - ROS2 Lifecycle Node implementation
   - Handles node configuration, state transitions, and parameter management
   - Entry point for camera integration with ROS2

5. **Main Entry Point (`zed_camera_main.cpp`)**:
   - ROS2 node main function
   - Initializes the ZED camera node and ROS environment

### Additional Components

1. **ZED 2i Driver (`zed_2i_driver.cpp`)**:
   - Specialized driver for ZED 2i USB cameras (currently not used in the system)

2. **Unified Camera Controller (`unified_camera_controller.cpp`)**:
   - Framework for controlling multiple cameras (work in progress)

## Camera Hardware

The system is designed to work with the following ZED cameras:

1. **ZED X0 (GMSL-0)**:
   - Serial Number: 40894785
   - Connection: GMSL
   - Status: Working with HD1080 resolution at 15 FPS

2. **ZED X1 (GMSL-1)**:
   - Serial Number: 41050786
   - Connection: GMSL
   - Status: Working with HD1080 resolution at 15 FPS

3. **ZED 2i (USB-0)**:
   - Serial Number: 37503998
   - Connection: USB
   - Status: Not available in the current setup

## Configuration

Camera parameters are defined in ROS2 parameters and include:

- **Serial Number**: Unique identifier for each camera
- **Resolution**: Image resolution (HD1080, HD720, VGA)
- **Frame Rate**: Min/Max FPS values
- **QoS Settings**: Reliability and history depth for ROS2 publishers
- **Model**: Camera model (ZED_X for GMSL cameras, ZED2i for USB cameras)

Example parameter configuration:
```yaml
camera:
  serial_number: 40894785
  model: "ZED_X"
  resolution: "HD1080"
  min_fps: 15.0
  max_fps: 15.0
  reliable_qos: true
  qos_history_depth: 5
```

## Usage

### Individual Camera Launch

To run a single ZED X camera:

```bash
ros2 run data_aquisition zed_camera_node --ros-args -p camera.serial_number:=40894785 -p camera.model:=ZED_X -p camera.resolution:=HD1080
```

For the second camera (after stopping the first one):

```bash
ros2 run data_aquisition zed_camera_node --ros-args -p camera.serial_number:=41050786 -p camera.model:=ZED_X -p camera.resolution:=HD1080
```

### Multi-Camera Launch Script

The system includes a specialized script for launching multiple cameras in a single terminal with separate processes. The script is located at:

```bash
scripts/camera/launch_all_cameras.sh
```

This script:
1. Launches each camera in its own process
2. Displays color-coded logs for each camera
3. Provides a unified control interface for stopping all cameras

**Note:** Due to limitations in the ZED SDK, cameras must run in separate processes to avoid conflicts.

## Implementation Details

### ZED SDK Integration

The ZED cameras are integrated through the official ZED SDK (version 4.2.5). Key features:

- **Camera Detection**: Automatic detection of available ZED cameras
- **Image Acquisition**: RGB, depth, and point cloud data acquisition
- **IMU Support**: Access to IMU data from supported cameras
- **Camera Control**: FPS control, resolution settings, etc.

### Image Processing Pipeline

1. Frame acquisition via ZED SDK
2. Conversion to ROS2 message formats
3. Integration with ROS2 publishers with configurable QoS
4. Data validation to ensure non-zero image content

### Error Handling

The implementation includes robust error handling mechanisms:

- Connection retry with backoff
- Data validation to detect all-zero images
- Detailed error reporting and diagnostics
- SDK version compatibility checks

## Troubleshooting

For detailed troubleshooting information, refer to the comprehensive [`ZED_CAMERA_GUIDE.md`](../../ZED_CAMERA_GUIDE.md) document.

## Future Improvements

1. Enabling true simultaneous multi-camera operation
2. Performance optimization for high frame rates
3. Enhanced error recovery mechanisms
4. Advanced camera calibration and synchronization
5. Integration with the data fusion pipeline