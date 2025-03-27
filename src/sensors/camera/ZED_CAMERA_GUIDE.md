# ZED Camera Integration Guide

This guide documents the current state of ZED camera integration, setup instructions, and troubleshooting tips.

## Available Cameras

The system is configured to work with the following ZED cameras:

1. **ZED X0 (GMSL-0)**:
   - Serial Number: 40894785
   - Connection: GMSL
   - Status: **Working** with HD1080 resolution

2. **ZED X1 (GMSL-1)**:
   - Serial Number: 41050786
   - Connection: GMSL
   - Status: **Working** with HD1080 resolution

3. **ZED 2i (USB-0)**:
   - Serial Number: 37503998
   - Connection: USB
   - Status: **Not Available** in the current setup

## ZED SDK Integration

The ZED SDK (version 4.2.5) has been successfully integrated into the ROS2 package. The following changes were made to ensure proper integration:

1. Added ZED SDK to CMakeLists.txt:
   ```cmake
   # Find CUDA and ZED SDK
   find_package(CUDA REQUIRED)
   set(ZED_DIR "/usr/local/zed/")
   find_package(zed REQUIRED)
   
   # Include directories
   include_directories(
     ${ZED_INCLUDE_DIRS}
     ${CUDA_INCLUDE_DIRS}
   )
   
   # Link libraries
   target_link_libraries(camera_lib
     ${ZED_LIBRARIES}
     ${CUDA_LIBRARIES}
   )
   ```

2. Fixed ZED camera driver initialization with proper resolution and FPS settings:
   - For ZED X cameras, HD1080 resolution at 15 FPS works correctly
   - Using PERFORMANCE depth mode for better compatibility
   - Changed SDK settings for improved compatibility

## Running ZED Cameras

### Single Camera Mode

To run a single ZED X camera with a specific serial number:

```bash
ros2 run data_aquisition zed_camera_node --ros-args -p camera.serial_number:=40894785 -p camera.model:=ZED_X -p camera.resolution:=HD1080
```

For the second camera (after stopping the first one):

```bash
ros2 run data_aquisition zed_camera_node --ros-args -p camera.serial_number:=41050786 -p camera.model:=ZED_X -p camera.resolution:=HD1080
```

### Multi-Camera Script (Recommended)

A specialized script is provided to launch multiple cameras in separate processes with color-coded output:

```bash
./scripts/camera/launch_all_cameras.sh
```

This script:
- Automatically checks if each camera is available
- Launches each camera in a separate process with its own namespace
- Creates three separate nodes, each publishing to its own namespace:
  - `/ZED_CAMERA_2i` for the ZED 2i USB camera
  - `/ZED_CAMERA_X0` for the ZED X GMSL-0 camera
  - `/ZED_CAMERA_X1` for the ZED X GMSL-1 camera
- Provides color-coded output for easier monitoring
- Handles proper cleanup when terminated (with Ctrl+C)

This is the recommended approach for working with multiple ZED cameras, as the ZED SDK has limitations when running multiple cameras in the same process.

### ROS2 Launch File (Preferred for Multiple Nodes)

A better approach using the ROS2 launch system is now available:

```bash
./scripts/launch_multicamera.sh
```

This method:
- Creates properly named ROS2 nodes for each camera
- Sets up distinct namespaces for each camera
- Uses the standard ROS2 launch system
- Ensures cleaner topic organization in tools like RViz

This is the preferred approach for a production environment, as it provides the best integration with ROS2 tools and practices.

### Original Launch File (Not Recommended)

The original launch file is still available but not recommended:

```bash
ros2 launch data_aquisition zed_x_cameras_launch.py
```

> **Note:** Running cameras in the same process leads to "CAMERA NOT DETECTED" errors due to ZED SDK limitations. The multi-camera script or new launch approach is recommended instead.

Parameters available in both the script and launch file:
- Resolution (default: HD1080)
- Frame rate (default: 15 FPS)
- QoS settings (default: Reliable with history=5)

## Published Topics

Each camera publishes to its own namespace. The topics follow this pattern:

- `/<namespace>/rgb/image_rect_color`: RGB images (1920x1080 for HD1080)
- `/<namespace>/depth/depth_registered`: Depth images (if depth enabled)
- `/<namespace>/point_cloud/cloud_registered`: 3D point clouds (if depth enabled)
- `/<namespace>/imu/data`: IMU data
- `/<namespace>/status`: Camera status information

Where `<namespace>` is the unique name for each camera:
- `ZED_CAMERA_2i` - For the ZED 2i USB camera
- `ZED_CAMERA_X0` - For the ZED X GMSL-0 camera 
- `ZED_CAMERA_X1` - For the ZED X GMSL-1 camera

You can view available topics with:
```bash
ros2 topic list | grep ZED_CAMERA
```

## Troubleshooting

### Common Issues and Solutions

1. **INVALID RESOLUTION Error**:
   - Resolution/FPS mismatch for camera model
   - Solution: Use HD1080 resolution at 15 FPS for ZED X cameras

2. **No Image Data (All Zeros)**:
   - Fixed in code by properly initializing the ZED SDK
   - Added pixel validation to detect all-zero images

3. **Camera Not Found**:
   - Check physical connections
   - Use `lsusb | grep -i stereolabs` to verify USB devices
   - Use ZED Explorer tool: `/usr/local/zed/tools/ZED_Explorer`

4. **Depth Not Working**:
   - Try PERFORMANCE depth mode instead of NEURAL or ULTRA
   - Ensure enough light and texture in the scene
   - Disable depth if not needed for better performance

5. **Multiple Camera Issues**:
   - The ZED SDK has limitations when running multiple cameras in the same process
   - When launching multiple cameras in a single process, "CAMERA NOT DETECTED" errors appear
   - Solution:
     - Use the provided `scripts/launch_multicamera.sh` script (preferred) to run each camera as a separate ROS2 node with its own namespace
     - Or use `scripts/camera/launch_all_cameras.sh` script to run each camera in its own process
     - Both scripts handle proper node naming and process management
     - This approach allows multiple cameras to run simultaneously
   - Verifying separate nodes:
     - Run `ros2 node list` to verify that separate camera nodes are running
     - Each camera should appear as its own node (e.g., `/ZED_CAMERA_X0`, `/ZED_CAMERA_X1`)
     - Check topics with `ros2 topic list | grep ZED_CAMERA_X0` for each camera
   - Alternative solutions:
     - Run each camera in a separate terminal window manually
     - Consider using the official ZED ROS2 wrapper from Stereolabs for multi-camera support

### Testing Cameras

A test script is available to verify camera connections:

```bash
./scripts/test_zed_cameras.sh
```

The script performs:
1. SDK installation check
2. Camera connection test with minimal settings
3. Image data validation test

## Next Steps

1. **Multi-Camera Support**:
   - ✓ Implemented multi-camera support using separate processes (with `launch_all_cameras.sh` script)
   - ✓ Added proper ROS2 node naming and namespacing for each camera
   - ✓ Created a ROS2 launch file approach for better integration (with `multi_camera_launch.py`)
   - Next improvements:
     - Add inter-process synchronization for camera timestamps
     - Implement a shared parameter server for controlling all cameras
     - Add visualization tools for checking camera alignment
     - Test with all three cameras simultaneously (when ZED 2i becomes available)

2. **Performance Optimization**:
   - Adjust resolution and FPS as needed for your use case
   - Consider using BestEffort QoS for real-time performance

3. **Integration with Other Sensors**:
   - Launch all sensors with `all_sensors_launch.py`
   - Test with GNSS and LiDAR sensors

4. **Camera Calibration**:
   - Consider running camera calibration if needed
   - ZED SDK handles most calibration automatically

5. **Improve Error Handling and Recovery**:
   - Add automatic reconnection for camera disconnects
   - Implement watchdog timers for camera health monitoring
   - Add diagnostics logging for camera performance

## References

- [ZED SDK Documentation](https://www.stereolabs.com/docs/)
- [ZED ROS2 Documentation](https://www.stereolabs.com/docs/ros2/)