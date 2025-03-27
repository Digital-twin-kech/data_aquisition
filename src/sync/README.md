# Sensor Synchronization Module

This module provides synchronized access to data from multiple sensor types, ensuring temporal alignment for sensor fusion and digital twin applications.

## Overview

The Sensor Synchronization module coordinates data from:
- ZED Cameras (images, point clouds, IMU)
- Livox LiDAR
- GNSS/RTK positioning

It uses ROS2 message filters with configurable sync policies (exact time or approximate time) to create temporally aligned data streams.

### QoS Settings
- Input subscribers: Best-effort reliability with history depth matching the cache_size parameter
- Output publishers: Reliable delivery with volatile durability and history depth matching the cache_size parameter

This ensures the module can receive high-rate sensor data without requiring reliable delivery, while providing guaranteed message delivery to downstream nodes.

## Topics

### Input Topics:
- `/ZED_CAMERA_2i/rgb/image_rect_color`
- `/ZED_CAMERA_2i/point_cloud/cloud_registered`
- `/ZED_CAMERA_2i/IMU`
- `/ZED_CAMERA_X0/rgb/image_rect_color`
- `/ZED_CAMERA_X0/point_cloud/cloud_registered`
- `/ZED_CAMERA_X0/IMU`
- `/ZED_CAMERA_X1/rgb/image_rect_color`
- `/ZED_CAMERA_X1/point_cloud/cloud_registered`
- `/ZED_CAMERA_X1/IMU`
- `/livox/lidar`
- `/gnss/fix`

### Output Topics:
- `/synchronized/<camera_name>/rgb/image_rect_color` - Synchronized camera data
- `/synchronized/<camera_name>/point_cloud/cloud_registered` - Synchronized camera point cloud 
- `/synchronized/<camera_name>/IMU` - Synchronized camera IMU data
- `/synchronized/livox/lidar` - Synchronized LiDAR data
- `/synchronized/gnss/fix` - Synchronized GNSS data
- `/sync/metrics` - Synchronization quality metrics

## Parameters

- `sync_policy` (string, default: "ApproximateTime") - Synchronization policy to use ("ExactTime", "ApproximateTime")
- `time_tolerance` (double, default: 0.10) - Time tolerance for synchronization (seconds)
- `cache_size` (int, default: 100) - Cache size for synchronization queue
- `max_delay` (double, default: 0.5) - Maximum allowable delay for synchronization (seconds)
- `camera_names` (string[], default: ["ZED_CAMERA_2i", "ZED_CAMERA_X0", "ZED_CAMERA_X1"]) - List of camera names to synchronize
- `sync_lidar` (bool, default: true) - Whether to synchronize LiDAR data
- `sync_gnss` (bool, default: true) - Whether to synchronize GNSS data
- `pass_through` (bool, default: true) - Whether to directly forward sensor messages without waiting for synchronization

## Running the Node

### From Launch File:
```bash
ros2 launch data_aquisition sync_launch.py
```

### Running with Custom Parameters:
```bash
# Use exact time policy with tight tolerance
ros2 launch data_aquisition sync_launch.py sync_policy:=ExactTime time_tolerance:=0.01

# Disable pass-through (only synchronized data will be published)
ros2 launch data_aquisition sync_launch.py pass_through:=false

# Increase time tolerance for sensors with large timestamp differences
ros2 launch data_aquisition sync_launch.py time_tolerance:=0.20
```

## Monitoring Synchronization

The node publishes diagnostics and metrics:

1. ROS2 Diagnostics:
   ```bash
   ros2 topic echo /diagnostics
   ```

2. Sync Metrics:
   ```bash
   ros2 topic echo /sync/metrics
   ```

## Common Issues

1. **Empty Synchronized Topics**: If the synchronized topics are empty but the original sensor topics have data:
   - Make sure `pass_through` parameter is set to `true` (default) to ensure data is always available
   - Check for large timestamp differences between sensor messages and increase `time_tolerance`
   - Verify that all required topics are being published

2. **High Latency**: If the max_delay parameter is too low, messages might be dropped. Monitor the ROS diagnostics and logs. You may need to increase the cache_size parameter if dealing with high-frequency data.

3. **Missing Data**: Check that all source topics are publishing with the expected frequency. Use this command to check topic publishers:
   ```bash
   ros2 topic info /ZED_CAMERA_2i/rgb/image_rect_color
   ```

4. **Poor Synchronization**: If timestamps between different sensors vary significantly, consider:
   - Increasing the time_tolerance parameter (e.g., to 0.20 seconds for better results)
   - Checking for clock synchronization issues between devices
   - Looking for excessive CPU load causing timing delays

5. **QoS Mismatch**: If you're not receiving synchronized messages, verify that your QoS settings match:
   ```bash
   # Check a topic's QoS profile
   ros2 topic info --verbose /livox/lidar
   ```
   The sync node subscribes using SensorDataQoS with best-effort reliability.

6. **Choosing between Pass-through and Synchronized Mode**:
   - Use pass-through (`pass_through:=true`) when you need to ensure all data is always available on synchronized topics
   - Disable pass-through (`pass_through:=false`) when you need to ensure data is perfectly time-synchronized
   - In applications where you need both, process the synchronized data when available but fall back to the most recent message otherwise

7. **Debugging Sync Performance**: Enable debug logging to see timestamp differences between synchronized messages:
   ```bash
   ros2 launch data_aquisition sync_launch.py --log-level sensor_synchronizer:=debug
   ```