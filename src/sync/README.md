# Sensor Synchronization Module

This module provides synchronized access to data from multiple sensor types, ensuring temporal alignment for sensor fusion and digital twin applications.

## Overview

The Sensor Synchronization module coordinates data from:
- ZED Cameras (images, point clouds, IMU)
- Livox LiDAR
- GNSS/RTK positioning

It uses ROS2 message filters with configurable sync policies (exact time or approximate time) to create temporally aligned data streams.

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

- `sync_policy` (string) - Synchronization policy to use ("ExactTime", "ApproximateTime")
- `time_tolerance` (double) - Time tolerance for synchronization (seconds)
- `cache_size` (int) - Cache size for synchronization
- `max_delay` (double) - Maximum allowable delay for synchronization (seconds)
- `camera_names` (string[]) - List of camera names to synchronize
- `sync_lidar` (bool) - Whether to synchronize LiDAR data
- `sync_gnss` (bool) - Whether to synchronize GNSS data

## Running the Node

### From Launch File:
```bash
ros2 launch data_aquisition sync_launch.py
```

### Running with Custom Parameters:
```bash
ros2 launch data_aquisition sync_launch.py sync_policy:=ExactTime time_tolerance:=0.01
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

1. **High Latency**: If the max_delay parameter is too low, messages might be dropped. Monitor the `/sync/metrics` topic and adjust as needed.

2. **Missing Data**: Check that all source topics are publishing with the expected frequency.

3. **Poor Synchronization**: If timestamps between different sensors vary significantly, consider adjusting the time_tolerance parameter, or implementing a custom time synchronization strategy.