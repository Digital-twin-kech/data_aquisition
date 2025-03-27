# Sensor Topic Naming Convention

This document outlines the standardized topic naming convention for all sensors in the Data Acquisition Digital Twin system.

## Topic Naming Structure

All sensor topics follow this pattern:

```
/sensors/<sensor_type>/<device_id>/<data_type>[/<sub_type>]
```

Where:
- `sensors`: Root namespace for all sensor data
- `sensor_type`: Type of sensor (camera, lidar, gnss)
- `device_id`: Specific device identifier
- `data_type`: Type of data being published
- `sub_type`: (Optional) Further classification of data type

## Camera Topics

### RGB Cameras

```
/sensors/camera/<camera_id>/rgb/image_rect_color     # Rectified color image
/sensors/camera/<camera_id>/rgb/camera_info          # Camera calibration and metadata
/sensors/camera/<camera_id>/rgb/raw                  # Raw unrectified image
```

### Depth Data

```
/sensors/camera/<camera_id>/depth/image              # Depth image
/sensors/camera/<camera_id>/depth/camera_info        # Depth camera calibration
```

### Point Cloud

```
/sensors/camera/<camera_id>/point_cloud/cloud        # Raw point cloud
/sensors/camera/<camera_id>/point_cloud/cloud_registered  # Point cloud in world frame
```

### IMU Data

```
/sensors/camera/<camera_id>/imu                      # IMU data
```

## LiDAR Topics

```
/sensors/lidar/<lidar_id>/point_cloud                # Main point cloud data
/sensors/lidar/<lidar_id>/imu                        # IMU data if available
/sensors/lidar/<lidar_id>/diagnostics                # Diagnostic information
```

## GNSS Topics

```
/sensors/gnss/<gnss_id>/fix                          # GNSS fix data
/sensors/gnss/<gnss_id>/fix_velocity                 # Velocity data
/sensors/gnss/<gnss_id>/imu                          # IMU data if available
/sensors/gnss/<gnss_id>/rtk                          # RTK correction data
/sensors/gnss/<gnss_id>/status                       # Status information
```

## Synchronized Topics

Synchronized topics follow this pattern:

```
/synchronized/<sensor_type>/<device_id>/<data_type>[/<sub_type>]
```

For example:
```
/synchronized/camera/ZED_CAMERA_2i/rgb/image_rect_color
/synchronized/lidar/livox/point_cloud
/synchronized/gnss/main/fix
```

## Topic QoS Settings

| Data Type | Reliability | Durability | History |
|-----------|-------------|------------|---------|
| Images    | Best Effort | Volatile   | Keep Last (5) |
| Point Clouds | Best Effort | Volatile | Keep Last (5) |
| IMU       | Best Effort | Volatile   | Keep Last (10) |
| GNSS      | Reliable    | Volatile   | Keep Last (5) |
| Status    | Reliable    | Transient Local | Keep Last (1) |