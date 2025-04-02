# ROS2 Bag Recording for Synchronized Sensor Data

This module provides automated recording of synchronized sensor data into ROS2 bags for later playback and analysis.

## Overview

The ROS2 Bag Recorder is designed to capture time-synchronized data from multiple sensors (cameras, LiDAR, GNSS) processed by the Sensor Synchronization module. The recorder runs as a lifecycle node with configurable parameters, automatically managing disk space and handling recording sessions. It works closely with the sensor synchronization module to ensure proper temporal alignment of all recorded data.

## Why Use ROS2 Bags?

ROS2 bags provide several key advantages for sensor data collection:

1. **Reproducible Testing**: Record real-world sensor data once and replay it multiple times for consistent algorithm development
2. **Offline Processing**: Capture high-bandwidth sensor data for later analysis without requiring physical sensors
3. **Data Sharing**: Package sensor data in a standard format that can be shared across development teams
4. **Temporal Alignment**: Maintain precise timestamps and synchronization between different sensor modalities
5. **Regression Testing**: Use consistent datasets for validation and comparison of different algorithm versions
6. **Extended Analysis**: Apply different processing techniques to the same raw data without recapturing

## Topics Recorded

The recorder can capture both synchronized topics and raw sensor topics:

### Synchronized Topics (default)

- `/synchronized/<camera_name>/rgb/image_rect_color` - RGB images from each camera
- `/synchronized/<camera_name>/point_cloud/cloud_registered` - Point clouds from each camera
- `/synchronized/<camera_name>/IMU` - IMU data from each camera
- `/synchronized/livox/lidar` - Synchronized LiDAR point cloud
- `/synchronized/gnss/fix` - Synchronized GNSS position data
- `/sync/metrics` - Synchronization quality metrics

### Raw Sensor Topics (optional)

- `/<camera_name>/rgb/image_rect_color` - Raw RGB images
- `/<camera_name>/point_cloud/cloud_registered` - Raw point clouds
- `/<camera_name>/IMU` - Raw IMU data
- `/livox/lidar` - Raw LiDAR data
- `/gnss/fix` - Raw GNSS data

### Transform Topics (always recorded)

- `/tf` - Transform data
- `/tf_static` - Static transform data

## Usage

### Launch with Default Settings

```bash
# Run the recorder node with default settings
ros2 launch data_aquisition rosbag_recorder_launch.py

# Run both synchronization and recorder
ros2 launch data_aquisition sync_with_recorder_launch.py

# Run with the convenience script (recommended)
./scripts/recording/run_synchronized_recording.sh
```

### Common Command-Line Options

```bash
# Specify a custom output directory
./scripts/recording/run_synchronized_recording.sh -o /data/recordings

# Use exact time synchronization with tight tolerance (high precision)
./scripts/recording/run_synchronized_recording.sh -e -t 0.05

# Disable automatic recording start (manual control via lifecycle commands)
./scripts/recording/run_synchronized_recording.sh -n

# Disable pass-through (only record perfectly synchronized data)
./scripts/recording/run_synchronized_recording.sh -p
```

### Manual Control via ROS2 Lifecycle Commands

```bash
# Configure the node
ros2 lifecycle set /rosbag_recorder configure

# Activate recording (start)
ros2 lifecycle set /rosbag_recorder activate

# Deactivate recording (stop)
ros2 lifecycle set /rosbag_recorder deactivate
```

## Configuration

The recorder is configured via ROS2 parameters in `config/recording/rosbag_recorder_params.yaml`.

Key parameters include:

| Parameter | Description | Default |
|-----------|-------------|---------|
| `recorder.output_directory` | Directory to store ROS2 bags | `/home/user/rosbags` |
| `recorder.bag_prefix` | Prefix for bag directory names | `sync_data` |
| `recorder.record_synchronized` | Record synchronized topics | `true` |
| `recorder.record_raw` | Record raw sensor topics | `false` |
| `recorder.camera_names` | List of cameras to record | `["ZED_CAMERA_2i", "ZED_CAMERA_X0", "ZED_CAMERA_X1"]` |
| `recorder.compression_mode` | Compression mode (`none`, `file`, `message`) | `file` |
| `recorder.split_size` | Size in MB to split bags (0 = no splitting) | `0` |
| `recorder.duration` | Max recording duration in seconds (0 = unlimited) | `0` |
| `recorder.max_bag_files` | Max number of bag files to keep (0 = unlimited) | `5` |

## Best Practices

### Recording High-Quality Data

1. **Verify Synchronization First**: Before starting a long recording session, verify that all sensors are properly synchronized by checking the `/sync/metrics` topic.
   ```bash
   ros2 topic echo /sync/metrics
   ```

2. **Check Disk Space**: Ensure sufficient disk space for your recording session. Point cloud data and images consume significant space.
   ```bash
   # 1 minute of data can use 500MB-2GB depending on sensors enabled
   df -h /path/to/output/directory
   ```

3. **Set Storage Constraints**: For long-term recording, configure `split_size` and `max_bag_files` to manage storage.
   ```bash
   # Edit configuration or use a custom parameter file
   nano config/recording/custom_recorder_params.yaml
   
   # Launch with custom parameters
   ros2 launch data_aquisition rosbag_recorder_launch.py params_file:=/path/to/custom_params.yaml
   ```

4. **Record Raw and Synchronized Data**: For critical data collection, consider recording both raw and synchronized data.
   ```bash
   # Edit config file to set record_raw: true
   # Or override via CLI
   ros2 launch data_aquisition rosbag_recorder_launch.py \
     recorder.record_raw:=true recorder.record_synchronized:=true
   ```

### Working with Recorded Data

1. **Listing Recorded Bags**:
   ```bash
   ls -la /home/user/rosbags
   ```

2. **Inspecting Bag Contents**:
   ```bash
   ros2 bag info /home/user/rosbags/sync_data_2025-03-28-14-30-00
   ```

3. **Replaying Recorded Data**:
   ```bash
   # Replay at normal speed
   ros2 bag play /home/user/rosbags/sync_data_2025-03-28-14-30-00
   
   # Replay at 0.5x speed
   ros2 bag play --rate 0.5 /home/user/rosbags/sync_data_2025-03-28-14-30-00
   ```

4. **Extracting Specific Topics**:
   ```bash
   # Create a new bag with only selected topics
   ros2 bag filter /path/to/input_bag /path/to/output_bag \
     --topics /synchronized/livox/lidar /synchronized/gnss/fix
   ```

5. **Converting to Other Formats**:
   ```bash
   # Export to CSV (for suitable topic types)
   ros2 topic echo --csv /synchronized/gnss/fix > gnss_data.csv
   
   # Convert images to files
   ros2 run image_transport republish raw --ros-args \
     -r in:=/synchronized/ZED_CAMERA_2i/rgb/image_rect_color \
     -r out:=/image_save
   ```

## Monitoring and Diagnostics

The recorder node publishes diagnostics information that can be monitored:

```bash
# Check recorder diagnostics
ros2 topic echo /diagnostics | grep rosbag_recorder -A 10

# Monitor disk space usage during recording
watch -n 5 'df -h /home/user/rosbags'
```

## Troubleshooting

1. **No Data Being Recorded**:
   - Check that the synchronization node is running and publishing data
   - Verify that the recorder node is in the ACTIVE state
   - Check permissions on the output directory

2. **Recorder Node Fails to Activate**:
   - Check disk space availability
   - Verify output directory permissions
   - Check the ROS2 logs for errors

3. **Missing Topics in Recording**:
   - Verify that the topics are being published
   - Check that the camera names in the configuration match the actual camera names
   - Verify that the synchronization node is running correctly

4. **High CPU Usage During Recording**:
   - Reduce the number of recorded topics
   - Try using a different compression mode
   - Consider using a separate physical drive for recording

5. **Playback Problems**:
   - Check that the correct ROS2 version is being used for playback
   - Ensure required message type definitions are available
   - Try replaying at a slower rate (e.g., `--rate 0.5`)