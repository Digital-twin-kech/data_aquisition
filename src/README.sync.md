# Sensor Synchronization Node

This document provides detailed technical documentation for the Sensor Synchronization subsystem in the Data Acquisition Digital Twin project.

## 1. Overview

The Sensor Synchronization node aligns data temporally from multiple sensors (cameras, LiDAR, GNSS) to facilitate sensor fusion and ensure consistent timestamps across all data streams. It uses ROS2's message_filters library to implement approximate time synchronization.

## 2. Architecture

### 2.1 Core Components

The synchronization system has the following components:

- `SensorSynchronizer` class: The main ROS2 node that handles synchronization
- Message filter subscribers for each sensor topic
- ApproximateTime synchronizers for groups of related topics
- Publishers for synchronized output topics

### 2.2 Class Structure

The `SensorSynchronizer` class contains:

```cpp
class SensorSynchronizer : public rclcpp::Node {
public:
  explicit SensorSynchronizer(const rclcpp::NodeOptions & options);
  virtual ~SensorSynchronizer();

private:
  // Parameters
  std::vector<std::string> camera_names_;
  bool sync_lidar_;
  bool sync_gnss_;
  
  // Publishers
  std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> sync_camera_rgb_pubs_;
  std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> sync_camera_pc_pubs_;
  std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> sync_camera_imu_pubs_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sync_lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr sync_gnss_pub_;
  
  // Message filter subscribers
  std::unordered_map<std::string, message_filters::Subscriber<sensor_msgs::msg::Image>> camera_rgb_subs_;
  std::unordered_map<std::string, message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> camera_pc_subs_;
  std::unordered_map<std::string, message_filters::Subscriber<sensor_msgs::msg::Imu>> camera_imu_subs_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> lidar_sub_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::NavSatFix>> gnss_sub_;
  
  // Synchronizers
  using CameraSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, 
    sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Imu>;
  std::unordered_map<std::string, std::shared_ptr<message_filters::Synchronizer<CameraSyncPolicy>>> camera_syncs_;
  
  using LidarGnssSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, sensor_msgs::msg::NavSatFix>;
  std::shared_ptr<message_filters::Synchronizer<LidarGnssSyncPolicy>> lidar_gnss_sync_;
  
  // Methods
  void initializeSubscribers();
  void initializePublishers();
  void initializeSynchronizers();
  void loadParameters();
  
  // Callback methods
  void cameraSyncCallback(...);
  void lidarGnssSyncCallback(...);
};
```

## 3. Synchronization Logic

### 3.1 Time-Based Synchronization

The node uses `message_filters::sync_policies::ApproximateTime` to synchronize messages. This policy:

- Groups messages from multiple topics based on their header timestamps
- Allows for slight variations in timestamps (configurable "slop")
- Invokes a callback when a complete set of messages with matching timestamps is available

### 3.2 Implementation Details

1. **Topic Subscription**: The node subscribes to each sensor topic using `message_filters::Subscriber` instead of regular ROS2 subscriptions.

2. **Synchronizer Setup**: For each camera, a separate synchronizer is created for RGB image + point cloud + IMU data. Another synchronizer is created for LiDAR + GNSS data.

3. **Queue Management**: Each synchronizer maintains a queue of messages (default size: 10) to find matching timestamp combinations.

4. **Callback Registration**: When synchronized messages are available, the corresponding callback is invoked with the entire set of messages.

5. **Republishing**: The synchronized data is republished on new topics with the `/synchronized/` prefix, preserving the original message content but ensuring consistent timestamps.

### 3.3 Synchronization Policies

The system currently uses two sync policies:

#### Camera Data Synchronization

```cpp
using CameraSyncPolicy = message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::Image, 
  sensor_msgs::msg::PointCloud2, 
  sensor_msgs::msg::Imu
>;
```

This policy synchronizes three messages types per camera.

#### LiDAR-GNSS Synchronization

```cpp
using LidarGnssSyncPolicy = message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::PointCloud2, 
  sensor_msgs::msg::NavSatFix
>;
```

This policy synchronizes LiDAR point clouds with GNSS fix data.

## 4. Configuration

### 4.1 ROS2 Parameters

The synchronization node supports these parameters:

| Parameter        | Type                | Default                                      | Description                            |
|------------------|---------------------|----------------------------------------------|----------------------------------------|
| `camera_names`   | string[]            | ["ZED_CAMERA_2i", "ZED_CAMERA_X0", "ZED_CAMERA_X1"] | Camera namespaces to synchronize        |
| `sync_lidar`     | bool                | true                                         | Whether to synchronize LiDAR data      |
| `sync_gnss`      | bool                | true                                         | Whether to synchronize GNSS data       |
| `sync_policy`    | string              | "ApproximateTime"                           | Synchronization policy                 |
| `time_tolerance` | double              | 0.02                                         | Timestamp tolerance in seconds         |
| `cache_size`     | int                 | 100                                          | Cache size for synchronization         |
| `max_delay`      | double              | 0.5                                          | Maximum allowable delay in seconds     |

### 4.2 Modifying Synchronization Policy

To change the synchronization policy:

1. **Change the Policy Type**:
   
   ```cpp
   // For exact time synchronization (timestamps must match exactly)
   using CameraSyncPolicy = message_filters::sync_policies::ExactTime<...>;
   
   // For approximate time synchronization (default)
   using CameraSyncPolicy = message_filters::sync_policies::ApproximateTime<...>;
   ```

2. **Adjust Time Tolerance** (for ApproximateTime only):

   The slop/tolerance can be configured using the `setMaxIntervalDuration` method:
   
   ```cpp
   // Set the maximum allowed timestamp difference to 50ms
   auto sync = std::make_shared<message_filters::Synchronizer<CameraSyncPolicy>>(CameraSyncPolicy(10));
   sync->setMaxIntervalDuration(rclcpp::Duration(0, 50000000)); // 50ms in nanoseconds
   ```

3. **Configure Queue Size**:
   
   The queue size is set when creating the synchronizer:
   
   ```cpp
   // Increase queue size to 20 (default is 10)
   auto sync = std::make_shared<message_filters::Synchronizer<CameraSyncPolicy>>(CameraSyncPolicy(20));
   ```

### 4.3 Adding New Sensor Types

To add a new sensor type:

1. Add message filter subscribers for the new sensor
2. Create a new synchronization policy that includes the message type
3. Set up the synchronizer and register a callback
4. Create publishers for the synchronized output

Example for adding a new radar sensor:

```cpp
// 1. Add subscriber members
std::unique_ptr<message_filters::Subscriber<radar_msgs::msg::RadarScan>> radar_sub_;

// 2. Define a new policy
using RadarGnssSyncPolicy = message_filters::sync_policies::ApproximateTime<
  radar_msgs::msg::RadarScan, sensor_msgs::msg::NavSatFix>;
std::shared_ptr<message_filters::Synchronizer<RadarGnssSyncPolicy>> radar_gnss_sync_;

// 3. Initialize the subscriber
radar_sub_ = std::make_unique<message_filters::Subscriber<radar_msgs::msg::RadarScan>>();
radar_sub_->subscribe(this, "/sensors/radar/main/scan", sensor_qos);

// 4. Create synchronizer
radar_gnss_sync_ = std::make_shared<message_filters::Synchronizer<RadarGnssSyncPolicy>>(
  RadarGnssSyncPolicy(10), *radar_sub_, *gnss_sub_);

// 5. Register callback
radar_gnss_sync_->registerCallback(
  std::bind(&SensorSynchronizer::radarGnssSyncCallback, this, _1, _2));
```

## 5. Running the Node

### 5.1 Standalone Execution

```bash
# Source workspace
source /path/to/install/setup.bash

# Run with default parameters
ros2 run data_aquisition sync_node

# Run with custom parameters
ros2 run data_aquisition sync_node --ros-args \
  -p camera_names:='["ZED_CAMERA_2i"]' \
  -p time_tolerance:=0.05 \
  -p cache_size:=20
```

### 5.2 Launch File

```bash
# Run with default configuration
ros2 launch data_aquisition sync_launch.py

# Run with custom parameters
ros2 launch data_aquisition sync_launch.py time_tolerance:=0.05 cache_size:=20
```

### 5.3 Combined with Sensors

Using the integrated script:

```bash
# Run all sensors + synchronization
./scripts/run_synchronized_sensors.sh

# Run with specific options
./scripts/run_synchronized_sensors.sh --no-lidar
```

## 6. Monitoring and Debugging

### 6.1 Viewing Log Messages

```bash
# Run with debug logging
ros2 run data_aquisition sync_node --ros-args --log-level debug

# Monitor sync node logs
ros2 topic echo /rosout | grep "sensor_synchronizer"
```

### 6.2 Checking Synchronized Topics

```bash
# List all synchronized topics
ros2 topic list | grep /synchronized

# Check message rate
ros2 topic hz /synchronized/camera/ZED_CAMERA_2i/rgb/image_rect_color

# View synchronized GNSS data
ros2 topic echo /synchronized/gnss/main/fix
```

### 6.3 Visualizing in RViz

Create an RViz configuration that subscribes to synchronized topics:

```bash
# Launch RViz with synchronized sensor configuration
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix data_aquisition)/share/data_aquisition/config/synchronized_view.rviz
```

## 7. Common Issues and Solutions

### 7.1 No Synchronized Output

- **Issue**: Subscriptions are established but no callbacks are triggered
- **Solution**: 
  - Check that all source topics are publishing
  - Verify timestamp synchronization between sensors
  - Increase the `time_tolerance` parameter

### 7.2 High Latency

- **Issue**: Synchronized data has high latency
- **Solution**:
  - Decrease `cache_size` and `time_tolerance`
  - Check CPU load - synchronization is computationally intensive
  - Consider optimizing sensor rates

### 7.3 Missing Messages

- **Issue**: Some synchronized messages are missing
- **Solution**:
  - Increase `cache_size` to buffer more messages
  - Increase `time_tolerance` if sensors have poor clock synchronization
  - Check if messages are being dropped due to QoS settings

## 8. Performance Considerations

- **Thread Safety**: Message filter callbacks may be called from different threads. Use proper synchronization if accessing shared resources.
- **Buffer Size**: The `cache_size` parameter affects memory usage. Larger values improve synchronization but increase memory consumption.
- **Computation Cost**: Time synchronization has O(n²) complexity in the worst case, where n is the number of messages in the queue.

## 9. Future Improvements

- Add synchronization metrics (jitter, latency)
- Support for dynamic reconfiguration of synchronization parameters
- Integration with transform system for spatially aligned data
- Advanced filtering options (outlier rejection, interpolation)