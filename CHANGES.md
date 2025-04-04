# Changes to Fix Point Cloud Visualization

## Issue Analysis
The point cloud visualization issues were caused by multiple factors:

1. **Missing TF Transforms**: Visualization tools like RViz require TF transforms to properly display point clouds in the right coordinate frames.

2. **QoS Mismatch**: The `queue full` errors occurred because RViz was using Reliable QoS while the high-throughput point cloud data works better with Best Effort QoS.

3. **Time Synchronization Tolerance**: The default time tolerance was too small (20ms) which can cause issues with high-throughput sensors.

4. **Frame ID Configuration**: The RViz configuration was using a fixed frame that didn't match the sensor frames.

5. **Color Encoding Mismatch**: Camera images showed a blue tint due to RGB/BGR channel mismatch, and LiDAR point clouds had incorrect colors due to field format issues.

## Fixes Implemented

1. **Enhanced run_sensors_node.sh Script**:
   - Increased synchronization time tolerance from 20ms to 100ms
   - Explicitly enabled pass-through mode for direct data forwarding
   - Added TF transform broadcasters for all sensor frames

2. **New Visualization Scripts**:
   - Created specialized script for ZED camera point cloud visualization
   - Created specialized script for Livox LiDAR point cloud visualization
   - Both scripts use Best Effort QoS to avoid queue full errors

3. **Optimized RViz Configuration**:
   - Customized RViz settings for different sensors
   - Used the correct Frame IDs for visualization
   - Added proper TF tree configuration

4. **Created Combined Script**:
   - `run_sensors_with_transformers.sh` runs the sensors and visualizes point clouds

5. **Fixed Color Visualization Issues**:
   - Camera:
     - Changed image quality from QUALITY to ULTRA for better color reproduction
     - Improved point cloud resolution from PERFORMANCE to QUALITY
     - Added color enhancement settings (brightness, contrast, saturation)
     - Implemented proper RGB to BGR channel swapping for RViz compatibility

   - LiDAR:
     - Changed point cloud type from intensity-based to RGB-based
     - Updated field definitions to use a single "rgb" field with UINT32 datatype
     - Implemented proper RGB packing for PCL/RViz compatibility

## Usage Instructions

### Running All Sensors with Visualizer
```bash
./scripts/run_sensors_with_transformers.sh
```

### Visualizing Specific Camera Point Cloud
```bash
# Visualize synchronized ZED_CAMERA_X0 point cloud
./scripts/visualize_zed_pointcloud.sh --camera ZED_CAMERA_X0 --topic-type synchronized

# Visualize raw ZED_CAMERA_X1 point cloud 
./scripts/visualize_zed_pointcloud.sh --camera ZED_CAMERA_X1 --topic-type raw
```

### Visualizing LiDAR Point Cloud
```bash
# Visualize synchronized LiDAR point cloud
./scripts/visualize_livox_pointcloud.sh --topic-type synchronized

# Visualize raw LiDAR point cloud
./scripts/visualize_livox_pointcloud.sh --topic-type raw
```

### Visualizing Fixed Camera Point Clouds
```bash
# Run the specialized script to fix and visualize camera point clouds
./scripts/fix_camera_pointclouds.sh
```

## Known Limitations

- Point clouds with very high point counts may still cause performance issues in RViz
- For best performance, use `--point-size 1` to reduce rendering load
- Some TF warnings may still appear in RViz if timestamps don't align perfectly