# Static Calibration System

This directory contains tools for performing static calibration between the Livox LiDAR and ZED 2i camera using ChArUco board detection.

## Overview

The static calibration system estimates the transformation matrix `T_LiDAR_Camera2i` that allows converting points from the LiDAR coordinate system to the camera coordinate system. This calibration is essential for sensor fusion applications and proper registration of data from multiple sensors.

## Prerequisites

- Python 3.8 or newer
- NumPy
- OpenCV (with ArUco module)
- Open3D (for point cloud processing)

Install dependencies with:
```bash
pip install numpy opencv-python open3d
```

## Data Collection

For a proper calibration session with the ChArUco board:

1. Create a ChArUco board using the included generator tool:
   ```bash
   python3 generate_charuco_board.py --output charuco_board.png
   ```

2. Print the board on a rigid material (preferably non-reflective)

3. Set up the calibration:
   - Position the ChArUco board so it's visible to both the LiDAR and camera
   - Place it approximately 1-3 meters from the sensors
   - Ensure good lighting conditions for camera detection
   - Hold the board steady or mount it on a stable surface

4. Record data:
   ```bash
   ./scripts/run_sensors_node.sh
   ```
   
5. Wait for 10-15 seconds to ensure enough frames are captured

## Running Calibration

After collecting data, run the calibration script:
```bash
./run_calibration.sh
```

## Calibration Process

The calibration consists of the following steps:

1. **Locate Latest Session**: Finds the most recent recording session in the records directory
2. **Detect ChArUco Board**: Identifies the ChArUco board in the camera image and estimates its pose
   - The script uses both newer and older OpenCV ArUco APIs for compatibility
   - Falls back to simplified detection if no markers are found
3. **Extract Point Clouds**: Extracts the ChArUco board region from both camera and LiDAR point clouds
   - Uses adaptive region selection based on point cloud characteristics
   - Handles different coordinate systems and orientations
4. **Point Cloud Registration**: Aligns the camera and LiDAR point clouds using ICP
   - Uses multiple parameter sets to find the best alignment
   - Includes robust error handling for real-world data
5. **Transform Calculation**: Computes the final transformation matrix between LiDAR and camera

## Output Files

The calibration generates the following output files in the `results` directory:

- `tf_statique.npy`: 4x4 transformation matrix from LiDAR to camera
- `tf_statique_inverse.npy`: 4x4 transformation matrix from camera to LiDAR
- `zed2i_intrinsics.npz`: Camera intrinsic parameters
- `calibration_timestamp.txt`: Timestamp and information about the calibration run

## Debug Information

The calibration process generates debug information and visualizations in the `debug` directory:

- `charuco_detection.png`: Debug image showing the detected ChArUco board
- `source_cloud.ply`: The source point cloud used for registration
- `target_cloud.ply`: The target point cloud used for registration
- `aligned_cloud.ply`: The aligned point cloud after registration

You can visualize these point clouds with tools like MeshLab or Open3D's visualizer.

## Error Handling

The calibration script includes robust error handling to deal with real-world data:

- Fallbacks for different OpenCV versions of ArUco detection
- Adaptive point cloud processing based on available data
- Graceful degradation when data is suboptimal
- Detailed diagnostic information for troubleshooting

## Usage

To use the calibration results in your application:

```python
import numpy as np

# Load the transformation matrix
T_LiDAR_Camera2i = np.load("results/tf_statique.npy")

# Transform a point from LiDAR to camera coordinates
point_lidar = np.array([x, y, z, 1])  # Homogeneous coordinates
point_camera = T_LiDAR_Camera2i @ point_lidar

# For the inverse transformation (camera to LiDAR)
T_Camera2i_LiDAR = np.load("results/tf_statique_inverse.npy")
point_camera = np.array([x, y, z, 1])  # Homogeneous coordinates
point_lidar = T_Camera2i_LiDAR @ point_camera

# To transform a point cloud
def transform_point_cloud(points, transform):
    """
    Transform a set of 3D points using a 4x4 transformation matrix
    
    Args:
        points: Nx3 array of points
        transform: 4x4 transformation matrix
    
    Returns:
        Nx3 array of transformed points
    """
    # Convert to homogeneous coordinates
    homogeneous_points = np.hstack((points, np.ones((points.shape[0], 1))))
    
    # Apply transformation
    transformed_points = (transform @ homogeneous_points.T).T
    
    # Convert back to 3D points
    return transformed_points[:, :3]
```

## Generate ChArUco Board

To generate a ChArUco board for calibration:

```bash
python3 generate_charuco_board.py --output charuco_board.png --squares_x 5 --squares_y 7 --square_length 30 --marker_length 23 --dpi 300
```

Parameters:
- `--output`: Output filename (PNG format)
- `--squares_x`: Number of squares in X direction (default: 5)
- `--squares_y`: Number of squares in Y direction (default: 7)
- `--square_length`: Length of each square in mm (default: 30)
- `--marker_length`: Length of ArUco markers in mm (default: 23)
- `--dpi`: Print resolution in dots per inch (default: 300)

## Troubleshooting

If the calibration doesn't produce good results:

1. Check the debug images to ensure proper ChArUco board detection
2. Verify that both sensors can see the ChArUco board clearly
3. Try running the calibration with newer data where the board is held steady
4. Examine the point cloud visualizations to ensure proper alignment
5. Adjust the position of the ChArUco board if needed (1-3 meters from sensors)

## References

- [OpenCV ArUco detection](https://docs.opencv.org/4.x/d9/d6a/group__aruco.html)
- [Open3D point cloud registration](http://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html)
- [ChArUco board detection](https://docs.opencv.org/4.x/df/d4a/tutorial_charuco_detection.html)
- [ICP Registration](http://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html)