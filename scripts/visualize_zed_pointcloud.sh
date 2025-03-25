#\!/bin/bash

# Kill existing processes
echo "Cleaning up any existing processes..."
killall -q rviz2
killall -q zed_camera_node

# Source ROS2
source /opt/ros/humble/setup.bash
source $(pwd)/install/setup.bash

# Create a static transform publisher to make map->camera_link transform
echo "Starting static transform publisher..."
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id map --child-frame-id camera_link &
TF_PUB_PID=$\!

# Start the camera node in the background
echo "Starting ZED camera node..."
ros2 run data_aquisition zed_camera_node \
  --ros-args \
  -r __ns:=/ZED_CAMERA_X0 \
  -r __node:=ZED_CAMERA_X0 \
  -p camera.model:=ZED_X \
  -p depth.enabled:=true \
  -p point_cloud.enabled:=true &
CAMERA_PID=$\!

# Wait for a moment to let the camera initialize
echo "Waiting for camera to initialize (10 seconds)..."
sleep 10

# Start RViz with our custom config
echo "Starting RViz with point cloud configuration..."
ros2 run rviz2 rviz2 -d $(pwd)/config/pointcloud_viewer.rviz

# When RViz closes, clean up
echo "Cleaning up..."
kill $TF_PUB_PID
kill $CAMERA_PID
echo "Done."
