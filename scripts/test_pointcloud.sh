#\!/bin/bash

echo "Testing ZED Camera Point Cloud publishing"
source /opt/ros/humble/setup.bash
source $(pwd)/install/setup.bash

# Start a node in the background
ros2 run data_aquisition zed_camera_node --ros-args -r __ns:=/ZED_CAMERA_TEST -r __node:=ZED_CAMERA_TEST -p camera.model:=ZED_X -p depth.enabled:=true -p point_cloud.enabled:=true &
NODE_PID=$\!

# Give it a moment to start
sleep 5

echo "Checking if point cloud topic is being published..."
ros2 topic list | grep -i "point_cloud"

echo "Checking point cloud message type..."
ros2 topic info /ZED_CAMERA_TEST/point_cloud/cloud_registered

echo "Checking point cloud message fields..."
ros2 topic echo /ZED_CAMERA_TEST/point_cloud/cloud_registered --field fields --no-arr --once

# Clean up
kill $NODE_PID
echo "Test complete."
