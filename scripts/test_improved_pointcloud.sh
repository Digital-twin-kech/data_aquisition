#\!/bin/bash
#
# Test script for improved point cloud functionality
#

# Define color codes
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RESET='\033[0m'

echo -e "${BLUE}===== ZED Camera Point Cloud Test Script =====${RESET}"
echo -e "${BLUE}This script tests the improved point cloud generation${RESET}"

# Source ROS setup
source /opt/ros/humble/setup.bash
source $(pwd)/install/setup.bash

# Kill any existing camera nodes
echo -e "${YELLOW}Cleaning up any existing camera processes...${RESET}"
killall -q zed_camera_node

# Print current ZED SDK version
echo -e "${YELLOW}ZED SDK Information:${RESET}"
if [ -f "/usr/local/zed/include/sl/Camera.hpp" ]; then
  SDK_VERSION=$(grep -a "ZED_SDK_MAJOR_VERSION" /usr/local/zed/include/sl/Camera.hpp | head -1 | awk '{print $3}')
  SDK_MINOR=$(grep -a "ZED_SDK_MINOR_VERSION" /usr/local/zed/include/sl/Camera.hpp | head -1 | awk '{print $3}')
  SDK_PATCH=$(grep -a "ZED_SDK_PATCH_VERSION" /usr/local/zed/include/sl/Camera.hpp | head -1 | awk '{print $3}')
  echo -e "${GREEN}ZED SDK Version: $SDK_VERSION.$SDK_MINOR.$SDK_PATCH${RESET}"
else
  echo -e "${RED}ZED SDK not found or cannot read version${RESET}"
fi

# Start the ZED camera node with explicit parameters
echo -e "${YELLOW}Starting ZED camera with improved point cloud settings...${RESET}"
ros2 run data_aquisition zed_camera_node \
  --ros-args \
  -r __ns:=/ZED_IMPROVED \
  -r __node:=ZED_IMPROVED \
  -p camera.model:=ZED_X \
  -p depth.enabled:=true \
  -p point_cloud.enabled:=true &

NODE_PID=$\!

# Wait for node to initialize
echo -e "${YELLOW}Waiting for camera initialization (15 seconds)...${RESET}"
sleep 15

# Check if node is still running
if ps -p $NODE_PID > /dev/null; then
  echo -e "${GREEN}Camera node is running (PID: $NODE_PID)${RESET}"
else
  echo -e "${RED}Camera node failed to start or crashed${RESET}"
  exit 1
fi

# Check for point cloud topic
echo -e "${YELLOW}Checking for point cloud topic...${RESET}"
if ros2 topic list | grep -q "/ZED_IMPROVED/point_cloud/cloud_registered"; then
  echo -e "${GREEN}Point cloud topic found\!${RESET}"
  
  # Get info about the point cloud topic
  echo -e "${YELLOW}Point cloud topic information:${RESET}"
  ros2 topic info /ZED_IMPROVED/point_cloud/cloud_registered
  
  # Print fields
  echo -e "${YELLOW}Point cloud message fields:${RESET}"
  ros2 topic echo /ZED_IMPROVED/point_cloud/cloud_registered --field fields --no-arr --once
  
  # Echo message once to check for data
  echo -e "${YELLOW}Checking point cloud message size...${RESET}"
  ros2 topic echo /ZED_IMPROVED/point_cloud/cloud_registered --field width --field height --field row_step --once
  
  # Check if we have subscribers to the point cloud (like RViz)
  SUB_COUNT=$(ros2 topic info /ZED_IMPROVED/point_cloud/cloud_registered | grep "Subscription count" | awk '{print $3}')
  if [ "$SUB_COUNT" -gt "0" ]; then
    echo -e "${GREEN}Point cloud has $SUB_COUNT subscribers (e.g., RViz)${RESET}"
  else
    echo -e "${YELLOW}Point cloud has no subscribers. Try running RViz2 to visualize:${RESET}"
    echo -e "ros2 run rviz2 rviz2 -d $(pwd)/config/camera_view.rviz"
  fi
else
  echo -e "${RED}Point cloud topic NOT found\!${RESET}"
fi

# Clean up
echo -e "${YELLOW}Cleaning up...${RESET}"
kill $NODE_PID
echo -e "${GREEN}Test complete.${RESET}"
