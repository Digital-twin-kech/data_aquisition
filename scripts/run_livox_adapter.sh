#!/bin/bash

# Run Livox Adapter with actual hardware
# This script runs the LiDAR subsystem with physical hardware

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}  LIVOX ADAPTER LAUNCHER        ${NC}"
echo -e "${BLUE}================================${NC}"

# Check for reference implementation
echo -e "${YELLOW}[1/3] Checking reference implementation...${NC}"
if [ -d "/home/user/Desktop/instll_liv/ws_livox" ]; then
  echo -e "${GREEN}✓ Reference implementation found${NC}"
  echo -e "Using reference LiDAR driver for hardware connection"
  source /home/user/Desktop/instll_liv/ws_livox/install/setup.bash
  
  # Check if LiDAR driver is properly sourced
  if ros2 pkg list | grep -q "livox_ros_driver2"; then
    echo -e "${GREEN}✓ Livox driver package found${NC}"
  else
    echo -e "${RED}✗ Livox driver package not found${NC}"
    echo -e "Please make sure the reference implementation is built and sourced"
    exit 1
  fi
else
  echo -e "${RED}✗ Reference implementation not found${NC}"
  echo -e "Expected at: /home/user/Desktop/instll_liv/ws_livox"
  echo -e "Will continue with our built-in implementation"
fi

# Check LiDAR connectivity
echo -e "${YELLOW}[2/3] Testing LiDAR connectivity...${NC}"
ping -c 3 192.168.1.100 > /dev/null 2>&1
if [ $? -eq 0 ]; then
  echo -e "${GREEN}✓ LiDAR is reachable via network${NC}"
else
  echo -e "${RED}✗ Cannot reach LiDAR at 192.168.1.100${NC}"
  echo -e "Please check physical connections and power"
  exit 1
fi

# Run LiDAR driver
echo -e "${YELLOW}[3/3] Launching LiDAR driver...${NC}"
echo -e "${BLUE}Starting reference LiDAR driver${NC}"
echo -e "${BLUE}This will launch the Livox ROS2 driver and our adapter${NC}"
echo -e "${BLUE}Press Ctrl+C to exit${NC}"

# Launch the reference driver in the background
echo -e "Launching reference Livox driver..."
ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args -p frame_id:=livox_frame &
REFERENCE_PID=$!

# Wait a moment for the driver to start
sleep 2

# Launch our adapter
echo -e "Launching our Livox adapter..."
ros2 run data_aquisition livox_adapter_node --ros-args \
  -p input_point_cloud_topic:=/livox/lidar \
  -p input_imu_topic:=/livox/imu \
  -p output_point_cloud_topic:=point_cloud \
  -p output_imu_topic:=imu \
  -p frame_id:=livox_frame \
  -p filter_points:=true \
  -p min_distance:=0.1 \
  -p max_distance:=100.0 \
  -p downsample_factor:=1 &
ADAPTER_PID=$!

# Wait for Ctrl+C
echo -e "${GREEN}LiDAR subsystem is running${NC}"
echo -e "Use the following command to verify point cloud data:"
echo -e "${BLUE}ros2 topic echo /livox/lidar --field-match x --max-msgs 1${NC}"
echo -e "Press Ctrl+C to exit"

# Wait for Ctrl+C
wait $REFERENCE_PID $ADAPTER_PID

# Cleanup
echo -e "${BLUE}Shutting down LiDAR subsystem...${NC}"
kill $REFERENCE_PID $ADAPTER_PID 2>/dev/null
exit 0