#!/bin/bash

# Simple script to run Livox LiDAR and verify topic publishing
# This version doesn't require xterm and runs in a single terminal

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}  LIVOX SIMPLE LAUNCHER         ${NC}"
echo -e "${BLUE}================================${NC}"

# Source ROS2
echo -e "${YELLOW}[1/4] Sourcing ROS2 environment...${NC}"
source /opt/ros/humble/setup.bash
if [ $? -eq 0 ]; then
  echo -e "${GREEN}✓ ROS2 environment sourced${NC}"
else
  echo -e "${RED}✗ Failed to source ROS2 environment${NC}"
  exit 1
fi

# Source reference implementation
echo -e "${YELLOW}[2/4] Sourcing reference Livox implementation...${NC}"
source /home/user/Desktop/instll_liv/ws_livox/install/setup.bash
if [ $? -eq 0 ]; then
  echo -e "${GREEN}✓ Livox reference implementation sourced${NC}"
else
  echo -e "${RED}✗ Failed to source Livox reference implementation${NC}"
  exit 1
fi

# Check connectivity
echo -e "${YELLOW}[3/4] Checking LiDAR connectivity...${NC}"
ping -c 3 192.168.1.100 > /dev/null 2>&1
if [ $? -eq 0 ]; then
  echo -e "${GREEN}✓ LiDAR is reachable${NC}"
else
  echo -e "${RED}✗ Cannot reach LiDAR at 192.168.1.100${NC}"
  echo -e "Please check physical connections and power"
  exit 1
fi

# Copy config file to reference path
echo -e "${YELLOW}[4/4] Setting up configuration...${NC}"
CONFIG_DIR="/home/user/Desktop/instll_liv/ws_livox/src/livox_ros_driver2/config"
if [ -d "$CONFIG_DIR" ]; then
  cp -f /home/user/Desktop/data-aquisition-digital-twin/data_aquisition/config/lidar/HAP_config.json "$CONFIG_DIR/"
  echo -e "${GREEN}✓ Configuration file copied to reference path${NC}"
else
  echo -e "${RED}✗ Reference config directory not found${NC}"
  echo -e "Will use our local configuration"
fi

# Launch the LiDAR
echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}Launching LiDAR in background...${NC}"
echo -e "${BLUE}================================${NC}"

# Run the LiDAR driver in the background
ros2 run livox_ros_driver2 livox_ros_driver2_node \
  --ros-args \
  -p xfer_format:=0 \
  -p multi_topic:=0 \
  -p data_src:=0 \
  -p publish_freq:=10.0 \
  -p output_data_type:=0 \
  -p frame_id:="livox_frame" \
  -p user_config_path:="${CONFIG_DIR}/HAP_config.json" \
  -p cmdline_input_bd_code:="livox0000000001" > /tmp/livox_driver.log 2>&1 &

DRIVER_PID=$!

# Wait for the driver to start
echo -e "${YELLOW}Waiting for LiDAR to start...${NC}"
sleep 5

# Check if the driver is still running
if kill -0 $DRIVER_PID > /dev/null 2>&1; then
  echo -e "${GREEN}✓ LiDAR driver is running (PID: $DRIVER_PID)${NC}"
  
  # Display the log
  echo -e "${YELLOW}Driver log:${NC}"
  cat /tmp/livox_driver.log
  
  # Check for Livox topics
  echo -e "${YELLOW}Checking for Livox topics...${NC}"
  LIVOX_TOPICS=$(ros2 topic list | grep livox)
  if [ -n "$LIVOX_TOPICS" ]; then
    echo -e "${GREEN}✓ Found Livox topics:${NC}"
    echo "$LIVOX_TOPICS" | while read topic; do
      echo -e "  ${GREEN}● $topic${NC}"
    done
    
    # Show topic info
    echo -e "${YELLOW}Getting topic info for /livox/lidar...${NC}"
    ros2 topic info /livox/lidar
    
    # Check topic publishing rate
    echo -e "${YELLOW}Checking topic publishing rate (press Ctrl+C after a few seconds):${NC}"
    ros2 topic hz /livox/lidar --window 5
    
    # Show a sample message
    echo -e "${YELLOW}Showing a sample message:${NC}"
    ros2 topic echo /livox/lidar --field-match header --max-msgs 1
  else
    echo -e "${RED}✗ No Livox topics found${NC}"
  fi
else
  echo -e "${RED}✗ LiDAR driver failed to start or has exited${NC}"
  echo -e "${YELLOW}Driver log:${NC}"
  cat /tmp/livox_driver.log
fi

# Cleanup
echo -e "${BLUE}================================${NC}"
echo -e "${YELLOW}Stopping LiDAR driver...${NC}"
kill $DRIVER_PID > /dev/null 2>&1
echo -e "${GREEN}LiDAR driver stopped${NC}"
echo -e "${BLUE}================================${NC}"

exit 0