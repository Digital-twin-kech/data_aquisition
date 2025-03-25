#!/bin/bash

# Run Livox LiDAR directly using reference driver
# This script runs the LiDAR using the reference implementation

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}  LIVOX DIRECT LAUNCH SCRIPT    ${NC}"
echo -e "${BLUE}================================${NC}"

# Source ROS2
echo -e "${YELLOW}[1/5] Sourcing ROS2 environment...${NC}"
source /opt/ros/humble/setup.bash
if [ $? -eq 0 ]; then
  echo -e "${GREEN}✓ ROS2 environment sourced${NC}"
else
  echo -e "${RED}✗ Failed to source ROS2 environment${NC}"
  exit 1
fi

# Source reference implementation
echo -e "${YELLOW}[2/5] Sourcing reference Livox implementation...${NC}"
source /home/user/Desktop/instll_liv/ws_livox/install/setup.bash
if [ $? -eq 0 ]; then
  echo -e "${GREEN}✓ Livox reference implementation sourced${NC}"
else
  echo -e "${RED}✗ Failed to source Livox reference implementation${NC}"
  exit 1
fi

# Source our implementation
echo -e "${YELLOW}[3/5] Sourcing our implementation...${NC}"
source /home/user/Desktop/data-aquisition-digital-twin/install/setup.bash 2>/dev/null
if [ $? -eq 0 ]; then
  echo -e "${GREEN}✓ Our implementation sourced${NC}"
else
  echo -e "${YELLOW}! Could not source our implementation${NC}"
  echo -e "Continuing with reference implementation only"
fi

# Check connectivity
echo -e "${YELLOW}[4/5] Checking LiDAR connectivity...${NC}"
ping -c 3 192.168.1.100 > /dev/null 2>&1
if [ $? -eq 0 ]; then
  echo -e "${GREEN}✓ LiDAR is reachable${NC}"
else
  echo -e "${RED}✗ Cannot reach LiDAR at 192.168.1.100${NC}"
  echo -e "Please check physical connections and power"
  exit 1
fi

# Copy config file to reference path
echo -e "${YELLOW}[5/5] Setting up configuration...${NC}"
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
echo -e "${GREEN}Launching LiDAR with reference driver${NC}"
echo -e "${BLUE}Press Ctrl+C to exit${NC}"
echo -e "${BLUE}================================${NC}"

# Launch using the reference implementation
ros2 launch livox_ros_driver2 rviz_HAP_launch.py