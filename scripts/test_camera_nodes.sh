#!/bin/bash
#
# Test Script for Camera Node Names
# This script tests if we can properly set unique ROS2 node names for camera nodes

# ANSI color codes for terminal output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
RESET='\033[0m'

# ROS setup
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$WORKSPACE_DIR/install/setup.bash"

echo -e "${CYAN}================================================${RESET}"
echo -e "${CYAN}   ZED Camera Node Name Test${RESET}"
echo -e "${CYAN}================================================${RESET}"
echo -e "${CYAN}This script tests if we can properly set unique${RESET}"
echo -e "${CYAN}ROS2 node names for ZED camera nodes.${RESET}"
echo -e "${CYAN}================================================${RESET}"
echo ""

# Cleanup any existing camera nodes
echo -e "${YELLOW}Cleaning up any existing camera processes...${RESET}"
killall -q zed_camera_node
rm -f /tmp/zed_camera_*.pid
echo -e "${GREEN}Cleanup complete${RESET}"
echo ""

# Start a node with a specific name directly (not through the launch script)
echo -e "${YELLOW}Starting test camera node with name 'test_camera_node1'...${RESET}"
ros2 run data_aquisition zed_camera_node --ros-args -r "/__node:=test_camera_node1" -p camera.serial_number:=40894785 > /tmp/test_node1.log 2>&1 &
NODE1_PID=$!
echo $NODE1_PID > /tmp/test_camera_node1.pid

# Wait a bit for the node to start
sleep 5

# Check if the node exists with the correct name
echo -e "${YELLOW}Checking ROS2 node list...${RESET}"
ros2 node list > /tmp/node_list.txt
cat /tmp/node_list.txt

# Check if our test node is in the list
if grep -q "/test_camera_node1" /tmp/node_list.txt; then
  echo -e "${GREEN}[✓] Successfully created node with custom name 'test_camera_node1'${RESET}"
else
  echo -e "${RED}[✗] Failed to create node with custom name - check if node remapping is working${RESET}"
fi

# Clean up
echo -e "${YELLOW}Cleaning up test node...${RESET}"
kill $NODE1_PID 2>/dev/null
rm -f /tmp/test_camera_node1.pid

echo -e "\n${GREEN}Test completed.${RESET}"