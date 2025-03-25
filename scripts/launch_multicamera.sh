#!/bin/bash
#
# Launch script for multiple ZED cameras using ROS2 launch
# This uses the proper ROS2 launch system to create multiple camera nodes

# ANSI color codes for terminal output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RESET='\033[0m'

# ROS setup
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$WORKSPACE_DIR/install/setup.bash"

echo -e "${CYAN}================================================${RESET}"
echo -e "${CYAN}   ZED Multi-Camera Launch Using ROS2 Launch${RESET}"
echo -e "${CYAN}================================================${RESET}"
echo -e "${CYAN}This script launches multiple ZED cameras as${RESET}"
echo -e "${CYAN}separate ROS2 nodes using the launch system.${RESET}"
echo -e "${CYAN}================================================${RESET}"
echo ""

# Clean up any previous camera nodes
echo -e "${YELLOW}Cleaning up any existing camera processes...${RESET}"
killall -q zed_camera_node
rm -f /tmp/zed_camera_*.pid

# Launch the cameras
echo -e "${YELLOW}Launching ZED cameras...${RESET}"
echo -e "${YELLOW}This will create separate nodes for each camera${RESET}"
echo -e "${YELLOW}with namespaces: /ZED_CAMERA_2i, /ZED_CAMERA_X0, /ZED_CAMERA_X1${RESET}"
echo ""

ros2 launch data_aquisition multi_camera_launch.py camera_resolution:=HD1080 camera_fps:=15.0

# The launch command will keep running until Ctrl+C is pressed