#!/bin/bash
#
# Multi-Camera Launch Script
# This script launches multiple ZED cameras in separate processes
# with color-coded output for easier monitoring.
#

# ANSI color codes for terminal output
BLUE='\033[0;34m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
RESET='\033[0m'

# ROS setup
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
source "$WORKSPACE_DIR/install/setup.bash"

# Camera configuration
ZED2I_SERIAL=37503998
ZEDX0_SERIAL=40894785
ZEDX1_SERIAL=41050786

# Camera names (these will be used as node names and namespaces)
ZED2I_NAME="ZED_CAMERA_2i"
ZEDX0_NAME="ZED_CAMERA_X0"
ZEDX1_NAME="ZED_CAMERA_X1"

# Resolution and FPS settings
RESOLUTION="HD1080"
FPS=15.0

# Prefix function for color-coded logs
prefix_output() {
  local prefix="$1"
  local color="$2"
  local prefix_len=${#prefix}
  # Calculate spacing for alignment
  local spacing=$((15 - prefix_len))
  local pad=$(printf "%${spacing}s" "")
  
  # Read input line by line and add prefix with color
  while IFS= read -r line; do
    echo -e "${color}[${prefix}]${pad}${RESET} $line"
  done
}

# Function to launch a single camera
launch_camera() {
  local name="$1"
  local serial="$2"
  local model="$3"
  local color="$4"
  
  echo -e "${color}Launching $name (SN: $serial)...${RESET}"
  
  # Run camera node with specific parameters and a unique node name
  # This ensures each camera runs as a separate node with its own namespace
  # Use ROS namespace and node name remapping with correct syntax
  ros2 run data_aquisition zed_camera_node \
    --ros-args \
    -r __ns:=/$name \
    -r __node:=$name \
    -p camera.serial_number:=$serial \
    -p camera.model:=$model \
    -p camera.resolution:=$RESOLUTION \
    -p camera.min_fps:=$FPS \
    -p camera.max_fps:=$FPS 2>&1 | prefix_output "$name" "$color" &

  
  # Store process ID
  echo $! > "/tmp/zed_camera_${name}.pid"
  sleep 2 # Small delay between camera launches
  
  echo -e "${color}Camera $name started as separate ROS2 node '/$name' with topics:${RESET}"
  echo -e "${color} - /$name/rgb/image_rect_color${RESET}"
  echo -e "${color} - /$name/depth/depth_registered${RESET}"
  echo -e "${color} - /$name/point_cloud/cloud_registered${RESET}"
  echo -e "${color} - /$name/imu/data${RESET}"
  echo -e "${color} - /$name/status${RESET}"
  echo -e "${color}You can verify with: ros2 node list | grep $name${RESET}"
}

# Function to check if camera is available
check_camera() {
  local serial="$1"
  local model="$2"
  local name="$3"
  local color="$4"
  
  # Check with ZED Explorer if camera is available
  if [[ -x "/usr/local/zed/tools/ZED_Explorer" ]]; then
    echo -e "${color}Checking if $name (SN: $serial) is available...${RESET}"
    timeout 3s /usr/local/zed/tools/ZED_Explorer -l | grep -q "$serial"
    if [ $? -eq 0 ]; then
      echo -e "${color}✓ $name camera detected${RESET}"
      return 0
    else
      echo -e "${RED}✗ $name camera not detected${RESET}"
      return 1
    fi
  else
    echo -e "${YELLOW}WARNING: ZED Explorer not found, cannot verify camera availability${RESET}"
    return 0 # Assume camera is available
  fi
}

# Cleanup function
cleanup() {
  echo -e "${YELLOW}Shutting down all camera processes...${RESET}"
  # Kill all camera processes
  for pid_file in /tmp/zed_camera_*.pid; do
    if [ -f "$pid_file" ]; then
      PID=$(cat "$pid_file")
      if kill -0 $PID 2>/dev/null; then
        kill $PID
        echo -e "${GREEN}Stopped camera process $PID${RESET}"
      fi
      rm "$pid_file"
    fi
  done
  
  # Extra cleanup for any hanging processes
  killall -q zed_camera_node
  
  echo -e "${GREEN}All camera processes terminated${RESET}"
  exit 0
}

# Set up trap for cleanup
trap cleanup SIGINT SIGTERM

# Display header
echo -e "${CYAN}================================================${RESET}"
echo -e "${CYAN}       ZED Multi-Camera Launch Script${RESET}"
echo -e "${CYAN}================================================${RESET}"
echo -e "${CYAN}This script launches multiple ZED cameras in separate${RESET}"
echo -e "${CYAN}processes. Press Ctrl+C to stop all cameras.${RESET}"
echo -e "${CYAN}================================================${RESET}"
echo ""

# Check ZED SDK installation
if [ ! -d "/usr/local/zed" ]; then
  echo -e "${RED}ERROR: ZED SDK not found at /usr/local/zed${RESET}"
  exit 1
fi

# Display SDK version
SDK_VERSION=$(grep -a "ZED_SDK_MAJOR_VERSION" /usr/local/zed/include/sl/Camera.hpp | head -1 | awk '{print $3}')
SDK_MINOR=$(grep -a "ZED_SDK_MINOR_VERSION" /usr/local/zed/include/sl/Camera.hpp | head -1 | awk '{print $3}')
SDK_PATCH=$(grep -a "ZED_SDK_PATCH_VERSION" /usr/local/zed/include/sl/Camera.hpp | head -1 | awk '{print $3}')
echo -e "${GREEN}ZED SDK Version: $SDK_VERSION.$SDK_MINOR.$SDK_PATCH${RESET}"
echo ""

# Launch ZED 2i (USB) camera if available
launch_camera "$ZED2I_NAME" $ZED2I_SERIAL "ZED2i" "$BLUE"

# Launch ZED X0 (GMSL-0) camera if available
launch_camera "$ZEDX0_NAME" $ZEDX0_SERIAL "ZED_X" "$GREEN"

# Launch ZED X1 (GMSL-1) camera if available
launch_camera "$ZEDX1_NAME" $ZEDX1_SERIAL "ZED_X" "$MAGENTA"


# Give nodes a moment to fully initialize
sleep 3

# Show all running nodes to confirm separate nodes are running
echo -e "${CYAN}================================================${RESET}"
echo -e "${CYAN}   Verifying Camera Nodes${RESET}"
echo -e "${CYAN}================================================${RESET}"
echo -e "${YELLOW}Running ROS2 nodes (should show multiple camera nodes):${RESET}"
ros2 node list

# Show available topics from the camera nodes
echo -e "\n${YELLOW}Available camera topics:${RESET}"
ros2 topic list | grep -E "ZED|zed"

echo -e "\n${YELLOW}All cameras launched. Press Ctrl+C to stop all cameras.${RESET}"

# Wait for all camera processes to finish or until Ctrl+C
wait

# Call cleanup to ensure proper termination
cleanup
