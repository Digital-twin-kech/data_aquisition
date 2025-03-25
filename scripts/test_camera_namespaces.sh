#!/bin/bash
#
# Test Script for Camera Namespace Validation
# This script launches ZED X cameras and verifies they use separate namespaces

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
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$WORKSPACE_DIR/install/setup.bash"

echo -e "${CYAN}================================================${RESET}"
echo -e "${CYAN}   ZED Camera Namespace Validation Script${RESET}"
echo -e "${CYAN}================================================${RESET}"
echo -e "${CYAN}This script launches ZED X cameras and verifies${RESET}"
echo -e "${CYAN}they publish to their respective namespaces.${RESET}"
echo -e "${CYAN}================================================${RESET}"
echo ""

# Cleanup any existing camera nodes
echo -e "${YELLOW}Cleaning up any existing camera processes...${RESET}"
killall -q zed_camera_node
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
echo -e "${GREEN}Cleanup complete${RESET}"
echo ""

# Launch the cameras using our script (background)
echo -e "${YELLOW}Launching ZED X cameras...${RESET}"
"$SCRIPT_DIR/camera/launch_all_cameras.sh" > /tmp/camera_launch.log 2>&1 &
LAUNCH_PID=$!

# Wait for cameras to start (10 seconds)
echo -e "${YELLOW}Waiting for cameras to initialize (10 seconds)...${RESET}"
sleep 10

# Check available topics to validate namespaces
echo -e "${YELLOW}Checking ROS2 topics for camera namespaces...${RESET}"
ros2 topic list > /tmp/camera_topics.txt

# Look for expected namespaces
ZEDX0_TOPICS=$(grep "/ZEDX-GMSL0/" /tmp/camera_topics.txt)
ZEDX1_TOPICS=$(grep "/ZEDX-GMSL1/" /tmp/camera_topics.txt)

# Display results
echo -e "\n${CYAN}Namespace Validation Results:${RESET}"
echo -e "${CYAN}================================${RESET}"

if [ -n "$ZEDX0_TOPICS" ]; then
  echo -e "${GREEN}[✓] ZEDX-GMSL0 namespace detected${RESET}"
  echo -e "Topics found:"
  echo "$ZEDX0_TOPICS" | sed 's/^/  /'
else
  echo -e "${RED}[✗] ZEDX-GMSL0 namespace NOT detected${RESET}"
fi

if [ -n "$ZEDX1_TOPICS" ]; then
  echo -e "${GREEN}[✓] ZEDX-GMSL1 namespace detected${RESET}"
  echo -e "Topics found:"
  echo "$ZEDX1_TOPICS" | sed 's/^/  /'
else
  echo -e "${RED}[✗] ZEDX-GMSL1 namespace NOT detected${RESET}"
fi

# Check for topic data to verify cameras are publishing
echo -e "\n${YELLOW}Checking for data on image topics...${RESET}"

# Function to check if topic is publishing
check_topic_publishing() {
  local topic=$1
  local timeout=5
  
  if [ -z "$topic" ]; then
    echo -e "${RED}[✗] Topic not found${RESET}"
    return 1
  fi
  
  echo -e "${YELLOW}Checking $topic...${RESET}"
  # Try to get a message with timeout
  timeout ${timeout}s ros2 topic echo -n 1 "$topic" > /tmp/topic_data.txt 2>&1
  
  if grep -q "header:" /tmp/topic_data.txt; then
    echo -e "${GREEN}[✓] Topic $topic is actively publishing data${RESET}"
    return 0
  else
    echo -e "${RED}[✗] No data received on topic $topic${RESET}"
    return 1
  fi
}

# Check first camera image topic
ZEDX0_RGB_TOPIC=$(grep "/ZEDX-GMSL0/rgb/image_rect_color" /tmp/camera_topics.txt)
check_topic_publishing "$ZEDX0_RGB_TOPIC"

# Check second camera image topic
ZEDX1_RGB_TOPIC=$(grep "/ZEDX-GMSL1/rgb/image_rect_color" /tmp/camera_topics.txt)
check_topic_publishing "$ZEDX1_RGB_TOPIC"

# Cleanup
echo -e "\n${YELLOW}Shutting down camera processes...${RESET}"
kill $LAUNCH_PID 2>/dev/null
killall -q zed_camera_node
for pid_file in /tmp/zed_camera_*.pid; do
  if [ -f "$pid_file" ]; then
    PID=$(cat "$pid_file")
    if kill -0 $PID 2>/dev/null; then
      kill $PID
    fi
    rm "$pid_file"
  fi
done

echo -e "\n${GREEN}Test completed.${RESET}"