#!/bin/bash

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default settings
LIDAR_IP="192.168.1.100"
HOST_IP="192.168.1.5"
INTERFACE="eth0"
LOG=false

# Display header
echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}  MINIMAL LIVOX LIDAR TEST      ${NC}"
echo -e "${BLUE}================================${NC}"

# Parse command line arguments
while getopts "i:l:h:v" opt; do
  case $opt in
    i)
      INTERFACE=$OPTARG
      ;;
    l)
      LIDAR_IP=$OPTARG
      ;;
    h)
      HOST_IP=$OPTARG
      ;;
    v)
      LOG=true
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
  esac
done

echo -e "${YELLOW}Test Configuration:${NC}"
echo -e "  LiDAR IP:    ${LIDAR_IP}"
echo -e "  Host IP:     ${HOST_IP}"
echo -e "  Interface:   ${INTERFACE}"
echo -e "  Verbose:     ${LOG}"
echo ""

# Step 1: Check if interface exists
echo -e "${YELLOW}[1/5] Checking network interface...${NC}"
if ip link show ${INTERFACE} > /dev/null 2>&1; then
  echo -e "${GREEN}✓ Interface ${INTERFACE} exists${NC}"
else
  echo -e "${RED}✗ Interface ${INTERFACE} does not exist${NC}"
  echo -e "Available interfaces:"
  ip link | grep -v "lo" | grep "state" | awk -F: '{print $2}' | sed 's/ //g'
  exit 1
fi

# Step 2: Check network configuration
echo -e "${YELLOW}[2/5] Checking network configuration...${NC}"
CURRENT_IP=$(ip -4 addr show ${INTERFACE} | grep -oP '(?<=inet\s)\d+(\.\d+){3}')
echo -e "Current IP address for ${INTERFACE}: ${CURRENT_IP}"
echo -e "Target LiDAR IP: ${LIDAR_IP}"

# Check if IP is in the same subnet
CURRENT_SUBNET=$(echo "${CURRENT_IP}" | cut -d. -f1-3)
LIDAR_SUBNET=$(echo "${LIDAR_IP}" | cut -d. -f1-3)

if [ "${CURRENT_SUBNET}" = "${LIDAR_SUBNET}" ]; then
  echo -e "${GREEN}✓ IP address is in the correct subnet${NC}"
  # Update HOST_IP to use current IP instead of requested one
  HOST_IP=${CURRENT_IP}
else
  echo -e "${YELLOW}! IP address is not in the same subnet as LiDAR${NC}"
  # Automatically configure network in script mode
  echo -e "Automatically configuring network..."
  echo -e "Configuring ${INTERFACE} with IP ${HOST_IP}/24..."
  sudo ip addr flush dev ${INTERFACE}
  sudo ip addr add ${HOST_IP}/24 dev ${INTERFACE}
  sudo ip link set ${INTERFACE} up
  
  if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Network configuration successful${NC}"
  else
    echo -e "${RED}✗ Failed to configure network${NC}"
    exit 1
  fi
fi

# Step 3: Check connectivity to LiDAR
echo -e "${YELLOW}[3/5] Testing connectivity to LiDAR...${NC}"
ping -c 3 ${LIDAR_IP} > /dev/null 2>&1
if [ $? -eq 0 ]; then
  echo -e "${GREEN}✓ LiDAR is reachable${NC}"
else
  echo -e "${RED}✗ Cannot reach LiDAR at ${LIDAR_IP}${NC}"
  echo -e "Possible issues:"
  echo -e "  - LiDAR is not powered on"
  echo -e "  - LiDAR IP is incorrect"
  echo -e "  - Network cable is disconnected"
  exit 1
fi

# Step 4: Check if ports are accessible
echo -e "${YELLOW}[4/5] Testing port connectivity...${NC}"
# Test command port (TCP 56000)
nc -z -v -w 1 ${LIDAR_IP} 56000 > /dev/null 2>&1
if [ $? -eq 0 ]; then
  echo -e "${GREEN}✓ Command port (56000) is accessible${NC}"
else
  echo -e "${RED}✗ Cannot access command port (56000)${NC}"
fi

# Test data port (UDP 57000)
echo -e "  (Data ports will be tested during runtime)"

# Step 5: Launch ROS2 test
echo -e "${YELLOW}[5/5] Launching minimal ROS2 test...${NC}"
echo -e "Starting LiDAR node and point cloud listener..."
echo -e "${BLUE}Press Ctrl+C after a few seconds to stop the test${NC}"

# Start a temporary listener for point cloud topics
ros2 topic list > /dev/null 2>&1
if [ $? -ne 0 ]; then
  echo -e "${RED}ROS2 environment not sourced. Please source your ROS2 workspace:${NC}"
  echo -e "  source /opt/ros/humble/setup.bash"
  echo -e "  source ~/data-aquisition-digital-twin/install/setup.bash"
  exit 1
fi

# Create temporary log directory
mkdir -p /tmp/lidar_test_logs

# Start listener in background
ros2 topic echo /livox/lidar --field-match position > /tmp/lidar_test_logs/points.log 2>&1 &
LISTENER_PID=$!

# Start LiDAR node (official driver for test only)
ros2 run livox_ros_driver2 livox_ros_driver2_node > /tmp/lidar_test_logs/driver.log 2>&1 &
DRIVER_PID=$!

# Wait for output
sleep 5

# Check if data is received
if grep -q "x:" /tmp/lidar_test_logs/points.log; then
  echo -e "${GREEN}✓ Successfully received point cloud data from LiDAR${NC}"
  TEST_PASSED=true
else
  echo -e "${RED}✗ No point cloud data received${NC}"
  TEST_PASSED=false
fi

# Cleanup
kill $LISTENER_PID $DRIVER_PID > /dev/null 2>&1
wait $LISTENER_PID $DRIVER_PID > /dev/null 2>&1

# Final summary
echo -e "${BLUE}================================${NC}"
if [ "$TEST_PASSED" = true ]; then
  echo -e "${GREEN}TEST PASSED: LiDAR is working correctly${NC}"
  echo -e "${BLUE}You can now use the full LiDAR system:${NC}"
  echo -e "  ros2 launch data_aquisition lidar_launch.py"
else
  echo -e "${RED}TEST FAILED: LiDAR is not functioning correctly${NC}"
  echo -e "${BLUE}Please check the troubleshooting section in README.md${NC}"
fi
echo -e "${BLUE}================================${NC}"

# Display logs if requested
if [ "$LOG" = true ]; then
  echo -e "${YELLOW}Driver logs:${NC}"
  cat /tmp/lidar_test_logs/driver.log
  echo -e "${YELLOW}Point cloud logs:${NC}"
  cat /tmp/lidar_test_logs/points.log
fi

# Clean up logs
rm -rf /tmp/lidar_test_logs

exit 0