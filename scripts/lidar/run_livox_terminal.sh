#!/bin/bash

# Run Livox LiDAR directly in terminal mode (no RViz)
# This script runs the LiDAR using the reference implementation without visualization

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}  LIVOX TERMINAL MODE LAUNCHER  ${NC}"
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

# Create a script to run in another terminal for topic monitoring
TMP_MONITOR_SCRIPT=$(mktemp)
cat > ${TMP_MONITOR_SCRIPT} << 'EOF'
#!/bin/bash
GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}  LIVOX TOPIC MONITOR          ${NC}"
echo -e "${BLUE}================================${NC}"

# Wait for a moment to let LiDAR start publishing
echo -e "${YELLOW}Waiting for LiDAR topics to become available...${NC}"
sleep 5

# Display active topics that include "livox"
echo -e "${YELLOW}[1/3] Checking for Livox topics...${NC}"
LIVOX_TOPICS=$(ros2 topic list | grep livox)
if [ -n "$LIVOX_TOPICS" ]; then
  echo -e "${GREEN}✓ Found Livox topics:${NC}"
  echo "$LIVOX_TOPICS" | while read topic; do
    echo -e "  ${GREEN}● $topic${NC}"
  done
else
  echo -e "${RED}✗ No Livox topics found${NC}"
fi

echo -e "${YELLOW}[2/3] Checking topic publishing rates...${NC}"
ros2 topic hz /livox/lidar --window 10 &
HZ_PID=$!
sleep 5
kill $HZ_PID > /dev/null 2>&1

echo -e "${YELLOW}[3/3] Checking point cloud data sample...${NC}"
echo -e "${BLUE}Showing first message from /livox/lidar:${NC}"
ros2 topic echo /livox/lidar --max-msgs 1 --field-match data[0:10]

echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}Monitoring complete.${NC}"
echo -e "${BLUE}================================${NC}"
echo -e "${YELLOW}To continuously monitor topics:${NC}"
echo -e "${BLUE}ros2 topic list | grep livox${NC}"
echo -e "${BLUE}ros2 topic hz /livox/lidar${NC}"
echo -e "${BLUE}ros2 topic echo /livox/lidar --field-match x --max-msgs 1${NC}"
echo -e "${BLUE}================================${NC}"
EOF
chmod +x ${TMP_MONITOR_SCRIPT}

# Launch the LiDAR
echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}Launching LiDAR in terminal mode${NC}"
echo -e "${BLUE}================================${NC}"

# Create a script to run the LiDAR node with all parameters
TMP_LAUNCH_SCRIPT=$(mktemp)
cat > ${TMP_LAUNCH_SCRIPT} << EOF
#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/user/Desktop/instll_liv/ws_livox/install/setup.bash

xfer_format=0
multi_topic=0
data_src=0
publish_freq=10.0
output_type=0
frame_id="livox_frame"
cmdline_bd_code="livox0000000001"
user_config_path="/home/user/Desktop/instll_liv/ws_livox/src/livox_ros_driver2/config/HAP_config.json"

ros2 run livox_ros_driver2 livox_ros_driver2_node \
  --ros-args \
  -p xfer_format:=\${xfer_format} \
  -p multi_topic:=\${multi_topic} \
  -p data_src:=\${data_src} \
  -p publish_freq:=\${publish_freq} \
  -p output_data_type:=\${output_type} \
  -p frame_id:=\${frame_id} \
  -p user_config_path:=\${user_config_path} \
  -p cmdline_input_bd_code:=\${cmdline_bd_code}
EOF
chmod +x ${TMP_LAUNCH_SCRIPT}

# Run topic monitor in another terminal
echo -e "${YELLOW}Starting topic monitor in separate terminal...${NC}"
xterm -title "Livox Topic Monitor" -e ${TMP_MONITOR_SCRIPT} &
MONITOR_PID=$!

# Run LiDAR driver
echo -e "${YELLOW}Starting Livox LiDAR driver...${NC}"
echo -e "${BLUE}Press Ctrl+C to exit${NC}"
${TMP_LAUNCH_SCRIPT}

# Cleanup
kill $MONITOR_PID > /dev/null 2>&1
rm -f ${TMP_LAUNCH_SCRIPT} ${TMP_MONITOR_SCRIPT}

exit 0