#!/bin/bash

# Script to test the direct data recorder with raw topics
# This script launches the recorder node and then the sensors

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m' # No Color

# Initialize variables
RUN_CAMERAS=true
RUN_GNSS=true
RUN_LIDAR=true
LOG_LEVEL="debug" # Set to debug for more verbose output
VERBOSE=true
ALL_PIDS=()

# Function to cleanup on exit
cleanup() {
    echo -e "${YELLOW}Shutting down all processes...${NC}"
    
    # Kill all known PIDs
    for pid in "${ALL_PIDS[@]}"; do
        if kill -0 $pid 2>/dev/null; then
            echo -e "Stopping process with PID $pid"
            kill -SIGINT $pid 2>/dev/null
            sleep 1
            # Force kill if still running
            kill -9 $pid 2>/dev/null
        fi
    done
    
    # Find and kill any remaining ROS2 nodes
    echo -e "${YELLOW}Looking for orphaned ROS2 nodes...${NC}"
    ROS_PIDS=$(ps -ef | grep "_ros2_node\|ros2 run\|ros2 launch" | grep -v grep | awk '{print $2}')
    if [[ ! -z "$ROS_PIDS" ]]; then
        echo -e "${YELLOW}Found orphaned ROS2 processes. Cleaning up...${NC}"
        for ros_pid in $ROS_PIDS; do
            echo -e "Stopping orphaned ROS2 process with PID $ros_pid"
            kill -SIGINT $ros_pid 2>/dev/null
            sleep 0.5
            kill -9 $ros_pid 2>/dev/null
        done
    fi
    
    echo -e "${GREEN}All processes stopped${NC}"
    exit 0
}

# Set trap for clean exit
trap cleanup EXIT INT TERM

echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}  TESTING DIRECT DATA RECORDER  ${NC}"
echo -e "${BLUE}================================${NC}"

# Source ROS2
echo -e "${YELLOW}[1/3] Sourcing ROS2 environment...${NC}"
source /opt/ros/humble/setup.bash
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ ROS2 environment sourced${NC}"
else
    echo -e "${RED}✗ Failed to source ROS2 environment${NC}"
    exit 1
fi

# Source our implementation
echo -e "${YELLOW}[2/3] Sourcing our implementation...${NC}"
INSTALL_DIR="/home/user/Desktop/data-aquisition-digital-twin/data_aquisition/install"
# Check if install directory exists
if [ -d "$INSTALL_DIR" ]; then
    source $INSTALL_DIR/setup.bash
else
    # Try parent directory
    source /home/user/Desktop/data-aquisition-digital-twin/install/setup.bash
fi

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Our implementation sourced${NC}"
else
    echo -e "${RED}✗ Failed to source our implementation${NC}"
    echo -e "Please make sure it's built with 'colcon build'"
    exit 1
fi

# Source Livox SDK if running LiDAR
if [ "$RUN_LIDAR" = true ]; then
    echo -e "${YELLOW}[3/3] Sourcing Livox SDK...${NC}"
    source /opt/livox-sdk/install/setup.bash 2>/dev/null
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Livox SDK sourced${NC}"
    else
        echo -e "${RED}✗ Failed to source Livox SDK${NC}"
        echo -e "Would you like to continue without LiDAR? (y/n)"
        read -n 1 -r
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            echo -e "Exiting."
            exit 1
        fi
        echo
        RUN_LIDAR=false
    fi
fi

# Create log directory
LOG_DIR="/tmp/recorder_test_logs"
mkdir -p $LOG_DIR
echo -e "${GREEN}✓ Log directory created: ${LOG_DIR}${NC}"

# Start the data recorder first
echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}Starting data recorder node...${NC}"
echo -e "${BLUE}================================${NC}"

# Set the records directory
RECORDS_DIR="/home/user/Desktop/data-aquisition-digital-twin/data_aquisition/records"
mkdir -p $RECORDS_DIR
echo -e "${GREEN}✓ Recording to: ${RECORDS_DIR}${NC}"

# Launch recorder with verbose logging
ros2 run data_aquisition data_recorder_node --ros-args \
    -p output_dir:="${RECORDS_DIR}" \
    --log-level data_recorder:=${LOG_LEVEL} > ${LOG_DIR}/recorder.log 2>&1 &
RECORDER_PID=$!
ALL_PIDS+=($RECORDER_PID)
echo -e "${GREEN}✓ Data recorder started with PID ${RECORDER_PID}${NC}"

# Wait a moment to ensure recorder is running
sleep 2

# Check if recorder is still running
if kill -0 $RECORDER_PID 2>/dev/null; then
    echo -e "${GREEN}✓ Data recorder is running${NC}"
else
    echo -e "${RED}✗ Data recorder failed to start${NC}"
    if [ "$VERBOSE" = true ]; then
        echo -e "${YELLOW}Recorder log output:${NC}"
        cat ${LOG_DIR}/recorder.log
    fi
    exit 1
fi

# Now start the sensor nodes
echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}Starting sensor nodes...${NC}"
echo -e "${BLUE}================================${NC}"

# Run the sensors node script
/home/user/Desktop/data-aquisition-digital-twin/data_aquisition/scripts/run_sensors_node.sh \
    --log-level $LOG_LEVEL --verbose &
SENSORS_PID=$!
ALL_PIDS+=($SENSORS_PID)
echo -e "${GREEN}✓ Sensors launched with PID ${SENSORS_PID}${NC}"

# Wait for everything to initialize
sleep 5

# Show active topics
echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}Active Topics:${NC}"
echo -e "${BLUE}================================${NC}"
ros2 topic list

# Show record directory contents
echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}Current recording status:${NC}"
echo -e "${BLUE}================================${NC}"

# Monitor for a bit, then show what's being recorded
sleep 10
echo -e "${YELLOW}Content in records directory:${NC}"
find $RECORDS_DIR -type f | wc -l
echo -e "${YELLOW}Directory structure:${NC}"
ls -la $RECORDS_DIR

# Keep running until user hits Ctrl+C
echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}System is running and recording data${NC}"
echo -e "${BLUE}Press Ctrl+C to stop${NC}"
echo -e "${BLUE}================================${NC}"

# Wait for Ctrl+C
wait $SENSORS_PID