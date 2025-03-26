#!/bin/bash

# Run Livox LiDAR directly using reference driver without RViz2
# This script runs only the LiDAR nodes publishing to /livox/lidar and /livox/status topics

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Initialize variables
LIDAR_IP="192.168.1.100"
LIDAR_DRIVER_PID=""

# Function to cleanup on exit
cleanup() {
    echo -e "${YELLOW}Shutting down LiDAR node...${NC}"
    if [ ! -z "$LIDAR_DRIVER_PID" ]; then
        if kill -0 $LIDAR_DRIVER_PID 2>/dev/null; then
            echo -e "Stopping LiDAR driver (PID: $LIDAR_DRIVER_PID)"
            kill -SIGINT $LIDAR_DRIVER_PID 2>/dev/null
            sleep 1
            # Force kill if still running
            kill -9 $LIDAR_DRIVER_PID 2>/dev/null
        fi
    fi
    echo -e "${GREEN}LiDAR node stopped${NC}"
    exit 0
}

# Set trap for clean exit
trap cleanup EXIT INT TERM

# Function to display help
show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo "Options:"
    echo "  -i, --ip IP        Set LiDAR IP address (default: 192.168.1.100)"
    echo "  -h, --help         Show this help message"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -i|--ip)
            LIDAR_IP="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}  LIVOX LIDAR LAUNCHER          ${NC}"
echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}LiDAR IP: ${NC}${LIDAR_IP}"
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
    echo -e "Expected at: /home/user/Desktop/instll_liv/ws_livox"
    exit 1
fi

# Check connectivity
echo -e "${YELLOW}[3/4] Checking LiDAR connectivity...${NC}"
ping -c 2 -W 1 ${LIDAR_IP} > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ LiDAR is reachable at ${LIDAR_IP}${NC}"
else
    echo -e "${RED}✗ Cannot reach LiDAR at ${LIDAR_IP}${NC}"
    echo -e "Would you like to continue anyway? (y/n)"
    read -n 1 -r
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo -e "Exiting."
        exit 1
    fi
    echo
    echo -e "Continuing despite connectivity issues..."
fi

# Copy config file to reference path
echo -e "${YELLOW}[4/4] Setting up configuration...${NC}"
CONFIG_DIR="/home/user/Desktop/instll_liv/ws_livox/src/livox_ros_driver2/config"
if [ -d "$CONFIG_DIR" ]; then
    cp -f /home/user/Desktop/data-aquisition-digital-twin/data_aquisition/config/lidar/HAP_config.json "$CONFIG_DIR/"
    echo -e "${GREEN}✓ Configuration file copied to reference path${NC}"
else
    echo -e "${RED}✗ Reference config directory not found${NC}"
    echo -e "Will try to use default configuration"
fi

# Launch the LiDAR node
echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}Launching LiDAR with reference driver${NC}"
echo -e "${BLUE}Press Ctrl+C to exit${NC}"
echo -e "${BLUE}================================${NC}"

# Start the LiDAR driver in the background - KEY DIFFERENCE FROM ORIGINAL SCRIPT
# Instead of launching the RViz launch file, we directly run the driver node
echo -e "${YELLOW}Starting Livox LiDAR driver...${NC}"
ros2 run livox_ros_driver2 livox_ros_driver2_node \
    --ros-args \
    -p xfer_format:=0 \
    -p multi_topic:=0 \
    -p data_src:=0 \
    -p publish_freq:=10.0 \
    -p output_data_type:=0 \
    -p frame_id:="livox_frame" \
    -p user_config_path:="${CONFIG_DIR}/HAP_config.json" \
    -p cmdline_input_bd_code:="livox0000000001" &

LIDAR_DRIVER_PID=$!
echo -e "${GREEN}LiDAR driver started with PID ${LIDAR_DRIVER_PID}${NC}"

# Wait for the driver to start
sleep 3

# Check if the driver is still running
if kill -0 $LIDAR_DRIVER_PID 2>/dev/null; then
    echo -e "${GREEN}✓ LiDAR driver is running${NC}"
    
    # Check for Livox topics
    echo -e "${YELLOW}Checking for Livox topics...${NC}"
    
    # Wait a bit to allow topics to be registered
    sleep 2
    
    LIVOX_TOPICS=$(ros2 topic list | grep livox)
    if [ -n "$LIVOX_TOPICS" ]; then
        echo -e "${GREEN}✓ Found Livox topics:${NC}"
        echo "$LIVOX_TOPICS" | while read topic; do
            echo -e "  ${GREEN}● $topic${NC}"
        done
        
        # Show topic info
        if ros2 topic list | grep -q "/livox/lidar"; then
            echo -e "${YELLOW}Checking /livox/lidar topic...${NC}"
            ros2 topic info /livox/lidar
        else
            echo -e "${YELLOW}! /livox/lidar topic not found (yet)${NC}"
        fi
        
        if ros2 topic list | grep -q "/livox/status"; then
            echo -e "${YELLOW}Checking /livox/status topic...${NC}"
            ros2 topic info /livox/status
        else
            echo -e "${YELLOW}! /livox/status topic not found (yet)${NC}"
        fi
    else
        echo -e "${YELLOW}! No Livox topics found yet. This could be normal during initialization.${NC}"
    fi
else
    echo -e "${RED}✗ LiDAR driver failed to start or exited immediately${NC}"
    exit 1
fi

echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}LiDAR is running and publishing data${NC}"
echo -e "${BLUE}To check data flow, run in another terminal:${NC}"
echo -e "${YELLOW}ros2 topic hz /livox/lidar${NC}"
echo -e "${BLUE}Press Ctrl+C to stop LiDAR and exit${NC}"
echo -e "${BLUE}================================${NC}"

# Wait for the LiDAR driver process to complete or be terminated
wait $LIDAR_DRIVER_PID

# The cleanup function will be called automatically by the trap handler