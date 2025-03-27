#!/bin/bash

# Run Livox LiDAR with synchronization and visualization
# This script runs the LiDAR with the synchronization node and RViz2

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to display help
show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo "Options:"
    echo "  -n, --no-sync       Run without synchronization node"
    echo "  -a, --no-adapter    Run without adapter node (direct integration only)"
    echo "  -r, --no-rviz       Run without RViz2 visualization"
    echo "  -h, --help          Show this help message"
}

# Initialize variables
USE_SYNC="true"
USE_ADAPTER="true"
USE_RVIZ="true"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -n|--no-sync)
            USE_SYNC="false"
            shift
            ;;
        -a|--no-adapter)
            USE_ADAPTER="false"
            shift
            ;;
        -r|--no-rviz)
            USE_RVIZ="false"
            shift
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
echo -e "${BLUE}  SYNCHRONIZED LIDAR LAUNCHER   ${NC}"
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

# Source our workspace
echo -e "${YELLOW}[2/3] Sourcing workspace...${NC}"
if [ -f /home/user/Desktop/data-aquisition-digital-twin/install/setup.bash ]; then
    source /home/user/Desktop/data-aquisition-digital-twin/install/setup.bash
    echo -e "${GREEN}✓ Workspace sourced${NC}"
else
    echo -e "${RED}✗ Could not find workspace${NC}"
    echo -e "Expected at: /home/user/Desktop/data-aquisition-digital-twin/install/setup.bash"
    exit 1
fi

# Check connectivity to LiDAR
echo -e "${YELLOW}[3/3] Checking LiDAR connectivity...${NC}"
ping -c 2 -W 1 192.168.1.100 > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ LiDAR is reachable${NC}"
else
    echo -e "${YELLOW}! Cannot reach LiDAR at 192.168.1.100${NC}"
    echo -e "Continue anyway? (y/n)"
    read -n 1 -r
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo -e "Exiting."
        exit 1
    fi
    echo
    echo -e "Continuing despite connectivity issues..."
fi

# Launch configuration
echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}Launching LiDAR with configuration:${NC}"
echo -e "${BLUE}- Synchronization: ${NC}${USE_SYNC}"
echo -e "${BLUE}- Adapter: ${NC}${USE_ADAPTER}"
echo -e "${BLUE}- RViz2: ${NC}${USE_RVIZ}"
echo -e "${BLUE}================================${NC}"

# Launch the LiDAR
echo -e "${YELLOW}Starting LiDAR with synchronization...${NC}"
ros2 launch data_aquisition lidar_with_sync_launch.py \
    use_sync:=${USE_SYNC} \
    use_adapter:=${USE_ADAPTER} \
    use_rviz:=${USE_RVIZ}

echo -e "${GREEN}Done!${NC}"
exit 0