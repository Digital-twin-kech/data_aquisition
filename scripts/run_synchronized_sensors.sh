#!/bin/bash

# Script to run all sensors and the synchronization node
# This enables complete data collection with synchronized output

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
WHITE='\033[1;37m'
NC='\033[0m' # No Color

# Initialize variables
RUN_CAMERAS=true
RUN_GNSS=true
RUN_LIDAR=true
RUN_SYNC=true
VERBOSE=false
ALL_PIDS=()

# Function to cleanup on exit
cleanup() {
    echo -e "${YELLOW}Shutting down all processes...${NC}"
    for pid in "${ALL_PIDS[@]}"; do
        if kill -0 $pid 2>/dev/null; then
            echo -e "Stopping process with PID $pid"
            kill -SIGINT $pid 2>/dev/null
            sleep 1
            # Force kill if still running
            kill -9 $pid 2>/dev/null
        fi
    done
    echo -e "${GREEN}All processes stopped${NC}"
    exit 0
}

# Set trap for clean exit
trap cleanup EXIT INT TERM

# Function to display help
show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo "Options:"
    echo "  --no-cameras         Don't run camera nodes"
    echo "  --no-gnss            Don't run GNSS node"
    echo "  --no-lidar           Don't run LiDAR node"
    echo "  --no-sync            Don't run synchronization node"
    echo "  -v, --verbose        Enable verbose output"
    echo "  -h, --help           Show this help message"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --no-cameras)
            RUN_CAMERAS=false
            shift
            ;;
        --no-gnss)
            RUN_GNSS=false
            shift
            ;;
        --no-lidar)
            RUN_LIDAR=false
            shift
            ;;
        --no-sync)
            RUN_SYNC=false
            shift
            ;;
        -v|--verbose)
            VERBOSE=true
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
echo -e "${WHITE}  SYNCHRONIZED SENSORS LAUNCHER ${NC}"
echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}Running cameras: ${NC}${RUN_CAMERAS}"
echo -e "${BLUE}Running GNSS:    ${NC}${RUN_GNSS}"
echo -e "${BLUE}Running LiDAR:   ${NC}${RUN_LIDAR}"
echo -e "${BLUE}Running sync:    ${NC}${RUN_SYNC}"
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

# Source our implementation
echo -e "${YELLOW}[2/5] Sourcing our implementation...${NC}"
source /home/user/Desktop/data-aquisition-digital-twin/data_aquisition/install/setup.bash 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Our implementation sourced${NC}"
else
    echo -e "${RED}✗ Failed to source our implementation${NC}"
    echo -e "Please make sure it's built with 'colcon build'"
    exit 1
fi

# Source reference Livox implementation
if [ "$RUN_LIDAR" = true ]; then
    echo -e "${YELLOW}[3/5] Sourcing Livox implementation...${NC}"
    source /opt/livox-sdk/install/setup.bash 2>/dev/null
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Livox implementation sourced${NC}"
    else
        source /home/user/Desktop/instll_liv/ws_livox/install/setup.bash 2>/dev/null
        if [ $? -eq 0 ]; then
            echo -e "${YELLOW}⚠ Using temporary Livox SDK location${NC}"
            echo -e "${YELLOW}⚠ Consider migrating to /opt/livox-sdk using migrate_livox_sdk.sh${NC}"
        else
            echo -e "${RED}✗ Failed to source Livox implementation${NC}"
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
fi

# Create a log directory
LOG_DIR="/tmp/sensor_logs"
echo -e "${YELLOW}[4/5] Setting up log directory...${NC}"
mkdir -p $LOG_DIR
echo -e "${GREEN}✓ Log directory created: ${LOG_DIR}${NC}"

# Run sensors
echo -e "${YELLOW}[5/5] Launching sensor processes...${NC}"

# Run script to start sensors
echo -e "${CYAN}Starting sensors...${NC}"
/home/user/Desktop/data-aquisition-digital-twin/data_aquisition/scripts/run_sensors_node.sh --no-cameras=$([[ "$RUN_CAMERAS" = false ]] && echo "true" || echo "false") --no-gnss=$([[ "$RUN_GNSS" = false ]] && echo "true" || echo "false") --no-lidar=$([[ "$RUN_LIDAR" = false ]] && echo "true" || echo "false") &
SENSORS_PID=$!
ALL_PIDS+=($SENSORS_PID)
echo -e "${GREEN}✓ Sensors script started with PID ${SENSORS_PID}${NC}"

# Wait for sensors to initialize
echo -e "${CYAN}Waiting for sensors to initialize...${NC}"
sleep 10

# Run synchronization node
if [ "$RUN_SYNC" = true ]; then
    echo -e "${MAGENTA}Starting synchronization node...${NC}"
    ros2 run data_aquisition sync_node --ros-args \
        -p camera_names:='["ZED_CAMERA_2i", "ZED_CAMERA_X0", "ZED_CAMERA_X1"]' \
        -p sync_lidar:=$RUN_LIDAR \
        -p sync_gnss:=$RUN_GNSS \
        -p time_tolerance:=0.02 \
        -p cache_size:=100 \
        --log-level info > ${LOG_DIR}/sync.log 2>&1 &
    SYNC_PID=$!
    ALL_PIDS+=($SYNC_PID)
    echo -e "${GREEN}✓ Synchronization node started with PID ${SYNC_PID}${NC}"

    # Wait for sync to initialize
    sleep 3
    
    # Check if sync process is still running
    if kill -0 $SYNC_PID 2>/dev/null; then
        echo -e "${GREEN}✓ Synchronization node is running${NC}"
    else
        echo -e "${RED}✗ Synchronization node failed to start${NC}"
        if [ "$VERBOSE" = true ]; then
            echo -e "${YELLOW}Log output:${NC}"
            tail -n 20 ${LOG_DIR}/sync.log
        else
            echo -e "${YELLOW}Use -v/--verbose to see log output${NC}"
        fi
    fi
fi

# Display active topics
echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}All requested nodes are running${NC}"
echo -e "${BLUE}================================${NC}"

# Wait a bit for topics to be registered
sleep 3

echo -e "${GREEN}Source topics:${NC}"
ros2 topic list | grep -E "/ZED_CAMERA|/gnss|/livox" | sort

if [ "$RUN_SYNC" = true ]; then
    echo -e "${GREEN}Synchronized topics:${NC}"
    ros2 topic list | grep "/synchronized" | sort
fi

# Display log directory
echo -e "${BLUE}Logs are being saved to: ${LOG_DIR}${NC}"

echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}Sensor system is running${NC}"
echo -e "${BLUE}To view logs in real-time, run in another terminal:${NC}"
echo -e "${YELLOW}tail -f ${LOG_DIR}/*.log${NC}"
echo -e "${BLUE}Press Ctrl+C to stop all nodes${NC}"
echo -e "${BLUE}================================${NC}"

# Periodically print status updates
while true; do
    sleep 30
    
    # Count active topics
    source_count=$(ros2 topic list | grep -E "/ZED_CAMERA|/gnss|/livox" | wc -l)
    
    if [ "$RUN_SYNC" = true ]; then
        sync_count=$(ros2 topic list | grep "/synchronized" | wc -l)
        echo -e "${GREEN}$(date +'%H:%M:%S')${NC} - System running - ${source_count} source topics, ${sync_count} synchronized topics"
    else
        echo -e "${GREEN}$(date +'%H:%M:%S')${NC} - System running - ${source_count} source topics"
    fi
done