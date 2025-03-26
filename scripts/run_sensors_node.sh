#!/bin/bash

# Comprehensive script to run all sensor nodes (cameras, GNSS, and LiDAR)
# This script handles launching all sensors and provides detailed logging

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
LIDAR_IP="192.168.1.100"
LOG_LEVEL="info"
VERBOSE=false
ALL_PIDS=()

# Function to cleanup on exit
cleanup() {
    echo -e "${YELLOW}Shutting down all sensor nodes...${NC}"
    for pid in "${ALL_PIDS[@]}"; do
        if kill -0 $pid 2>/dev/null; then
            echo -e "Stopping process with PID $pid"
            kill -SIGINT $pid 2>/dev/null
            sleep 1
            # Force kill if still running
            kill -9 $pid 2>/dev/null
        fi
    done
    echo -e "${GREEN}All sensor nodes stopped${NC}"
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
    echo "  --lidar-ip IP        Set LiDAR IP address (default: 192.168.1.100)"
    echo "  --log-level LEVEL    Set log level (debug, info, warn, error) (default: info)"
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
        --lidar-ip)
            LIDAR_IP="$2"
            shift 2
            ;;
        --log-level)
            LOG_LEVEL="$2"
            shift 2
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
echo -e "${BLUE}  SENSOR NODES LAUNCHER         ${NC}"
echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}Running cameras: ${NC}${RUN_CAMERAS}"
echo -e "${BLUE}Running GNSS:    ${NC}${RUN_GNSS}"
echo -e "${BLUE}Running LiDAR:   ${NC}${RUN_LIDAR} (IP: ${LIDAR_IP})"
echo -e "${BLUE}Log level:       ${NC}${LOG_LEVEL}"
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

# Source our implementation
echo -e "${YELLOW}[2/4] Sourcing our implementation...${NC}"
source /home/user/Desktop/data-aquisition-digital-twin/install/setup.bash 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Our implementation sourced${NC}"
else
    echo -e "${RED}✗ Failed to source our implementation${NC}"
    echo -e "Please make sure it's built with 'colcon build'"
    exit 1
fi

# Source reference Livox implementation
if [ "$RUN_LIDAR" = true ]; then
    echo -e "${YELLOW}[3/4] Sourcing reference Livox implementation...${NC}"
    source /home/user/Desktop/instll_liv/ws_livox/install/setup.bash 2>/dev/null
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Reference Livox implementation sourced${NC}"
    else
        echo -e "${RED}✗ Failed to source reference Livox implementation${NC}"
        echo -e "Expected at: /home/user/Desktop/instll_liv/ws_livox"
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

# Create a log directory
LOG_DIR="/tmp/sensor_logs"
echo -e "${YELLOW}[4/4] Setting up log directory...${NC}"
mkdir -p $LOG_DIR
echo -e "${GREEN}✓ Log directory created: ${LOG_DIR}${NC}"

# Launch Camera Nodes
if [ "$RUN_CAMERAS" = true ]; then
    echo -e "${BLUE}================================${NC}"
    echo -e "${CYAN}Starting Camera Nodes...${NC}"
    echo -e "${BLUE}================================${NC}"
    
    # Check available ZED cameras
    echo -e "${CYAN}Detecting ZED cameras...${NC}"
    
    # ZED X0 Camera (GMSL-0)
    ZED_X0_SERIAL="40894785"
    echo -e "${CYAN}Starting ZED X0 Camera (GMSL-0, SN: ${ZED_X0_SERIAL})...${NC}"
    ros2 run data_aquisition zed_camera_node --ros-args \
        -r __ns:=/ZED_CAMERA_X0 \
        -r __node:=ZED_CAMERA_X0 \
        -p camera.serial_number:=${ZED_X0_SERIAL} \
        -p camera.model:=ZED_X \
        -p camera.resolution:=HD720 \
        -p camera.min_fps:=15.0 \
        -p camera.max_fps:=15.0 \
        --log-level ${LOG_LEVEL} > ${LOG_DIR}/camera_x0.log 2>&1 &
    CAMERA_X0_PID=$!
    ALL_PIDS+=($CAMERA_X0_PID)
    echo -e "${GREEN}✓ ZED X0 Camera started with PID ${CAMERA_X0_PID}${NC}"
    
    # ZED X1 Camera (GMSL-1)
    ZED_X1_SERIAL="41050786"
    echo -e "${CYAN}Starting ZED X1 Camera (GMSL-1, SN: ${ZED_X1_SERIAL})...${NC}"
    ros2 run data_aquisition zed_camera_node --ros-args \
        -r __ns:=/ZED_CAMERA_X1 \
        -r __node:=ZED_CAMERA_X1 \
        -p camera.serial_number:=${ZED_X1_SERIAL} \
        -p camera.model:=ZED_X \
        -p camera.resolution:=HD720 \
        -p camera.min_fps:=15.0 \
        -p camera.max_fps:=15.0 \
        --log-level ${LOG_LEVEL} > ${LOG_DIR}/camera_x1.log 2>&1 &
    CAMERA_X1_PID=$!
    ALL_PIDS+=($CAMERA_X1_PID)
    echo -e "${GREEN}✓ ZED X1 Camera started with PID ${CAMERA_X1_PID}${NC}"
    
    # ZED 2i Camera (USB)
    ZED_2I_SERIAL="37503998"
    echo -e "${CYAN}Starting ZED 2i Camera (USB, SN: ${ZED_2I_SERIAL})...${NC}"
    ros2 run data_aquisition zed_camera_node --ros-args \
        -r __ns:=/ZED_CAMERA_2i \
        -r __node:=ZED_CAMERA_2i \
        -p camera.serial_number:=${ZED_2I_SERIAL} \
        -p camera.model:=ZED2i \
        -p camera.resolution:=HD720 \
        -p camera.min_fps:=15.0 \
        -p camera.max_fps:=15.0 \
        --log-level ${LOG_LEVEL} > ${LOG_DIR}/camera_2i.log 2>&1 &
    CAMERA_2I_PID=$!
    ALL_PIDS+=($CAMERA_2I_PID)
    echo -e "${GREEN}✓ ZED 2i Camera started with PID ${CAMERA_2I_PID}${NC}"
    
    # Wait for cameras to initialize
    echo -e "${CYAN}Waiting for cameras to initialize...${NC}"
    sleep 5
    
    # Check if camera processes are still running
    for cam_pid in $CAMERA_X0_PID $CAMERA_X1_PID $CAMERA_2I_PID; do
        if kill -0 $cam_pid 2>/dev/null; then
            echo -e "${GREEN}✓ Camera process ${cam_pid} is running${NC}"
        else
            echo -e "${RED}✗ Camera process ${cam_pid} failed to start${NC}"
            if [ "$VERBOSE" = true ]; then
                echo -e "${YELLOW}Log output:${NC}"
                tail -n 20 ${LOG_DIR}/camera_*.log
            else
                echo -e "${YELLOW}Use -v/--verbose to see log output${NC}"
            fi
        fi
    done
fi

# Launch GNSS Node
if [ "$RUN_GNSS" = true ]; then
    echo -e "${BLUE}================================${NC}"
    echo -e "${MAGENTA}Starting GNSS Node...${NC}"
    echo -e "${BLUE}================================${NC}"
    
    echo -e "${MAGENTA}Checking serial port...${NC}"
    # Check if GNSS device is available
    if [ -e "/dev/ttyACM0" ]; then
        echo -e "${GREEN}✓ GNSS device found at /dev/ttyACM0${NC}"
        # Set serial port permissions if needed
        if [ ! -r "/dev/ttyACM0" ] || [ ! -w "/dev/ttyACM0" ]; then
            echo -e "${YELLOW}Setting permissions for serial port...${NC}"
            sudo chmod 666 /dev/ttyACM0
        fi
    else
        echo -e "${RED}✗ GNSS device not found at /dev/ttyACM0${NC}"
        echo -e "${YELLOW}Will try to start GNSS node anyway${NC}"
    fi
    
    # Launch GNSS node
    echo -e "${MAGENTA}Starting GNSS node...${NC}"
    ros2 run data_aquisition gnss_node --ros-args \
        -p gnss.serial_port:=/dev/ttyACM0 \
        -p gnss.baud_rate:=115200 \
        -p gnss.frequency:=10.0 \
        -p gnss.frame_id:=gnss_frame \
        -p gnss.use_rtcm_corrections:=false \
        --log-level ${LOG_LEVEL} > ${LOG_DIR}/gnss.log 2>&1 &
    GNSS_PID=$!
    ALL_PIDS+=($GNSS_PID)
    echo -e "${GREEN}✓ GNSS node started with PID ${GNSS_PID}${NC}"
    
    # Wait for GNSS to initialize
    echo -e "${MAGENTA}Waiting for GNSS to initialize...${NC}"
    sleep 3
    
    # Check if GNSS process is still running
    if kill -0 $GNSS_PID 2>/dev/null; then
        echo -e "${GREEN}✓ GNSS process is running${NC}"
    else
        echo -e "${RED}✗ GNSS process failed to start${NC}"
        if [ "$VERBOSE" = true ]; then
            echo -e "${YELLOW}Log output:${NC}"
            tail -n 20 ${LOG_DIR}/gnss.log
        else
            echo -e "${YELLOW}Use -v/--verbose to see log output${NC}"
        fi
    fi
fi

# Launch LiDAR Node
if [ "$RUN_LIDAR" = true ]; then
    echo -e "${BLUE}================================${NC}"
    echo -e "${YELLOW}Starting LiDAR Node...${NC}"
    echo -e "${BLUE}================================${NC}"
    
    # Check LiDAR connectivity
    echo -e "${YELLOW}Checking LiDAR connectivity...${NC}"
    ping -c 1 -W 1 ${LIDAR_IP} > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ LiDAR is reachable at ${LIDAR_IP}${NC}"
    else
        echo -e "${RED}✗ Cannot reach LiDAR at ${LIDAR_IP}${NC}"
        echo -e "${YELLOW}Will try to start LiDAR node anyway${NC}"
    fi
    
    # Copy config file to reference path
    CONFIG_DIR="/home/user/Desktop/instll_liv/ws_livox/src/livox_ros_driver2/config"
    if [ -d "$CONFIG_DIR" ]; then
        cp -f /home/user/Desktop/data-aquisition-digital-twin/data_aquisition/config/lidar/HAP_config.json "$CONFIG_DIR/"
        echo -e "${GREEN}✓ Configuration file copied to reference path${NC}"
    else
        echo -e "${RED}✗ Reference config directory not found${NC}"
        echo -e "${YELLOW}Will try to use default configuration${NC}"
    fi
    
    # Start the LiDAR driver
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
        -p cmdline_input_bd_code:="livox0000000001" \
        --log-level ${LOG_LEVEL} > ${LOG_DIR}/lidar.log 2>&1 &
    LIDAR_PID=$!
    ALL_PIDS+=($LIDAR_PID)
    echo -e "${GREEN}✓ LiDAR driver started with PID ${LIDAR_PID}${NC}"
    
    # Wait for LiDAR to initialize
    echo -e "${YELLOW}Waiting for LiDAR to initialize...${NC}"
    sleep 5
    
    # Check if LiDAR process is still running
    if kill -0 $LIDAR_PID 2>/dev/null; then
        echo -e "${GREEN}✓ LiDAR process is running${NC}"
    else
        echo -e "${RED}✗ LiDAR process failed to start${NC}"
        if [ "$VERBOSE" = true ]; then
            echo -e "${YELLOW}Log output:${NC}"
            tail -n 20 ${LOG_DIR}/lidar.log
        else
            echo -e "${YELLOW}Use -v/--verbose to see log output${NC}"
        fi
    fi
fi

# Check all active topics
echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}All requested sensor nodes started${NC}"
echo -e "${BLUE}================================${NC}"

echo -e "${GREEN}Active sensors:${NC}"
[ "$RUN_CAMERAS" = true ] && echo -e "  ${CYAN}● ZED Cameras${NC}"
[ "$RUN_GNSS" = true ] && echo -e "  ${MAGENTA}● GNSS${NC}"
[ "$RUN_LIDAR" = true ] && echo -e "  ${YELLOW}● LiDAR${NC}"

# Display log directory
echo -e "${BLUE}Logs are being saved to: ${LOG_DIR}${NC}"
echo -e "  ${CYAN}● Camera logs: ${LOG_DIR}/camera_*.log${NC}"
[ "$RUN_GNSS" = true ] && echo -e "  ${MAGENTA}● GNSS log: ${LOG_DIR}/gnss.log${NC}"
[ "$RUN_LIDAR" = true ] && echo -e "  ${YELLOW}● LiDAR log: ${LOG_DIR}/lidar.log${NC}"

# Display active topics
echo -e "${BLUE}Getting list of active topics...${NC}"
sleep 2
ros2 topic list | grep -E "/ZED_CAMERA|/gnss|/livox" | sort

echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}All sensor nodes are running${NC}"
echo -e "${BLUE}To view logs in real-time, run in another terminal:${NC}"
echo -e "${YELLOW}tail -f ${LOG_DIR}/*.log${NC}"
echo -e "${BLUE}Press Ctrl+C to stop all sensor nodes${NC}"
echo -e "${BLUE}================================${NC}"

# Wait until user interrupts with Ctrl+C
# This keeps the script running until user decides to stop
echo -e "${GREEN}Monitoring sensors... (Press Ctrl+C to stop)${NC}"

# Periodically print status updates
while true; do
    sleep 10
    echo -e "${GREEN}$(date +'%H:%M:%S')${NC} - All sensors running - $(ps -p $(echo ${ALL_PIDS[@]} | tr ' ' ',') -o comm= | sort | uniq | tr '\n' ' ')"
done

# The cleanup function will be called automatically by the trap handler when Ctrl+C is pressed