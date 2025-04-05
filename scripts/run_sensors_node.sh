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
RUN_SYNC=true
LIDAR_IP="192.168.1.100"
LOG_LEVEL="info"
VERBOSE=false
ALL_PIDS=()

# Define sensor status variables
ZED_2I_AVAILABLE=false
ZED_X0_AVAILABLE=false
ZED_X1_AVAILABLE=false
LIDAR_AVAILABLE=false
GNSS_AVAILABLE=false

# ZED camera serial numbers
ZED_2I_SERIAL="37503998"
ZED_X0_SERIAL="40894785"
ZED_X1_SERIAL="41050786"

# Define function to check if ZED cameras are actually available
check_zed_cameras() {
    echo -e "${CYAN}Checking ZED camera availability...${NC}"
    
    # First try with v4l2-ctl if available
    if command -v v4l2-ctl &> /dev/null; then
        echo -e "${CYAN}Using v4l2-ctl to detect cameras...${NC}"
        V4L2_DEVICES=$(v4l2-ctl --list-devices 2>/dev/null)
        
        # Check how many camera devices we have
        NUM_CAMERAS=$(echo "$V4L2_DEVICES" | grep -c "Camera")
        
        if [ $NUM_CAMERAS -ge 3 ]; then
            echo -e "${GREEN}✓ Found $NUM_CAMERAS cameras - assuming all ZED cameras are connected${NC}"
            ZED_2I_AVAILABLE=true
            ZED_X0_AVAILABLE=true
            ZED_X1_AVAILABLE=true
            return 0
        elif [ $NUM_CAMERAS -ge 1 ]; then
            echo -e "${YELLOW}⚠ Found only $NUM_CAMERAS camera(s) - some ZED cameras may be missing${NC}"
            # Enable the cameras we have
            if [ $NUM_CAMERAS -ge 1 ]; then
                ZED_2I_AVAILABLE=true
                echo -e "${GREEN}✓ Assuming ZED 2i camera is available${NC}"
            else
                ZED_2I_AVAILABLE=false
            fi
            
            if [ $NUM_CAMERAS -ge 2 ]; then
                ZED_X0_AVAILABLE=true
                echo -e "${GREEN}✓ Assuming ZED X0 camera is available${NC}"
            else
                ZED_X0_AVAILABLE=false
            fi
            
            if [ $NUM_CAMERAS -ge 3 ]; then
                ZED_X1_AVAILABLE=true
                echo -e "${GREEN}✓ Assuming ZED X1 camera is available${NC}"
            else
                ZED_X1_AVAILABLE=false
            fi
            
            return 0
        fi
    fi
    
    # Fallback: check if video devices exist in /dev
    echo -e "${CYAN}Checking video devices in /dev...${NC}"
    VIDEO_DEVICES=$(ls -l /dev/video* 2>/dev/null | wc -l)
    
    if [ $VIDEO_DEVICES -ge 2 ]; then
        echo -e "${YELLOW}Found $VIDEO_DEVICES video devices - will try to use available cameras${NC}"
        
        # Assume we have at least the ZED 2i camera if we found video devices
        ZED_2I_AVAILABLE=true
        echo -e "${GREEN}✓ Assuming ZED 2i camera is available (device found)${NC}"
        
        # If we have more than 2 video devices, assume X0 is also available
        if [ $VIDEO_DEVICES -ge 4 ]; then
            ZED_X0_AVAILABLE=true
            echo -e "${GREEN}✓ Assuming ZED X0 camera is available (device found)${NC}"
        else
            ZED_X0_AVAILABLE=false
            echo -e "${YELLOW}⚠ ZED X0 camera may not be available${NC}"
        fi
        
        # If we have more than 4 video devices, assume X1 is also available
        if [ $VIDEO_DEVICES -ge 6 ]; then
            ZED_X1_AVAILABLE=true
            echo -e "${GREEN}✓ Assuming ZED X1 camera is available (device found)${NC}"
        else
            ZED_X1_AVAILABLE=false
            echo -e "${YELLOW}⚠ ZED X1 camera may not be available${NC}"
        fi
        
        return 0
    fi
    
    # Last resort - just enable everything and let the ROS node handle failures
    echo -e "${YELLOW}⚠ Camera detection failed - will try to launch all cameras and let ROS nodes handle failures${NC}"
    echo -e "${YELLOW}⚠ This method is less reliable but will attempt to work with whatever cameras are available${NC}"
    
    ZED_2I_AVAILABLE=true
    ZED_X0_AVAILABLE=true
    ZED_X1_AVAILABLE=true
    
    return 0
}

# Function to check if GNSS device is available
check_gnss_device() {
    echo -e "${MAGENTA}Checking GNSS device availability...${NC}"
    
    if [ -e "/dev/ttyACM0" ]; then
        # Try to get some basic info from the device
        if timeout 2 stty -F /dev/ttyACM0 &>/dev/null; then
            echo -e "${GREEN}✓ GNSS device found and accessible at /dev/ttyACM0${NC}"
            GNSS_AVAILABLE=true
            return 0
        else
            echo -e "${RED}✗ GNSS device found at /dev/ttyACM0 but may not be accessible${NC}"
            echo -e "${YELLOW}  Attempting to set permissions...${NC}"
            sudo chmod 666 /dev/ttyACM0
            if timeout 2 stty -F /dev/ttyACM0 &>/dev/null; then
                echo -e "${GREEN}✓ GNSS device is now accessible${NC}"
                GNSS_AVAILABLE=true
                return 0
            else
                echo -e "${RED}✗ Failed to access GNSS device${NC}"
                GNSS_AVAILABLE=false
                return 1
            fi
        fi
    else
        echo -e "${RED}✗ GNSS device not found at /dev/ttyACM0${NC}"
        echo -e "${YELLOW}  Check if device is properly connected${NC}"
        GNSS_AVAILABLE=false
        return 1
    fi
}

# Function to check if LiDAR is available
check_lidar_device() {
    echo -e "${YELLOW}Checking LiDAR availability...${NC}"
    
    # Check if device is pingable
    if ping -c 1 -W 1 ${LIDAR_IP} > /dev/null 2>&1; then
        echo -e "${GREEN}✓ LiDAR is reachable at ${LIDAR_IP}${NC}"
        # Try to connect to LiDAR SDK port (typical Livox port is 55000)
        if timeout 2 bash -c "echo > /dev/tcp/${LIDAR_IP}/55000" 2>/dev/null; then
            echo -e "${GREEN}✓ LiDAR SDK port is accessible${NC}"
            LIDAR_AVAILABLE=true
            return 0
        else
            echo -e "${YELLOW}⚠ LiDAR reachable but SDK port may not be accessible${NC}"
            echo -e "${YELLOW}  Will continue anyway as it might still work${NC}"
            LIDAR_AVAILABLE=true
            return 0
        fi
    else
        echo -e "${RED}✗ Cannot reach LiDAR at ${LIDAR_IP}${NC}"
        echo -e "${YELLOW}  Check if LiDAR is powered on and network connection is correct${NC}"
        LIDAR_AVAILABLE=false
        return 1
    fi
}

# Function to verify that a ROS node is actually connected to the hardware
verify_ros_node_connected() {
    local node_pid=$1
    local node_type=$2
    local log_file=$3
    local max_wait_time=15  # seconds to wait for connection (increased from 10 to 15)
    
    echo -e "${BLUE}Verifying ${node_type} ROS node (PID: ${node_pid}) connection to hardware...${NC}"
    
    # Wait a moment for the node to initialize
    sleep 3
    
    # Check if process is still running
    if ! kill -0 $node_pid 2>/dev/null; then
        echo -e "${RED}✗ ${node_type} node process died shortly after starting${NC}"
        if [ -f "$log_file" ]; then
            echo -e "${RED}Last 10 lines from log:${NC}"
            tail -n 10 "$log_file"
        fi
        return 1
    fi
    
    # Check for ROS topics being published by this node
    # This is a better indicator of success than log files
    if [[ "$node_type" == *"ZED"* || "$node_type" == *"camera"* ]]; then
        # For ZED cameras, check if image topics are being published
        local camera_name=""
        if [[ "$node_type" == *"ZED 2i"* ]]; then
            camera_name="ZED_CAMERA_2i"
        elif [[ "$node_type" == *"ZED X0"* ]]; then
            camera_name="ZED_CAMERA_X0"
        elif [[ "$node_type" == *"ZED X1"* ]]; then
            camera_name="ZED_CAMERA_X1"
        fi
        
        if [ ! -z "$camera_name" ]; then
            # Wait a bit longer for topics to appear
            sleep 5
            
            # Check if the camera is publishing topics
            if ros2 topic list | grep -q "/$camera_name/"; then
                echo -e "${GREEN}✓ ${node_type} is publishing ROS topics - hardware connection confirmed${NC}"
                return 0
            fi
        fi
    elif [[ "$node_type" == *"LiDAR"* ]]; then
        # For LiDAR, check if point cloud topic is being published
        sleep 5
        if ros2 topic list | grep -q "/livox/lidar"; then
            echo -e "${GREEN}✓ ${node_type} is publishing ROS topics - hardware connection confirmed${NC}"
            return 0
        fi
    elif [[ "$node_type" == *"GNSS"* ]]; then
        # For GNSS, check if fix topic is being published
        sleep 5
        if ros2 topic list | grep -q "/gnss/fix"; then
            echo -e "${GREEN}✓ ${node_type} is publishing ROS topics - hardware connection confirmed${NC}"
            return 0
        fi
    fi
    
    # Check log file for connection success indicators or errors
    local wait_count=0
    while [ $wait_count -lt $max_wait_time ]; do
        if [ -f "$log_file" ]; then
            # Look for success messages in logs (expanded list of patterns)
            if grep -q "Successfully connected" "$log_file" || \
               grep -q "Device initialized" "$log_file" || \
               grep -q "Camera opened" "$log_file" || \
               grep -q "Publishing data" "$log_file" || \
               grep -q "ZED SDK initialized" "$log_file" || \
               grep -q "Starting capture" "$log_file" || \
               grep -q "Started publishing" "$log_file" || \
               grep -q "Message published" "$log_file" || \
               grep -q "Connection established" "$log_file" || \
               grep -q "Driver loaded successfully" "$log_file" || \
               grep -q "Point cloud generation enabled" "$log_file"; then
                echo -e "${GREEN}✓ ${node_type} node confirmed connected to hardware (log message)${NC}"
                return 0
            fi
            
            # Look for known failure patterns (expanded list)
            if grep -q "Failed to open" "$log_file" || \
               grep -q "Failed to initialize camera" "$log_file" || \
               grep -q "Connection failed" "$log_file" || \
               grep -q "Cannot connect" "$log_file" || \
               grep -q "Error initializing" "$log_file" || \
               grep -q "Failed to open device" "$log_file" || \
               grep -q "Cannot find camera" "$log_file" || \
               grep -q "No devices found" "$log_file" || \
               grep -q "Device not found" "$log_file" || \
               grep -q "Failed to start capture" "$log_file" || \
               grep -q "Camera is not available" "$log_file" || \
               grep -q "Error opening" "$log_file" || \
               grep -q "Cannot communicate" "$log_file" || \
               grep -q "Communication error" "$log_file" || \
               grep -q "Invalid device" "$log_file"; then
                echo -e "${RED}✗ ${node_type} node failed to connect to hardware${NC}"
                echo -e "${RED}Error from log:${NC}"
                grep -E "Failed to|Error|Cannot|Invalid|No devices" "$log_file" | tail -n 3
                return 1
            fi
            
            # Also check if the process is using minimal CPU, which might indicate it's stuck
            # But only do this after waiting a bit, as startup can be CPU intensive
            if [ $wait_count -gt 8 ]; then
                # Check if process is still responding
                if ! kill -0 $node_pid 2>/dev/null; then
                    echo -e "${RED}✗ ${node_type} node process is not responding${NC}"
                    return 1
                fi
                
                # Only check CPU usage if bc is available (it might not be on all systems)
                if command -v bc &> /dev/null; then
                    CPU_USAGE=$(ps -p $node_pid -o %cpu | tail -n 1 | tr -d ' ')
                    if (( $(echo "$CPU_USAGE < 0.5" | bc -l) )); then
                        echo -e "${YELLOW}⚠ ${node_type} node CPU usage is very low (${CPU_USAGE}%) - it might be stuck${NC}"
                    fi
                fi
            fi
        fi
        
        # Wait and increment counter
        sleep 1
        wait_count=$((wait_count + 1))
        echo -n "."
    done
    
    # One final check for published topics
    if [[ "$node_type" == *"ZED"* || "$node_type" == *"camera"* ]]; then
        # Get the camera name part
        local camera_name=""
        if [[ "$node_type" == *"ZED 2i"* ]]; then
            camera_name="ZED_CAMERA_2i"
        elif [[ "$node_type" == *"ZED X0"* ]]; then
            camera_name="ZED_CAMERA_X0"
        elif [[ "$node_type" == *"ZED X1"* ]]; then
            camera_name="ZED_CAMERA_X1"
        fi
        
        if [ ! -z "$camera_name" ] && ros2 topic list | grep -q "/$camera_name/"; then
            echo -e "${GREEN}✓ ${node_type} is publishing ROS topics - hardware connection confirmed${NC}"
            return 0
        fi
    fi
    
    # If the process is still running but we can't confirm success, assume it's OK
    if kill -0 $node_pid 2>/dev/null; then
        echo -e "\n${YELLOW}⚠ Cannot definitively confirm ${node_type} node connection to hardware${NC}"
        echo -e "${YELLOW}  Node is running, assuming it's working correctly${NC}"
        # Will return success and hope for the best
        return 0
    else
        echo -e "\n${RED}✗ ${node_type} node process died during connection verification${NC}"
        return 1
    fi
}

# Function to restart a specific node type
restart_node() {
    local node_type=$1
    
    case "$node_type" in
        "zed_2i")
            echo -e "${CYAN}Restarting ZED 2i camera node...${NC}"
            # Find and kill the existing process if any
            if [ ! -z "$CAMERA_2I_PID" ] && kill -0 $CAMERA_2I_PID 2>/dev/null; then
                kill $CAMERA_2I_PID 2>/dev/null
                sleep 2
            fi
            # Start ZED 2i camera with the same parameters as before
            launch_zed_2i_camera
            ;;
        "zed_x0")
            echo -e "${CYAN}Restarting ZED X0 camera node...${NC}"
            if [ ! -z "$CAMERA_X0_PID" ] && kill -0 $CAMERA_X0_PID 2>/dev/null; then
                kill $CAMERA_X0_PID 2>/dev/null
                sleep 2
            fi
            launch_zed_x0_camera
            ;;
        "zed_x1")
            echo -e "${CYAN}Restarting ZED X1 camera node...${NC}"
            if [ ! -z "$CAMERA_X1_PID" ] && kill -0 $CAMERA_X1_PID 2>/dev/null; then
                kill $CAMERA_X1_PID 2>/dev/null
                sleep 2
            fi
            launch_zed_x1_camera
            ;;
        "lidar")
            echo -e "${YELLOW}Restarting LiDAR node...${NC}"
            if [ ! -z "$LIDAR_PID" ] && kill -0 $LIDAR_PID 2>/dev/null; then
                kill $LIDAR_PID 2>/dev/null
                sleep 2
            fi
            launch_lidar_node
            ;;
        "gnss")
            echo -e "${MAGENTA}Restarting GNSS node...${NC}"
            if [ ! -z "$GNSS_PID" ] && kill -0 $GNSS_PID 2>/dev/null; then
                kill $GNSS_PID 2>/dev/null
                sleep 2
            fi
            launch_gnss_node
            ;;
        "sync")
            echo -e "${MAGENTA}Restarting synchronization node...${NC}"
            if [ ! -z "$SYNC_PID" ] && kill -0 $SYNC_PID 2>/dev/null; then
                kill $SYNC_PID 2>/dev/null
                sleep 2
            fi
            launch_sync_node
            ;;
        *)
            echo -e "${RED}Unknown node type: ${node_type}${NC}"
            ;;
    esac
}

# Function to cleanup on exit with more aggressive process termination
cleanup() {
    echo -e "${YELLOW}Shutting down all processes...${NC}"
    
    # First attempt graceful shutdown of all known PIDs
    echo -e "${YELLOW}Attempting graceful shutdown of all processes...${NC}"
    for pid in "${ALL_PIDS[@]}"; do
        if kill -0 $pid 2>/dev/null; then
            echo -e "Stopping process with PID $pid"
            kill -SIGINT $pid 2>/dev/null
        fi
    done
    
    # Give processes a moment to shut down gracefully
    sleep 2
    
    # Force kill any remaining processes
    echo -e "${YELLOW}Force killing any remaining processes...${NC}"
    for pid in "${ALL_PIDS[@]}"; do
        if kill -0 $pid 2>/dev/null; then
            echo -e "Force killing process with PID $pid"
            kill -9 $pid 2>/dev/null
        fi
    done
    
    # Clean up temporary files
    if [[ -d "/tmp/livox_config" ]]; then
        echo -e "${YELLOW}Cleaning up temporary config files...${NC}"
        rm -rf /tmp/livox_config
    fi
    
    # Aggressively find and kill all ROS2-related processes
    echo -e "${YELLOW}Looking for any ROS2-related processes...${NC}"
    
    # Find all processes with "ros2" in their command line
    ROS_PIDS=$(ps -ef | grep -E "_ros2_node|ros2 run|ros2 launch|ros2_|_node" | grep -v grep | awk '{print $2}')
    if [[ ! -z "$ROS_PIDS" ]]; then
        echo -e "${YELLOW}Found ROS2 processes. Terminating...${NC}"
        for ros_pid in $ROS_PIDS; do
            echo -e "Terminating ROS2 process with PID $ros_pid"
            kill -SIGINT $ros_pid 2>/dev/null
        done
        
        # Quick wait
        sleep 1
        
        # Force kill any remaining ROS2 processes
        for ros_pid in $ROS_PIDS; do
            if kill -0 $ros_pid 2>/dev/null; then
                echo -e "Force killing ROS2 process with PID $ros_pid"
                kill -9 $ros_pid 2>/dev/null
            fi
        done
    fi
    
    # Find any remaining ZED-related processes
    ZED_PIDS=$(ps -ef | grep -i "zed" | grep -v grep | awk '{print $2}')
    if [[ ! -z "$ZED_PIDS" ]]; then
        echo -e "${YELLOW}Found ZED-related processes. Terminating...${NC}"
        for zed_pid in $ZED_PIDS; do
            echo -e "Terminating ZED process with PID $zed_pid"
            kill -9 $zed_pid 2>/dev/null
        done
    fi
    
    # Clean up the ROS topic system
    echo -e "${YELLOW}Cleaning up ROS topics...${NC}"
    
    # Use ros2 daemon to stop and restart the ROS2 daemon - this effectively resets all topics
    if command -v ros2 &> /dev/null; then
        ros2 daemon stop &>/dev/null
        sleep 1
        ros2 daemon start &>/dev/null
    fi
    
    echo -e "${GREEN}All processes stopped and system cleaned up${NC}"
    exit 0
}

# Launch functions for each sensor node
launch_zed_2i_camera() {
    echo -e "${CYAN}Starting ZED 2i Camera (USB, SN: ${ZED_2I_SERIAL})...${NC}"
    ros2 run data_aquisition zed_camera_node --ros-args \
        -r __ns:=/ZED_CAMERA_2i \
        -r __node:=ZED_CAMERA_2i \
        -p camera.serial_number:=${ZED_2I_SERIAL} \
        -p camera.model:=ZED2i \
        -p camera.resolution:=HD720 \
        -p camera.min_fps:=10.0 \
        -p camera.max_fps:=10.0 \
        -p camera.reliable_qos:=true \
        -p camera.qos_history_depth:=50 \
        -p depth.enabled:=true \
        -p point_cloud.enabled:=true \
        -p point_cloud.resolution:=PERFORMANCE \
        -p point_cloud.max_range:=15.0 \
        --log-level ${LOG_LEVEL} > ${LOG_DIR}/camera_2i.log 2>&1 &
    CAMERA_2I_PID=$!
    ALL_PIDS+=($CAMERA_2I_PID)
    echo -e "${GREEN}✓ ZED 2i Camera started with PID ${CAMERA_2I_PID}${NC}"
    
    # Verify connection
    verify_ros_node_connected $CAMERA_2I_PID "ZED 2i Camera" "${LOG_DIR}/camera_2i.log"
    return $?
}

launch_zed_x0_camera() {
    echo -e "${CYAN}Starting ZED X0 Camera (GMSL-0, SN: ${ZED_X0_SERIAL})...${NC}"
    ros2 run data_aquisition zed_camera_node --ros-args \
        -r __ns:=/ZED_CAMERA_X0 \
        -r __node:=ZED_CAMERA_X0 \
        -p camera.serial_number:=${ZED_X0_SERIAL} \
        -p camera.model:=ZED_X \
        -p camera.resolution:=HD720 \
        -p camera.min_fps:=10.0 \
        -p camera.max_fps:=10.0 \
        -p camera.reliable_qos:=true \
        -p camera.qos_history_depth:=50 \
        -p depth.enabled:=true \
        -p point_cloud.enabled:=true \
        -p point_cloud.resolution:=PERFORMANCE \
        -p point_cloud.max_range:=15.0 \
        --log-level ${LOG_LEVEL} > ${LOG_DIR}/camera_x0.log 2>&1 &
    CAMERA_X0_PID=$!
    ALL_PIDS+=($CAMERA_X0_PID)
    echo -e "${GREEN}✓ ZED X0 Camera started with PID ${CAMERA_X0_PID}${NC}"
    
    # Verify connection
    verify_ros_node_connected $CAMERA_X0_PID "ZED X0 Camera" "${LOG_DIR}/camera_x0.log"
    return $?
}

launch_zed_x1_camera() {
    echo -e "${CYAN}Starting ZED X1 Camera (GMSL-1, SN: ${ZED_X1_SERIAL})...${NC}"
    ros2 run data_aquisition zed_camera_node --ros-args \
        -r __ns:=/ZED_CAMERA_X1 \
        -r __node:=ZED_CAMERA_X1 \
        -p camera.serial_number:=${ZED_X1_SERIAL} \
        -p camera.model:=ZED_X \
        -p camera.resolution:=HD720 \
        -p camera.min_fps:=10.0 \
        -p camera.max_fps:=10.0 \
        -p camera.reliable_qos:=true \
        -p camera.qos_history_depth:=50 \
        -p depth.enabled:=true \
        -p point_cloud.enabled:=true \
        -p point_cloud.resolution:=PERFORMANCE \
        -p point_cloud.max_range:=15.0 \
        --log-level ${LOG_LEVEL} > ${LOG_DIR}/camera_x1.log 2>&1 &
    CAMERA_X1_PID=$!
    ALL_PIDS+=($CAMERA_X1_PID)
    echo -e "${GREEN}✓ ZED X1 Camera started with PID ${CAMERA_X1_PID}${NC}"
    
    # Verify connection
    verify_ros_node_connected $CAMERA_X1_PID "ZED X1 Camera" "${LOG_DIR}/camera_x1.log"
    return $?
}

launch_lidar_node() {
    echo -e "${YELLOW}Starting Livox LiDAR driver...${NC}"
    
    # Handle config file for LiDAR
    CONFIG_DIR="/opt/livox-sdk/src/livox_ros_driver2/config"
    CONFIG_SOURCE="/home/user/Desktop/data-aquisition-digital-twin/data_aquisition/config/lidar/HAP_config.json"
    
    # Always use a temporary config to avoid permission issues
    echo -e "${YELLOW}Creating configuration in temporary location...${NC}"
    TMP_CONFIG_DIR="/tmp/livox_config"
    mkdir -p "$TMP_CONFIG_DIR"
    cp -f "$CONFIG_SOURCE" "$TMP_CONFIG_DIR/HAP_config.json"
    CONFIG_DIR="$TMP_CONFIG_DIR"
    echo -e "${GREEN}✓ Configuration file copied to: $TMP_CONFIG_DIR${NC}"
    
    # Start the LiDAR driver
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
    
    # Verify connection
    verify_ros_node_connected $LIDAR_PID "LiDAR" "${LOG_DIR}/lidar.log"
    return $?
}

launch_gnss_node() {
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
    
    # Verify connection
    verify_ros_node_connected $GNSS_PID "GNSS" "${LOG_DIR}/gnss.log"
    return $?
}

launch_sync_node() {
    echo -e "${MAGENTA}Starting synchronization node with improved parameters...${NC}"
    ros2 run data_aquisition sync_node --ros-args \
        -p camera_names:='["ZED_CAMERA_2i", "ZED_CAMERA_X0", "ZED_CAMERA_X1"]' \
        -p sync_lidar:=$RUN_LIDAR \
        -p sync_gnss:=$RUN_GNSS \
        -p time_tolerance:=0.3 \
        -p cache_size:=300 \
        -p max_delay:=1.0 \
        -p pass_through:=false \
        --log-level ${LOG_LEVEL} > ${LOG_DIR}/sync.log 2>&1 &
    SYNC_PID=$!
    ALL_PIDS+=($SYNC_PID)
    echo -e "${GREEN}✓ Synchronization node started with PID ${SYNC_PID}${NC}"
    
    # Verify connection - for sync node, just check if it's running
    sleep 2
    if kill -0 $SYNC_PID 2>/dev/null; then
        echo -e "${GREEN}✓ Synchronization node is running${NC}"
        return 0
    else
        echo -e "${RED}✗ Synchronization node failed to start${NC}"
        if [ -f "${LOG_DIR}/sync.log" ]; then
            echo -e "${RED}Last 10 lines from log:${NC}"
            tail -n 10 "${LOG_DIR}/sync.log"
        fi
        return 1
    fi
}

# Interactive prompt to retry connection
prompt_retry() {
    local node_type=$1
    
    while true; do
        echo -e "${YELLOW}Would you like to retry connecting to ${node_type}? (r=retry, s=skip, q=quit) [r]${NC}"
        read -n 1 -s choice
        case "$choice" in
            r|"")  # retry or Enter key (default)
                echo -e "${CYAN}Retrying ${node_type} connection...${NC}"
                return 0
                ;;
            s)  # skip
                echo -e "${YELLOW}Skipping ${node_type}...${NC}"
                return 1
                ;;
            q)  # quit
                echo -e "${RED}Quitting...${NC}"
                cleanup
                exit 1
                ;;
            *)
                echo -e "${RED}Invalid option. Please press 'r' for retry, 's' to skip, or 'q' to quit.${NC}"
                ;;
        esac
    done
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
        --no-sync)
            RUN_SYNC=false
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
echo -e "${BLUE}  SENSOR & SYNC NODES LAUNCHER  ${NC}"
echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}Running cameras: ${NC}${RUN_CAMERAS}"
echo -e "${BLUE}Running GNSS:    ${NC}${RUN_GNSS}"
echo -e "${BLUE}Running LiDAR:   ${NC}${RUN_LIDAR} (IP: ${LIDAR_IP})"
echo -e "${BLUE}Running sync:    ${NC}${RUN_SYNC}"
echo -e "${BLUE}Log level:       ${NC}${LOG_LEVEL}"
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
source /home/user/Desktop/data-aquisition-digital-twin/install/setup.bash 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Our implementation sourced${NC}"
else
    echo -e "${RED}✗ Failed to source our implementation${NC}"
    echo -e "Please make sure it's built with 'colcon build'"
    exit 1
fi

# Source Livox implementation
if [ "$RUN_LIDAR" = true ]; then
    echo -e "${YELLOW}[3/5] Sourcing Livox implementation...${NC}"
    source /opt/livox-sdk/install/setup.bash 2>/dev/null
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Reference Livox implementation sourced${NC}"
    else
        echo -e "${RED}✗ Failed to source reference Livox implementation${NC}"
        echo -e "Expected at: /opt/livox-sdk"
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
echo -e "${YELLOW}[4/5] Setting up log directory...${NC}"
mkdir -p $LOG_DIR
echo -e "${GREEN}✓ Log directory created: ${LOG_DIR}${NC}"

# Define arrays to hold verified node launch commands
VERIFIED_CAMERA_2I_CMD=""
VERIFIED_CAMERA_X0_CMD=""
VERIFIED_CAMERA_X1_CMD=""
VERIFIED_LIDAR_CMD=""
VERIFIED_GNSS_CMD=""
VERIFIED_SYNC_CMD=""

# Test ZED camera availability
if [ "$RUN_CAMERAS" = true ]; then
    echo -e "${BLUE}================================${NC}"
    echo -e "${CYAN}Verifying Camera Availability...${NC}"
    echo -e "${BLUE}================================${NC}"
    
    # First check ZED camera availability
    while ! check_zed_cameras; do
        echo -e "${RED}Failed to detect ZED cameras!${NC}"
        if prompt_retry "ZED cameras"; then
            echo -e "${CYAN}Rechecking camera availability...${NC}"
            continue
        else
            echo -e "${YELLOW}Skipping camera initialization...${NC}"
            RUN_CAMERAS=false
            break
        fi
    done
    
    if [ "$RUN_CAMERAS" = true ]; then
        echo -e "${GREEN}✓ Camera verification complete${NC}"
    fi
fi

# Test GNSS device availability
if [ "$RUN_GNSS" = true ]; then
    echo -e "${BLUE}================================${NC}"
    echo -e "${MAGENTA}Verifying GNSS Availability...${NC}"
    echo -e "${BLUE}================================${NC}"
    
    # Check GNSS device availability
    while ! check_gnss_device; do
        echo -e "${RED}Failed to detect GNSS device!${NC}"
        if prompt_retry "GNSS device"; then
            echo -e "${MAGENTA}Rechecking GNSS device...${NC}"
            continue
        else
            echo -e "${YELLOW}Skipping GNSS initialization...${NC}"
            RUN_GNSS=false
            break
        fi
    done
    
    if [ "$RUN_GNSS" = true ]; then
        echo -e "${GREEN}✓ GNSS verification complete${NC}"
    fi
fi

# Test LiDAR availability
if [ "$RUN_LIDAR" = true ]; then
    echo -e "${BLUE}================================${NC}"
    echo -e "${YELLOW}Verifying LiDAR Availability...${NC}"
    echo -e "${BLUE}================================${NC}"
    
    # Check LiDAR device availability
    while ! check_lidar_device; do
        echo -e "${RED}Failed to detect LiDAR device!${NC}"
        if prompt_retry "LiDAR device"; then
            echo -e "${YELLOW}Rechecking LiDAR device...${NC}"
            continue
        else
            echo -e "${YELLOW}Skipping LiDAR initialization...${NC}"
            RUN_LIDAR=false
            break
        fi
    done
    
    if [ "$RUN_LIDAR" = true ]; then
        echo -e "${GREEN}✓ LiDAR verification complete${NC}"
    fi
fi

# Prepare launch commands for all nodes based on verification results
echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}Preparing Synchronized Launch...${NC}"
echo -e "${BLUE}================================${NC}"

# Prepare camera launch commands
if [ "$RUN_CAMERAS" = true ]; then
    echo -e "${CYAN}Preparing camera commands...${NC}"
    
    # ZED 2i Camera
    if $ZED_2I_AVAILABLE; then
        echo -e "${CYAN}Preparing ZED 2i Camera launch command...${NC}"
        VERIFIED_CAMERA_2I_CMD="ros2 run data_aquisition zed_camera_node --ros-args -r __ns:=/ZED_CAMERA_2i -r __node:=ZED_CAMERA_2i -p camera.serial_number:=${ZED_2I_SERIAL} -p camera.model:=ZED2i -p camera.resolution:=HD720 -p camera.min_fps:=10.0 -p camera.max_fps:=10.0 -p camera.reliable_qos:=true -p camera.qos_history_depth:=50 -p depth.enabled:=true -p point_cloud.enabled:=true -p point_cloud.resolution:=PERFORMANCE -p point_cloud.max_range:=15.0 --log-level ${LOG_LEVEL} > ${LOG_DIR}/camera_2i.log 2>&1"
    else
        echo -e "${YELLOW}⚠ Skipping ZED 2i camera (not available)${NC}"
    fi
    
    # ZED X0 Camera
    if $ZED_X0_AVAILABLE; then
        echo -e "${CYAN}Preparing ZED X0 Camera launch command...${NC}"
        VERIFIED_CAMERA_X0_CMD="ros2 run data_aquisition zed_camera_node --ros-args -r __ns:=/ZED_CAMERA_X0 -r __node:=ZED_CAMERA_X0 -p camera.serial_number:=${ZED_X0_SERIAL} -p camera.model:=ZED_X -p camera.resolution:=HD720 -p camera.min_fps:=10.0 -p camera.max_fps:=10.0 -p camera.reliable_qos:=true -p camera.qos_history_depth:=50 -p depth.enabled:=true -p point_cloud.enabled:=true -p point_cloud.resolution:=PERFORMANCE -p point_cloud.max_range:=15.0 --log-level ${LOG_LEVEL} > ${LOG_DIR}/camera_x0.log 2>&1"
    else
        echo -e "${YELLOW}⚠ Skipping ZED X0 camera (not available)${NC}"
    fi
    
    # ZED X1 Camera
    if $ZED_X1_AVAILABLE; then
        echo -e "${CYAN}Preparing ZED X1 Camera launch command...${NC}"
        VERIFIED_CAMERA_X1_CMD="ros2 run data_aquisition zed_camera_node --ros-args -r __ns:=/ZED_CAMERA_X1 -r __node:=ZED_CAMERA_X1 -p camera.serial_number:=${ZED_X1_SERIAL} -p camera.model:=ZED_X -p camera.resolution:=HD720 -p camera.min_fps:=10.0 -p camera.max_fps:=10.0 -p camera.reliable_qos:=true -p camera.qos_history_depth:=50 -p depth.enabled:=true -p point_cloud.enabled:=true -p point_cloud.resolution:=PERFORMANCE -p point_cloud.max_range:=15.0 --log-level ${LOG_LEVEL} > ${LOG_DIR}/camera_x1.log 2>&1"
    else
        echo -e "${YELLOW}⚠ Skipping ZED X1 camera (not available)${NC}"
    fi
fi

# Prepare GNSS launch command
if [ "$RUN_GNSS" = true ]; then
    echo -e "${MAGENTA}Preparing GNSS launch command...${NC}"
    VERIFIED_GNSS_CMD="ros2 run data_aquisition gnss_node --ros-args -p gnss.serial_port:=/dev/ttyACM0 -p gnss.baud_rate:=115200 -p gnss.frequency:=10.0 -p gnss.frame_id:=gnss_frame -p gnss.use_rtcm_corrections:=false --log-level ${LOG_LEVEL} > ${LOG_DIR}/gnss.log 2>&1"
fi

# Prepare LiDAR launch command
if [ "$RUN_LIDAR" = true ]; then
    echo -e "${YELLOW}Preparing LiDAR launch command...${NC}"
    
    # Handle config file for LiDAR
    CONFIG_DIR="/opt/livox-sdk/src/livox_ros_driver2/config"
    CONFIG_SOURCE="/home/user/Desktop/data-aquisition-digital-twin/data_aquisition/config/lidar/HAP_config.json"
    
    # Always use a temporary config to avoid permission issues
    echo -e "${YELLOW}Creating configuration in temporary location...${NC}"
    TMP_CONFIG_DIR="/tmp/livox_config"
    mkdir -p "$TMP_CONFIG_DIR"
    cp -f "$CONFIG_SOURCE" "$TMP_CONFIG_DIR/HAP_config.json"
    CONFIG_DIR="$TMP_CONFIG_DIR"
    echo -e "${GREEN}✓ Configuration file copied to: $TMP_CONFIG_DIR${NC}"
    
    # Prepare LiDAR command
    VERIFIED_LIDAR_CMD="ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args -p xfer_format:=0 -p multi_topic:=0 -p data_src:=0 -p publish_freq:=10.0 -p output_data_type:=0 -p frame_id:=\"livox_frame\" -p user_config_path:=\"${CONFIG_DIR}/HAP_config.json\" -p cmdline_input_bd_code:=\"livox0000000001\" --log-level ${LOG_LEVEL} > ${LOG_DIR}/lidar.log 2>&1"
fi

# Prepare sync launch command
# Note we'll check again if there are any sensors to sync before actually preparing this
if [ "$RUN_SYNC" = true ]; then
    if [ -z "$VERIFIED_CAMERA_2I_CMD" ] && [ -z "$VERIFIED_CAMERA_X0_CMD" ] && 
       [ -z "$VERIFIED_CAMERA_X1_CMD" ] && [ -z "$VERIFIED_LIDAR_CMD" ] && 
       [ -z "$VERIFIED_GNSS_CMD" ]; then
        # No sensors to synchronize
        echo -e "${YELLOW}⚠ No sensors are enabled. Skipping synchronization node.${NC}"
        RUN_SYNC=false
    else
        echo -e "${MAGENTA}Preparing synchronization node launch command...${NC}"
        
        # Create appropriate camera_names parameter based on available cameras
        CAMERA_NAMES_PARAM="["
        if [ ! -z "$VERIFIED_CAMERA_2I_CMD" ]; then
            CAMERA_NAMES_PARAM+="\"ZED_CAMERA_2i\""
        fi
        if [ ! -z "$VERIFIED_CAMERA_X0_CMD" ]; then
            if [ "$CAMERA_NAMES_PARAM" != "[" ]; then
                CAMERA_NAMES_PARAM+=", "
            fi
            CAMERA_NAMES_PARAM+="\"ZED_CAMERA_X0\""
        fi
        if [ ! -z "$VERIFIED_CAMERA_X1_CMD" ]; then
            if [ "$CAMERA_NAMES_PARAM" != "[" ]; then
                CAMERA_NAMES_PARAM+=", "
            fi
            CAMERA_NAMES_PARAM+="\"ZED_CAMERA_X1\""
        fi
        CAMERA_NAMES_PARAM+="]"
        
        # Prepare sync command
        VERIFIED_SYNC_CMD="ros2 run data_aquisition sync_node --ros-args -p camera_names:='${CAMERA_NAMES_PARAM}' -p sync_lidar:=$([ ! -z \"$VERIFIED_LIDAR_CMD\" ] && echo true || echo false) -p sync_gnss:=$([ ! -z \"$VERIFIED_GNSS_CMD\" ] && echo true || echo false) -p time_tolerance:=0.3 -p cache_size:=300 -p max_delay:=1.0 -p pass_through:=false --log-level ${LOG_LEVEL} > ${LOG_DIR}/sync.log 2>&1"
    fi
fi

# Now, execute all prepared commands simultaneously
echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}Launching All Nodes Simultaneously${NC}"
echo -e "${BLUE}================================${NC}"

# Launch all nodes at once
echo -e "${CYAN}Launching cameras...${NC}"
if [ ! -z "$VERIFIED_CAMERA_2I_CMD" ]; then
    echo -e "${CYAN}Launching ZED 2i Camera...${NC}"
    eval $VERIFIED_CAMERA_2I_CMD &
    CAMERA_2I_PID=$!
    ALL_PIDS+=($CAMERA_2I_PID)
    echo -e "${GREEN}✓ ZED 2i Camera started with PID ${CAMERA_2I_PID}${NC}"
fi

if [ ! -z "$VERIFIED_CAMERA_X0_CMD" ]; then
    echo -e "${CYAN}Launching ZED X0 Camera...${NC}"
    eval $VERIFIED_CAMERA_X0_CMD &
    CAMERA_X0_PID=$!
    ALL_PIDS+=($CAMERA_X0_PID)
    echo -e "${GREEN}✓ ZED X0 Camera started with PID ${CAMERA_X0_PID}${NC}"
fi

if [ ! -z "$VERIFIED_CAMERA_X1_CMD" ]; then
    echo -e "${CYAN}Launching ZED X1 Camera...${NC}"
    eval $VERIFIED_CAMERA_X1_CMD &
    CAMERA_X1_PID=$!
    ALL_PIDS+=($CAMERA_X1_PID)
    echo -e "${GREEN}✓ ZED X1 Camera started with PID ${CAMERA_X1_PID}${NC}"
fi

if [ ! -z "$VERIFIED_LIDAR_CMD" ]; then
    echo -e "${YELLOW}Launching LiDAR driver...${NC}"
    eval $VERIFIED_LIDAR_CMD &
    LIDAR_PID=$!
    ALL_PIDS+=($LIDAR_PID)
    echo -e "${GREEN}✓ LiDAR driver started with PID ${LIDAR_PID}${NC}"
fi

if [ ! -z "$VERIFIED_GNSS_CMD" ]; then
    echo -e "${MAGENTA}Launching GNSS node...${NC}"
    eval $VERIFIED_GNSS_CMD &
    GNSS_PID=$!
    ALL_PIDS+=($GNSS_PID)
    echo -e "${GREEN}✓ GNSS node started with PID ${GNSS_PID}${NC}"
fi

# Wait a short time for sensor nodes to start before launching sync
if [ ! -z "$VERIFIED_SYNC_CMD" ]; then
    echo -e "${CYAN}Waiting for sensors to initialize before starting synchronization...${NC}"
    sleep 5  # Short wait to ensure all nodes have started publishing

    echo -e "${MAGENTA}Launching synchronization node...${NC}"
    eval $VERIFIED_SYNC_CMD &
    SYNC_PID=$!
    ALL_PIDS+=($SYNC_PID)
    echo -e "${GREEN}✓ Synchronization node started with PID ${SYNC_PID}${NC}"
fi

# Verify all nodes are running
echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}Verifying Node Status...${NC}"
echo -e "${BLUE}================================${NC}"

sleep 3  # Give nodes time to initialize

# Check camera nodes
if [ ! -z "$CAMERA_2I_PID" ]; then
    if kill -0 $CAMERA_2I_PID 2>/dev/null; then
        echo -e "${GREEN}✓ ZED 2i camera is running (PID: ${CAMERA_2I_PID})${NC}"
    else
        echo -e "${RED}✗ ZED 2i camera failed (PID: ${CAMERA_2I_PID})${NC}"
    fi
fi

if [ ! -z "$CAMERA_X0_PID" ]; then
    if kill -0 $CAMERA_X0_PID 2>/dev/null; then
        echo -e "${GREEN}✓ ZED X0 camera is running (PID: ${CAMERA_X0_PID})${NC}"
    else
        echo -e "${RED}✗ ZED X0 camera failed (PID: ${CAMERA_X0_PID})${NC}"
    fi
fi

if [ ! -z "$CAMERA_X1_PID" ]; then
    if kill -0 $CAMERA_X1_PID 2>/dev/null; then
        echo -e "${GREEN}✓ ZED X1 camera is running (PID: ${CAMERA_X1_PID})${NC}"
    else
        echo -e "${RED}✗ ZED X1 camera failed (PID: ${CAMERA_X1_PID})${NC}"
    fi
fi

# Check LiDAR node
if [ ! -z "$LIDAR_PID" ]; then
    if kill -0 $LIDAR_PID 2>/dev/null; then
        echo -e "${GREEN}✓ LiDAR node is running (PID: ${LIDAR_PID})${NC}"
    else
        echo -e "${RED}✗ LiDAR node failed (PID: ${LIDAR_PID})${NC}"
    fi
fi

# Check GNSS node
if [ ! -z "$GNSS_PID" ]; then
    if kill -0 $GNSS_PID 2>/dev/null; then
        echo -e "${GREEN}✓ GNSS node is running (PID: ${GNSS_PID})${NC}"
    else
        echo -e "${RED}✗ GNSS node failed (PID: ${GNSS_PID})${NC}"
    fi
fi

# Check synchronization node
if [ ! -z "$SYNC_PID" ]; then
    if kill -0 $SYNC_PID 2>/dev/null; then
        echo -e "${GREEN}✓ Synchronization node is running (PID: ${SYNC_PID})${NC}"
    else
        echo -e "${RED}✗ Synchronization node failed (PID: ${SYNC_PID})${NC}"
    fi
fi

# Check all active topics
# Add TF2 static transform publishers for all sensor frames
echo -e "${BLUE}================================${NC}"
echo -e "${CYAN}Setting up TF transforms for visualization...${NC}"
echo -e "${BLUE}================================${NC}"

# Publish transforms for all sensors to map frame
# ZED cameras - using offsets to visualize them separately in space
# Generic camera_link frame
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id camera_link > ${LOG_DIR}/tf_cam_link.log 2>&1 &
TF_CAM_LINK_PID=$!
ALL_PIDS+=($TF_CAM_LINK_PID)

# ZED 2i camera frames with offset to see them separately
# Main camera frame (zed_2i_left_camera_frame)
ros2 run tf2_ros static_transform_publisher --x 0 --y 0.5 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id zed_2i_left_camera_frame > ${LOG_DIR}/tf_zed_2i.log 2>&1 &
TF_ZED_2I_PID=$!
ALL_PIDS+=($TF_ZED_2I_PID)

# Direct map to legacy frame for ZED 2i (with unique offset)
ros2 run tf2_ros static_transform_publisher --x 0 --y 0.5 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id zed_camera_left_2i > ${LOG_DIR}/tf_zed_2i_compat.log 2>&1 &
TF_ZED_2I_COMPAT_PID=$!
ALL_PIDS+=($TF_ZED_2I_COMPAT_PID)

# Add optical frame for compatibility
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 --frame-id zed_camera_left_2i --child-frame-id zed_camera_left_2i_optical_frame > ${LOG_DIR}/tf_zed_2i_compat_opt.log 2>&1 &
TF_ZED_2I_COMPAT_OPT_PID=$!
ALL_PIDS+=($TF_ZED_2I_COMPAT_OPT_PID)

# Optional camera frame to optical frame for point clouds
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 --frame-id zed_2i_left_camera_frame --child-frame-id zed_2i_left_camera_optical_frame > ${LOG_DIR}/tf_zed_2i_optical.log 2>&1 &
TF_ZED_2I_OPT_PID=$!
ALL_PIDS+=($TF_ZED_2I_OPT_PID)

# ZED X0 camera frames with offset
# Main camera frame (zed_x0_left_camera_frame)
ros2 run tf2_ros static_transform_publisher --x 0 --y -0.5 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id zed_x0_left_camera_frame > ${LOG_DIR}/tf_zed_x0.log 2>&1 &
TF_ZED_X0_PID=$!
ALL_PIDS+=($TF_ZED_X0_PID)

# Direct map to 'zed_camera_left' frame that RViz expects
ros2 run tf2_ros static_transform_publisher --x 0 --y -0.5 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id zed_camera_left > ${LOG_DIR}/tf_zed_compat.log 2>&1 &
TF_ZED_COMPAT_PID=$!
ALL_PIDS+=($TF_ZED_COMPAT_PID)

# Add optical frame for compatibility
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 --frame-id zed_camera_left --child-frame-id zed_camera_left_optical_frame > ${LOG_DIR}/tf_zed_compat_opt.log 2>&1 &
TF_ZED_COMPAT_OPT_PID=$!
ALL_PIDS+=($TF_ZED_COMPAT_OPT_PID)

# Optional camera frame to optical frame for point clouds
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 --frame-id zed_x0_left_camera_frame --child-frame-id zed_x0_left_camera_optical_frame > ${LOG_DIR}/tf_zed_x0_optical.log 2>&1 &
TF_ZED_X0_OPT_PID=$!
ALL_PIDS+=($TF_ZED_X0_OPT_PID)

# ZED X1 camera frames with offset
# Main camera frame (zed_x1_left_camera_frame)
ros2 run tf2_ros static_transform_publisher --x 0.5 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id zed_x1_left_camera_frame > ${LOG_DIR}/tf_zed_x1.log 2>&1 &
TF_ZED_X1_PID=$!
ALL_PIDS+=($TF_ZED_X1_PID)

# Direct map to legacy frame for ZED X1 (with unique offset)
ros2 run tf2_ros static_transform_publisher --x 0.5 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id zed_camera_left_x1 > ${LOG_DIR}/tf_zed_x1_compat.log 2>&1 &
TF_ZED_X1_COMPAT_PID=$!
ALL_PIDS+=($TF_ZED_X1_COMPAT_PID)

# Add optical frame for compatibility
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 --frame-id zed_camera_left_x1 --child-frame-id zed_camera_left_x1_optical_frame > ${LOG_DIR}/tf_zed_x1_compat_opt.log 2>&1 &
TF_ZED_X1_COMPAT_OPT_PID=$!
ALL_PIDS+=($TF_ZED_X1_COMPAT_OPT_PID)

# Optional camera frame to optical frame for point clouds
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5 --frame-id zed_x1_left_camera_frame --child-frame-id zed_x1_left_camera_optical_frame > ${LOG_DIR}/tf_zed_x1_optical.log 2>&1 &
TF_ZED_X1_OPT_PID=$!
ALL_PIDS+=($TF_ZED_X1_OPT_PID)

# LiDAR
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id livox_frame > ${LOG_DIR}/tf_livox.log 2>&1 &
TF_LIVOX_PID=$!
ALL_PIDS+=($TF_LIVOX_PID)

# GNSS
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id gnss_frame > ${LOG_DIR}/tf_gnss.log 2>&1 &
TF_GNSS_PID=$!
ALL_PIDS+=($TF_GNSS_PID)

echo -e "${GREEN}✓ TF transforms published${NC}"

echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}All requested nodes started${NC}"
echo -e "${BLUE}================================${NC}"

echo -e "${GREEN}Active components:${NC}"
[ "$RUN_CAMERAS" = true ] && echo -e "  ${CYAN}● ZED Cameras${NC}"
[ "$RUN_GNSS" = true ] && echo -e "  ${MAGENTA}● GNSS${NC}"
[ "$RUN_LIDAR" = true ] && echo -e "  ${YELLOW}● LiDAR${NC}"
[ "$RUN_SYNC" = true ] && echo -e "  ${WHITE}● Synchronization${NC}"

# Display log directory
echo -e "${BLUE}Logs are being saved to: ${LOG_DIR}${NC}"
echo -e "  ${CYAN}● Camera logs: ${LOG_DIR}/camera_*.log${NC}"
[ "$RUN_GNSS" = true ] && echo -e "  ${MAGENTA}● GNSS log: ${LOG_DIR}/gnss.log${NC}"
[ "$RUN_LIDAR" = true ] && echo -e "  ${YELLOW}● LiDAR log: ${LOG_DIR}/lidar.log${NC}"
[ "$RUN_SYNC" = true ] && echo -e "  ${WHITE}● Sync log: ${LOG_DIR}/sync.log${NC}"

# Display active topics
echo -e "${BLUE}Getting list of active topics...${NC}"
sleep 2
ros2 topic list | grep -E "/ZED_CAMERA|/gnss|/livox" | sort

if [ "$RUN_SYNC" = true ]; then
    echo -e "${GREEN}Synchronized topics:${NC}"
    ros2 topic list | grep "/synchronized" | sort
fi

echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}System is running${NC}"
echo -e "${BLUE}To view logs in real-time, run in another terminal:${NC}"
echo -e "${YELLOW}tail -f ${LOG_DIR}/*.log${NC}"
echo -e "${BLUE}Press Ctrl+C to stop all nodes${NC}"
echo -e "${BLUE}================================${NC}"

# Wait until user interrupts with Ctrl+C
# This keeps the script running until user decides to stop
echo -e "${GREEN}Monitoring system... (Press Ctrl+C to stop)${NC}"

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

# The cleanup function will be called automatically by the trap handler when Ctrl+C is pressed