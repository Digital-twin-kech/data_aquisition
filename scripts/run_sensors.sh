#!/bin/bash

# Script to run GNSS and camera nodes

# Set the directory of the script as the working directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Display banner
echo "========================================================"
echo "      Data Acquisition - Sensors Launcher Script        "
echo "========================================================"
echo "Launching ZED cameras (ZED2i, ZEDX0, ZEDX1) and GNSS"
echo "========================================================"

# Source ROS environment if not already sourced
if [[ -z "$ROS_DISTRO" ]]; then
    echo "ROS environment not sourced. Sourcing now..."
    source /opt/ros/humble/setup.bash
fi

# Check if the workspace is built and source it
if [[ -f "$PROJECT_DIR/install/setup.bash" ]]; then
    echo "Sourcing local workspace..."
    source "$PROJECT_DIR/install/setup.bash"
else
    echo "Local workspace not built. Building now..."
    cd "$PROJECT_DIR"
    colcon build --symlink-install
    source "$PROJECT_DIR/install/setup.bash"
fi

# Run the launch file
echo "Starting all sensors..."
ros2 launch data_aquisition all_sensors_launch.py

exit 0