#!/bin/bash

# Simple script to run the rosbag recorder node

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source ~/Desktop/data-aquisition-digital-twin/data_aquisition/install/setup.bash

# Launch the recorder
ros2 launch data_aquisition data_recorder_only_launch.py "$@"