#!/bin/bash

# Script to run synchronized recording using ROS2 bags
# This launches both the synchronization node and rosbag recorder

# Default values
OUTPUT_DIR="/home/user/rosbags"
AUTO_RECORD="true" 
SYNC_POLICY="ApproximateTime"
TIME_TOLERANCE="0.10"
PASS_THROUGH="true"

# Display help message
function show_help {
  echo "Usage: $0 [options]"
  echo
  echo "Options:"
  echo "  -o, --output-dir DIR    Directory to store ROS2 bags (default: $OUTPUT_DIR)"
  echo "  -n, --no-auto-record    Don't automatically start recording (must use ROS2 lifecycle commands)"
  echo "  -e, --exact-time        Use ExactTime synchronization policy (stricter)"
  echo "  -t, --tolerance VALUE   Time tolerance for synchronization in seconds (default: $TIME_TOLERANCE)"
  echo "  -p, --no-pass-through   Disable pass-through mode (only synchronized messages will be published)"
  echo "  -h, --help              Display this help message"
  echo
  echo "Example:"
  echo "  $0 -o /data/recordings -e -t 0.05"
  echo
  echo "This will synchronize and record all sensor data with high precision (ExactTime, 50ms tolerance)"
  echo "and store the ROS2 bags in /data/recordings"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  key="$1"
  case $key in
    -o|--output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    -n|--no-auto-record)
      AUTO_RECORD="false"
      shift
      ;;
    -e|--exact-time)
      SYNC_POLICY="ExactTime"
      shift
      ;;
    -t|--tolerance)
      TIME_TOLERANCE="$2"
      shift 2
      ;;
    -p|--no-pass-through)
      PASS_THROUGH="false"
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

# Ensure output directory exists
mkdir -p "$OUTPUT_DIR"

# Source ROS2 setup
source /opt/ros/humble/setup.bash
source ~/Desktop/data-aquisition-digital-twin/data_aquisition/install/setup.bash

# Launch the synchronized recording
echo "Starting synchronized recording with:"
echo "  Output directory: $OUTPUT_DIR"
echo "  Sync policy: $SYNC_POLICY"
echo "  Time tolerance: $TIME_TOLERANCE"
echo "  Pass through: $PASS_THROUGH"
echo "  Auto record: $AUTO_RECORD"
echo

ros2 launch data_aquisition sync_with_recorder_launch.py \
  output_directory:=$OUTPUT_DIR \
  auto_record:=$AUTO_RECORD \
  sync_policy:=$SYNC_POLICY \
  time_tolerance:=$TIME_TOLERANCE \
  pass_through:=$PASS_THROUGH