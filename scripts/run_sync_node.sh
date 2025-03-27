#!/bin/bash

# Script to launch just the sensor synchronization node

# Define colors for better readability
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}Starting Sensor Synchronization Node...${NC}"

# Default parameters
SYNC_POLICY="ApproximateTime"
TIME_TOLERANCE="0.10"
CACHE_SIZE="100"
LOG_LEVEL="info"
PASS_THROUGH="true"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  key="$1"
  case $key in
    -p|--policy)
      SYNC_POLICY="$2"
      shift
      shift
      ;;
    -t|--tolerance)
      TIME_TOLERANCE="$2"
      shift
      shift
      ;;
    -c|--cache)
      CACHE_SIZE="$2"
      shift
      shift
      ;;
    -d|--debug)
      LOG_LEVEL="debug"
      shift
      ;;
    -n|--no-pass-through)
      PASS_THROUGH="false"
      shift
      ;;
    -h|--help)
      echo -e "${YELLOW}Usage: $0 [OPTIONS]${NC}"
      echo -e "  -p, --policy      Synchronization policy (ExactTime/ApproximateTime)"
      echo -e "  -t, --tolerance   Time tolerance in seconds (default: 0.10)"
      echo -e "  -c, --cache       Cache size (default: 100)"
      echo -e "  -d, --debug       Enable debug logging"
      echo -e "  -n, --no-pass-through  Disable direct pass-through (default: enabled)"
      echo -e "  -h, --help        Show this help message"
      exit 0
      ;;
    *)
      echo -e "${RED}Unknown option: $key${NC}"
      exit 1
      ;;
  esac
done

echo -e "${YELLOW}Configuration:${NC}"
echo -e "  Sync Policy:    ${SYNC_POLICY}"
echo -e "  Time Tolerance: ${TIME_TOLERANCE} s"
echo -e "  Cache Size:     ${CACHE_SIZE}"
echo -e "  Pass Through:   ${PASS_THROUGH}"
echo -e "  Log Level:      ${LOG_LEVEL}"

# Launch the synchronization node using ROS2 launch
ros2 launch data_aquisition sync_launch.py \
  sync_policy:=${SYNC_POLICY} \
  time_tolerance:=${TIME_TOLERANCE} \
  cache_size:=${CACHE_SIZE} \
  pass_through:=${PASS_THROUGH} \
  --log-level sensor_synchronizer:=${LOG_LEVEL}

exit 0