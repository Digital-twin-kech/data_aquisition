#!/bin/bash

# Script to visualize Livox LiDAR point clouds with proper TF setup
# This is a standalone script that doesn't require sensors to be running previously

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default values
TOPIC_TYPE="synchronized" # or "raw"
POINT_SIZE=2

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --topic-type)
            TOPIC_TYPE="$2"
            shift 2
            ;;
        --point-size)
            POINT_SIZE="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--topic-type raw|synchronized] [--point-size SIZE]"
            exit 1
            ;;
    esac
done

# Construct topic path based on topic type
if [ "$TOPIC_TYPE" == "synchronized" ]; then
    TOPIC="/${TOPIC_TYPE}/livox/lidar"
else
    TOPIC="/livox/lidar"
fi

# Frame ID is always livox_frame
FRAME_ID="livox_frame"

echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}Livox LiDAR Point Cloud Visualizer${NC}"
echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}Topic:${NC} $TOPIC"
echo -e "${GREEN}Frame:${NC} $FRAME_ID"
echo -e "${BLUE}================================${NC}"

# Check if we're already sourced in ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}Sourcing ROS2 environment...${NC}"
    source /opt/ros/humble/setup.bash
    source /home/user/Desktop/data-aquisition-digital-twin/install/setup.bash
fi

# Start TF publisher to connect lidar frame to map
echo -e "${YELLOW}Publishing TF transform...${NC}"
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id "$FRAME_ID" &
TF_PID=$!

# Create a temporary RViz configuration optimized for LiDAR point cloud
TEMP_RVIZ_CONFIG="/tmp/livox_pointcloud_viewer.rviz"
cat > "$TEMP_RVIZ_CONFIG" << EOF
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /PointCloud21/Status1
      Splitter Ratio: 0.5
    Tree Height: 719
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 255; 255
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 255
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: PointCloud2
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): $POINT_SIZE
      Size (m): 0.009999999776482582
      Style: Points
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: $TOPIC
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.785398006439209
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.785398006439209
EOF

# Launch RViz2 with the custom configuration
echo -e "${GREEN}Launching RViz2 for point cloud visualization...${NC}"
ros2 run rviz2 rviz2 -d "$TEMP_RVIZ_CONFIG"

# When RViz2 exits, kill the TF publisher
echo -e "${YELLOW}Shutting down TF publisher...${NC}"
kill -SIGINT $TF_PID 2>/dev/null
echo -e "${GREEN}Done.${NC}"