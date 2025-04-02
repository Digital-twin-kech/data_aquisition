#!/bin/bash

# Script to launch all sensors, synchronization, and visualize point clouds in RViz2
# This is a wrapper around run_sensors_node.sh that also launches RViz2

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if we're already sourced in ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}Sourcing ROS2 environment...${NC}"
    source /opt/ros/humble/setup.bash
    source /home/user/Desktop/data-aquisition-digital-twin/install/setup.bash
fi

# Start sensors in the background
echo -e "${BLUE}Starting all sensors and synchronization...${NC}"
# Pass through all arguments to the run_sensors_node.sh script
./scripts/run_sensors_node.sh "$@" &

# Wait a bit for sensors to start
echo -e "${YELLOW}Waiting for sensors to start...${NC}"
sleep 10

# Create a temporary RViz configuration
TEMP_RVIZ_CONFIG="/tmp/pointcloud_viewer.rviz"
cat > "$TEMP_RVIZ_CONFIG" << EOF
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /TF1/Frames1
        - /Camera PointCloud1
        - /LiDAR PointCloud1
      Splitter Ratio: 0.5
    Tree Height: 617
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
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: false
        camera_link:
          Value: true
        gnss_frame:
          Value: true
        livox_frame:
          Value: true
        map:
          Value: true
        zed_2i_camera_link:
          Value: true
        zed_x0_camera_link:
          Value: true
        zed_x1_camera_link:
          Value: true
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree:
        map:
          camera_link:
            {}
          gnss_frame:
            {}
          livox_frame:
            {}
          zed_2i_camera_link:
            {}
          zed_x0_camera_link:
            {}
          zed_x1_camera_link:
            {}
      Update Interval: 0
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: rgb
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 255; 255
      Color Transformer: RGB8
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: Camera PointCloud
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 2
      Size (m): 0.009999999776482582
      Style: Points
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /synchronized/ZED_CAMERA_X0/point_cloud/cloud_registered
      Use Fixed Frame: true
      Use rainbow: true
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
      Name: LiDAR PointCloud
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 2
      Size (m): 0.009999999776482582
      Style: Points
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /synchronized/livox/lidar
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
ros2 run rviz2 rviz2 -d $TEMP_RVIZ_CONFIG

# When RViz2 exits, kill the sensors
echo -e "${YELLOW}RViz2 has closed. Stopping all sensors...${NC}"
pkill -P $(pgrep -f "run_sensors_node.sh")
echo -e "${GREEN}Done.${NC}"