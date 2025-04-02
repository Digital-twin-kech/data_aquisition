#!/bin/bash

# Script to run all cameras and view their point clouds in RViz2
# with properly configured transforms

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}Sourcing ROS2 and workspace...${NC}"
    source /opt/ros/humble/setup.bash
    source /home/user/Desktop/data-aquisition-digital-twin/install/setup.bash
fi

# Create a temporary directory for logs
LOG_DIR="/tmp/camera_cloud_logs"
mkdir -p $LOG_DIR

# Create a temporary directory for the RViz config
echo -e "${BLUE}Creating RViz configuration...${NC}"

# Define camera namespaces
CAMERA_1="ZED_CAMERA_X0"
CAMERA_2="ZED_CAMERA_X1"
CAMERA_3="ZED_CAMERA_2i"

# Create a temporary RViz configuration
RVIZ_CONFIG_FILE="/tmp/camera_clouds.rviz"
cat > $RVIZ_CONFIG_FILE << EOF
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /TF1/Frames1
        - /Camera 1 PointCloud1
        - /Camera 2 PointCloud1
        - /Camera 3 PointCloud1
      Splitter Ratio: 0.5
    Tree Height: 569
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
        All Enabled: true
        camera_link:
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
      Name: Camera 1 PointCloud
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 2
      Size (m): 0.009999999776482582
      Style: Points
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /${CAMERA_1}/point_cloud/cloud_registered
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
      Name: Camera 2 PointCloud
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 2
      Size (m): 0.009999999776482582
      Style: Points
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /${CAMERA_2}/point_cloud/cloud_registered
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
      Name: Camera 3 PointCloud
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 2
      Size (m): 0.009999999776482582
      Style: Points
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /${CAMERA_3}/point_cloud/cloud_registered
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
      Distance: 5
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
    Saved: ~
EOF

# Print a message about the script
echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}Camera Point Cloud Visualizer${NC}"
echo -e "${BLUE}================================${NC}"
echo -e "This script will:"
echo -e "1. Start ZED cameras with proper configuration"
echo -e "2. Publish necessary TF transforms"
echo -e "3. Launch RViz2 with the right configuration"
echo -e "${BLUE}================================${NC}"

# Start cameras
echo -e "${YELLOW}Starting cameras...${NC}"

# ZED X0 Camera (GMSL-0)
ZED_X0_SERIAL="40894785"
echo -e "${GREEN}Starting ZED X0 Camera (GMSL-0, SN: ${ZED_X0_SERIAL})...${NC}"
ros2 run data_aquisition zed_camera_node --ros-args \
    -r __ns:=/${CAMERA_1} \
    -r __node:=${CAMERA_1} \
    -p camera.serial_number:=${ZED_X0_SERIAL} \
    -p camera.model:=ZED_X \
    -p camera.resolution:=HD720 \
    -p camera.min_fps:=15.0 \
    -p camera.max_fps:=15.0 \
    -p depth.enabled:=true \
    -p point_cloud.enabled:=true \
    --log-level info > ${LOG_DIR}/camera_x0.log 2>&1 &
CAMERA_X0_PID=$!

# ZED X1 Camera (GMSL-1)
ZED_X1_SERIAL="41050786"
echo -e "${GREEN}Starting ZED X1 Camera (GMSL-1, SN: ${ZED_X1_SERIAL})...${NC}"
ros2 run data_aquisition zed_camera_node --ros-args \
    -r __ns:=/${CAMERA_2} \
    -r __node:=${CAMERA_2} \
    -p camera.serial_number:=${ZED_X1_SERIAL} \
    -p camera.model:=ZED_X \
    -p camera.resolution:=HD720 \
    -p camera.min_fps:=15.0 \
    -p camera.max_fps:=15.0 \
    -p depth.enabled:=true \
    -p point_cloud.enabled:=true \
    --log-level info > ${LOG_DIR}/camera_x1.log 2>&1 &
CAMERA_X1_PID=$!

# ZED 2i Camera (USB)
ZED_2I_SERIAL="37503998"
echo -e "${GREEN}Starting ZED 2i Camera (USB, SN: ${ZED_2I_SERIAL})...${NC}"
ros2 run data_aquisition zed_camera_node --ros-args \
    -r __ns:=/${CAMERA_3} \
    -r __node:=${CAMERA_3} \
    -p camera.serial_number:=${ZED_2I_SERIAL} \
    -p camera.model:=ZED2i \
    -p camera.resolution:=HD720 \
    -p camera.min_fps:=15.0 \
    -p camera.max_fps:=15.0 \
    -p depth.enabled:=true \
    -p point_cloud.enabled:=true \
    --log-level info > ${LOG_DIR}/camera_2i.log 2>&1 &
CAMERA_2I_PID=$!

# Wait for cameras to initialize
echo -e "${YELLOW}Waiting for cameras to initialize...${NC}"
sleep 5

# Start TF publishers
echo -e "${YELLOW}Publishing TF transforms...${NC}"

# Generic camera_link frame
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id camera_link > ${LOG_DIR}/tf_cam_link.log 2>&1 &
TF_CAM_LINK_PID=$!

# Camera-specific frames with offsets to visualize them separately in space
ros2 run tf2_ros static_transform_publisher --x 0 --y -0.5 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id zed_x0_camera_link > ${LOG_DIR}/tf_zed_x0.log 2>&1 &
TF_ZED_X0_PID=$!

ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id zed_x1_camera_link > ${LOG_DIR}/tf_zed_x1.log 2>&1 &
TF_ZED_X1_PID=$!

ros2 run tf2_ros static_transform_publisher --x 0 --y 0.5 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id zed_2i_camera_link > ${LOG_DIR}/tf_zed_2i.log 2>&1 &
TF_ZED_2I_PID=$!

echo -e "${GREEN}All TF transforms published${NC}"

# Start RViz2
echo -e "${YELLOW}Starting RViz2...${NC}"
echo -e "${BLUE}Look for the three camera point clouds in different positions${NC}"
ros2 run rviz2 rviz2 -d $RVIZ_CONFIG_FILE

# Clean up when RViz2 exits
echo -e "${YELLOW}Cleaning up...${NC}"

# Kill all processes
kill $CAMERA_X0_PID $CAMERA_X1_PID $CAMERA_2I_PID $TF_CAM_LINK_PID $TF_ZED_X0_PID $TF_ZED_X1_PID $TF_ZED_2I_PID 2>/dev/null

echo -e "${GREEN}Done${NC}"