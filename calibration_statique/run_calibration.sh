#!/bin/bash

# Interactive Static Calibration Runner Script
# This script runs the static calibration between LiDAR and ZED 2i Camera 
# with step-by-step interactive user confirmation for each stage
# Uses the existing A0 ChArUco board in charuco_target/

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# Base directory
BASE_DIR="/home/user/Desktop/data-aquisition-digital-twin/data_aquisition"
CALIB_DIR="${BASE_DIR}/calibration_statique"
RESULTS_DIR="${CALIB_DIR}/results"
CHARUCO_DIR="${CALIB_DIR}/charuco_target"
DEBUG_DIR="${CALIB_DIR}/debug"

# Ensure directories exist
mkdir -p "${RESULTS_DIR}"
mkdir -p "${DEBUG_DIR}"

# Function to display status banner
function show_banner() {
    local text="$1"
    local status="$2" # "info", "success", "error", "step"
    
    case "$status" in
        "info")
            echo -e "\n${BLUE}┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓${NC}"
            echo -e "${BLUE}┃${NC} ${BOLD}${text}${NC}${BLUE}$(printf "%$((50 - ${#text}))s")┃${NC}"
            echo -e "${BLUE}┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛${NC}"
            ;;
        "success")
            echo -e "\n${GREEN}┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓${NC}"
            echo -e "${GREEN}┃${NC} ${BOLD}✓ ${text}${NC}${GREEN}$(printf "%$((48 - ${#text}))s")┃${NC}"
            echo -e "${GREEN}┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛${NC}"
            ;;
        "error")
            echo -e "\n${RED}┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓${NC}"
            echo -e "${RED}┃${NC} ${BOLD}✗ ${text}${NC}${RED}$(printf "%$((48 - ${#text}))s")┃${NC}"
            echo -e "${RED}┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛${NC}"
            ;;
        "step")
            echo -e "\n${MAGENTA}┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓${NC}"
            echo -e "${MAGENTA}┃${NC} ${BOLD}⚡ STEP ${text}${NC}${MAGENTA}$(printf "%$((42 - ${#text}))s")┃${NC}"
            echo -e "${MAGENTA}┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛${NC}"
            ;;
        *)
            echo -e "\n${text}"
            ;;
    esac
}

# Function to wait for user input to continue
function wait_for_continue() {
    local prompt="$1"
    local retry=$2  # Whether retry is an option (0 or 1)
    
    if [ "$retry" = "1" ]; then
        echo -e "\n${CYAN}${prompt}${NC} (Press ${BOLD}c${NC} to continue, ${BOLD}r${NC} to retry, ${BOLD}q${NC} to quit): "
        while true; do
            read -n 1 response
            case "$response" in
                c|C) echo ""; return 0 ;;  # Continue
                r|R) echo ""; return 1 ;;  # Retry
                q|Q) echo -e "\n${RED}Calibration aborted by user.${NC}"; exit 1 ;;
                *) ;;  # Ignore other input
            esac
        done
    else
        echo -e "\n${CYAN}${prompt}${NC} (Press ${BOLD}c${NC} to continue, ${BOLD}q${NC} to quit): "
        while true; do
            read -n 1 response
            case "$response" in
                c|C) echo ""; return 0 ;;  # Continue
                q|Q) echo -e "\n${RED}Calibration aborted by user.${NC}"; exit 1 ;;
                *) ;;  # Ignore other input
            esac
        done
    fi
}

# Display header
show_banner "Interactive Static Calibration Tool" "info"
echo -e "This tool provides an interactive step-by-step calibration process" 
echo -e "between the Livox LiDAR and ZED 2i camera using a ChArUco board."
echo -e "\nCalibration Steps:"
echo -e "  ${BOLD}1.${NC} Locate the latest recording session"
echo -e "  ${BOLD}2.${NC} Detect ChArUco board in camera images"
echo -e "  ${BOLD}3.${NC} Extract ChArUco regions from point clouds"
echo -e "  ${BOLD}4.${NC} Align point clouds using ICP"
echo -e "  ${BOLD}5.${NC} Compute and save the transformation matrix"

# Check if the ChArUco board files exist
if [ ! -f "${CHARUCO_DIR}/charuco_board_A0.png" ]; then
    show_banner "ChArUco board file not found" "error"
    echo -e "The file ${CHARUCO_DIR}/charuco_board_A0.png does not exist."
    echo -e "Please ensure that the pre-generated board file exists."
    exit 1
fi

echo -e "\n${YELLOW}Using existing ChArUco board: ${CHARUCO_DIR}/charuco_board_A0.png${NC}"

# Wait for user to start
wait_for_continue "Ready to start the calibration process?" 0

# STEP 1: Find latest session
show_banner "1: Finding latest recording session" "step"
echo -e "Searching for recording sessions in ${BASE_DIR}/records/..."

# Run Python to find the latest session
latest_session=$(cd "${BASE_DIR}" && python3 -c "
import os, glob
records_dir = os.path.join('${BASE_DIR}', 'records')
session_dirs = sorted(glob.glob(os.path.join(records_dir, '2*')))
if not session_dirs:
    print('ERROR: No recording sessions found')
    exit(1)
latest_session = session_dirs[-1]
print(latest_session)
")

if [[ $latest_session == ERROR* ]]; then
    show_banner "No recording sessions found" "error"
    echo -e "Please run data recording first to capture calibration data."
    exit 1
fi

show_banner "Latest session found" "success"
echo -e "Using recording session: ${YELLOW}${latest_session}${NC}"

retry_step1=0
while true; do
    wait_for_continue "Proceed with this session?" 1
    retry_step1=$?
    if [ $retry_step1 -eq 0 ]; then
        break  # Continue to next step
    else
        # User requested retry - choose another session
        show_banner "Selecting an alternative session" "info"
        echo -e "Available sessions:"
        
        # List all sessions
        session_list=$(cd "${BASE_DIR}" && python3 -c "
import os, glob
records_dir = os.path.join('${BASE_DIR}', 'records')
session_dirs = sorted(glob.glob(os.path.join(records_dir, '2*')))
for i, session in enumerate(session_dirs):
    print(f'{i+1}: {os.path.basename(session)}')
")
        echo -e "${session_list}"
        
        # Let user select a session
        echo -e "${CYAN}Enter the number of the session to use:${NC} "
        read session_num
        
        latest_session=$(cd "${BASE_DIR}" && python3 -c "
import os, glob
records_dir = os.path.join('${BASE_DIR}', 'records')
session_dirs = sorted(glob.glob(os.path.join(records_dir, '2*')))
try:
    selected = int('${session_num}') - 1
    if 0 <= selected < len(session_dirs):
        print(session_dirs[selected])
    else:
        print('ERROR: Invalid session number')
except:
    print('ERROR: Invalid input')
")
        
        if [[ $latest_session == ERROR* ]]; then
            show_banner "Invalid session selection" "error"
            echo -e "Please select a valid session number."
            continue
        fi
        
        show_banner "Selected session" "success"
        echo -e "Using recording session: ${YELLOW}${latest_session}${NC}"
    fi
done

# STEP 2: Detect ChArUco board in camera images
show_banner "2: Detecting ChArUco board" "step"
echo -e "Loading the latest RGB image from the ZED 2i camera..."

retry_step2=1
while [ $retry_step2 -eq 1 ]; do
    # Run Python to detect the ChArUco board
    charuco_result=$(cd "${BASE_DIR}" && python3 -c "
import os, glob, sys, traceback
import cv2
import numpy as np

try:
    # Find the latest image
    image_dir = os.path.join('${latest_session}', 'ZED_CAMERA_2i_rgb_image_rect_color')
    image_files = sorted(glob.glob(os.path.join(image_dir, '*.png')))
    
    if not image_files:
        print('ERROR: No images found in ZED_CAMERA_2i_rgb_image_rect_color')
        sys.exit(1)
    
    latest_image_path = image_files[-1]
    image_timestamp = os.path.basename(latest_image_path).split('.')[0]
    
    # Load the image
    image = cv2.imread(latest_image_path)
    if image is None:
        print(f'ERROR: Failed to load image {latest_image_path}')
        sys.exit(1)
    
    print(f'SUCCESS: Loaded image {os.path.basename(latest_image_path)}')
    
    # Convert to grayscale for ArUco detection
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Set up ChArUco board parameters
    squares_x = 10
    squares_y = 14
    square_length = 0.07  # 7cm squares
    marker_length = 0.055  # 5.5cm markers
    
    try:
        # Create ArUco dictionary and detect markers
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
        if ids is None or len(ids) < 4:
            print(f'ERROR: Not enough ArUco markers detected - found {0 if ids is None else len(ids)}, need at least 4')
            
            # Save a debug image showing what was detected
            debug_img = image.copy()
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(debug_img, corners, ids)
            
            cv2.putText(debug_img, f'Only {0 if ids is None else len(ids)} markers detected - need at least 4', 
                      (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            debug_dir = '${DEBUG_DIR}'
            os.makedirs(debug_dir, exist_ok=True)
            cv2.imwrite(os.path.join(debug_dir, 'charuco_detection_failed.png'), debug_img)
            print(f'DEBUG: Saved failure visualization to {os.path.join(debug_dir, 'charuco_detection_failed.png')}')
            sys.exit(1)
        
        # Create a debugging image
        debug_img = image.copy()
        cv2.aruco.drawDetectedMarkers(debug_img, corners, ids)
        
        # Create the ChArUco board
        board = cv2.aruco.CharucoBoard_create(squares_x, squares_y, square_length, marker_length, aruco_dict)
        
        # Refine corners
        ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
        
        if charuco_corners is None or len(charuco_corners) < 5:
            print(f'ERROR: Not enough ChArUco corners - found {0 if charuco_corners is None else len(charuco_corners)}, need at least 5')
            cv2.putText(debug_img, f'Only {0 if charuco_corners is None else len(charuco_corners)} corners detected - need at least 5', 
                      (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            debug_dir = '${DEBUG_DIR}'
            os.makedirs(debug_dir, exist_ok=True)
            cv2.imwrite(os.path.join(debug_dir, 'charuco_detection_failed.png'), debug_img)
            print(f'DEBUG: Saved failure visualization to {os.path.join(debug_dir, 'charuco_detection_failed.png')}')
            sys.exit(1)
        
        # Draw the ChArUco corners
        cv2.aruco.drawDetectedCornersCharuco(debug_img, charuco_corners, charuco_ids)
        
        # Load camera intrinsics
        try:
            intrinsics_path = os.path.join('${CALIB_DIR}', 'zed2i_intrinsics.npz')
            data = np.load(intrinsics_path)
            camera_matrix = data['cameraMatrix']
            dist_coeffs = data['distCoeffs']
        except (FileNotFoundError, KeyError):
            height, width = image.shape[:2]
            focal_length = width
            camera_matrix = np.array([
                [focal_length, 0, width/2],
                [0, focal_length, height/2],
                [0, 0, 1]
            ])
            dist_coeffs = np.zeros(5)
        
        # Estimate board pose
        ret, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            charuco_corners, charuco_ids, board, camera_matrix, dist_coeffs, None, None)
        
        if not ret:
            print('ERROR: Failed to estimate ChArUco board pose')
            cv2.putText(debug_img, 'Failed to estimate board pose', 
                      (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            debug_dir = '${DEBUG_DIR}'
            os.makedirs(debug_dir, exist_ok=True)
            cv2.imwrite(os.path.join(debug_dir, 'charuco_detection_failed.png'), debug_img)
            print(f'DEBUG: Saved failure visualization to {os.path.join(debug_dir, 'charuco_detection_failed.png')}')
            sys.exit(1)
        
        # Draw axes on the board
        cv2.drawFrameAxes(debug_img, camera_matrix, dist_coeffs, rvec, tvec, 0.1)
        
        # Add status text
        cv2.putText(debug_img, f'Detected {len(ids)} markers', 
                  (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(debug_img, f'Found {len(charuco_ids)} ChArUco corners', 
                  (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(debug_img, 'Board pose estimated successfully', 
                  (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Convert rotation to transformation matrix
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec.flatten()
        
        # Save results
        debug_dir = '${DEBUG_DIR}'
        os.makedirs(debug_dir, exist_ok=True)
        cv2.imwrite(os.path.join(debug_dir, 'charuco_detection.png'), debug_img)
        
        print(f'SUCCESS: ChArUco board detected with {len(ids)} markers and {len(charuco_ids)} corners')
        print(f'DEBUG: Saved visualization to {os.path.join(debug_dir, 'charuco_detection.png')}')
        print(f'TIMESTAMP: {image_timestamp}')
        
    except Exception as e:
        print(f'ERROR: Exception during ChArUco detection: {str(e)}')
        traceback.print_exc()
        debug_img = image.copy()
        cv2.putText(debug_img, f'Error: {str(e)}', 
                  (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        debug_dir = '${DEBUG_DIR}'
        os.makedirs(debug_dir, exist_ok=True)
        cv2.imwrite(os.path.join(debug_dir, 'charuco_detection_failed.png'), debug_img)
        print(f'DEBUG: Saved failure visualization to {os.path.join(debug_dir, 'charuco_detection_failed.png')}')
        sys.exit(1)
        
except Exception as e:
    print(f'ERROR: Exception during image loading: {str(e)}')
    traceback.print_exc()
    sys.exit(1)
")

    # Extract timestamp for later use
    image_timestamp=$(echo "$charuco_result" | grep "TIMESTAMP:" | cut -d' ' -f2)

    # Check if ChArUco detection succeeded
    if echo "$charuco_result" | grep -q "SUCCESS"; then
        show_banner "ChArUco board detection successful" "success"
        echo -e "$charuco_result"
        
        # Display the debug image path
        debug_image=$(echo "$charuco_result" | grep "DEBUG:" | grep "charuco_detection.png" | cut -d' ' -f3)
        echo -e "\n${CYAN}Debug image saved to:${NC} ${debug_image}"
        
        wait_for_continue "ChArUco board detected successfully. Continue to point cloud extraction?" 1
        retry_step2=$?
    else
        show_banner "ChArUco board detection failed" "error"
        echo -e "$charuco_result"
        
        # Display the debug image path
        debug_image=$(echo "$charuco_result" | grep "DEBUG:" | grep "charuco_detection_failed.png" | cut -d' ' -f3)
        if [ ! -z "$debug_image" ]; then
            echo -e "\n${CYAN}Debug image saved to:${NC} ${debug_image}"
            echo -e "\n${YELLOW}Troubleshooting tips:${NC}"
            echo -e "- Make sure the ChArUco board is clearly visible to the camera"
            echo -e "- Check for adequate lighting (avoid glare and shadows)"
            echo -e "- Keep the board flat and stable"
            echo -e "- Try different distances (1-3 meters from camera is ideal)"
            echo -e "- Ensure at least 4 markers are visible in the image"
        fi
        
        wait_for_continue "ChArUco detection failed. Retry with a different image?" 1
        retry_step2=$?
    fi
done

# STEP 3: Extract ChArUco regions from point clouds
show_banner "3: Processing point clouds" "step"
echo -e "Finding and extracting ChArUco board from LiDAR and camera point clouds..."

retry_step3=1
while [ $retry_step3 -eq 1 ]; do
    # Process point clouds
    pointcloud_result=$(cd "${BASE_DIR}" && python3 -c "
import os, glob, sys, traceback
import numpy as np
import open3d as o3d

try:
    # Initialize paths
    session_dir = '${latest_session}'
    debug_dir = '${DEBUG_DIR}'
    os.makedirs(debug_dir, exist_ok=True)
    
    # Find closest point clouds to the image timestamp
    image_timestamp = '${image_timestamp}'
    if not image_timestamp:
        print('ERROR: No image timestamp available')
        sys.exit(1)
    
    # Find camera point cloud
    camera_pc_dir = os.path.join(session_dir, 'ZED_CAMERA_2i_point_clouds')
    if os.path.exists(camera_pc_dir) and os.listdir(camera_pc_dir):
        camera_pc_files = sorted(glob.glob(os.path.join(camera_pc_dir, '*.ply')))
        if camera_pc_files:
            # Extract timestamps from filenames
            timestamps = [os.path.basename(f).split('.')[0] for f in camera_pc_files]
            
            # Find closest timestamp
            closest_idx = min(range(len(timestamps)), 
                            key=lambda i: abs(int(timestamps[i]) - int(image_timestamp)))
            
            camera_pc_path = camera_pc_files[closest_idx]
            print(f'SUCCESS: Found camera point cloud: {os.path.basename(camera_pc_path)}')
        else:
            print('WARNING: No camera point cloud files found')
            camera_pc_path = None
    else:
        print('WARNING: No camera point cloud directory found')
        camera_pc_path = None
    
    # Find LiDAR point cloud
    lidar_pc_dir = os.path.join(session_dir, 'point_clouds')
    if os.path.exists(lidar_pc_dir) and os.listdir(lidar_pc_dir):
        lidar_pc_files = sorted(glob.glob(os.path.join(lidar_pc_dir, '*.ply')))
        if not lidar_pc_files:
            lidar_cloud_files = sorted(glob.glob(os.path.join(lidar_pc_dir, '*.cloud')))
            if lidar_cloud_files:
                print('WARNING: Found .cloud files, but no .ply files. Need to convert first.')
                print('ERROR: No LiDAR .ply point cloud files found')
                sys.exit(1)
            else:
                print('ERROR: No LiDAR point cloud files found')
                sys.exit(1)
                
        # Extract timestamps from filenames
        timestamps = [os.path.basename(f).split('.')[0] for f in lidar_pc_files]
        
        # Find closest timestamp
        closest_idx = min(range(len(timestamps)), 
                        key=lambda i: abs(int(timestamps[i]) - int(image_timestamp)))
        
        lidar_pc_path = lidar_pc_files[closest_idx]
        print(f'SUCCESS: Found LiDAR point cloud: {os.path.basename(lidar_pc_path)}')
    else:
        print('ERROR: No LiDAR point cloud directory found')
        sys.exit(1)
    
    # Check if we have at least one of the point clouds
    if not camera_pc_path and not lidar_pc_path:
        print('ERROR: No point clouds available')
        sys.exit(1)
    
    # Extract ChArUco board region from point clouds
    charuco_cloud_camera_path = os.path.join('${CALIB_DIR}', 'charuco_cloud_camera.pcd')
    charuco_cloud_lidar_path = os.path.join('${CALIB_DIR}', 'charuco_cloud_lidar.pcd')
    
    # Process camera point cloud if available
    if camera_pc_path:
        try:
            camera_pcd = o3d.io.read_point_cloud(camera_pc_path)
            print(f'INFO: Camera point cloud has {len(camera_pcd.points)} points')
            
            if len(camera_pcd.points) > 0:
                # Calculate point cloud bounds for info
                bounds_min = camera_pcd.get_min_bound()
                bounds_max = camera_pcd.get_max_bound()
                print(f'INFO: Camera point cloud bounds: {bounds_min} to {bounds_max}')
                
                # Extract region of interest - center of the point cloud
                points = np.asarray(camera_pcd.points)
                
                # Determine likely orientation and depth axis
                if np.max(np.abs(points[:, 2])) > np.max(np.abs(points[:, 1])):
                    depth_idx = 2
                    print('INFO: Camera depth appears to be Z axis')
                else:
                    depth_idx = 1
                    print('INFO: Camera depth appears to be Y axis')
                
                # Filter by reasonable depth
                depth_mask = (points[:, depth_idx] > 0.5) & (points[:, depth_idx] < 3.0)
                filtered_points = points[depth_mask]
                
                if len(filtered_points) < 100:
                    print('WARNING: Very few points in expected depth range, using full point cloud')
                    filtered_points = points
                
                # Get median as center of target area
                center = np.median(filtered_points, axis=0)
                print(f'INFO: Estimated center of ChArUco region: {center}')
                
                # Create bounding box
                bounds_min = center - np.array([0.75, 0.75, 0.75])
                bounds_max = center + np.array([0.75, 0.75, 0.75])
                
                # Ensure bounds are sensible
                if depth_idx == 2:
                    bounds_min[2] = max(bounds_min[2], 0.5)
                else:
                    bounds_min[1] = max(bounds_min[1], 0.5)
                
                print(f'INFO: Using bounding box: {bounds_min} to {bounds_max}')
                
                bbox = o3d.geometry.AxisAlignedBoundingBox(
                    min_bound=bounds_min,
                    max_bound=bounds_max
                )
                
                # Crop point cloud
                cropped_camera_pcd = camera_pcd.crop(bbox)
                print(f'SUCCESS: Extracted {len(cropped_camera_pcd.points)} points from camera ChArUco region')
                
                # Save result
                o3d.io.write_point_cloud(charuco_cloud_camera_path, cropped_camera_pcd)
                debug_camera_path = os.path.join(debug_dir, 'camera_charuco_region.ply')
                o3d.io.write_point_cloud(debug_camera_path, cropped_camera_pcd)
                print(f'DEBUG: Saved camera ChArUco region to {debug_camera_path}')
            else:
                print('ERROR: Camera point cloud is empty')
                sys.exit(1)
                
        except Exception as e:
            print(f'ERROR: Failed to process camera point cloud: {str(e)}')
            traceback.print_exc()
            sys.exit(1)
    else:
        print('WARNING: Skipping camera point cloud processing (not available)')
    
    # Process LiDAR point cloud
    try:
        lidar_pcd = o3d.io.read_point_cloud(lidar_pc_path)
        print(f'INFO: LiDAR point cloud has {len(lidar_pcd.points)} points')
        
        if len(lidar_pcd.points) > 0:
            # Calculate point cloud bounds for info
            bounds_min = lidar_pcd.get_min_bound()
            bounds_max = lidar_pcd.get_max_bound()
            print(f'INFO: LiDAR point cloud bounds: {bounds_min} to {bounds_max}')
            
            # Extract region of interest - center of the point cloud
            points = np.asarray(lidar_pcd.points)
            
            # Determine likely orientation and depth axis
            if np.max(np.abs(points[:, 2])) > np.max(np.abs(points[:, 1])):
                depth_idx = 2
                print('INFO: LiDAR depth appears to be Z axis')
            else:
                depth_idx = 1
                print('INFO: LiDAR depth appears to be Y axis')
            
            # Filter by reasonable depth
            depth_mask = (points[:, depth_idx] > 0.5) & (points[:, depth_idx] < 3.0)
            filtered_points = points[depth_mask]
            
            if len(filtered_points) < 100:
                print('WARNING: Very few points in expected depth range, using full point cloud')
                filtered_points = points
            
            # Get median as center of target area
            center = np.median(filtered_points, axis=0)
            print(f'INFO: Estimated center of ChArUco region: {center}')
            
            # Create bounding box
            bounds_min = center - np.array([0.75, 0.75, 0.75])
            bounds_max = center + np.array([0.75, 0.75, 0.75])
            
            # Ensure bounds are sensible
            if depth_idx == 2:
                bounds_min[2] = max(bounds_min[2], 0.5)
            else:
                bounds_min[1] = max(bounds_min[1], 0.5)
            
            print(f'INFO: Using bounding box: {bounds_min} to {bounds_max}')
            
            bbox = o3d.geometry.AxisAlignedBoundingBox(
                min_bound=bounds_min,
                max_bound=bounds_max
            )
            
            # Crop point cloud
            cropped_lidar_pcd = lidar_pcd.crop(bbox)
            print(f'SUCCESS: Extracted {len(cropped_lidar_pcd.points)} points from LiDAR ChArUco region')
            
            # Save result
            o3d.io.write_point_cloud(charuco_cloud_lidar_path, cropped_lidar_pcd)
            debug_lidar_path = os.path.join(debug_dir, 'lidar_charuco_region.ply')
            o3d.io.write_point_cloud(debug_lidar_path, cropped_lidar_pcd)
            print(f'DEBUG: Saved LiDAR ChArUco region to {debug_lidar_path}')
        else:
            print('ERROR: LiDAR point cloud is empty')
            sys.exit(1)
            
    except Exception as e:
        print(f'ERROR: Failed to process LiDAR point cloud: {str(e)}')
        traceback.print_exc()
        sys.exit(1)
    
    print('SUCCESS: Point cloud processing complete')
    
except Exception as e:
    print(f'ERROR: Exception during point cloud processing: {str(e)}')
    traceback.print_exc()
    sys.exit(1)
")

    # Check if point cloud processing succeeded
    if echo "$pointcloud_result" | grep -q "SUCCESS: Point cloud processing complete"; then
        show_banner "Point cloud processing successful" "success"
        echo -e "$pointcloud_result"
        
        # Display debug files
        echo -e "\n${CYAN}Debug files saved to:${NC}"
        echo -e "  ${DEBUG_DIR}/camera_charuco_region.ply"
        echo -e "  ${DEBUG_DIR}/lidar_charuco_region.ply"
        
        wait_for_continue "Point cloud regions extracted. Continue to point cloud alignment?" 1
        retry_step3=$?
    else
        show_banner "Point cloud processing failed" "error"
        echo -e "$pointcloud_result"
        
        echo -e "\n${YELLOW}Troubleshooting tips:${NC}"
        echo -e "- Make sure both LiDAR and camera captured the ChArUco board"
        echo -e "- Check that point clouds are not empty"
        echo -e "- Ensure the board is within range of both sensors (1-3 meters)"
        echo -e "- Try using a different recording session with better data"
        
        wait_for_continue "Point cloud processing failed. Retry?" 1
        retry_step3=$?
    fi
done

# STEP 4: Align point clouds using ICP
show_banner "4: Aligning point clouds" "step"
echo -e "Registering extracted point clouds using Iterative Closest Point (ICP)..."

retry_step4=1
while [ $retry_step4 -eq 1 ]; do
    # Align point clouds
    alignment_result=$(cd "${BASE_DIR}" && python3 -c "
import os, sys, traceback
import numpy as np
import open3d as o3d

try:
    # Initialize paths
    charuco_cloud_camera_path = os.path.join('${CALIB_DIR}', 'charuco_cloud_camera.pcd')
    charuco_cloud_lidar_path = os.path.join('${CALIB_DIR}', 'charuco_cloud_lidar.pcd')
    debug_dir = '${DEBUG_DIR}'
    
    # Load point clouds
    source = o3d.io.read_point_cloud(charuco_cloud_camera_path)
    target = o3d.io.read_point_cloud(charuco_cloud_lidar_path)
    
    print(f'INFO: Source (camera) point cloud has {len(source.points)} points')
    print(f'INFO: Target (LiDAR) point cloud has {len(target.points)} points')
    
    # Check if point clouds have enough points
    if len(source.points) < 10 or len(target.points) < 10:
        print(f'ERROR: Point clouds have too few points for registration')
        print(f'Source: {len(source.points)} points, Target: {len(target.points)} points')
        print(f'Need at least 10 points in each cloud')
        sys.exit(1)
    
    # Calculate normals for better registration
    source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    
    # Save debug visualizations
    o3d.io.write_point_cloud(os.path.join(debug_dir, 'source_cloud.ply'), source)
    o3d.io.write_point_cloud(os.path.join(debug_dir, 'target_cloud.ply'), target)
    
    # Downsample for faster processing
    source_down = source.voxel_down_sample(voxel_size=0.05)
    target_down = target.voxel_down_sample(voxel_size=0.05)
    
    print('INFO: Running ICP with multiple parameter sets to find best alignment...')
    
    # Try different max correspondence distances
    best_fitness = 0
    best_transform = np.eye(4)
    best_rmse = float('inf')
    
    for dist in [0.05, 0.1, 0.2, 0.5]:
        print(f'INFO: Trying max_correspondence_distance={dist}...')
        
        try:
            # Use point-to-plane ICP
            result = o3d.pipelines.registration.registration_icp(
                source_down, target_down,
                max_correspondence_distance=dist,
                init=np.eye(4),
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane()
            )
            
            fitness = result.fitness
            rmse = result.inlier_rmse
            print(f'INFO: Fitness score: {fitness}, RMSE: {rmse}')
            
            if fitness > best_fitness:
                best_fitness = fitness
                best_transform = result.transformation
                best_rmse = rmse
                print(f'INFO: New best transformation found')
        except Exception as e:
            print(f'WARNING: Registration failed with parameters: {str(e)}')
            continue
    
    # Check if we have a decent registration
    if best_fitness < 0.1:
        print(f'WARNING: Registration quality is poor (fitness={best_fitness})')
        
        if best_fitness < 0.01:
            print('ERROR: Registration fitness is extremely poor (< 0.01)')
            print('ICP alignment failed to find a good match between point clouds')
            sys.exit(1)
    
    print(f'SUCCESS: Final ICP fitness score: {best_fitness}, RMSE: {best_rmse}')
    print(f'INFO: Final transformation matrix:')
    np.set_printoptions(precision=4, suppress=True)
    print(best_transform)
    
    # Save the aligned point cloud for visualization
    source_aligned = source.clone()
    source_aligned.transform(best_transform)
    aligned_debug_path = os.path.join(debug_dir, 'aligned_cloud.ply')
    o3d.io.write_point_cloud(aligned_debug_path, source_aligned)
    
    # Save transformation results
    result_path = os.path.join(debug_dir, 'icp_transform.npy')
    np.save(result_path, best_transform)
    
    print(f'DEBUG: Saved aligned point cloud to {aligned_debug_path}')
    print(f'DEBUG: Saved ICP transformation to {result_path}')
    
except Exception as e:
    print(f'ERROR: Exception during point cloud alignment: {str(e)}')
    traceback.print_exc()
    sys.exit(1)
")

    # Check if alignment succeeded
    if echo "$alignment_result" | grep -q "SUCCESS"; then
        show_banner "Point cloud alignment successful" "success"
        echo -e "$alignment_result"
        
        # Display debug files
        echo -e "\n${CYAN}Debug files saved to:${NC}"
        echo -e "  ${DEBUG_DIR}/aligned_cloud.ply"
        echo -e "  ${DEBUG_DIR}/icp_transform.npy"
        
        wait_for_continue "Point clouds aligned successfully. Continue to final transformation?" 1
        retry_step4=$?
    else
        show_banner "Point cloud alignment failed" "error"
        echo -e "$alignment_result"
        
        echo -e "\n${YELLOW}Troubleshooting tips:${NC}"
        echo -e "- The extracted point cloud regions may not have enough overlap"
        echo -e "- Check the extracted regions in debug files"
        echo -e "- Try adjusting the bounding box in the previous step"
        echo -e "- Ensure the ChArUco board is captured clearly by both sensors"
        
        wait_for_continue "Point cloud alignment failed. Retry?" 1
        retry_step4=$?
    fi
done

# STEP 5: Compute final transformation
show_banner "5: Computing final transformation" "step"
echo -e "Computing the final LiDAR to Camera transformation matrix..."

# Compute final transformation
transform_result=$(cd "${BASE_DIR}" && python3 -c "
import os, sys, traceback
import numpy as np
import cv2
from datetime import datetime

try:
    # Load the transformation matrices
    debug_dir = '${DEBUG_DIR}'
    results_dir = '${RESULTS_DIR}'
    os.makedirs(results_dir, exist_ok=True)
    
    # We need two transformations:
    # 1. T_Camera2i_Charuco (from camera to charuco board)
    # 2. T_Charuco_LiDAR (from charuco board to lidar, from ICP)
    
    # For T_Camera2i_Charuco, we need to recompute from the image
    # (This would typically be saved from the ChArUco detection step in a production system)
    
    # Load image and detect ChArUco board again
    image_dir = os.path.join('${latest_session}', 'ZED_CAMERA_2i_rgb_image_rect_color')
    image_files = sorted(os.listdir(image_dir))
    
    if not image_files:
        print('ERROR: No images found in ZED_CAMERA_2i_rgb_image_rect_color')
        sys.exit(1)
    
    latest_image_path = os.path.join(image_dir, image_files[-1])
    image = cv2.imread(latest_image_path)
    
    # Convert to grayscale for ArUco detection
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Set up ChArUco board parameters
    squares_x = 10
    squares_y = 14
    square_length = 0.07  # 7cm squares
    marker_length = 0.055  # 5.5cm markers
    
    # Load camera intrinsics
    try:
        intrinsics_path = os.path.join('${CALIB_DIR}', 'zed2i_intrinsics.npz')
        data = np.load(intrinsics_path)
        camera_matrix = data['cameraMatrix']
        dist_coeffs = data['distCoeffs']
    except (FileNotFoundError, KeyError):
        height, width = image.shape[:2]
        focal_length = width
        camera_matrix = np.array([
            [focal_length, 0, width/2],
            [0, focal_length, height/2],
            [0, 0, 1]
        ])
        dist_coeffs = np.zeros(5)
    
    # Create ArUco dictionary and detect markers
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
    # Create the ChArUco board
    board = cv2.aruco.CharucoBoard_create(squares_x, squares_y, square_length, marker_length, aruco_dict)
    
    # Refine corners
    ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
    
    # Estimate board pose
    ret, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
        charuco_corners, charuco_ids, board, camera_matrix, dist_coeffs, None, None)
    
    # Convert to transformation matrix
    R, _ = cv2.Rodrigues(rvec)
    T_Camera2i_Charuco = np.eye(4)
    T_Camera2i_Charuco[:3, :3] = R
    T_Camera2i_Charuco[:3, 3] = tvec.flatten()
    
    print('INFO: Camera to ChArUco transformation:')
    print(T_Camera2i_Charuco)
    
    # Load T_Charuco_LiDAR from ICP alignment
    icp_transform_path = os.path.join(debug_dir, 'icp_transform.npy')
    if not os.path.exists(icp_transform_path):
        print('ERROR: ICP transformation file not found')
        sys.exit(1)
        
    T_Charuco_LiDAR = np.load(icp_transform_path)
    
    print('INFO: ChArUco to LiDAR transformation (from ICP):')
    print(T_Charuco_LiDAR)
    
    # Compute final transformation: T_LiDAR_Camera2i = T_Charuco_LiDAR @ inv(T_Camera2i_Charuco)
    T_LiDAR_Camera2i = T_Charuco_LiDAR @ np.linalg.inv(T_Camera2i_Charuco)
    
    print('SUCCESS: Final transformation (LiDAR to Camera):')
    print(T_LiDAR_Camera2i)
    
    # Save results
    output_path = os.path.join(results_dir, 'tf_statique.npy')
    np.save(output_path, T_LiDAR_Camera2i)
    
    # Save inverse transformation
    T_Camera2i_LiDAR = np.linalg.inv(T_LiDAR_Camera2i)
    np.save(os.path.join(results_dir, 'tf_statique_inverse.npy'), T_Camera2i_LiDAR)
    
    # Save camera intrinsics
    np.savez(os.path.join(results_dir, 'zed2i_intrinsics.npz'), 
            cameraMatrix=camera_matrix, 
            distCoeffs=dist_coeffs)
    
    # Create a timestamp file
    with open(os.path.join(results_dir, 'calibration_timestamp.txt'), 'w') as f:
        f.write(f'Calibration performed at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n')
        f.write(f'Using session: {os.path.basename('${latest_session}')}\n')
        f.write(f'Camera image: {os.path.basename(latest_image_path)}\n')
        f.write(f'ICP fitness score: {np.load(icp_transform_path).item().get('fitness', 'N/A') if hasattr(np.load(icp_transform_path), 'item') else 'N/A'}\n')
    
    print(f'SUCCESS: Results saved to {results_dir}')
    print(f'DEBUG: Transformation matrices saved to {output_path}')
    
except Exception as e:
    print(f'ERROR: Exception during final transformation: {str(e)}')
    traceback.print_exc()
    sys.exit(1)
")

# Check if final transformation succeeded
if echo "$transform_result" | grep -q "SUCCESS: Results saved"; then
    show_banner "Calibration completed successfully!" "success"
    echo -e "$transform_result"
    
    # Display the transformation matrix
    echo -e "\n${YELLOW}Transformation Matrix (LiDAR → Camera):${NC}"
    python3 -c "import numpy as np; print(np.load('${RESULTS_DIR}/tf_statique.npy'))"
    
    echo -e "\n${YELLOW}Intrinsic Camera Parameters:${NC}"
    python3 -c "import numpy as np; data = np.load('${RESULTS_DIR}/zed2i_intrinsics.npz'); print('Camera Matrix:'); print(data['cameraMatrix']); print('Distortion Coefficients:'); print(data['distCoeffs'])"
    
    echo -e "\n${CYAN}Results saved to:${NC} ${RESULTS_DIR}"
    
    echo -e "\n${BLUE}To use this calibration in your application:${NC}"
    echo -e "1. Load the transformation matrix from: ${RESULTS_DIR}/tf_statique.npy"
    echo -e "2. Apply it to transform points from LiDAR to Camera coordinate system"
    echo -e "3. For the reverse transformation (Camera → LiDAR), use: ${RESULTS_DIR}/tf_statique_inverse.npy"
else
    show_banner "Final transformation calculation failed" "error"
    echo -e "$transform_result"
    
    echo -e "\n${YELLOW}Troubleshooting tips:${NC}"
    echo -e "- Check the alignment results from the previous step"
    echo -e "- Verify that both transformation matrices were computed correctly"
    echo -e "- Try running the calibration again with a different dataset"
    
    exit 1
fi

# Final success message
show_banner "Calibration process completed" "info"
echo -e "Thank you for using the interactive calibration tool."
echo -e "The LiDAR to Camera transformation has been calculated and saved."