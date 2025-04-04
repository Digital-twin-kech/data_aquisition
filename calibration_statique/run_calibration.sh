#!/bin/bash

# Static Calibration Runner Script
# This script runs the static calibration between LiDAR and ZED 2i Camera

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Base directory
BASE_DIR="/home/user/Desktop/data-aquisition-digital-twin/data_aquisition"
CALIB_DIR="${BASE_DIR}/calibration_statique"
RESULTS_DIR="${CALIB_DIR}/results"

# Ensure directories exist
mkdir -p "${RESULTS_DIR}"
mkdir -p "${CALIB_DIR}/debug"

# Display header
echo -e "${BLUE}================================${NC}"
echo -e "${GREEN}Static Calibration Tool${NC}"
echo -e "${BLUE}================================${NC}"
echo -e "This script will:"
echo -e "1. Locate the latest recording session"
echo -e "2. Extract ChArUco board from images"
echo -e "3. Extract and align point clouds"
echo -e "4. Compute and save transformation matrix"
echo -e "${BLUE}================================${NC}"

# Run the calibration script
echo -e "${YELLOW}Starting calibration...${NC}"
cd "${BASE_DIR}" && python3 calibration_statique/static_calibration.py
RESULT=$?

if [ $RESULT -eq 0 ]; then
    echo -e "${GREEN}Calibration completed successfully!${NC}"
    echo -e "Results saved to: ${RESULTS_DIR}/tf_statique.npy"
    
    # Display the transformation matrix
    echo -e "${YELLOW}Transformation Matrix (LiDAR → Camera):${NC}"
    python3 -c "import numpy as np; print(np.load('${RESULTS_DIR}/tf_statique.npy'))"
    
    echo -e "\n${YELLOW}Intrinsic Camera Parameters:${NC}"
    python3 -c "import numpy as np; data = np.load('${RESULTS_DIR}/zed2i_intrinsics.npz'); print('Camera Matrix:'); print(data['cameraMatrix']); print('Distortion Coefficients:'); print(data['distCoeffs'])"
    
    echo -e "\n${BLUE}To use this calibration in your application:${NC}"
    echo -e "1. Load the transformation matrix from: ${RESULTS_DIR}/tf_statique.npy"
    echo -e "2. Apply it to transform points from LiDAR to Camera coordinate system"
    echo -e "3. For the reverse transformation (Camera → LiDAR), use: ${RESULTS_DIR}/tf_statique_inverse.npy"
else
    echo -e "${RED}Calibration failed with error code ${RESULT}${NC}"
    echo -e "Please check the error message above and try again."
fi