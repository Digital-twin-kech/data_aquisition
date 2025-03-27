#!/bin/bash

# Script to migrate Livox SDK from Desktop location to a more standard location
# and update references in the codebase

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
OLD_LIVOX_PATH="/opt/livox-sdk"
NEW_LIVOX_PATH="/opt/livox-sdk"
PROJECT_PATH="/home/user/Desktop/data-aquisition-digital-twin/data_aquisition"

echo -e "${BLUE}====================================${NC}"
echo -e "${BLUE}  LIVOX SDK MIGRATION TOOL         ${NC}"
echo -e "${BLUE}====================================${NC}"
echo -e "${YELLOW}This script will:${NC}"
echo -e "1. Copy Livox SDK from ${OLD_LIVOX_PATH} to ${NEW_LIVOX_PATH}"
echo -e "2. Update paths in the codebase"
echo -e "${BLUE}====================================${NC}"

# Check if source directory exists
if [ ! -d "${OLD_LIVOX_PATH}" ]; then
    echo -e "${RED}ERROR: Source directory ${OLD_LIVOX_PATH} does not exist${NC}"
    exit 1
fi

# Create destination directory if it doesn't exist
echo -e "${YELLOW}Creating destination directory...${NC}"
sudo mkdir -p ${NEW_LIVOX_PATH}
if [ $? -ne 0 ]; then
    echo -e "${RED}ERROR: Failed to create destination directory${NC}"
    exit 1
fi

# Copy files
echo -e "${YELLOW}Copying Livox SDK files...${NC}"
sudo cp -r ${OLD_LIVOX_PATH}/install ${NEW_LIVOX_PATH}/
sudo cp -r ${OLD_LIVOX_PATH}/src ${NEW_LIVOX_PATH}/

# Set permissions
echo -e "${YELLOW}Setting permissions...${NC}"
sudo chown -R $(whoami):$(whoami) ${NEW_LIVOX_PATH}
sudo chmod -R 755 ${NEW_LIVOX_PATH}

# Verify copy
if [ -d "${NEW_LIVOX_PATH}/install" ] && [ -d "${NEW_LIVOX_PATH}/src" ]; then
    echo -e "${GREEN}✓ Livox SDK files successfully copied to ${NEW_LIVOX_PATH}${NC}"
else
    echo -e "${RED}ERROR: Failed to copy Livox SDK files${NC}"
    exit 1
fi

# Find files that need to be updated
echo -e "${YELLOW}Files that need to be updated with the new path:${NC}"
grep -r --include="*.sh" --include="*.cpp" --include="*.hpp" --include="*.h" --include="*.py" --include="*.md" "${OLD_LIVOX_PATH}" ${PROJECT_PATH}

# Update paths in files
echo -e "${YELLOW}Updating paths in files...${NC}"
find ${PROJECT_PATH} -type f \( -name "*.sh" -o -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "*.py" -o -name "*.md" \) -exec sed -i "s|${OLD_LIVOX_PATH}|${NEW_LIVOX_PATH}|g" {} \;

# List updated files
echo -e "${YELLOW}Verifying updated files...${NC}"
grep -r --include="*.sh" --include="*.cpp" --include="*.hpp" --include="*.h" --include="*.py" --include="*.md" "${NEW_LIVOX_PATH}" ${PROJECT_PATH}

echo -e "${BLUE}====================================${NC}"
echo -e "${GREEN}Migration complete!${NC}"
echo -e "${YELLOW}Please verify the changes and rebuild the project.${NC}"
echo -e "${BLUE}====================================${NC}"