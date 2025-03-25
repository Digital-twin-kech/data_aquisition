#!/bin/bash

# ZED Camera Connection Test Script
# This script tries connecting to each ZED camera with minimal settings
# to verify the physical connections and SDK integration

echo "ZED Camera Connection Test Script"
echo "================================="

# Check if ZED SDK is installed
if [ -d "/usr/local/zed" ]; then
    echo "[✓] ZED SDK is installed at /usr/local/zed"
    
    # Get ZED SDK version from Camera.hpp
    SDK_VERSION=$(grep -a "ZED_SDK_MAJOR_VERSION" /usr/local/zed/include/sl/Camera.hpp | head -1 | awk '{print $3}')
    SDK_MINOR=$(grep -a "ZED_SDK_MINOR_VERSION" /usr/local/zed/include/sl/Camera.hpp | head -1 | awk '{print $3}')
    SDK_PATCH=$(grep -a "ZED_SDK_PATCH_VERSION" /usr/local/zed/include/sl/Camera.hpp | head -1 | awk '{print $3}')
    
    echo "[✓] ZED SDK Version: $SDK_VERSION.$SDK_MINOR.$SDK_PATCH"
else
    echo "[✗] ZED SDK not found at /usr/local/zed"
    echo "    Please install the ZED SDK from https://www.stereolabs.com/developers/release/"
    exit 1
fi

# Check for ZED cameras using ZED Explorer
echo -e "\nChecking for connected ZED cameras with ZED tool..."
if [ -f "/usr/local/zed/tools/ZED_Explorer" ]; then
    # Run ZED Explorer with minimal UI to get camera info
    timeout 5s /usr/local/zed/tools/ZED_Explorer -l > /tmp/zed_camera_list.txt 2>&1 || true
    
    # Check if any cameras were found
    if grep -q "Found" /tmp/zed_camera_list.txt; then
        echo "[✓] ZED Explorer found cameras"
        grep "Found" /tmp/zed_camera_list.txt
        grep "Serial" /tmp/zed_camera_list.txt
    else
        echo "[✗] ZED Explorer did not find any cameras"
        echo "    Check physical connections and permissions"
    fi
else
    echo "[!] ZED Explorer not found, skipping check"
fi

# Create minimal C++ test program to verify ZED SDK functionality
echo -e "\nCreating minimal ZED test program..."
TMP_DIR=$(mktemp -d)
cat > $TMP_DIR/zed_test.cpp << 'EOL'
#include <iostream>
#include <sl/Camera.hpp>

int main() {
    std::cout << "ZED SDK Version: " << ZED_SDK_MAJOR_VERSION << "." << ZED_SDK_MINOR_VERSION << "." << ZED_SDK_PATCH_VERSION << std::endl;
    
    // Get list of cameras
    auto devList = sl::Camera::getDeviceList();
    std::cout << "Found " << devList.size() << " camera(s)" << std::endl;
    for (auto& dev : devList) {
        std::cout << "  - SN: " << dev.serial_number << ", Model: " << sl::toString(dev.camera_model) 
                  << ", State: " << (dev.camera_state == sl::CAMERA_STATE::AVAILABLE ? "Available" : "In use") << std::endl;
    }
    
    // Try to connect to each camera with minimal settings
    for (auto& dev : devList) {
        std::cout << "\nTesting connection to camera SN: " << dev.serial_number << std::endl;
        sl::Camera zed;
        sl::InitParameters init_params;
        init_params.camera_resolution = sl::RESOLUTION::VGA;
        init_params.camera_fps = 15;
        init_params.depth_mode = sl::DEPTH_MODE::NONE;
        init_params.sdk_verbose = true;
        init_params.input.setFromSerialNumber(dev.serial_number);
        
        sl::ERROR_CODE err = zed.open(init_params);
        if (err == sl::ERROR_CODE::SUCCESS) {
            std::cout << "  [✓] Successfully connected to camera!" << std::endl;
            
            // Get camera information
            auto info = zed.getCameraInformation();
            std::cout << "  - Model: " << sl::toString(info.camera_model) << std::endl;
            std::cout << "  - Serial: " << info.serial_number << std::endl;
            std::cout << "  - Firmware: " << info.camera_configuration.firmware_version << std::endl;
            
            // Try to grab a frame
            if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
                std::cout << "  [✓] Successfully grabbed a frame!" << std::endl;
                
                // Retrieve an image
                sl::Mat image;
                if (zed.retrieveImage(image, sl::VIEW::LEFT) == sl::ERROR_CODE::SUCCESS) {
                    std::cout << "  [✓] Successfully retrieved an image: " 
                              << image.getWidth() << "x" << image.getHeight() 
                              << ", channels: " << image.getChannels() << std::endl;
                    
                    // Check if the image has some non-zero data
                    bool has_data = false;
                    for (int i = 0; i < 10; i++) {
                        unsigned char* ptr = image.getPtr<sl::uchar1>() + (image.getWidth() * image.getChannels() * i);
                        for (int j = 0; j < image.getWidth() * image.getChannels(); j++) {
                            if (ptr[j] > 0) {
                                has_data = true;
                                break;
                            }
                        }
                        if (has_data) break;
                    }
                    
                    if (has_data) {
                        std::cout << "  [✓] Image contains valid data (non-zero pixels)" << std::endl;
                    } else {
                        std::cout << "  [✗] Image appears to contain all zeros!" << std::endl;
                    }
                } else {
                    std::cout << "  [✗] Failed to retrieve an image" << std::endl;
                }
            } else {
                std::cout << "  [✗] Failed to grab a frame" << std::endl;
            }
            
            zed.close();
        } else {
            std::cout << "  [✗] Failed to connect: " << sl::toString(err) << std::endl;
        }
    }
    
    return 0;
}
EOL

# Compile and run the test program
echo "Compiling and running ZED test program..."
g++ -std=c++17 $TMP_DIR/zed_test.cpp -o $TMP_DIR/zed_test -I/usr/local/zed/include -L/usr/local/zed/lib -lsl_zed -lGL -lGLU -lcuda

if [ -f "$TMP_DIR/zed_test" ]; then
    echo "Running ZED test program..."
    $TMP_DIR/zed_test > /tmp/zed_native_test.log 2>&1
    cat /tmp/zed_native_test.log
    
    # Check for successful connections
    if grep -q "Successfully connected to camera" /tmp/zed_native_test.log; then
        echo -e "\n[✓] Native ZED SDK test passed: Successfully connected to at least one camera"
    else
        echo -e "\n[✗] Native ZED SDK test failed: Could not connect to any camera"
    fi
    
    # Check for valid image data
    if grep -q "Image contains valid data" /tmp/zed_native_test.log; then
        echo "[✓] Native ZED SDK test passed: Successfully captured valid image data"
    else
        echo "[✗] Native ZED SDK test failed: Could not capture valid image data"
    fi
else
    echo "[✗] Failed to compile ZED test program"
fi

# Test with the ROS2 node for each camera
echo -e "\nTesting with ROS2 camera node..."

# Source ROS2 setup
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source $SCRIPT_DIR/../install/setup.bash

# Check simple camera connection with minimal settings
echo -e "\nRunning zero-serial camera test (first available)..."
LOGFILE="/tmp/zed_camera_zero_test.log"
timeout 10s ros2 run data_aquisition zed_camera_node --ros-args -p camera.serial_number:=0 -p camera.resolution:=VGA -p camera.max_fps:=15 > $LOGFILE 2>&1 &
PID=$!

# Wait for a few seconds to see if connection is successful
sleep 10
if ps -p $PID > /dev/null; then
    kill $PID 2>/dev/null
fi

# Check if connection was successful
if grep -q "connected successfully" $LOGFILE && grep -q "Test image check" $LOGFILE; then
    echo "[✓] ROS2 node test passed: Successfully connected to the first available camera"
    # Check if image test found non-zero pixels
    NON_ZERO=$(grep "Test image check" $LOGFILE | grep -o '[0-9]\+/[0-9]\+' | cut -d'/' -f1)
    TOTAL=$(grep "Test image check" $LOGFILE | grep -o '[0-9]\+/[0-9]\+' | cut -d'/' -f2)
    
    if [ "$NON_ZERO" -gt 0 ]; then
        echo "[✓] ROS2 node test passed: Successfully captured valid image data ($NON_ZERO/$TOTAL non-zero pixels)"
    else
        echo "[✗] ROS2 node test failed: Captured image contains all zeros (0/$TOTAL non-zero pixels)"
    fi
else
    echo "[✗] ROS2 node test failed: Could not connect to any camera"
fi

# Cleanup
rm -rf $TMP_DIR
echo -e "\nTest completed. See logs for details."