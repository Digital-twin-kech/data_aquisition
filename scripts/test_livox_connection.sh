#!/bin/bash

# Test Livox LiDAR connection
# This script tests the connection to the Livox LiDAR hardware

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default settings
LIDAR_IP="192.168.1.100"
HOST_IP="192.168.1.50"  # Using current existing IP from previous test
INTERFACE="eth0"
LOG=false
NMAP=false

# Display header
echo -e "${BLUE}================================${NC}"
echo -e "${BLUE}  LIVOX LIDAR CONNECTION TEST   ${NC}"
echo -e "${BLUE}================================${NC}"

# Parse command line arguments
while getopts "i:l:h:vn" opt; do
  case $opt in
    i)
      INTERFACE=$OPTARG
      ;;
    l)
      LIDAR_IP=$OPTARG
      ;;
    h)
      HOST_IP=$OPTARG
      ;;
    v)
      LOG=true
      ;;
    n)
      NMAP=true
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
  esac
done

echo -e "${YELLOW}Test Configuration:${NC}"
echo -e "  LiDAR IP:    ${LIDAR_IP}"
echo -e "  Host IP:     ${HOST_IP}"
echo -e "  Interface:   ${INTERFACE}"
echo -e "  Verbose:     ${LOG}"
echo -e "  Nmap Scan:   ${NMAP}"
echo ""

# Step 1: Check network configuration
echo -e "${YELLOW}[1/4] Checking network configuration...${NC}"
CURRENT_IP=$(ip -4 addr show ${INTERFACE} | grep -oP '(?<=inet\s)\d+(\.\d+){3}')
if [ -n "$CURRENT_IP" ]; then
  echo -e "Current IP address for ${INTERFACE}: ${CURRENT_IP}"
  echo -e "${GREEN}✓ Interface has a valid IP address${NC}"
else
  echo -e "${RED}✗ No valid IP address found for ${INTERFACE}${NC}"
  exit 1
fi

# Step 2: Ping the LiDAR
echo -e "${YELLOW}[2/4] Pinging LiDAR at ${LIDAR_IP}...${NC}"
ping -c 3 ${LIDAR_IP} > /dev/null 2>&1
if [ $? -eq 0 ]; then
  echo -e "${GREEN}✓ LiDAR is reachable via ping${NC}"
else
  echo -e "${RED}✗ Cannot ping LiDAR at ${LIDAR_IP}${NC}"
  echo -e "Possible issues:"
  echo -e "  - LiDAR is not powered on"
  echo -e "  - LiDAR IP is incorrect"
  echo -e "  - Network cable is disconnected"
  exit 1
fi

# Step 3: Test port connectivity
echo -e "${YELLOW}[3/4] Testing port connectivity...${NC}"

# Check if nc (netcat) is installed
if ! command -v nc &> /dev/null; then
  echo -e "${YELLOW}! netcat (nc) is not installed, skipping port checks${NC}"
else
  # Test command port (TCP 56000)
  nc -z -v -w 1 ${LIDAR_IP} 56000 2>&1 | grep -q "succeeded" 
  if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Command port (56000) is accessible${NC}"
  else
    echo -e "${RED}✗ Cannot access command port (56000)${NC}"
  fi

  # Test data port (UDP 57000)
  echo -e "${YELLOW}  Note: UDP ports cannot be reliably tested with nc${NC}"
  echo -e "${YELLOW}  Data port (57000) will be tested during runtime${NC}"
fi

# Step 4: Optional Nmap scan
if [ "$NMAP" = true ]; then
  echo -e "${YELLOW}[4/4] Running network scan with nmap...${NC}"
  
  # Check if nmap is installed
  if ! command -v nmap &> /dev/null; then
    echo -e "${RED}✗ nmap is not installed${NC}"
    echo -e "Install with: sudo apt install nmap"
  else
    echo -e "Scanning LiDAR for open ports..."
    NMAP_OUTPUT=$(nmap -p 56000-58000 ${LIDAR_IP})
    echo -e "${NMAP_OUTPUT}"
    
    if echo "${NMAP_OUTPUT}" | grep -q "56000.*open"; then
      echo -e "${GREEN}✓ Command port (56000) is open${NC}"
    else
      echo -e "${RED}✗ Command port (56000) is not open${NC}"
    fi
  fi
else
  echo -e "${YELLOW}[4/4] Skipping network scan (use -n to enable)${NC}"
fi

# Final summary
echo -e "${BLUE}================================${NC}"
ping -c 1 ${LIDAR_IP} > /dev/null 2>&1
if [ $? -eq 0 ]; then
  echo -e "${GREEN}CONNECTION TEST SUMMARY:${NC}"
  echo -e "${GREEN}✓ LiDAR is reachable via ping${NC}"
  
  if command -v nc &> /dev/null && nc -z -v -w 1 ${LIDAR_IP} 56000 2>&1 | grep -q "succeeded"; then
    echo -e "${GREEN}✓ Command port is accessible${NC}"
    echo -e "${GREEN}LiDAR appears to be properly connected${NC}"
  else
    echo -e "${RED}✗ Command port is not accessible${NC}"
    echo -e "${YELLOW}The LiDAR is online but not fully accessible.${NC}"
    echo -e "This could be due to:"
    echo -e "  - Firewall blocking the ports"
    echo -e "  - LiDAR software not running"
    echo -e "  - LiDAR in a special mode"
  fi
else
  echo -e "${RED}CONNECTION TEST FAILED:${NC}"
  echo -e "${RED}✗ LiDAR is not reachable${NC}"
  echo -e "Please check physical connections and power."
fi
echo -e "${BLUE}================================${NC}"

exit 0