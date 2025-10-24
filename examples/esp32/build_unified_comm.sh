#!/bin/bash

# Build script for TMC9660 Unified Communication Interface Example
# This script builds the ESP32 example that demonstrates real-time SPI/UART switching

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}TMC9660 Unified Communication Interface Build Script${NC}"
echo "=============================================================="

# Check if we're in the right directory
if [ ! -f "main/UnifiedCommMain.cpp" ]; then
    echo -e "${RED}Error: Please run this script from the examples/esp32 directory${NC}"
    exit 1
fi

# Set build type (default to Debug)
BUILD_TYPE=${1:-Debug}
echo -e "${YELLOW}Build type: ${BUILD_TYPE}${NC}"

# Clean previous build
echo "Cleaning previous build..."
idf.py fullclean

# Configure the project
echo "Configuring project..."
idf.py set-target esp32c6

# Set the main source file
export APP_SOURCE_FILE="UnifiedCommMain.cpp"
export APP_TYPE="UnifiedComm"
export BUILD_TYPE="${BUILD_TYPE}"

echo -e "${YELLOW}Building with source file: ${APP_SOURCE_FILE}${NC}"

# Build the project
echo "Building project..."
idf.py build

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ Build successful!${NC}"
    echo ""
    echo "To flash the firmware:"
    echo "  idf.py -p /dev/ttyUSB0 flash monitor"
    echo ""
    echo "To flash and monitor:"
    echo "  idf.py -p /dev/ttyUSB0 flash monitor"
    echo ""
    echo "Note: Replace /dev/ttyUSB0 with your actual serial port"
else
    echo -e "${RED}❌ Build failed!${NC}"
    exit 1
fi