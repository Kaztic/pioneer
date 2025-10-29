#!/bin/bash

################################################################################
# Pioneer Multi-Robot Launch Script
# 
# This script:
# 1. Compiles the pioneer_controller if needed
# 2. Launches Webots with the pioneer world
# 3. Automatically starts the simulation with all 5 robots
################################################################################

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo -e "${BLUE}╔════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  Pioneer Multi-Robot Simulation Launcher          ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════╝${NC}"
echo ""

# Check if Webots is installed
if ! command -v webots &> /dev/null; then
    echo -e "${RED}✗ Error: Webots is not installed or not in PATH${NC}"
    echo -e "${YELLOW}  Please install Webots or add it to your PATH${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Webots found: $(which webots)${NC}"

# Navigate to controller directory
CONTROLLER_DIR="${SCRIPT_DIR}/controllers/controller"

if [ ! -d "$CONTROLLER_DIR" ]; then
    echo -e "${RED}✗ Error: Controller directory not found${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Controller directory found${NC}"

# Build the controller
echo ""
echo -e "${YELLOW}Building controller...${NC}"
cd "$CONTROLLER_DIR"

if [ ! -f "Makefile" ]; then
    echo -e "${RED}✗ Error: Makefile not found in controller directory${NC}"
    exit 1
fi

# Clean and build
make clean > /dev/null 2>&1
if make; then
    echo -e "${GREEN}✓ Controller compiled successfully${NC}"
else
    echo -e "${RED}✗ Error: Failed to compile controller${NC}"
    echo -e "${YELLOW}  Check the compilation errors above${NC}"
    exit 1
fi

# Check if world file exists
WORLD_FILE="${SCRIPT_DIR}/worlds/pioneer_world.wbt"

if [ ! -f "$WORLD_FILE" ]; then
    echo -e "${RED}✗ Error: World file not found: $WORLD_FILE${NC}"
    exit 1
fi

echo -e "${GREEN}✓ World file found${NC}"

# Summary
echo ""
echo -e "${BLUE}════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}Configuration Summary:${NC}"
echo -e "  World File:  ${WORLD_FILE}"
echo -e "  Controller:  controller (Braitenberg obstacle avoidance)"
echo -e "  Robots:      5 Pioneer3at robots with Sick LMS 291 lidar"
echo -e "${BLUE}════════════════════════════════════════════════════${NC}"
echo ""

# Launch Webots
echo -e "${YELLOW}Launching Webots...${NC}"
echo ""

cd "$SCRIPT_DIR"

# Launch with different options based on preference
# Default: launch normally
# You can modify this to use --mode=fast, --minimize, etc.

webots "$WORLD_FILE"

echo ""
echo -e "${GREEN}✓ Simulation ended${NC}"


