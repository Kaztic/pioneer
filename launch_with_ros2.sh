#!/bin/bash

################################################################################
# Launch Pioneer Simulation with ROS 2 Integration
# 
# This script:
# 1. Builds supervisor controller if needed
# 2. Starts ROS 2 supervisor node in background
# 3. Launches Webots simulation
# 4. Handles cleanup on exit
################################################################################

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROS2_WS="${SCRIPT_DIR}/ros2_ws"
SUPERVISOR_DIR="${SCRIPT_DIR}/controllers/supervisor"
WORLD_FILE="${SCRIPT_DIR}/worlds/pioneer_world.wbt"

echo -e "${BLUE}╔════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  Pioneer ROS 2 Integration Launcher             ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════╝${NC}"
echo ""

# Check ROS 2 installation
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo -e "${GREEN}✓ Sourced ROS 2 Humble${NC}"
    else
        echo -e "${RED}✗ Error: ROS 2 not found${NC}"
        echo -e "${YELLOW}  Please install ROS 2 Humble or source it manually${NC}"
        exit 1
    fi
fi

# Build supervisor controller
echo -e "${YELLOW}Building supervisor controller...${NC}"
cd "$SUPERVISOR_DIR"
if make clean && make; then
    echo -e "${GREEN}✓ Supervisor controller built${NC}"
else
    echo -e "${RED}✗ Error: Failed to build supervisor controller${NC}"
    exit 1
fi

# Build ROS 2 package
echo -e "${YELLOW}Building ROS 2 package...${NC}"
cd "$ROS2_WS"
if [ ! -f "install/setup.bash" ]; then
    if colcon build; then
        echo -e "${GREEN}✓ ROS 2 package built${NC}"
    else
        echo -e "${RED}✗ Error: Failed to build ROS 2 package${NC}"
        exit 1
    fi
fi

source install/setup.bash
echo -e "${GREEN}✓ ROS 2 workspace sourced${NC}"

# Check if Webots is installed
if ! command -v webots &> /dev/null; then
    echo -e "${RED}✗ Error: Webots not found${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Webots found: $(which webots)${NC}"

# Create shared file for communication
POSE_FILE="/tmp/pioneer_poses.txt"
touch "$POSE_FILE"
echo -e "${GREEN}✓ Created communication file: $POSE_FILE${NC}"

# Start ROS 2 supervisor node in background
echo ""
echo -e "${YELLOW}Starting ROS 2 supervisor node...${NC}"
ros2 run pioneer_ros2 supervisor_node > /tmp/ros2_supervisor.log 2>&1 &
ROS2_PID=$!
echo -e "${GREEN}✓ ROS 2 node started (PID: $ROS2_PID)${NC}"

# Wait a moment for ROS 2 to initialize
sleep 2

# Function to cleanup on exit
cleanup() {
    echo ""
    echo -e "${YELLOW}Cleaning up...${NC}"
    kill $ROS2_PID 2>/dev/null
    rm -f "$POSE_FILE"
    echo -e "${GREEN}✓ Cleanup complete${NC}"
}

trap cleanup EXIT INT TERM

# Launch Webots
echo ""
echo -e "${YELLOW}Launching Webots...${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════${NC}"
echo ""
cd "$SCRIPT_DIR"
webots "$WORLD_FILE"

# Cleanup will happen automatically via trap


