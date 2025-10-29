#!/bin/bash
# Script to start the ROS 2 supervisor node

echo "Starting ROS 2 Supervisor Node..."
echo ""

# Source ROS 2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Error: ROS 2 not found at /opt/ros/humble/setup.bash"
    exit 1
fi

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Source workspace
cd "$SCRIPT_DIR/ros2_ws"
if [ ! -f "install/setup.bash" ]; then
    echo "Error: ROS 2 workspace not built. Run: cd ros2_ws && colcon build"
    exit 1
fi

source install/setup.bash

# Check if already running
if pgrep -f "supervisor_node" > /dev/null; then
    echo "Supervisor node is already running!"
    echo "To stop it: pkill -f supervisor_node"
    exit 0
fi

# Start the node
echo "Launching supervisor node..."
echo "Press Ctrl+C to stop"
echo ""

cd "$SCRIPT_DIR"
ros2 run pioneer_ros2 supervisor_node

