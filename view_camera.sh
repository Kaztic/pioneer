#!/bin/bash
#
# View camera images from robots using rqt_image_view
#

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Source ROS2
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi
fi

# Source workspace
source ros2_ws/install/setup.bash

# Default to robot_1 if no argument provided
ROBOT_ID=${1:-robot_1}
TOPIC="/${ROBOT_ID}/camera/image_raw"

echo "Viewing camera from ${ROBOT_ID} on topic: ${TOPIC}"
echo "Press Ctrl+C to exit"
echo ""

# Run rqt_image_view
ros2 run rqt_image_view rqt_image_view ${TOPIC}
