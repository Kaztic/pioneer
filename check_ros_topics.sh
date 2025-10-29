#!/bin/bash
# Quick script to check ROS 2 topics

echo "=== ROS 2 Topic Checker ==="
echo ""

# Source ROS 2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ Sourced ROS 2 Humble"
else
    echo "✗ ROS 2 not found at /opt/ros/humble/setup.bash"
    exit 1
fi

echo ""
echo "1. Listing all topics..."
echo "---"
ros2 topic list

echo ""
echo "2. Checking robot odometry topics..."
echo "---"

for i in {1..5}; do
    topic="/robot_$i/odom"
    echo -n "Checking $topic: "
    if ros2 topic info $topic &>/dev/null; then
        echo "✓ EXISTS"
        echo "   Publishing rate:"
        timeout 1 ros2 topic hz $topic 2>&1 | head -3 || echo "   (checking...)"

    else
        echo "✗ NOT FOUND"
    fi
done

echo ""
echo "3. Checking TF topic..."
echo "---"
if ros2 topic info /tf &>/dev/null; then
    echo "✓ /tf topic exists"
else
    echo "✗ /tf topic not found"
fi

echo ""
echo "=== To view robot positions, use: ==="
echo "  ros2 topic echo /robot_1/odom"
echo "  ros2 topic echo /robot_2/odom"
echo "  etc."


