#!/bin/bash
# Quick system check for ROS 2 Pioneer setup

echo "=== Pioneer ROS 2 System Check ==="
echo ""

# Check ROS 2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash 2>/dev/null
    echo "✓ ROS 2 sourced"
else
    echo "✗ ROS 2 not found"
    exit 1
fi

echo ""
echo "1. Supervisor Node Status:"
if pgrep -f "supervisor_node" > /dev/null; then
    echo "   ✓ Running (PID: $(pgrep -f supervisor_node))"
else
    echo "   ✗ NOT RUNNING - Start with: ./start_supervisor_node.sh"
fi

echo ""
echo "2. ROS Nodes:"
ros2 node list 2>/dev/null | grep -E "(supervisor|printer)" || echo "   (none found)"

echo ""
echo "3. Robot Odometry Topics:"
TOPIC_COUNT=$(ros2 topic list 2>/dev/null | grep -E "robot_[1-5]/odom" | wc -l)
if [ "$TOPIC_COUNT" -eq 5 ]; then
    echo "   ✓ All 5 topics exist"
else
    echo "   ✗ Only $TOPIC_COUNT/5 topics found"
fi

echo ""
echo "4. Topic Publishing Status:"
for i in {1..5}; do
    RATE_OUTPUT=$(timeout 2 ros2 topic hz "/robot_$i/odom" 2>&1)
    if echo "$RATE_OUTPUT" | grep -q "average rate"; then
        RATE=$(echo "$RATE_OUTPUT" | grep "average rate" | tail -1 | awk '{print $3}')
        echo "   ✓ robot_$i/odom: $RATE Hz"
    else
        # Try echo to verify topic exists
        if timeout 1 ros2 topic echo "/robot_$i/odom" --once > /dev/null 2>&1; then
            echo "   ? robot_$i/odom: Publishing (rate check failed)"
        else
            echo "   ✗ robot_$i/odom: NOT PUBLISHING"
        fi
    fi
done

echo ""
echo "5. Pose File:"
if [ -f "/tmp/pioneer_poses.txt" ]; then
    FILE_SIZE=$(wc -l < /tmp/pioneer_poses.txt)
    if [ "$FILE_SIZE" -gt 0 ]; then
        echo "   ✓ Exists with $FILE_SIZE lines"
        echo "   Latest: $(tail -1 /tmp/pioneer_poses.txt | cut -d: -f1-4)"
    else
        echo "   ✗ Exists but empty"
    fi
else
    echo "   ✗ Not found - Webots supervisor controller may not be running"
fi

echo ""
echo "6. Webots Supervisor Controller:"
if pgrep -f "controllers/supervisor/supervisor" > /dev/null; then
    echo "   ✓ Running"
else
    echo "   ✗ Not running - Check Webots simulation"
fi

echo ""
echo "=== Summary ==="
if pgrep -f "supervisor_node" > /dev/null && [ -f "/tmp/pioneer_poses.txt" ]; then
    echo "System should be working! Topics should be publishing."
else
    echo "Issues found above. Fix them and rerun this check."
fi

