#!/bin/bash
# Camera Status Checker

echo "=== Camera Integration Status Check ==="
echo ""

echo "1. Checking if Webots is running..."
if pgrep -f webots > /dev/null; then
    echo "✓ Webots is running"
else
    echo "✗ Webots is NOT running - start it first!"
    exit 1
fi

echo ""
echo "2. Checking camera bridge nodes..."
CAMERA_NODES=$(pgrep -f camera_bridge_node | wc -l)
if [ "$CAMERA_NODES" -gt 0 ]; then
    echo "✓ Camera bridge nodes running: $CAMERA_NODES"
else
    echo "✗ Camera bridge nodes not running"
fi

echo ""
echo "3. Checking camera files..."
CAMERA_FILES=$(ls /tmp/pioneer_camera_*.txt 2>/dev/null | wc -l)
if [ "$CAMERA_FILES" -gt 0 ]; then
    echo "✓ Camera files found: $CAMERA_FILES"
    ls -lh /tmp/pioneer_camera_*.txt | head -5
else
    echo "✗ No camera files found in /tmp/"
    echo ""
    echo "   This means the controller isn't creating camera files."
    echo "   Possible causes:"
    echo "   - Camera device name doesn't match 'camera' in Webots"
    echo "   - Controller needs to be restarted in Webots"
    echo "   - Camera not properly attached to robots"
    echo ""
    echo "   Check Webots console for messages like:"
    echo "   'WARNING: Camera device not found for robot_X (device name: 'camera')'"
fi

echo ""
echo "4. Checking ROS2 camera topics..."
source ros2_ws/install/setup.bash 2>/dev/null
if ros2 topic list 2>/dev/null | grep -q "camera/image_raw"; then
    echo "✓ Camera topics exist:"
    ros2 topic list | grep camera | head -5
    echo ""
    echo "   Checking if topics have data..."
    if timeout 2 ros2 topic hz /robot_1/camera/image_raw 2>&1 | grep -q "average rate"; then
        echo "✓ Camera images are being published!"
    else
        echo "✗ Camera topics exist but no data (waiting for camera files)"
    fi
else
    echo "✗ Camera topics not found"
fi

echo ""
echo "5. Checking controller binary..."
if [ -f "controllers/controller/controller" ]; then
    echo "✓ Controller binary exists"
    echo "   Last modified: $(stat -c %y controllers/controller/controller | cut -d. -f1)"
else
    echo "✗ Controller binary not found - need to rebuild!"
fi

echo ""
echo "=== Summary ==="
if [ "$CAMERA_FILES" -gt 0 ]; then
    echo "✓ Camera pipeline appears to be working!"
else
    echo "⚠ Camera files not being created - check Webots console for camera device warnings"
    echo ""
    echo "To fix:"
    echo "1. Check Webots console for camera device warnings"
    echo "2. Verify camera device name is exactly 'camera' (case-sensitive)"
    echo "3. Restart Webots or reload the world to restart controllers"
fi


