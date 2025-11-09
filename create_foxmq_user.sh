#!/bin/bash
#
# Helper script to create a FoxMQ user
# Uses Python script to manually add user to users.toml
#

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

echo "FoxMQ User Creator"
echo "=================="
echo ""
echo "This script will create a user by manually adding it to users.toml"
echo ""

# Check if Python script exists
if [ ! -f "create_foxmq_user_manual.py" ]; then
    echo "Error: create_foxmq_user_manual.py not found"
    exit 1
fi

# Run Python script
python3 create_foxmq_user_manual.py

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ User creation complete!"
    echo ""
    echo "Next steps:"
    echo "1. Update ros2_ws/src/pioneer_ros2/launch/foxmq_bridge.launch.py"
    echo "2. Rebuild: cd ros2_ws && colcon build --packages-select pioneer_ros2"
    echo "3. Restart bridge nodes"
else
    echo ""
    echo "Failed to create user"
    exit 1
fi

