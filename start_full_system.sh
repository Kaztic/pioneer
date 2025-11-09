#!/bin/bash
#
# Simple All-in-One System Startup Script
# Starts everything needed for the multi-robot system
#

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

echo -e "${BLUE}╔════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  Pioneer Multi-Robot System Startup              ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════╝${NC}"
echo ""

# Source ROS2
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        echo -e "${GREEN}✓ Sourced ROS 2 Humble${NC}"
    else
        echo -e "${RED}✗ Error: ROS 2 not found${NC}"
        exit 1
    fi
fi

# Build workspace if needed
if [ ! -f "ros2_ws/install/setup.bash" ]; then
    echo -e "${YELLOW}Building ROS2 workspace...${NC}"
    cd ros2_ws
    colcon build
    cd ..
fi

# Source workspace
source ros2_ws/install/setup.bash
echo -e "${GREEN}✓ ROS2 workspace sourced${NC}"

# Step 1: Start FoxMQ Cluster
echo ""
echo -e "${YELLOW}[1/5] Starting FoxMQ cluster...${NC}"
if pgrep -f "foxmq run" > /dev/null; then
    echo -e "${GREEN}✓ FoxMQ cluster already running${NC}"
else
    cd foxmq_cluster
    nohup ./foxmq run --secret-key-file=foxmq.d/key_0.pem --mqtt-addr=0.0.0.0:1883 --cluster-addr=0.0.0.0:19793 > /tmp/foxmq_broker_0.log 2>&1 &
    sleep 1
    nohup ./foxmq run --secret-key-file=foxmq.d/key_1.pem --mqtt-addr=0.0.0.0:1884 --cluster-addr=0.0.0.0:19794 > /tmp/foxmq_broker_1.log 2>&1 &
    sleep 1
    nohup ./foxmq run --secret-key-file=foxmq.d/key_2.pem --mqtt-addr=0.0.0.0:1885 --cluster-addr=0.0.0.0:19795 > /tmp/foxmq_broker_2.log 2>&1 &
    sleep 1
    nohup ./foxmq run --secret-key-file=foxmq.d/key_3.pem --mqtt-addr=0.0.0.0:1886 --cluster-addr=0.0.0.0:19796 > /tmp/foxmq_broker_3.log 2>&1 &
    sleep 2
    cd ..
    if pgrep -f "foxmq run" > /dev/null; then
        echo -e "${GREEN}✓ FoxMQ cluster started (4 brokers)${NC}"
    else
        echo -e "${RED}✗ Failed to start FoxMQ cluster${NC}"
        exit 1
    fi
fi

# Step 2: Start Supervisor Node
echo ""
echo -e "${YELLOW}[2/5] Starting supervisor node...${NC}"
if pgrep -f "supervisor_node" > /dev/null; then
    echo -e "${GREEN}✓ Supervisor node already running${NC}"
else
    nohup ros2 run pioneer_ros2 supervisor_node > /tmp/supervisor_node.log 2>&1 &
    sleep 2
    if pgrep -f "supervisor_node" > /dev/null; then
        echo -e "${GREEN}✓ Supervisor node started${NC}"
    else
        echo -e "${RED}✗ Failed to start supervisor node${NC}"
        exit 1
    fi
fi

# Step 3: Start Bridge Nodes
echo ""
echo -e "${YELLOW}[3/5] Starting FoxMQ bridge nodes...${NC}"
if pgrep -f "foxmq_bridge_node" > /dev/null; then
    echo -e "${GREEN}✓ Bridge nodes already running${NC}"
else
    nohup ros2 launch pioneer_ros2 foxmq_bridge.launch.py > /tmp/foxmq_bridge.log 2>&1 &
    sleep 3
    if pgrep -f "foxmq_bridge_node" > /dev/null; then
        echo -e "${GREEN}✓ Bridge nodes started (5 robots)${NC}"
    else
        echo -e "${RED}✗ Failed to start bridge nodes${NC}"
        exit 1
    fi
fi

# Step 4: Start LiDAR Bridge Node (required for occupancy grid)
echo ""
echo -e "${YELLOW}[4/6] Starting LiDAR bridge node (robot_1)...${NC}"
if pgrep -f "lidar_bridge_node.*robot_1" > /dev/null; then
    echo -e "${GREEN}✓ LiDAR bridge node already running${NC}"
else
    nohup ros2 run pioneer_ros2 lidar_bridge_node --ros-args -r __ns:=/robot_1 > /tmp/robot_1_lidar.log 2>&1 &
    sleep 2
    if pgrep -f "lidar_bridge_node.*robot_1" > /dev/null; then
        echo -e "${GREEN}✓ LiDAR bridge node started${NC}"
    else
        echo -e "${RED}✗ Failed to start LiDAR bridge node${NC}"
        exit 1
    fi
fi

# Step 5: Start Exploration Nodes (one robot for testing)
echo ""
echo -e "${YELLOW}[5/6] Starting exploration nodes (robot_1 only for testing)...${NC}"
if pgrep -f "occupancy_grid_builder_node.*robot_1" > /dev/null; then
    echo -e "${GREEN}✓ Exploration nodes already running${NC}"
else
    nohup ros2 run pioneer_path_planner occupancy_grid_builder_node --ros-args -r __ns:=/robot_1 > /tmp/robot_1_grid.log 2>&1 &
    sleep 1
    nohup ros2 run pioneer_path_planner frontier_detector_node --ros-args -r __ns:=/robot_1 > /tmp/robot_1_frontier.log 2>&1 &
    sleep 1
    nohup ros2 run pioneer_path_planner goal_selector_node --ros-args -r __ns:=/robot_1 > /tmp/robot_1_goal.log 2>&1 &
    sleep 1
    nohup ros2 run pioneer_path_planner path_planner_node --ros-args -r __ns:=/robot_1 > /tmp/robot_1_planner.log 2>&1 &
    sleep 1
    nohup ros2 run pioneer_path_planner path_follower_node --ros-args -r __ns:=/robot_1 > /tmp/robot_1_follower.log 2>&1 &
    sleep 2
    echo -e "${GREEN}✓ Exploration nodes started for robot_1${NC}"
fi

# Step 6: Start Spatial Analysis (optional)
echo ""
echo -e "${YELLOW}[6/6] Starting spatial analysis visualizer...${NC}"
if pgrep -f "spatial_analysis_visualizer_node" > /dev/null; then
    echo -e "${GREEN}✓ Spatial analysis already running${NC}"
else
    nohup ros2 launch pioneer_ros2 spatial_analysis.launch.py > /tmp/spatial_analysis.log 2>&1 &
    sleep 2
    echo -e "${GREEN}✓ Spatial analysis started${NC}"
fi

# Summary
echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  System Started Successfully!                    ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${BLUE}Running processes:${NC}"
echo "  - FoxMQ brokers: $(pgrep -f 'foxmq run' | wc -l)"
echo "  - Supervisor node: $(pgrep -f 'supervisor_node' | wc -l)"
echo "  - Bridge nodes: $(pgrep -f 'foxmq_bridge_node' | wc -l)"
echo "  - LiDAR bridge: $(pgrep -f 'lidar_bridge_node.*robot_1' | wc -l)"
echo "  - Exploration nodes: $(pgrep -f 'occupancy_grid_builder_node.*robot_1' | wc -l)"
echo ""
echo -e "${YELLOW}Next steps:${NC}"
echo "  1. Start Webots: webots worlds/pioneer_world.wbt"
echo "  2. View RViz: rviz2 -d pioneer_spatial_analysis.rviz"
echo "  3. Check logs: tail -f /tmp/*.log"
echo ""
echo -e "${YELLOW}To stop everything:${NC}"
echo "  ./stop_full_system.sh"
echo ""

