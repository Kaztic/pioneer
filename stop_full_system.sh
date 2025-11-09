#!/bin/bash
#
# Stop All System Components
#

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${YELLOW}Stopping all system components...${NC}"

# Stop FoxMQ brokers
if pgrep -f "foxmq run" > /dev/null; then
    pkill -f "foxmq run"
    echo -e "${GREEN}✓ Stopped FoxMQ brokers${NC}"
fi

# Stop ROS2 nodes
pkill -f "supervisor_node"
pkill -f "foxmq_bridge_node"
pkill -f "lidar_bridge_node"
pkill -f "occupancy_grid_builder_node"
pkill -f "frontier_detector_node"
pkill -f "goal_selector_node"
pkill -f "path_planner_node"
pkill -f "path_follower_node"
pkill -f "spatial_analysis_visualizer_node"

echo -e "${GREEN}✓ Stopped all ROS2 nodes${NC}"
echo -e "${GREEN}✓ System stopped${NC}"

