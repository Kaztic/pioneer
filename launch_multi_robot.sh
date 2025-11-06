#!/bin/bash
#
# Multi-Robot Launch Script
# Launches LiDAR bridge, occupancy grid builder, frontier detector, goal selector,
# path planner, path follower, and cmd_vel bridge nodes for all specified robots
#
# REQUIRED: Supervisor node must be running first (./start_supervisor_node.sh)
# REQUIRED: Webots simulation must be running (./launch_with_ros2.sh)
#
# Usage:
#   ./launch_multi_robot.sh                    # Launch all 5 robots (default)
#   ./launch_multi_robot.sh robot_1 robot_2    # Launch specific robots
#   ./launch_multi_robot.sh --robot-list robot_1 robot_3 robot_5
#

# Don't exit on error in verification sections
# set -e  # Exit on error - disabled for better error handling

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default: All 5 robots
DEFAULT_ROBOTS=("robot_1" "robot_2" "robot_3" "robot_4" "robot_5")

# Parse command line arguments
ROBOTS=()
if [ $# -eq 0 ]; then
    # No arguments: use all robots
    ROBOTS=("${DEFAULT_ROBOTS[@]}")
elif [ "$1" == "--robot-list" ]; then
    # Explicit list provided
    shift
    ROBOTS=("$@")
else
    # Robot IDs as arguments
    ROBOTS=("$@")
fi

# Validate robot IDs
VALID_ROBOTS=("robot_1" "robot_2" "robot_3" "robot_4" "robot_5")
for robot in "${ROBOTS[@]}"; do
    if [[ ! " ${VALID_ROBOTS[@]} " =~ " ${robot} " ]]; then
        echo -e "${RED}✗${NC} Error: Invalid robot ID: $robot"
        echo "Valid IDs: ${VALID_ROBOTS[*]}"
        exit 1
    fi
done

# Configuration
LOG_DIR="logs"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_PREFIX="${LOG_DIR}/multi_robot_${TIMESTAMP}"

# Storage for PIDs (robot_id -> array of PIDs)
declare -A ROBOT_PIDS

# Cleanup function
cleanup() {
    echo -e "\n${YELLOW}Cleaning up processes...${NC}"
    
    # Kill all nodes for all robots
    for robot_id in "${ROBOTS[@]}"; do
        if [ -n "${ROBOT_PIDS[$robot_id]}" ]; then
            echo -e "${BLUE}Stopping nodes for ${robot_id}...${NC}"
            # Kill each PID in the array
            IFS=',' read -ra PIDS <<< "${ROBOT_PIDS[$robot_id]}"
            for pid in "${PIDS[@]}"; do
                kill $pid 2>/dev/null || true
            done
        fi
    done
    
    # Also kill by process name (safety)
    pkill -f "lidar_bridge_node" || true
    pkill -f "occupancy_grid_builder_node" || true
    pkill -f "frontier_detector_node" || true
    pkill -f "goal_selector_node" || true
    pkill -f "path_planner_node" || true
    pkill -f "path_follower_node" || true
    pkill -f "cmd_vel_bridge_node" || true
    
    sleep 2
    echo -e "${GREEN}Cleanup complete${NC}"
}

# Trap Ctrl+C and call cleanup
trap cleanup EXIT INT TERM

echo -e "${GREEN}╔════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  Multi-Robot Launch Script                       ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${BLUE}Robots to launch: ${ROBOTS[*]}${NC}"
echo ""

# Create logs directory
mkdir -p "${LOG_PREFIX}"
echo -e "${GREEN}✓${NC} Log directory created: ${LOG_PREFIX}"

# ═══════════════════════════════════════════════════════════════
# CRITICAL PRE-FLIGHT CHECKS
# ═══════════════════════════════════════════════════════════════

echo ""
echo -e "${YELLOW}═══ Pre-flight Checks ═══${NC}"

# Check 1: Supervisor controller binary exists
# Get script directory if not already set
if [ -z "$SCRIPT_DIR" ]; then
    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
fi

SUPERVISOR_BIN="$SCRIPT_DIR/controllers/supervisor/supervisor"
if [ ! -f "$SUPERVISOR_BIN" ]; then
    echo -e "${RED}✗${NC} Supervisor controller not built"
    echo -e "${YELLOW}  Building...${NC}"
    cd "$SCRIPT_DIR/controllers/supervisor" || {
        echo -e "${RED}  ✗${NC} Cannot find supervisor directory: $SCRIPT_DIR/controllers/supervisor"
        exit 1
    }
    if make clean && make; then
        echo -e "${GREEN}  ✓${NC} Supervisor built"
    else
        echo -e "${RED}  ✗${NC} Build failed"
        exit 1
    fi
    cd "$SCRIPT_DIR"
else
    echo -e "${GREEN}  ✓${NC} Supervisor controller binary exists"
fi

# Check 2: Webots is running
if ! pgrep -f "webots" > /dev/null; then
    echo -e "${RED}✗${NC} Webots is NOT running!"
    echo -e "${YELLOW}  Start with: ${BLUE}webots $SCRIPT_DIR/worlds/pioneer_world.wbt${NC}"
    exit 1
fi
echo -e "${GREEN}  ✓${NC} Webots is running"

# Check 3: Supervisor is writing poses (CRITICAL!)
POSE_FILE="/tmp/pioneer_poses.txt"
echo -e "${YELLOW}  Checking supervisor controller...${NC}"
if [ ! -f "$POSE_FILE" ]; then
    echo -e "${YELLOW}    ⚠  Pose file not found. Waiting 5 seconds...${NC}"
    sleep 5
    if [ ! -f "$POSE_FILE" ]; then
        echo ""
        echo -e "${RED}╔══════════════════════════════════════════════════════════╗${NC}"
        echo -e "${RED}║  CRITICAL: Supervisor Controller Not Running            ║${NC}"
        echo -e "${RED}╚══════════════════════════════════════════════════════════╝${NC}"
        echo ""
        echo -e "${YELLOW}The Webots supervisor is NOT writing pose data to:${NC}"
        echo -e "  ${BLUE}/tmp/pioneer_poses.txt${NC}"
        echo ""
        echo -e "${YELLOW}Without poses, the ROS2 system CANNOT work:${NC}"
        echo -e "  • No odometry → No path planning"
        echo -e "  • No navigation → Robots won't move autonomously"
        echo ""
        echo -e "${YELLOW}FIX (Required):${NC}"
        echo -e "  ${BLUE}1.${NC} Close Webots COMPLETELY"
        echo -e "  ${BLUE}2.${NC} Reopen: ${BLUE}webots $SCRIPT_DIR/worlds/pioneer_world.wbt${NC}"
        echo -e "  ${BLUE}3.${NC} Wait 3 seconds for supervisor to load"
        echo -e "  ${BLUE}4.${NC} Verify: ${BLUE}cat /tmp/pioneer_poses.txt${NC}"
        echo -e "      (Should show: POSE:robot_1:... POSE:robot_2:... etc.)"
        echo -e "  ${BLUE}5.${NC} Run this script again"
        echo ""
        exit 1
    fi
fi

# Check 4: Pose file is fresh (being updated)
POSE_FILE_AGE=$(stat -c %Y "$POSE_FILE" 2>/dev/null || echo 0)
CURRENT_TIME=$(date +%s)
AGE_SECONDS=$((CURRENT_TIME - POSE_FILE_AGE))
if [ $AGE_SECONDS -gt 5 ]; then
    echo -e "${RED}✗${NC} Pose file is stale (${AGE_SECONDS}s old)"
    echo -e "${YELLOW}  Supervisor crashed. Restart Webots.${NC}"
    exit 1
fi
echo -e "${GREEN}  ✓${NC} Supervisor controller is ACTIVE"

# Check 5: Clean up crashed nodes
echo -e "${YELLOW}  Cleaning up old nodes...${NC}"
pkill -f "pioneer_ros2" 2>/dev/null || true
pkill -f "pioneer_path_planner" 2>/dev/null || true
sleep 1
echo -e "${GREEN}  ✓${NC} Pre-flight checks complete"

echo ""

# Source ROS2 workspace
if [ -f "ros2_ws/install/setup.bash" ]; then
    source ros2_ws/install/setup.bash
    echo -e "${GREEN}✓${NC} ROS2 workspace sourced"
else
    echo -e "${RED}✗${NC} Error: ROS2 workspace not found. Run 'colcon build' first."
    exit 1
fi

# Start supervisor node (if not already running)
if ! pgrep -f "supervisor_node" > /dev/null; then
    echo -e "${YELLOW}Starting supervisor node...${NC}"
    ros2 run pioneer_ros2 supervisor_node > "${LOG_PREFIX}/supervisor.log" 2>&1 &
    SUPERVISOR_PID=$!
    sleep 2
    if ps -p $SUPERVISOR_PID > /dev/null 2>&1; then
        echo -e "${GREEN}  ✓${NC} Supervisor node started (PID: $SUPERVISOR_PID)"
    else
        echo -e "${RED}  ✗${NC} Supervisor node failed to start. Check ${LOG_PREFIX}/supervisor.log"
        exit 1
    fi
else
    echo -e "${GREEN}  ✓${NC} Supervisor node already running"
fi

echo ""
echo -e "${GREEN}Starting ROS2 nodes for ${#ROBOTS[@]} robot(s)...${NC}"
echo ""

# Function to launch nodes for a single robot
launch_robot_nodes() {
    local robot_id=$1
    local robot_log_dir="${LOG_PREFIX}/${robot_id}"
    mkdir -p "$robot_log_dir"
    
    local pids=()
    
    echo -e "${BLUE}[${robot_id}]${NC} Launching nodes..."
    
    # Start LiDAR Bridge Node (needed by occupancy grid builder)
    ros2 run pioneer_ros2 lidar_bridge_node \
        --ros-args -r __ns:=/${robot_id} \
        > "${robot_log_dir}/lidar_bridge.log" 2>&1 &
    local lidar_bridge_pid=$!
    pids+=($lidar_bridge_pid)
    sleep 1
    
    # Start Occupancy Grid Builder Node (needed by path planner)
    ros2 run pioneer_path_planner occupancy_grid_builder_node \
        --ros-args -r __ns:=/${robot_id} \
        > "${robot_log_dir}/occupancy_grid_builder.log" 2>&1 &
    local grid_builder_pid=$!
    pids+=($grid_builder_pid)
    sleep 1
    
    # Start Frontier Detector Node (for autonomous exploration)
    ros2 run pioneer_path_planner frontier_detector_node \
        --ros-args -r __ns:=/${robot_id} \
        > "${robot_log_dir}/frontier_detector.log" 2>&1 &
    local frontier_detector_pid=$!
    pids+=($frontier_detector_pid)
    sleep 1
    
    # Start Goal Selector Node (selects best frontier as goal)
    ros2 run pioneer_path_planner goal_selector_node \
        --ros-args -r __ns:=/${robot_id} \
        > "${robot_log_dir}/goal_selector.log" 2>&1 &
    local goal_selector_pid=$!
    pids+=($goal_selector_pid)
    sleep 1
    
    # Start Path Planner Node
    ros2 run pioneer_path_planner path_planner_node \
        --ros-args -r __ns:=/${robot_id} \
        > "${robot_log_dir}/path_planner.log" 2>&1 &
    local planner_pid=$!
    pids+=($planner_pid)
    sleep 1
    
    # Start Path Follower Node
    ros2 run pioneer_path_planner path_follower_node \
        --ros-args -r __ns:=/${robot_id} \
        > "${robot_log_dir}/path_follower.log" 2>&1 &
    local follower_pid=$!
    pids+=($follower_pid)
    sleep 1
    
    # Start cmd_vel Bridge Node
    ros2 run pioneer_ros2 cmd_vel_bridge_node \
        --ros-args -r __ns:=/${robot_id} \
        > "${robot_log_dir}/cmd_vel_bridge.log" 2>&1 &
    local bridge_pid=$!
    pids+=($bridge_pid)
    sleep 1
    
    # Verify all nodes started
    local all_started=true
    if ! ps -p $lidar_bridge_pid > /dev/null 2>&1; then
        echo -e "${RED}  ✗${NC} LiDAR bridge failed to start"
        all_started=false
    fi
    if ! ps -p $grid_builder_pid > /dev/null 2>&1; then
        echo -e "${RED}  ✗${NC} Occupancy grid builder failed to start"
        all_started=false
    fi
    if ! ps -p $frontier_detector_pid > /dev/null 2>&1; then
        echo -e "${RED}  ✗${NC} Frontier detector failed to start"
        all_started=false
    fi
    if ! ps -p $goal_selector_pid > /dev/null 2>&1; then
        echo -e "${RED}  ✗${NC} Goal selector failed to start"
        all_started=false
    fi
    if ! ps -p $planner_pid > /dev/null 2>&1; then
        echo -e "${RED}  ✗${NC} Path planner failed to start"
        all_started=false
    fi
    if ! ps -p $follower_pid > /dev/null 2>&1; then
        echo -e "${RED}  ✗${NC} Path follower failed to start"
        all_started=false
    fi
    if ! ps -p $bridge_pid > /dev/null 2>&1; then
        echo -e "${RED}  ✗${NC} cmd_vel bridge failed to start"
        all_started=false
    fi
    
    if [ "$all_started" = true ]; then
        echo -e "${GREEN}  ✓${NC} All nodes started (PIDs: ${pids[*]})"
        # Store PIDs as comma-separated string
        ROBOT_PIDS[$robot_id]=$(IFS=','; echo "${pids[*]}")
        
        # Create summary file
        cat > "${robot_log_dir}/summary.txt" <<EOF
Robot ID: ${robot_id}
Launch Time: $(date)
PIDs:
  LiDAR Bridge: ${lidar_bridge_pid}
  Occupancy Grid Builder: ${grid_builder_pid}
  Frontier Detector: ${frontier_detector_pid}
  Goal Selector: ${goal_selector_pid}
  Path Planner: ${planner_pid}
  Path Follower: ${follower_pid}
  cmd_vel Bridge: ${bridge_pid}
Log Directory: ${robot_log_dir}
EOF
    else
        echo -e "${RED}  ✗${NC} Some nodes failed to start. Check logs in ${robot_log_dir}/"
        return 1
    fi
    
    return 0
}

# Launch nodes for each robot
FAILED_ROBOTS=()
for robot_id in "${ROBOTS[@]}"; do
    if ! launch_robot_nodes "$robot_id"; then
        FAILED_ROBOTS+=("$robot_id")
    fi
    echo ""
done

# Check if any robots failed
if [ ${#FAILED_ROBOTS[@]} -gt 0 ]; then
    echo -e "${RED}✗${NC} Failed to launch nodes for: ${FAILED_ROBOTS[*]}"
    echo -e "${YELLOW}Continuing with successfully launched robots...${NC}"
fi

# Wait for nodes to initialize
echo -e "${YELLOW}Waiting 5 seconds for nodes to initialize...${NC}"
sleep 5

# Function to check topic message rate
check_topic_rate() {
    local topic=$1
    local robot_id=$2
    local min_rate=${3:-0.1}  # Default minimum rate: 0.1 Hz
    
    local rate=$(timeout 2 ros2 topic hz $topic 2>/dev/null | tail -n 1 | grep -oE '[0-9]+\.[0-9]+' | head -n 1 || echo "0.0")
    
    if [ -n "$rate" ] && (( $(echo "$rate >= $min_rate" | bc -l) )); then
        echo -e "${GREEN}  ✓${NC} $topic (rate: ${rate} Hz)"
        return 0
    else
        echo -e "${YELLOW}  ⚠${NC} $topic (rate: ${rate} Hz - may need more time)"
        return 1
    fi
}

# Function to assert required topics exist
assert_topics_exist() {
    local robot_id=$1
    local required_topics=(
        "${robot_id}/scan"
        "${robot_id}/odom"
        "${robot_id}/occupancy_map"
        "${robot_id}/frontiers"
        "${robot_id}/goal"
        "${robot_id}/global_path"
        "${robot_id}/cmd_vel"
    )
    local missing_topics=()
    
    for topic in "${required_topics[@]}"; do
        if ! ros2 topic list 2>/dev/null | grep -q "/${topic}" > /dev/null 2>&1; then
            missing_topics+=("${topic}")
        fi
    done
    
    if [ ${#missing_topics[@]} -gt 0 ]; then
        echo -e "${YELLOW}  ⚠${NC} Missing topics for ${robot_id}: ${missing_topics[*]}"
        return 1
    else
        return 0
    fi
}

# Verify topics for all robots
echo ""
echo -e "${GREEN}Verifying topics...${NC}"
TOPIC_LOG="${LOG_PREFIX}/topics.log"
ros2 topic list > "$TOPIC_LOG" 2>&1

SUCCESS_COUNT=0
TOPIC_ASSERTIONS_FAILED=0
for robot_id in "${ROBOTS[@]}"; do
    if [[ ! " ${FAILED_ROBOTS[@]} " =~ " ${robot_id} " ]]; then
        echo -e "${BLUE}[${robot_id}]${NC} Topics:"
        
        # Assert required topics
        if ! assert_topics_exist "$robot_id"; then
            ((TOPIC_ASSERTIONS_FAILED++))
        fi
        
        # Check topic existence and message rates
        if grep -q "/${robot_id}/odom" "$TOPIC_LOG"; then
            check_topic_rate "/${robot_id}/odom" "$robot_id" 5.0
            ((SUCCESS_COUNT++))
        else
            echo -e "${YELLOW}  ⚠${NC} /${robot_id}/odom not found (supervisor node must be running!)"
        fi
        
        if grep -q "/${robot_id}/scan" "$TOPIC_LOG"; then
            check_topic_rate "/${robot_id}/scan" "$robot_id" 10.0
        else
            echo -e "${YELLOW}  ⚠${NC} /${robot_id}/scan not found"
        fi
        
        if grep -q "/${robot_id}/occupancy_map" "$TOPIC_LOG"; then
            check_topic_rate "/${robot_id}/occupancy_map" "$robot_id" 1.0
        else
            echo -e "${YELLOW}  ⚠${NC} /${robot_id}/occupancy_map not found"
        fi
        
        if grep -q "/${robot_id}/frontiers" "$TOPIC_LOG"; then
            echo -e "${GREEN}  ✓${NC} /${robot_id}/frontiers"
        else
            echo -e "${YELLOW}  ⚠${NC} /${robot_id}/frontiers not found (may appear after exploration starts)"
        fi
        
        if grep -q "/${robot_id}/goal" "$TOPIC_LOG"; then
            echo -e "${GREEN}  ✓${NC} /${robot_id}/goal"
        else
            echo -e "${YELLOW}  ⚠${NC} /${robot_id}/goal not found"
        fi
        
        if grep -q "/${robot_id}/global_path" "$TOPIC_LOG"; then
            echo -e "${GREEN}  ✓${NC} /${robot_id}/global_path"
        else
            echo -e "${YELLOW}  ⚠${NC} /${robot_id}/global_path not found"
        fi
        
        if grep -q "/${robot_id}/cmd_vel" "$TOPIC_LOG"; then
            echo -e "${GREEN}  ✓${NC} /${robot_id}/cmd_vel"
        else
            echo -e "${YELLOW}  ⚠${NC} /${robot_id}/cmd_vel not found"
        fi
        echo ""
    fi
done

# Create master summary
MASTER_SUMMARY="${LOG_PREFIX}/master_summary.txt"
{
    echo "=========================================="
    echo "Multi-Robot Launch Summary"
    echo "=========================================="
    echo "Timestamp: $(date)"
    echo "Robots Launched: ${ROBOTS[*]}"
    echo "Total Robots: ${#ROBOTS[@]}"
    echo ""
    echo "Log Directory: ${LOG_PREFIX}"
    echo ""
    echo "Robot Status:"
    for robot_id in "${ROBOTS[@]}"; do
        if [[ " ${FAILED_ROBOTS[@]} " =~ " ${robot_id} " ]]; then
            echo "  ${robot_id}: ${RED}FAILED${NC}"
        else
            echo "  ${robot_id}: ${GREEN}RUNNING${NC}"
            echo "    PIDs: ${ROBOT_PIDS[$robot_id]}"
            echo "    Logs: ${LOG_PREFIX}/${robot_id}/"
        fi
    done
    echo ""
    echo "Topic Verification:"
    echo "  Robots with topics: ${SUCCESS_COUNT}/${#ROBOTS[@]}"
    echo ""
} > "$MASTER_SUMMARY"

echo ""
echo -e "${GREEN}════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}All robots launched!${NC}"
echo -e "${GREEN}════════════════════════════════════════════════════${NC}"
echo ""
echo "Summary: ${MASTER_SUMMARY}"
echo ""
echo "To publish goals:"
echo "  ./publish_multi_goals.sh"
echo ""
echo "To stop all nodes:"
echo "  Press Ctrl+C or run: pkill -f 'lidar_bridge_node|occupancy_grid_builder_node|frontier_detector_node|goal_selector_node|path_planner_node|path_follower_node|cmd_vel_bridge_node'"
echo ""
echo -e "${YELLOW}Nodes are running in background. Press Ctrl+C to stop all.${NC}"
echo ""

# Keep script running and monitoring
echo ""
echo "Monitoring nodes... (Press Ctrl+C to stop)"
COUNTER=0
while true; do
    sleep 1
    ((COUNTER++))
    
    # Periodically check node health (every 10 seconds)
    if [ $((COUNTER % 10)) -eq 0 ]; then
        for robot_id in "${ROBOTS[@]}"; do
            if [[ ! " ${FAILED_ROBOTS[@]} " =~ " ${robot_id} " ]]; then
                IFS=',' read -ra PIDS <<< "${ROBOT_PIDS[$robot_id]}"
                all_alive=true
                for pid in "${PIDS[@]}"; do
                    if ! ps -p $pid > /dev/null 2>&1; then
                        echo -e "${RED}✗${NC} Node (PID: $pid) for ${robot_id} died!"
                        all_alive=false
                    fi
                done
                if [ "$all_alive" = true ]; then
                    echo -e "${GREEN}✓${NC} ${robot_id} nodes healthy"
                fi
            fi
        done
    fi
done

