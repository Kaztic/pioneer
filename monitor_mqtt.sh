#!/bin/bash
#
# MQTT Message Monitor for Pioneer System
# Monitors MQTT topics via FoxMQ broker
#

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# MQTT broker configuration (matches foxmq_bridge_node.py)
# FoxMQ cluster has multiple brokers:
# - Broker 0: port 1883 (default, used by bridge nodes)
# - Broker 1: port 1884
# - Broker 2: port 1885
# - Broker 3: port 1886
MQTT_HOST="${MQTT_HOST:-127.0.0.1}"
MQTT_PORT="${MQTT_PORT:-1883}"
MQTT_USER="${MQTT_USER:-pioneer_robot}"
# Default password matches foxmq_bridge.launch.py configuration
MQTT_PASS="${MQTT_PASS:-test123}"

echo -e "${GREEN}╔════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  MQTT Message Monitor                            ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${BLUE}MQTT Broker:${NC} ${MQTT_HOST}:${MQTT_PORT}"
echo -e "${BLUE}Username:${NC} ${MQTT_USER}"
echo ""

# Check if authentication might be needed
if [ -z "$MQTT_PASS" ] && [ -f "foxmq_cluster/foxmq.d/users.toml" ]; then
    echo -e "${YELLOW}Note:${NC} User exists in FoxMQ. If connection fails, you may need to:"
    echo -e "  1. Set MQTT_PASS environment variable"
    echo -e "  2. Or create user with empty password: cd foxmq_cluster && echo '' | ./foxmq user add pioneer_robot"
    echo ""
fi

# Check if mosquitto_sub is available
if ! command -v mosquitto_sub &> /dev/null; then
    echo -e "${RED}Error: mosquitto_sub not found${NC}"
    echo "Install with: sudo apt-get install mosquitto-clients"
    exit 1
fi

# Function to monitor all coordination topics
monitor_all() {
    echo -e "${YELLOW}Monitoring all coordination topics...${NC}"
    echo -e "${BLUE}Topics:${NC} coordination/#"
    echo ""
    # Use MQTT v5 protocol (FoxMQ requires v5)
    mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" \
        -u "$MQTT_USER" -P "$MQTT_PASS" \
        -V 5 \
        -t "coordination/#" \
        -v
}

# Function to monitor all robot topics
monitor_robots() {
    echo -e "${YELLOW}Monitoring all robot topics...${NC}"
    echo -e "${BLUE}Topics:${NC} robots/#"
    echo ""
    # Use MQTT v5 protocol (FoxMQ requires v5)
    mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" \
        -u "$MQTT_USER" -P "$MQTT_PASS" \
        -V 5 \
        -t "robots/#" \
        -v
}

# Function to monitor frontiers only
monitor_frontiers() {
    echo -e "${YELLOW}Monitoring frontiers topic...${NC}"
    echo -e "${BLUE}Topic:${NC} coordination/frontiers"
    echo ""
    # Use MQTT v5 protocol (FoxMQ requires v5)
    mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" \
        -u "$MQTT_USER" -P "$MQTT_PASS" \
        -V 5 \
        -t "coordination/frontiers" \
        -F "%t: %p"
}

# Function to monitor specific robot
monitor_robot() {
    local robot_id=$1
    if [ -z "$robot_id" ]; then
        robot_id="robot_1"
    fi
    echo -e "${YELLOW}Monitoring topics for ${robot_id}...${NC}"
    echo -e "${BLUE}Topics:${NC} robots/${robot_id}/#"
    echo ""
    # Use MQTT v5 protocol (FoxMQ requires v5)
    mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" \
        -u "$MQTT_USER" -P "$MQTT_PASS" \
        -V 5 \
        -t "robots/${robot_id}/#" \
        -v
}

# Function to show help
show_help() {
    echo "Usage: $0 [OPTION]"
    echo ""
    echo "Options:"
    echo "  all          Monitor all coordination topics (default)"
    echo "  robots       Monitor all robot topics"
    echo "  frontiers    Monitor frontiers topic only"
    echo "  robot N      Monitor topics for robot_N (e.g., robot 1)"
    echo "  help         Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                    # Monitor all coordination topics"
    echo "  $0 frontiers          # Monitor only frontiers"
    echo "  $0 robot 1            # Monitor robot_1 topics"
    echo ""
    echo "Environment variables:"
    echo "  MQTT_HOST    MQTT broker host (default: 127.0.0.1)"
    echo "  MQTT_PORT    MQTT broker port (default: 1883, also: 1884, 1885, 1886)"
    echo "  MQTT_USER    MQTT username (default: pioneer_robot)"
    echo "  MQTT_PASS    MQTT password (default: empty)"
    echo ""
    echo "Note: Bridge nodes connect to broker 0 (port 1883)"
}

# Parse arguments
case "${1:-all}" in
    all)
        monitor_all
        ;;
    robots)
        monitor_robots
        ;;
    frontiers)
        monitor_frontiers
        ;;
    robot)
        monitor_robot "$2"
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        echo -e "${RED}Unknown option: $1${NC}"
        show_help
        exit 1
        ;;
esac

