#!/bin/bash
#
# Start FoxMQ Cluster
# Launches 4-node FoxMQ cluster for high availability
#

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}╔════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  FoxMQ Cluster Launcher                           ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════╝${NC}"
echo ""

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

FOXMQ_DIR="$SCRIPT_DIR/foxmq_cluster"

# Check if FoxMQ exists
if [ ! -f "$FOXMQ_DIR/foxmq" ]; then
    echo -e "${RED}✗${NC} FoxMQ binary not found at $FOXMQ_DIR/foxmq"
    echo -e "${YELLOW}Downloading FoxMQ...${NC}"
    mkdir -p "$FOXMQ_DIR"
    cd "$FOXMQ_DIR"
    wget https://github.com/tashigg/foxmq/releases/download/v0.1.0/foxmq_0.1.0_linux-amd64.zip
    unzip foxmq_0.1.0_linux-amd64.zip
    chmod +x foxmq
    cd "$SCRIPT_DIR"
fi

# Check if address book exists
if [ ! -f "$FOXMQ_DIR/foxmq.d/address-book.toml" ]; then
    echo -e "${YELLOW}Generating address book...${NC}"
    cd "$FOXMQ_DIR"
    ./foxmq address-book from-range 127.0.0.1 19793 19796
    cd "$SCRIPT_DIR"
fi

# Check if users exist
if [ ! -f "$FOXMQ_DIR/foxmq.d/users.toml" ]; then
    echo -e "${YELLOW}Creating MQTT user...${NC}"
    cd "$FOXMQ_DIR"
    echo "pioneer_robot" | ./foxmq user add
    cd "$SCRIPT_DIR"
fi

# Cleanup function
cleanup() {
    echo -e "\n${YELLOW}Stopping FoxMQ brokers...${NC}"
    pkill -f "foxmq run" || true
    sleep 2
    echo -e "${GREEN}Cleanup complete${NC}"
}

# Trap Ctrl+C
trap cleanup EXIT INT TERM

# Check if brokers are already running
if pgrep -f "foxmq run" > /dev/null; then
    echo -e "${YELLOW}FoxMQ brokers already running${NC}"
    echo "PIDs:"
    pgrep -f "foxmq run"
    read -p "Kill existing brokers and restart? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        pkill -f "foxmq run"
        sleep 2
    else
        exit 0
    fi
fi

echo -e "${BLUE}Starting FoxMQ cluster (4 nodes)...${NC}"
echo ""

# Start broker 0
echo -e "${GREEN}Starting Broker 0 (port 1883/19793)...${NC}"
cd "$FOXMQ_DIR"
./foxmq run --secret-key-file=foxmq.d/key_0.pem --mqtt-addr=0.0.0.0:1883 --cluster-addr=0.0.0.0:19793 > /tmp/foxmq_broker_0.log 2>&1 &
BROKER_0_PID=$!
sleep 2

# Start broker 1
echo -e "${GREEN}Starting Broker 1 (port 1884/19794)...${NC}"
./foxmq run --secret-key-file=foxmq.d/key_1.pem --mqtt-addr=0.0.0.0:1884 --cluster-addr=0.0.0.0:19794 > /tmp/foxmq_broker_1.log 2>&1 &
BROKER_1_PID=$!
sleep 2

# Start broker 2
echo -e "${GREEN}Starting Broker 2 (port 1885/19795)...${NC}"
./foxmq run --secret-key-file=foxmq.d/key_2.pem --mqtt-addr=0.0.0.0:1885 --cluster-addr=0.0.0.0:19795 > /tmp/foxmq_broker_2.log 2>&1 &
BROKER_2_PID=$!
sleep 2

# Start broker 3
echo -e "${GREEN}Starting Broker 3 (port 1886/19796)...${NC}"
./foxmq run --secret-key-file=foxmq.d/key_3.pem --mqtt-addr=0.0.0.0:1886 --cluster-addr=0.0.0.0:19796 > /tmp/foxmq_broker_3.log 2>&1 &
BROKER_3_PID=$!
sleep 2

cd "$SCRIPT_DIR"

# Verify brokers started
ALL_STARTED=true
for i in 0 1 2 3; do
    PID_VAR="BROKER_${i}_PID"
    PID=${!PID_VAR}
    if ps -p $PID > /dev/null 2>&1; then
        echo -e "${GREEN}✓${NC} Broker $i running (PID: $PID)"
    else
        echo -e "${RED}✗${NC} Broker $i failed to start"
        echo "Check log: /tmp/foxmq_broker_${i}.log"
        ALL_STARTED=false
    fi
done

if [ "$ALL_STARTED" = true ]; then
    echo ""
    echo -e "${GREEN}════════════════════════════════════════════════════${NC}"
    echo -e "${GREEN}FoxMQ cluster is running!${NC}"
    echo -e "${GREEN}════════════════════════════════════════════════════${NC}"
    echo ""
    echo "MQTT endpoints:"
    echo "  - Broker 0: mqtt://127.0.0.1:1883"
    echo "  - Broker 1: mqtt://127.0.0.1:1884"
    echo "  - Broker 2: mqtt://127.0.0.1:1885"
    echo "  - Broker 3: mqtt://127.0.0.1:1886"
    echo ""
    echo "Cluster addresses:"
    echo "  - 127.0.0.1:19793, 127.0.0.1:19794, 127.0.0.1:19795, 127.0.0.1:19796"
    echo ""
    echo "Logs:"
    echo "  - /tmp/foxmq_broker_0.log"
    echo "  - /tmp/foxmq_broker_1.log"
    echo "  - /tmp/foxmq_broker_2.log"
    echo "  - /tmp/foxmq_broker_3.log"
    echo ""
    echo -e "${YELLOW}Press Ctrl+C to stop all brokers${NC}"
    echo ""
    
    # Wait for interrupt
    while true; do
        sleep 1
        # Check if brokers are still running
        if ! pgrep -f "foxmq run" > /dev/null; then
            echo -e "${RED}Brokers stopped unexpectedly${NC}"
            break
        fi
    done
else
    echo -e "${RED}Some brokers failed to start. Check logs above.${NC}"
    exit 1
fi

