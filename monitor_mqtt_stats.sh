#!/bin/bash
#
# MQTT Message Statistics Monitor
# Shows message counts and statistics for MQTT topics
#

MQTT_HOST="${MQTT_HOST:-127.0.0.1}"
MQTT_PORT="${MQTT_PORT:-1883}"
MQTT_USER="${MQTT_USER:-pioneer_robot}"
MQTT_PASS="${MQTT_PASS:-test123}"
TOPIC="${1:-coordination/frontiers}"

echo "╔════════════════════════════════════════════════════╗"
echo "║  MQTT Message Statistics Monitor                  ║"
echo "╚════════════════════════════════════════════════════╝"
echo ""
echo "Topic: $TOPIC"
echo "Press Ctrl+C to stop and see statistics"
echo ""

MESSAGE_COUNT=0
START_TIME=$(date +%s)
LAST_MESSAGE_TIME=$(date +%s)

# Trap Ctrl+C to show statistics
trap 'echo ""; echo "────────────────────────────────────────────────────"; echo "Statistics:"; echo "  Total messages: $MESSAGE_COUNT"; if [ $MESSAGE_COUNT -gt 0 ]; then ELAPSED=$(($(date +%s) - $START_TIME)); RATE=$(echo "scale=2; $MESSAGE_COUNT / $ELAPSED" | bc 2>/dev/null || echo "N/A"); echo "  Duration: ${ELAPSED}s"; echo "  Rate: ${RATE} msg/s"; fi; exit 0' INT

mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" \
    -u "$MQTT_USER" -P "$MQTT_PASS" \
    -V 5 \
    -t "$TOPIC" \
    -F "%t" | while read topic; do
        read payload
        MESSAGE_COUNT=$((MESSAGE_COUNT + 1))
        LAST_MESSAGE_TIME=$(date +%s)
        
        # Extract robot_id and frontier count from JSON
        if command -v jq &> /dev/null; then
            ROBOT_ID=$(echo "$payload" | jq -r '.robot_id // "unknown"' 2>/dev/null)
            FRONTIER_COUNT=$(echo "$payload" | jq '.frontiers | length' 2>/dev/null)
            TIMESTAMP=$(echo "$payload" | jq -r '.timestamp // "unknown"' 2>/dev/null)
            
            echo "[$(date '+%H:%M:%S')] #$MESSAGE_COUNT | Robot: $ROBOT_ID | Frontiers: $FRONTIER_COUNT | Topic: $topic"
        else
            echo "[$(date '+%H:%M:%S')] #$MESSAGE_COUNT | $topic"
        fi
    done






