#!/bin/bash
#
# MQTT Message Monitor with JSON Formatting
# Monitors MQTT topics and formats JSON output nicely
#

MQTT_HOST="${MQTT_HOST:-127.0.0.1}"
MQTT_PORT="${MQTT_PORT:-1883}"
MQTT_USER="${MQTT_USER:-pioneer_robot}"
MQTT_PASS="${MQTT_PASS:-test123}"
TOPIC="${1:-coordination/#}"

echo "╔════════════════════════════════════════════════════╗"
echo "║  MQTT Message Monitor (Formatted)                  ║"
echo "╚════════════════════════════════════════════════════╝"
echo ""
echo "Topic: $TOPIC"
echo "Press Ctrl+C to stop"
echo ""

# Check if jq is available for JSON formatting
if command -v jq &> /dev/null; then
    echo "Using jq for JSON formatting"
    echo "────────────────────────────────────────────────────"
    echo ""
    mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" \
        -u "$MQTT_USER" -P "$MQTT_PASS" \
        -V 5 \
        -t "$TOPIC" \
        -F "%t" | while read topic; do
            read payload
            echo "[$(date '+%H:%M:%S')] $topic"
            echo "$payload" | jq . 2>/dev/null || echo "$payload"
            echo ""
        done
else
    echo "jq not found - showing raw output"
    echo "Install jq for better formatting: sudo apt-get install jq"
    echo "────────────────────────────────────────────────────"
    echo ""
    mosquitto_sub -h "$MQTT_HOST" -p "$MQTT_PORT" \
        -u "$MQTT_USER" -P "$MQTT_PASS" \
        -V 5 \
        -t "$TOPIC" \
        -v | while IFS= read -r line; do
            echo "[$(date '+%H:%M:%S')] $line"
        done
fi






