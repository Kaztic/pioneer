#!/bin/bash
# Quick MQTT monitor - simplest way to see messages
# Usage: ./quick_mqtt_monitor.sh [topic]

TOPIC="${1:-coordination/#}"
echo "Monitoring MQTT topic: $TOPIC"
echo "Press Ctrl+C to stop"
echo ""

mosquitto_sub -h 127.0.0.1 -p 1883 \
    -u pioneer_robot -P "test123" \
    -V 5 \
    -t "$TOPIC" \
    -v
