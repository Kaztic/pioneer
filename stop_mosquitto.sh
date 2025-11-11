#!/bin/bash
#
# Stop system mosquitto service to free port 1883 for FoxMQ
#

echo "Stopping mosquitto service..."

if systemctl is-active --quiet snap.mosquitto.mosquitto.service 2>/dev/null; then
    echo "Mosquitto service is running. Stopping..."
    sudo systemctl stop snap.mosquitto.mosquitto.service
    if [ $? -eq 0 ]; then
        echo "✓ Mosquitto stopped successfully"
        echo ""
        echo "To prevent it from starting on boot, run:"
        echo "  sudo systemctl disable snap.mosquitto.mosquitto.service"
    else
        echo "✗ Failed to stop mosquitto (may need sudo password)"
        exit 1
    fi
else
    echo "Mosquitto service is not running"
fi

# Verify port is free
sleep 1
if ss -tuln 2>/dev/null | grep -q ":1883 " || netstat -tuln 2>/dev/null | grep -q ":1883 "; then
    echo "⚠ Warning: Port 1883 is still in use"
    echo "Check with: ss -tuln | grep 1883"
else
    echo "✓ Port 1883 is now free"
fi

