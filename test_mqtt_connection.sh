#!/bin/bash
# Quick test to verify MQTT connection works

echo "Testing MQTT connection..."
echo "Username: pioneer_robot"
echo "Password: test123"
echo "Protocol: MQTT v5"
echo ""

# Test connection using Python (more reliable)
python3 << 'PYEOF'
import paho.mqtt.client as mqtt
import time
import sys

def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("✓ Connection successful!")
        print("✓ Authentication successful!")
        client.subscribe("coordination/#")
        print("✓ Subscribed to coordination/#")
        print("")
        print("Connection is working! You can now monitor MQTT messages.")
        sys.exit(0)
    else:
        print(f"✗ Connection failed with code {rc}")
        if rc == 5:
            print("  Error: Not authorized (check password)")
        sys.exit(1)

def on_message(client, userdata, msg):
    print(f"Message: {msg.topic}")

client = mqtt.Client(protocol=mqtt.MQTTv5)
client.username_pw_set("pioneer_robot", "test123")
client.on_connect = on_connect
client.on_message = on_message

try:
    client.connect("127.0.0.1", 1883, 60)
    client.loop_start()
    time.sleep(2)
    client.loop_stop()
    client.disconnect()
except Exception as e:
    print(f"✗ Error: {e}")
    print("")
    print("Check:")
    print("  1. FoxMQ cluster is running: ps aux | grep foxmq")
    print("  2. Bridge node is running: ros2 node list | grep foxmq")
    sys.exit(1)
PYEOF

if [ $? -eq 0 ]; then
    echo ""
    echo "To monitor messages, run:"
    echo "  ./monitor_mqtt.sh"
    echo "  or"
    echo "  ./quick_mqtt_monitor.sh"
fi

