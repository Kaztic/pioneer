# Full System Testing Guide

## **Step 1: Start the Complete System**

```bash
# Quick start (recommended)
./start_full_system.sh
```

This automatically starts:
- ✅ FoxMQ cluster (4 brokers)
- ✅ Supervisor node
- ✅ FoxMQ bridge nodes (5 robots)
- ✅ LiDAR bridge node (robot_1)
- ✅ Exploration nodes (robot_1)
- ✅ Spatial analysis visualizer

### **Step 2: Start Webots Simulation**

```bash
# In a new terminal
webots worlds/pioneer_world.wbt
```

**Important:** Wait 3-5 seconds after Webots starts for all controllers to initialize.

### **Step 3: Verification Checks**

**A. Check all processes are running:**
```bash
ps aux | grep -E "foxmq|supervisor|bridge|occupancy|frontier|goal|path" | grep -v grep
```

**B. Check ROS2 topics:**
```bash
ros2 topic list | wc -l  # Should show 40+ topics
ros2 topic list | grep coordination  # Should show /coordination/frontiers
```

**C. Check data flow:**
```bash
# Odometry (should be ~30 Hz)
ros2 topic hz /robot_1/odom

# LiDAR scans (should be ~15 Hz)
ros2 topic hz /robot_1/scan

# Occupancy map (should be ~2 Hz)
ros2 topic hz /robot_1/occupancy_map

# Frontiers (should be ~2 Hz)
ros2 topic hz /robot_1/frontiers

# Coordination frontiers (from MQTT, ~10 Hz)
ros2 topic hz /coordination/frontiers

# Robot movement
ros2 topic echo /robot_1/cmd_vel
```

**D. Check FoxMQ:**
```bash
# Test MQTT connection
./test_mqtt_connection.sh

# Monitor MQTT messages
./monitor_mqtt.sh frontiers

# Or monitor all coordination topics
./monitor_mqtt.sh
```

**E. Check logs:**
```bash
# View all logs
tail -f /tmp/*.log

# Or specific components
tail -f /tmp/foxmq_bridge.log | grep -E "Received MQTT|Published.*coordination"
tail -f /tmp/robot_1_grid.log
tail -f /tmp/robot_1_frontier.log
```

### **Step 4: Visual Verification**

**A. Watch Webots:**
- Robots should start moving autonomously
- They should explore and avoid obstacles
- Movement should start within 10 seconds

**B. View in RViz:**
```bash
rviz2 -d pioneer_spatial_analysis.rviz
```

You should see:
- Occupancy grids
- Frontier points
- Robot trajectories
- Planned paths

### **Step 5: Test FoxMQ Specifically**

**A. Verify MQTT message flow:**
```bash
# Terminal 1: Monitor ROS2
ros2 topic echo /coordination/frontiers

# Terminal 2: Monitor MQTT
./monitor_mqtt.sh frontiers

# Terminal 3: Publish test message
mosquitto_pub -h 127.0.0.1 -p 1883 -u pioneer_robot -P test123 -V 5 \
  -t "coordination/frontiers" \
  -m '{"robot_id": "robot_2", "timestamp": 1234567890, "frontiers": [{"x": 10.0, "y": 20.0}]}'
```

**B. Check bridge node logs:**
```bash
tail -f /tmp/foxmq_bridge.log | grep -E "Received MQTT|Routing|Publishing.*coordination"
```

### **Step 6: Stop the System**

```bash
./stop_full_system.sh
```

---

## Quick Testing Checklist

- [ ] All processes running (FoxMQ, supervisor, bridges, exploration nodes)
- [ ] Webots simulation running with robots visible
- [ ] Odometry publishing (~30 Hz)
- [ ] LiDAR scans publishing (~15 Hz)
- [ ] Occupancy map updating (~2 Hz)
- [ ] Frontiers detected and published
- [ ] Robots moving autonomously
- [ ] FoxMQ connection working (test_mqtt_connection.sh)
- [ ] MQTT messages visible in monitor_mqtt.sh
- [ ] `/coordination/frontiers` receiving messages
- [ ] No errors in logs

---

## Troubleshooting

**If robots don't move:**
```bash
# Check if path follower is publishing commands
ros2 topic echo /robot_1/cmd_vel

# Check path planner
ros2 topic echo /robot_1/global_path

# Check goal selector
ros2 topic echo /robot_1/goal
```

**If MQTT not working:**
```bash
# Check FoxMQ brokers
ps aux | grep "foxmq run"

# Check bridge nodes
ros2 node list | grep foxmq

# Check logs
tail -f /tmp/foxmq_bridge.log | grep -i error
```

**If no frontiers:**
```bash
# Check if map is being built
ros2 topic echo /robot_1/occupancy_map --once | grep -E "data:|-1|100"

# Check frontier detector logs
tail -f /tmp/robot_1_frontier.log
```

