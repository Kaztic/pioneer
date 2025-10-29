# Quick Start Guide: ROS 2 Integration

## Prerequisites Checklist

- [ ] ROS 2 Humble installed (`/opt/ros/humble`)
- [ ] Webots installed and in PATH
- [ ] GCC compiler available
- [ ] Python 3 installed

## Installation Steps

### 1. Install ROS 2 (if not installed)

```bash
# For Ubuntu 22.04
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions
```

### 2. Source ROS 2

```bash
source /opt/ros/humble/setup.bash
# Or add to ~/.bashrc:
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 3. Build Supervisor Controller

```bash
cd /home/karthik/ws/tashi/pioneer/controllers/supervisor
make clean && make
```

### 4. Build ROS 2 Package

```bash
cd /home/karthik/ws/tashi/pioneer/ros2_ws
colcon build
source install/setup.bash
```

## Running the System

### Option 1: Using Launch Script (Easiest)

```bash
cd /home/karthik/ws/tashi/pioneer
./launch_with_ros2.sh
```

This will:
- Build supervisor controller
- Build ROS 2 package
- Start ROS 2 supervisor node
- Launch Webots

### Option 2: Manual Launch

**Terminal 1: ROS 2 Supervisor Node**
```bash
cd /home/karthik/ws/tashi/pioneer/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run pioneer_ros2 supervisor_node
```

**Terminal 2: Webots (with supervisor robot)**
```bash
cd /home/karthik/ws/tashi/pioneer
webots worlds/pioneer_world.wbt
```

**Note:** Ensure the world file has a Supervisor robot node that runs the `supervisor` controller.

## Verification

### Check ROS 2 Topics

```bash
# List all topics
ros2 topic list

# Should see:
# /robot_1/odom
# /robot_2/odom
# /robot_3/odom
# /robot_4/odom
# /robot_5/odom
# /tf
```

### View Robot Positions

```bash
# Robot 1
ros2 topic echo /robot_1/odom

# Robot 2
ros2 topic echo /robot_2/odom
```

### Visualize in RViz

```bash
rviz2

# In RViz:
# 1. Add TF display
# 2. Set Fixed Frame to "robot_1/odom" (or any robot frame)
# 3. You should see all 5 robot frames in the TF tree
```

### Check Communication File

```bash
# Monitor pose file updates
watch -n 0.1 cat /tmp/pioneer_poses.txt
```

## Troubleshooting

### No topics published?

1. Check ROS 2 node is running: `ps aux | grep supervisor_node`
2. Check supervisor controller output: Check Webots console
3. Verify pose file exists: `ls -l /tmp/pioneer_poses.txt`
4. Check robot DEF names in world file match `ROBOT_1`, `ROBOT_2`, etc.

### Supervisor not reading robots?

1. Verify DEF names in `worlds/pioneer_world.wbt`:
   ```vrml
   DEF ROBOT_1 Pioneer3at { ... }
   DEF ROBOT_2 Pioneer3at { ... }
   ```
2. Check supervisor controller is set to a Supervisor robot node
3. Check Webots console for error messages

### Build errors?

1. **Supervisor controller**: Ensure Webots libraries are accessible
   ```bash
   export WEBOTS_HOME=/usr/local/webots
   make
   ```

2. **ROS 2 package**: Install dependencies
   ```bash
   cd ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build
   ```

## Next Steps

After verifying basic communication works:

1. **Add velocity estimation** in supervisor controller
2. **Implement coordinate transforms** (Webots to ROS ENU)
3. **Create aggregator node** to combine all robot poses
4. **Add visualization** with markers in RViz
5. **Implement robot-to-robot coordination** (each robot subscribes to others)

## Architecture Diagram

```
Webots Supervisor Controller (reads ROBOT_1-5 positions)
         ↓
   Writes to /tmp/pioneer_poses.txt
         ↓
ROS 2 Supervisor Node (reads file, publishes to ROS)
         ↓
   Publishes: /robot_1/odom, /robot_2/odom, ... /robot_5/odom
   Publishes: /tf (TF transforms for all robots)
```

## File Locations

- Supervisor Controller: `controllers/supervisor/supervisor.c`
- ROS 2 Package: `ros2_ws/src/pioneer_ros2/`
- World File: `worlds/pioneer_world.wbt`
- Launch Script: `launch_with_ros2.sh`
- Communication File: `/tmp/pioneer_poses.txt`


