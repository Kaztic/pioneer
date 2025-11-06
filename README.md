# Pioneer Multi-Robot Simulation

A multi-robot autonomous exploration system built with Webots and ROS 2, featuring 5 Pioneer3at robots with autonomous navigation, frontier-based exploration, and path planning capabilities.

## Prerequisites

Before setting up the system, ensure you have the following installed:

### Required Software

1. **Webots** (R2023a or later)
   - Download from: https://cyberbotics.com/
   - Ensure Webots is in your PATH or set `WEBOTS_HOME` environment variable

2. **ROS 2 Humble Hawksbill**
   - Installation guide: https://docs.ros.org/en/humble/Installation.html
   - Ubuntu/Debian: Follow the official ROS 2 Humble installation instructions

3. **System Dependencies**
   ```bash
   sudo apt-get update
   sudo apt-get install -y \
       build-essential \
       gcc \
       make \
       python3-pip \
       python3-colcon-common-extensions
   ```

4. **Python Dependencies**
   ```bash
   pip3 install numpy
   ```

### ROS 2 Packages

The following ROS 2 packages are required (included as dependencies):
- `rclpy`
- `nav_msgs`
- `geometry_msgs`
- `sensor_msgs`
- `std_msgs`
- `tf2_ros`
- `tf2_msgs`

These are typically installed with ROS 2 Humble.

## Installation

### 1. Clone the Repository

```bash
cd ~/your_workspace
git clone <repository_url> pioneer
cd pioneer
```

### 2. Verify Installation

Ensure Webots is accessible:
```bash
which webots
# Should output: /usr/local/webots/webots (or similar)
```

Verify ROS 2 is sourced:
```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

## Setup

### 1. Environment Configuration

Source ROS 2 in your shell (add to `~/.bashrc` for persistence):
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Webots Configuration

If Webots is not in your PATH, set the `WEBOTS_HOME` environment variable:
```bash
export WEBOTS_HOME=/usr/local/webots
# Or add to ~/.bashrc for persistence
echo "export WEBOTS_HOME=/usr/local/webots" >> ~/.bashrc
```

## Building

### 1. Build Supervisor Controller

The supervisor controller reads robot poses from Webots and writes them to a shared file for ROS 2:

```bash
cd controllers/supervisor
make clean
make
cd ../..
```

The compiled binary will be at: `controllers/supervisor/supervisor`

### 2. Build ROS 2 Workspace

Build all ROS 2 packages:

```bash
cd ros2_ws
colcon build
source install/setup.bash
cd ..
```

The build process creates:
- `ros2_ws/build/` - Build artifacts
- `ros2_ws/install/` - Installed packages
- `ros2_ws/log/` - Build logs

**Note:** Always source `install/setup.bash` after building:
```bash
source ros2_ws/install/setup.bash
```

## Running the Application

### Complete Startup Procedure

The system requires three components to run in sequence:

#### Step 1: Start Webots Simulation

Launch the Webots world:
```bash
webots worlds/pioneer_world.wbt
```

**Important:** Wait 3-5 seconds after Webots starts for all controllers to initialize.

#### Step 2: Verify Supervisor Controller is Running

Check that the supervisor controller is writing pose data:
```bash
cat /tmp/pioneer_poses.txt
```

You should see output like:
```
POSE:robot_1:-5.814:10.073:0.098:...
POSE:robot_2:-7.435:8.384:0.098:...
POSE:robot_3:-9.627:4.962:0.098:...
POSE:robot_4:-8.728:6.483:0.098:...
POSE:robot_5:-10.597:3.255:0.098:...
```

If the file doesn't exist or is empty:
- Check Webots console for errors (View → Console)
- Ensure the supervisor controller binary exists: `ls controllers/supervisor/supervisor`
- Restart Webots completely

#### Step 3: Start Supervisor ROS 2 Node

In a new terminal, source ROS 2 and start the supervisor node:
```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
./start_supervisor_node.sh
```

Or use the integrated launcher:
```bash
./launch_with_ros2.sh
```

This script:
- Builds supervisor controller if needed
- Builds ROS 2 workspace if needed
- Starts the supervisor ROS 2 node
- Launches Webots

#### Step 4: Launch Robot Nodes for Autonomous Operation

In another terminal, launch all robot navigation nodes:
```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
./launch_multi_robot.sh
```

This script:
- Verifies supervisor controller is running
- Checks Webots is running
- Verifies pose file exists and is fresh
- Automatically starts supervisor ROS 2 node if not running
- Launches all navigation nodes for all 5 robots

### Verification

After launching, verify the system is working:

1. **Check ROS 2 Topics:**
   ```bash
   ros2 topic list | wc -l
   # Should show 40+ topics
   ```

2. **Check Odometry Publishing:**
   ```bash
   ros2 topic hz /robot_1/odom
   # Should show ~30 Hz publishing rate
   ```

3. **Check LiDAR Data:**
   ```bash
   ros2 topic hz /robot_1/scan
   # Should show publishing rate
   ```

4. **Monitor Robot Exploration:**
   - Watch the Webots simulation
   - Robots should start moving autonomously within 10 seconds
   - They will build occupancy maps and explore frontiers

### Stopping the System

1. Press `Ctrl+C` in the terminal running `launch_multi_robot.sh`
2. Press `Ctrl+C` in the terminal running `start_supervisor_node.sh` (if running separately)
3. Close Webots

## Quick Start Scripts

The repository includes the following scripts for autonomous operation:

- **`launch_with_ros2.sh`** - Starts Webots and supervisor ROS 2 node together
- **`launch_multi_robot.sh`** - Launches all robot navigation nodes for autonomous exploration
- **`start_supervisor_node.sh`** - Standalone supervisor ROS 2 node launcher

## Troubleshooting

### Issue: Pose file not created

**Symptoms:** `/tmp/pioneer_poses.txt` doesn't exist after Webots starts

**Solutions:**
1. Check Webots console for supervisor controller errors
2. Verify supervisor binary exists: `ls controllers/supervisor/supervisor`
3. Rebuild supervisor: `cd controllers/supervisor && make clean && make`
4. Fully close and restart Webots

### Issue: No ROS 2 topics

**Symptoms:** `ros2 topic list` shows only 2 topics (default ROS topics)

**Solutions:**
1. Verify pose file exists and has data: `cat /tmp/pioneer_poses.txt`
2. Check supervisor ROS 2 node is running: `ps aux | grep supervisor_node`
3. Source ROS 2 workspace: `source ros2_ws/install/setup.bash`
4. Restart supervisor node: `./start_supervisor_node.sh`

### Issue: Robots not moving autonomously

**Symptoms:** Robots idle or only moving randomly

**Solutions:**
1. Check navigation nodes are running: `ros2 node list`
2. Verify topics exist: `ros2 topic list | grep robot_1`
3. Check logs in `logs/multi_robot_*/robot_*/` directories
4. Verify occupancy grid is building: `ros2 topic echo /robot_1/occupancy_map --once`

### Issue: Build errors

**Symptoms:** `make` or `colcon build` fails

**Solutions:**
1. Ensure Webots is installed and `WEBOTS_HOME` is set correctly
2. Check ROS 2 is sourced: `source /opt/ros/humble/setup.bash`
3. Install missing dependencies: `sudo apt-get install build-essential python3-colcon-common-extensions`
4. Clean and rebuild: `make clean && make` or `colcon build --cmake-clean-cache`

## System Architecture

For detailed information about ROS nodes, data flow, topics, and autonomous exploration, see [ARCHITECTURE.md](ARCHITECTURE.md).

## License

Apache-2.0

