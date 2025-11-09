# Pioneer Multi-Robot Autonomous Exploration System

A multi-robot autonomous exploration system built with Webots and ROS 2, featuring 5 Pioneer3at robots with autonomous navigation, frontier-based exploration, path planning, and inter-robot coordination via FoxMQ/MQTT.

## Features

- **Multi-Robot Simulation**: 5 Pioneer3at robots in Webots simulation environment
- **Autonomous Exploration**: Frontier-based exploration with A* path planning
- **Mapping**: Real-time occupancy grid building from LiDAR data
- **Inter-Robot Communication**: FoxMQ/MQTT bridge for robot coordination
- **Spatial Analysis**: Visualization tools for exploration metrics and heatmaps
- **Path Planning**: A* algorithm with obstacle avoidance
- **Goal Validation**: Intelligent goal selection with occupancy grid validation

## Prerequisites

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
       python3-colcon-common-extensions \
       paho-mqtt
   ```

4. **Python Dependencies**
   ```bash
   pip3 install numpy paho-mqtt
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
- `visualization_msgs`

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

### 3. FoxMQ Cluster Setup

The system uses FoxMQ for inter-robot communication. The cluster is automatically set up on first run:

```bash
./start_foxmq_cluster.sh
```

This will:
- Download FoxMQ binary if needed
- Generate address book for 4-node cluster
- Create MQTT user credentials
- Start FoxMQ brokers on ports 19793-19796

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

### Quick Start (Recommended)

The easiest way to start the complete system:

```bash
./start_full_system.sh
```

This script:
1. Starts FoxMQ cluster
2. Starts Webots simulation
3. Starts supervisor node
4. Starts LiDAR bridge node
5. Starts exploration nodes (path planning, frontier detection, etc.)
6. Starts spatial analysis visualizer (optional)

### Manual Startup Procedure

If you prefer to start components manually:

#### Step 1: Start FoxMQ Cluster

```bash
./start_foxmq_cluster.sh
```

#### Step 2: Start Webots Simulation

Launch the Webots world:
```bash
webots worlds/pioneer_world.wbt
```

**Important:** Wait 3-5 seconds after Webots starts for all controllers to initialize.

#### Step 3: Verify Supervisor Controller is Running

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

#### Step 4: Start ROS 2 Nodes

In a new terminal, source ROS 2 and start nodes:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

# Start supervisor node
ros2 run pioneer_ros2 supervisor_node

# In another terminal, start LiDAR bridge
ros2 run pioneer_ros2 lidar_bridge_node --ros-args -r __ns:=/robot_1

# Start exploration nodes
ros2 run pioneer_path_planner occupancy_grid_builder_node --ros-args -r __ns:=/robot_1
ros2 run pioneer_path_planner frontier_detector_node --ros-args -r __ns:=/robot_1
ros2 run pioneer_path_planner goal_selector_node --ros-args -r __ns:=/robot_1
ros2 run pioneer_path_planner path_planner_node --ros-args -r __ns:=/robot_1
ros2 run pioneer_path_planner path_follower_node --ros-args -r __ns:=/robot_1

# Start FoxMQ bridge (for inter-robot communication)
ros2 run pioneer_ros2 foxmq_bridge_node --ros-args -r __ns:=/robot_1
```

### Stopping the System

Use the stop script to cleanly shut down all components:

```bash
./stop_full_system.sh
```

This will:
- Stop all ROS 2 nodes
- Stop FoxMQ brokers
- Clean up temporary files

## Verification

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

4. **Check Occupancy Grid:**
   ```bash
   ros2 topic echo /robot_1/occupancy_map --once
   # Should show grid with free/occupied/unknown cells
   ```

5. **Check FoxMQ Connection:**
   ```bash
   ros2 topic list | grep coordination
   # Should show coordination topics if FoxMQ bridge is running
   ```

6. **Monitor Robot Exploration:**
   - Watch the Webots simulation
   - Robots should start moving autonomously within 10 seconds
   - They will build occupancy maps and explore frontiers

## System Architecture

### Components

- **Webots Simulation**: 5 Pioneer3at robots with LiDAR sensors
- **Supervisor Controller**: C controller that reads robot poses from Webots
- **ROS 2 Nodes**: Python nodes for navigation, mapping, and exploration
- **FoxMQ Cluster**: MQTT broker cluster for inter-robot communication
- **FoxMQ Bridge**: ROS2-MQTT bridge for robot coordination

### Key Nodes

- `supervisor_node`: Publishes odometry for all robots
- `lidar_bridge_node`: Bridges LiDAR data from Webots to ROS 2
- `occupancy_grid_builder_node`: Builds 2D occupancy grid from LiDAR scans
- `frontier_detector_node`: Detects exploration frontiers
- `goal_selector_node`: Selects best exploration goals
- `path_planner_node`: Plans paths using A* algorithm
- `path_follower_node`: Generates velocity commands to follow paths
- `foxmq_bridge_node`: Bridges ROS 2 topics to MQTT for coordination
- `spatial_analysis_visualizer_node`: Visualizes exploration metrics

For detailed architecture information, see [ARCHITECTURE.md](ARCHITECTURE.md).

## FoxMQ/MQTT Integration

The system uses FoxMQ (a distributed MQTT broker) for inter-robot communication:

- **Purpose**: Enable robots to share frontiers, paths, and coordinate exploration
- **MQTT Topics**:
  - `coordination/frontiers` - Shared frontier information
  - `coordination/paths` - Planned paths for collision avoidance
  - `robots/robot_i/status` - Mission status updates
  - `robots/robot_i/detection` - Object detection reports (future)
  - `robots/robot_i/territory` - Territory assignments (future)

- **Consensus**: FoxMQ consensus features are planned but not yet implemented

## Visualization

### RViz Configuration

A pre-configured RViz file is available for spatial analysis:

```bash
rviz2 -d pioneer_spatial_analysis.rviz
```

This displays:
- Robot trajectories
- Exploration heatmaps
- Frontier points
- Planned paths
- Occupancy grids

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
4. Restart supervisor node

### Issue: Robots not moving autonomously

**Symptoms:** Robots idle or only moving randomly

**Solutions:**
1. Check navigation nodes are running: `ros2 node list`
2. Verify topics exist: `ros2 topic list | grep robot_1`
3. Check logs in `/tmp/robot_1_*.log` files
4. Verify occupancy grid is building: `ros2 topic echo /robot_1/occupancy_map --once`
5. Check for path planning errors: `tail -f /tmp/robot_1_planner.log`

### Issue: FoxMQ connection failed

**Symptoms:** FoxMQ bridge node shows connection errors

**Solutions:**
1. Verify FoxMQ cluster is running: `ps aux | grep foxmq`
2. Check FoxMQ brokers are listening: `netstat -tuln | grep 19793`
3. Verify user credentials in `foxmq_cluster/foxmq.d/users.toml`
4. Restart FoxMQ cluster: `./start_foxmq_cluster.sh`

### Issue: Build errors

**Symptoms:** `make` or `colcon build` fails

**Solutions:**
1. Ensure Webots is installed and `WEBOTS_HOME` is set correctly
2. Check ROS 2 is sourced: `source /opt/ros/humble/setup.bash`
3. Install missing dependencies: `sudo apt-get install build-essential python3-colcon-common-extensions paho-mqtt`
4. Clean and rebuild: `make clean && make` or `colcon build --cmake-clean-cache`

## Project Status

### Completed Features
- ✅ Multi-robot simulation setup
- ✅ LiDAR integration and occupancy grid building
- ✅ Frontier-based exploration
- ✅ A* path planning
- ✅ Path following and velocity control
- ✅ FoxMQ cluster setup
- ✅ ROS2-MQTT bridge for frontier sharing
- ✅ Goal validation and adjustment
- ✅ Spatial analysis visualization

### In Progress
- 🔄 Multi-robot coordination via FoxMQ
- 🔄 Enhanced goal selection with occupancy grid validation

### Planned Features
- ⏳ FoxMQ consensus for critical decisions
- ⏳ Object detection and multi-robot verification
- ⏳ Territory assignment and coordination
- ⏳ Collision avoidance via path sharing

## License

Apache-2.0
