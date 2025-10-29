# Implementation Summary: ROS 2 Integration for 5 Pioneer Robots

## What Was Implemented

This implementation enables 5 Pioneer robots in Webots to share their position data via ROS 2 communication. Each robot publishes its odometry to a namespaced topic, allowing all robots (and external systems) to know where each robot is located.

## Components Created

### 1. Supervisor Controller (`controllers/supervisor/`)
   - **File**: `supervisor.c` + `Makefile`
   - **Purpose**: Reads positions of all 5 robots from Webots using Supervisor API
   - **Output**: Writes robot poses to `/tmp/pioneer_poses.txt` file
   - **Format**: `POSE:robot_name:x:y:z:qx:qy:qz:qw:roll:pitch:yaw`

### 2. ROS 2 Package (`ros2_ws/src/pioneer_ros2/`)
   - **Package Structure**: Standard ROS 2 Python package
   - **Main Node**: `supervisor_node.py` - Reads pose file and publishes to ROS 2
   - **Aggregator Node**: `aggregator_node.py` - Optional node to combine all poses
   - **Launch File**: `launch/supervisor.launch.py`

### 3. World File Updates (`worlds/pioneer_world.wbt`)
   - Added `DEF ROBOT_1` through `DEF ROBOT_5` to all Pioneer robots
   - Enables supervisor controller to access robots by DEF name

### 4. Launch Script (`launch_with_ros2.sh`)
   - Automated script to build and launch entire system
   - Handles ROS 2 node startup and Webots launch

### 5. Documentation
   - `IMPLEMENTATION_GUIDE.md` - Detailed step-by-step guide
   - `QUICK_START.md` - Quick reference for running the system
   - `README.md` - Overview and architecture

## Architecture

```
┌─────────────────────────────────────────┐
│         Webots Simulation               │
│                                         │
│  ┌──────────┐  ┌──────────┐  ...      │
│  │ ROBOT_1  │  │ ROBOT_2  │  (5 bots) │
│  └────┬─────┘  └────┬─────┘            │
│       │             │                   │
│       └──────┬──────┘                   │
│              │                          │
│       ┌──────▼──────┐                  │
│       │ Supervisor  │                  │
│       │ Controller  │                  │
│       └──────┬──────┘                  │
│              │                         │
│              │ (writes poses)          │
│              ▼                         │
│      /tmp/pioneer_poses.txt            │
└──────────────┼─────────────────────────┘
               │ (reads file)
               ▼
┌─────────────────────────────────────────┐
│     ROS 2 Supervisor Node (Python)     │
│                                         │
│  Reads file → Parses poses → Publishes │
└─────────────────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────┐
│         ROS 2 Topics Published          │
│                                         │
│  /robot_1/odom (nav_msgs/Odometry)      │
│  /robot_2/odom                          │
│  /robot_3/odom                          │
│  /robot_4/odom                          │
│  /robot_5/odom                          │
│  /tf (TF transforms)                    │
└─────────────────────────────────────────┘
```

## Communication Flow

1. **Webots Simulation Runs**
   - 5 Pioneer robots with obstacle avoidance controllers
   - 1 Supervisor robot with supervisor controller

2. **Supervisor Controller** (C code)
   - Each timestep (32ms), reads positions of all 5 robots
   - Uses `wb_supervisor_node_get_from_def()` to access robots
   - Extracts position (translation) and orientation (rotation)
   - Converts rotation (axis-angle) to quaternion
   - Converts quaternion to Euler angles
   - Writes poses to `/tmp/pioneer_poses.txt`

3. **ROS 2 Supervisor Node** (Python)
   - Continuously reads `/tmp/pioneer_poses.txt`
   - Parses pose lines
   - Publishes `nav_msgs/Odometry` to `/robot_N/odom` topics
   - Publishes TF transforms via `tf2_ros`

4. **Any ROS 2 Node**
   - Can subscribe to `/robot_1/odom`, `/robot_2/odom`, etc.
   - Can subscribe to `/tf` to get robot transforms
   - Can use `tf2` library to query robot positions

## Key Features

### 1. Namespaced Topics
Each robot has its own namespace:
- `/robot_1/odom`
- `/robot_2/odom`
- `/robot_3/odom`
- `/robot_4/odom`
- `/robot_5/odom`

This allows easy filtering and prevents topic collisions.

### 2. TF Transforms
Each robot publishes:
- `robot_N/odom` → `robot_N/base_link` transform
- Enables visualization in RViz
- Enables coordinate transformations using tf2

### 3. Ground Truth Positions
Uses Webots Supervisor API to get exact robot positions (ground truth).
- No sensor noise
- No drift
- Perfect accuracy for testing algorithms

### 4. File-Based Communication
Simple file-based IPC between Webots and ROS 2:
- No complex socket programming
- Easy to debug (can inspect file directly)
- Works reliably across different systems

## Usage

### Running the System

```bash
# Method 1: Automated launch script
cd /home/karthik/ws/tashi/pioneer
./launch_with_ros2.sh

# Method 2: Manual launch
# Terminal 1:
ros2 run pioneer_ros2 supervisor_node

# Terminal 2:
webots worlds/pioneer_world.wbt
# (Ensure world has Supervisor robot with "supervisor" controller)
```

### Verifying Communication

```bash
# List topics
ros2 topic list

# View robot positions
ros2 topic echo /robot_1/odom
ros2 topic echo /robot_2/odom

# View TF tree
ros2 run tf2_tools view_frames

# Visualize in RViz
rviz2
```

## File Locations

| Component | Location |
|-----------|----------|
| Supervisor Controller | `controllers/supervisor/supervisor.c` |
| ROS 2 Package | `ros2_ws/src/pioneer_ros2/` |
| World File | `worlds/pioneer_world.wbt` |
| Launch Script | `launch_with_ros2.sh` |
| Communication File | `/tmp/pioneer_poses.txt` |
| Documentation | `ros2_integration/` |

## Coordinate System

- **Webots**: X=forward, Y=left, Z=up
- **ROS 2**: Currently uses Webots coordinates directly
- **Future**: Add ENU (East-North-Up) transform for ROS standard

## Current Limitations & Future Improvements

### Current Limitations
1. No velocity estimation (twist is zero)
2. Uses Webots coordinates (not ROS ENU standard)
3. File-based communication (not real-time socket)
4. No error handling for robot disconnects
5. Fixed covariance values (not based on sensor noise)

### Future Improvements
1. **Velocity Computation**: Calculate from pose differences
2. **Coordinate Transform**: Webots → ROS ENU
3. **Socket Communication**: Replace file with TCP socket for lower latency
4. **Covariance Estimation**: Add realistic sensor noise models
5. **Multi-Robot Coordination**: Enable robots to react to each other's positions
6. **Aggregator Message**: Create custom message with all robot poses in one topic
7. **Visualization**: Add RViz markers to show robot trails

## Testing Checklist

- [x] Supervisor controller compiles
- [x] Supervisor controller reads robot positions
- [x] Pose file is created and updated
- [x] ROS 2 package builds
- [x] ROS 2 node reads pose file
- [x] Topics are published (`/robot_N/odom`)
- [x] TF transforms are published
- [ ] Verify robot positions match Webots (manual inspection)
- [ ] RViz visualization works
- [ ] Multiple subscribers can receive data
- [ ] System works for extended duration

## Dependencies

### Webots
- Webots R2025a or compatible
- Controller C API
- Supervisor API access

### ROS 2
- ROS 2 Humble (or compatible)
- Packages: `rclpy`, `nav_msgs`, `geometry_msgs`, `tf2_ros`, `tf2_msgs`

### System
- GCC compiler
- Python 3
- Make utility

## Troubleshooting Common Issues

1. **No topics published**: Check supervisor controller is running and writing to file
2. **File permission errors**: Ensure `/tmp/pioneer_poses.txt` is writable
3. **Robot DEF names not found**: Verify DEF names match in world file
4. **ROS 2 node not reading**: Check file exists and has correct format
5. **Build errors**: Ensure Webots libraries are in PATH and ROS 2 is sourced

## Conclusion

This implementation provides a complete ROS 2 integration for 5 Pioneer robots in Webots, enabling position sharing and multi-robot coordination. The architecture is modular, well-documented, and ready for extension with additional features like velocity estimation, coordinate transforms, and multi-robot behaviors.


