# Complete Implementation Guide: ROS 2 Integration for Pioneer Robots

## Overview

This guide provides step-by-step instructions to implement ROS 2 communication for 5 Pioneer robots in Webots, enabling them to share position data.

## Architecture Summary

```
┌─────────────────────────────────────────────────────┐
│            Webots Simulation                         │
│                                                      │
│  ┌────────┐  ┌────────┐  ┌────────┐  ...  (5 bots) │
│  │ROBOT_1 │  │ROBOT_2 │  │ROBOT_3 │                 │
│  └───┬────┘  └───┬────┘  └───┬────┘                 │
│      │           │           │                       │
│      └───────────┼───────────┘                       │
│                  │                                   │
│           ┌──────▼──────┐                           │
│           │  Supervisor │  (reads positions)        │
│           │  Controller │                           │
│           └──────┬──────┘                           │
│                  │                                   │
│                  │ (stdout: POSE:... format)        │
└──────────────────┼───────────────────────────────────┘
                   │
                   │ (pipe/subprocess)
                   │
┌──────────────────▼───────────────────────────────────┐
│           ROS 2 Supervisor Node                      │
│  (Python: pioneer_ros2/supervisor_node.py)          │
│                                                      │
│  Reads stdout → Parses poses → Publishes ROS 2      │
└──────────────────────────────────────────────────────┘
│                                                      │
│  Topics Published:                                   │
│  - /robot_1/odom                                     │
│  - /robot_2/odom                                     │
│  - /robot_3/odom                                     │
│  - /robot_4/odom                                     │
│  - /robot_5/odom                                     │
│  - /tf (TF transforms)                               │
└──────────────────────────────────────────────────────┘
```

## Step-by-Step Implementation

### Step 1: Install Prerequisites

```bash
# Install ROS 2 (Ubuntu 22.04 - ROS 2 Humble)
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update
sudo apt install -y ros-humble-desktop

# Install ROS 2 build tools
sudo apt install -y python3-colcon-common-extensions \
                    python3-rosdep \
                    python3-vcstool

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Build Supervisor Controller

```bash
cd /home/karthik/ws/tashi/pioneer/controllers/supervisor
make clean
make
```

This creates the `supervisor` binary that will read robot positions from Webots.

### Step 3: Build ROS 2 Package

```bash
cd /home/karthik/ws/tashi/pioneer/ros2_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build

# Source the workspace
source install/setup.bash
```

### Step 4: Add Supervisor Robot to World (Optional)

The supervisor controller can be run in one of two ways:

**Option A: As a Webots Supervisor Node (Recommended)**

Add to `worlds/pioneer_world.wbt` at the end:

```vrml
Robot {
  controller "supervisor"
  supervisor TRUE
  translation 0 0 0
  children [
    Shape {
      appearance Appearance {
        material Material {
          diffuseColor 1 0 0
        }
      }
      geometry Box {
        size 0.01 0.01 0.01
      }
    }
  ]
}
```

**Option B: Run separately (Current Implementation)**

The supervisor node reads from stdin, so we'll run it separately and capture Webots supervisor output.

### Step 5: Integration Method

The current implementation uses stdout/stderr capture. Here's how it works:

1. **Webots runs the supervisor controller** - reads robot positions every timestep
2. **Supervisor outputs to stdout** in format: `POSE:robot_name:x:y:z:qx:qy:qz:qw:roll:pitch:yaw`
3. **ROS 2 supervisor_node.py** reads from stdin (piped from supervisor) or a file
4. **ROS 2 node parses and publishes** to `/robot_N/odom` topics

## Running the System

### Method 1: Using Launch Script (Recommended)

Create a launch script that:
1. Starts ROS 2 supervisor node
2. Launches Webots with supervisor controller
3. Pipes supervisor output to ROS node

```bash
# Create launch script
cd /home/karthik/ws/tashi/pioneer
./launch_with_ros2.sh
```

### Method 2: Manual Launch

**Terminal 1: Start ROS 2 Supervisor Node**
```bash
cd /home/karthik/ws/tashi/pioneer/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run supervisor node (reads from stdin)
ros2 run pioneer_ros2 supervisor_node
```

**Terminal 2: Launch Webots with Supervisor**
```bash
cd /home/karthik/ws/tashi/pioneer

# Option A: If supervisor is a robot in world
webots worlds/pioneer_world.wbt

# Option B: Run supervisor separately (need to modify)
# This requires Webots to output supervisor controller stdout
```

**Note:** Webots doesn't easily expose supervisor controller stdout to external processes. We need a better integration method.

## Improved Integration: Using File-Based Communication

A better approach is to have the supervisor write to a file, and the ROS 2 node reads from that file.

### Update Supervisor Controller

Modify `supervisor.c` to write to a shared memory location or file:

```c
// At top of main loop
FILE *pose_file = fopen("/tmp/pioneer_poses.txt", "w");

// In loop
for (int i = 0; i < NUM_ROBOTS; i++) {
  if (robot_poses[i].valid) {
    fprintf(pose_file, "POSE:%s:%.6f:%.6f:%.6f:%.6f:%.6f:%.6f:%.6f:%.6f:%.6f:%.6f\n",
            robot_poses[i].name, ...);
    fflush(pose_file);
  }
}
```

### Update ROS 2 Node

Modify `supervisor_node.py` to read from file instead of stdin:

```python
def read_supervisor_output(self):
    pose_file_path = "/tmp/pioneer_poses.txt"
    last_position = 0
    
    while self.reading and rclpy.ok():
        try:
            with open(pose_file_path, 'r') as f:
                f.seek(last_position)
                for line in f:
                    if line.startswith('POSE:'):
                        self.parse_pose_line(line.strip())
                last_position = f.tell()
        except FileNotFoundError:
            pass
        time.sleep(0.01)  # 10ms delay
```

## Testing

### 1. Verify Topics

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
# /tf_static
```

### 2. Echo Robot Positions

```bash
# Check robot 1 position
ros2 topic echo /robot_1/odom

# Check robot 2 position
ros2 topic echo /robot_2/odom
```

### 3. Visualize with RViz

```bash
# Start RViz
rviz2

# Add displays:
# - TF (to see robot frames)
# - MarkerArray or Odometry (to see robot positions)
```

### 4. View TF Tree

```bash
ros2 run tf2_tools view_frames
# Generates frames.pdf with TF tree
```

## Troubleshooting

### Issue: No topics published
- Check supervisor controller is running
- Verify robot DEF names match in world file
- Check ROS 2 node logs: `ros2 topic list`

### Issue: Poses not updating
- Verify supervisor is reading robot positions
- Check file permissions on `/tmp/pioneer_poses.txt`
- Monitor supervisor output

### Issue: TF not publishing
- Check frame IDs match convention: `robot_N/odom`, `robot_N/base_link`
- Verify quaternion values are normalized

## Next Steps

1. **Velocity Estimation**: Compute linear/angular velocity from pose differences
2. **Covariance**: Add proper covariance matrices based on sensor noise
3. **Coordinate Transform**: Map Webots coordinate system to ROS standard (ENU)
4. **Aggregator Node**: Create combined message with all robot positions
5. **Robot-to-Robot Communication**: Each robot subscribes to others' positions for coordination

## File Locations

- **Supervisor Controller**: `controllers/supervisor/supervisor.c`
- **ROS 2 Package**: `ros2_ws/src/pioneer_ros2/`
- **World File**: `worlds/pioneer_world.wbt`
- **Launch Script**: `launch_with_ros2.sh` (to be created)

## Coordinate System

Webots uses:
- X: forward
- Y: left
- Z: up

ROS 2 standard (ENU):
- X: east (forward)
- Y: north (left)
- Z: up

Current implementation uses Webots coordinates directly. For production, add coordinate transform.


