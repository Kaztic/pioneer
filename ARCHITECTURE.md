# Pioneer Multi-Robot System Architecture

This document describes the complete architecture of the Pioneer multi-robot autonomous exploration system, including all ROS nodes, data flow, topics, and how autonomous exploration works.

## System Overview

The system consists of:
- **Webots Simulation**: Runs 5 Pioneer3at robots with LiDAR sensors
- **C Controllers**: Supervisor and robot controllers that interface with Webots
- **File-Based Bridge**: Temporary files in `/tmp/` for communication between C and Python
- **ROS 2 Nodes**: Python nodes implementing navigation, path planning, and exploration
- **Autonomous Exploration**: Frontier-based exploration with A* path planning

## ROS Node Architecture

### Package: `pioneer_ros2`

This package contains the bridge nodes that connect Webots (C controllers) to ROS 2.

#### 1. `supervisor_node`

**Purpose:** Reads robot pose data from Webots supervisor controller and publishes it as ROS 2 odometry.

**Subscribes to:** None (reads from file system)

**Publishes:**
- `/robot_1/odom` - Odometry for robot 1 (nav_msgs/Odometry)
- `/robot_2/odom` - Odometry for robot 2
- `/robot_3/odom` - Odometry for robot 3
- `/robot_4/odom` - Odometry for robot 4
- `/robot_5/odom` - Odometry for robot 5
- `/tf` - Transform tree (tf2_msgs/TFMessage)

**Data Flow:**
1. Webots supervisor controller (C) writes robot poses to `/tmp/pioneer_poses.txt`
2. Supervisor node reads this file continuously
3. Parses pose data for each robot
4. Publishes odometry messages at ~30 Hz (Webots timestep)

**Key Features:**
- Single node instance handles all 5 robots
- Publishes TF transforms for each robot
- Handles pose file parsing and error handling

#### 2. `lidar_bridge_node`

**Purpose:** Bridges LiDAR scan data from Webots robot controllers to ROS 2.

**Subscribes to:** None (reads from file system)

**Publishes:**
- `/{robot_id}/scan` - LiDAR scan data (sensor_msgs/LaserScan)

**Data Flow:**
1. Each robot controller (C) writes LiDAR data to `/tmp/pioneer_lidar_robot_X.txt`
2. One `lidar_bridge_node` instance per robot reads its corresponding file
3. Parses LiDAR ranges and converts to ROS 2 LaserScan message
4. Publishes scan data at robot's update rate

**Key Features:**
- One instance per robot (launched with namespace `/{robot_id}`)
- Converts file-based data to ROS 2 messages
- Handles Sick LMS 291 LiDAR format

#### 3. `cmd_vel_bridge_node`

**Purpose:** Bridges velocity commands from ROS 2 to Webots robot controllers.

**Subscribes to:**
- `/{robot_id}/cmd_vel` - Velocity commands (geometry_msgs/Twist)

**Publishes:** None

**Data Flow:**
1. Path follower node publishes `cmd_vel` commands to ROS 2 topic
2. `cmd_vel_bridge_node` subscribes to this topic
3. Writes velocity commands to `/tmp/pioneer_cmd_vel_robot_X.txt`
4. Robot controller (C) reads this file and executes commands

**Key Features:**
- One instance per robot (launched with namespace `/{robot_id}`)
- Converts ROS 2 Twist messages to file format
- Includes timestamp for synchronization

#### 4. `aggregator_node`

**Purpose:** Aggregates data from multiple robots (if used for multi-robot coordination).

**Note:** This node exists in the package but may not be actively used in the current autonomous exploration mode.

### Package: `pioneer_path_planner`

This package contains the navigation and exploration nodes.

#### 1. `occupancy_grid_builder_node`

**Purpose:** Builds a 2D occupancy grid map from LiDAR scan data.

**Subscribes to:**
- `/{robot_id}/scan` - LiDAR scans (sensor_msgs/LaserScan)

**Publishes:**
- `/{robot_id}/occupancy_map` - Occupancy grid map (nav_msgs/OccupancyGrid)

**Data Flow:**
1. Receives LiDAR scans from `lidar_bridge_node`
2. Converts scan data to occupancy grid cells
3. Updates grid with obstacles (high values) and free space (low values)
4. Publishes grid updates at configurable rate

**Key Features:**
- Maintains local map per robot
- Configurable resolution (typically 0.1-0.5m per cell)
- Handles unknown, free, and occupied cells

#### 2. `frontier_detector_node`

**Purpose:** Detects frontiers (boundaries between known and unknown space) for exploration.

**Subscribes to:**
- `/{robot_id}/occupancy_map` - Occupancy grid (nav_msgs/OccupancyGrid)

**Publishes:**
- `/{robot_id}/frontiers` - Detected frontier points (std_msgs/Float32MultiArray)

**Data Flow:**
1. Receives occupancy grid updates
2. Analyzes grid to find cells that are:
   - Known (free or occupied)
   - Adjacent to unknown cells
3. Groups frontier cells into frontier regions
4. Publishes frontier points as array of (x, y) coordinates

**Key Features:**
- Detects exploration frontiers in real-time
- Groups frontier cells into regions
- Filters small frontiers (noise reduction)

#### 3. `goal_selector_node`

**Purpose:** Selects the best frontier as an exploration goal for the robot.

**Subscribes to:**
- `/{robot_id}/frontiers` - Frontier points (std_msgs/Float32MultiArray)
- `/{robot_id}/odom` - Current robot pose (nav_msgs/Odometry)
- `/{robot_id}/occupancy_map` - Occupancy grid (nav_msgs/OccupancyGrid)

**Publishes:**
- `/{robot_id}/goal` - Selected exploration goal (geometry_msgs/PoseStamped)

**Data Flow:**
1. Receives frontier points from `frontier_detector_node`
2. Evaluates each frontier based on:
   - Distance from robot (closer is better)
   - Frontier size/information gain
   - Avoids selecting same frontier as other robots (name-based hashing)
3. Selects best frontier and publishes as goal
4. If no frontiers available, generates initial exploration goal

**Key Features:**
- Autonomous goal selection for exploration
- Multi-robot coordination (simple mutex via name hashing)
- Fallback to initial exploration goals when no frontiers detected
- Goal timeout and re-selection logic

#### 4. `path_planner_node`

**Purpose:** Plans a path from current robot pose to goal using A* algorithm.

**Subscribes to:**
- `/{robot_id}/odom` - Current robot pose (nav_msgs/Odometry)
- `/{robot_id}/goal` - Goal pose (geometry_msgs/PoseStamped)
- `/{robot_id}/occupancy_map` - Occupancy grid (nav_msgs/OccupancyGrid)

**Publishes:**
- `/{robot_id}/global_path` - Planned path (nav_msgs/Path)

**Data Flow:**
1. Receives current pose and goal
2. Uses A* path planning algorithm on occupancy grid
3. Plans path avoiding obstacles
4. Publishes path as sequence of waypoints

**Key Features:**
- A* path planning algorithm
- Obstacle avoidance
- Allows traversal through unknown cells (exploration mode)
- Path smoothing and optimization

#### 5. `path_follower_node`

**Purpose:** Follows the planned path by generating velocity commands.

**Subscribes to:**
- `/{robot_id}/global_path` - Planned path (nav_msgs/Path)
- `/{robot_id}/odom` - Current robot pose (nav_msgs/Odometry)

**Publishes:**
- `/{robot_id}/cmd_vel` - Velocity commands (geometry_msgs/Twist)

**Data Flow:**
1. Receives planned path from `path_planner_node`
2. Tracks current waypoint in path
3. Calculates linear and angular velocity to reach next waypoint
4. Publishes `cmd_vel` commands
5. Advances to next waypoint when current one is reached

**Key Features:**
- Proportional control or pure pursuit algorithm
- Waypoint arrival threshold handling
- Path completion detection
- Smooth velocity commands

## Data Flow Architecture

### Complete Data Flow Pipeline

```
┌─────────────────────────────────────────────────────────────┐
│ WEBOTS SIMULATION (C Controllers)                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Supervisor Controller (C)                                  │
│  ├─ Reads: Robot poses from Webots API                     │
│  └─ Writes: /tmp/pioneer_poses.txt                         │
│      Format: POSE:robot_1:x:y:z:qx:qy:qz:qw:timestamp      │
│                                                             │
│  Robot Controllers (C) x 5                                  │
│  ├─ Read: /tmp/pioneer_cmd_vel_robot_X.txt                 │
│  │   Format: linear_x:angular_z:timestamp                  │
│  └─ Write: /tmp/pioneer_lidar_robot_X.txt                  │
│      Format: LIDAR:robot_X:timestamp:ranges...              │
└─────────────────────────────────────────────────────────────┘
                   ↓ FILE BRIDGE ↓
┌─────────────────────────────────────────────────────────────┐
│ ROS2 BRIDGE NODES (Python)                                 │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  supervisor_node.py (1 instance)                            │
│  ├─ Reads: /tmp/pioneer_poses.txt                          │
│  └─ Publishes: /robot_X/odom, /tf                         │
│                                                             │
│  lidar_bridge_node.py (5 instances)                        │
│  ├─ Reads: /tmp/pioneer_lidar_robot_X.txt                  │
│  └─ Publishes: /robot_X/scan                               │
│                                                             │
│  cmd_vel_bridge_node.py (5 instances)                       │
│  ├─ Subscribes: /robot_X/cmd_vel                           │
│  └─ Writes: /tmp/pioneer_cmd_vel_robot_X.txt               │
└─────────────────────────────────────────────────────────────┘
                   ↓ ROS2 TOPICS ↓
┌─────────────────────────────────────────────────────────────┐
│ NAVIGATION STACK (5 instances, one per robot)              │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  occupancy_grid_builder_node.py                             │
│  ├─ Subscribes: /robot_X/scan                               │
│  └─ Publishes: /robot_X/occupancy_map                       │
│                                                             │
│  frontier_detector_node.py                                  │
│  ├─ Subscribes: /robot_X/occupancy_map                     │
│  └─ Publishes: /robot_X/frontiers                          │
│                                                             │
│  goal_selector_node.py                                      │
│  ├─ Subscribes: /robot_X/frontiers, /robot_X/odom          │
│  └─ Publishes: /robot_X/goal                                │
│                                                             │
│  path_planner_node.py                                       │
│  ├─ Subscribes: /robot_X/goal, /robot_X/odom                │
│  └─ Publishes: /robot_X/global_path                        │
│                                                             │
│  path_follower_node.py                                      │
│  ├─ Subscribes: /robot_X/global_path, /robot_X/odom         │
│  └─ Publishes: /robot_X/cmd_vel                            │
└─────────────────────────────────────────────────────────────┘
```

## Topic Architecture

### Per-Robot Topics

Each robot has its own namespace (`/robot_1`, `/robot_2`, etc.) with the following topics:

#### Sensor Topics
- `/{robot_id}/scan` - LiDAR scan data (sensor_msgs/LaserScan)
  - Published by: `lidar_bridge_node`
  - Frequency: ~10-30 Hz (depends on Webots timestep)

#### Odometry Topics
- `/{robot_id}/odom` - Robot odometry (nav_msgs/Odometry)
  - Published by: `supervisor_node`
  - Frequency: ~30 Hz (Webots timestep)

#### Mapping Topics
- `/{robot_id}/occupancy_map` - Occupancy grid map (nav_msgs/OccupancyGrid)
  - Published by: `occupancy_grid_builder_node`
  - Frequency: ~1-5 Hz (configurable)

#### Exploration Topics
- `/{robot_id}/frontiers` - Detected frontier points (std_msgs/Float32MultiArray)
  - Published by: `frontier_detector_node`
  - Format: Array of (x, y) coordinates
  - Frequency: ~1-2 Hz

#### Planning Topics
- `/{robot_id}/goal` - Exploration goal (geometry_msgs/PoseStamped)
  - Published by: `goal_selector_node`
  - Frequency: On-demand (when goal changes)
  
- `/{robot_id}/global_path` - Planned path (nav_msgs/Path)
  - Published by: `path_planner_node`
  - Frequency: On-demand (when goal changes or path replanning)

#### Control Topics
- `/{robot_id}/cmd_vel` - Velocity commands (geometry_msgs/Twist)
  - Published by: `path_follower_node`
  - Subscribed by: `cmd_vel_bridge_node`
  - Frequency: ~10-20 Hz

### Global Topics
- `/tf` - Transform tree (tf2_msgs/TFMessage)
  - Published by: `supervisor_node`
  - Contains transforms for all robots

## Autonomous Exploration Flow

### How Robots Explore Autonomously

The autonomous exploration system works in a continuous loop:

1. **Map Building**
   - `lidar_bridge_node` receives LiDAR scans from Webots
   - `occupancy_grid_builder_node` processes scans and builds occupancy grid
   - Map is continuously updated as robot moves

2. **Frontier Detection**
   - `frontier_detector_node` analyzes occupancy grid
   - Detects boundaries between known and unknown space
   - Publishes frontier points

3. **Goal Selection**
   - `goal_selector_node` receives frontier points
   - Evaluates each frontier based on:
     - Distance from robot
     - Information gain (frontier size)
     - Multi-robot coordination (avoid selecting same frontier)
   - Selects best frontier and publishes as goal
   - If no frontiers, generates initial exploration goal

4. **Path Planning**
   - `path_planner_node` receives goal
   - Uses A* algorithm to plan path from current pose to goal
   - Plans path avoiding obstacles in occupancy grid
   - Publishes path as sequence of waypoints

5. **Path Following**
   - `path_follower_node` receives planned path
   - Tracks current waypoint
   - Calculates velocity commands (linear and angular)
   - Publishes `cmd_vel` commands

6. **Command Execution**
   - `cmd_vel_bridge_node` receives velocity commands
   - Writes commands to file for robot controller
   - Robot controller executes commands in Webots
   - Robot moves toward goal

7. **Loop**
   - Robot moves, updates map, detects new frontiers
   - Process repeats for continuous exploration

### Multi-Robot Coordination

The system uses simple coordination mechanisms:

- **Name-based Frontier Assignment**: Each robot's name is hashed to assign frontiers, reducing collisions
- **Independent Maps**: Each robot maintains its own occupancy grid
- **Shared TF Tree**: All robots share the same transform tree for coordinate reference

### Exploration Phases

1. **Initial Exploration**: When no frontiers are detected, robots use pre-defined exploration goals
2. **Frontier-Based Exploration**: Once frontiers are detected, robots select best frontiers
3. **Continuous Exploration**: As map builds, new frontiers are discovered and explored

## Node Launch Architecture

Nodes are launched in a specific order:

1. **Supervisor Node** (1 instance)
   - Must be running before other nodes
   - Provides odometry for all robots

2. **Bridge Nodes** (per robot)
   - `lidar_bridge_node` - Must start before occupancy grid builder
   - `cmd_vel_bridge_node` - Receives commands from path follower

3. **Navigation Nodes** (per robot)
   - `occupancy_grid_builder_node` - Depends on lidar_bridge
   - `frontier_detector_node` - Depends on occupancy_grid_builder
   - `goal_selector_node` - Depends on frontier_detector and odom
   - `path_planner_node` - Depends on goal and odom
   - `path_follower_node` - Depends on path_planner and odom

The `launch_multi_robot.sh` script handles this dependency order automatically.

## File-Based Communication

The system uses file-based communication between Webots (C) and ROS 2 (Python):

### Pose File: `/tmp/pioneer_poses.txt`
- Written by: Webots supervisor controller (C)
- Read by: `supervisor_node.py`
- Format: `POSE:robot_1:x:y:z:qx:qy:qz:qw:timestamp`
- Update rate: ~30 Hz (Webots timestep)

### LiDAR Files: `/tmp/pioneer_lidar_robot_X.txt`
- Written by: Webots robot controllers (C)
- Read by: `lidar_bridge_node.py` (one per robot)
- Format: `LIDAR:robot_X:timestamp:ranges...`
- Update rate: ~10-30 Hz

### Command Files: `/tmp/pioneer_cmd_vel_robot_X.txt`
- Written by: `cmd_vel_bridge_node.py` (one per robot)
- Read by: Webots robot controllers (C)
- Format: `linear_x:angular_z:timestamp`
- Update rate: ~10-20 Hz

## Summary

The Pioneer multi-robot system implements a complete autonomous exploration pipeline:
- **Sensing**: LiDAR scans from Webots
- **Mapping**: Occupancy grid building
- **Exploration**: Frontier detection and goal selection
- **Planning**: A* path planning
- **Control**: Path following and velocity commands
- **Execution**: File-based bridge to Webots controllers

All nodes run independently per robot, enabling true multi-robot autonomous exploration with coordination through frontier selection.

