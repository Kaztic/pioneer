# Pioneer Multi-Robot System Architecture

This document describes the complete architecture of the Pioneer multi-robot autonomous exploration system, including all ROS nodes, data flow, topics, FoxMQ/MQTT integration, and how autonomous exploration works.

## System Overview

The system consists of:
- **Webots Simulation**: Runs 5 Pioneer3at robots with LiDAR sensors
- **C Controllers**: Supervisor and robot controllers that interface with Webots
- **File-Based Bridge**: Temporary files in `/tmp/` for communication between C and Python
- **ROS 2 Nodes**: Python nodes implementing navigation, path planning, and exploration
- **FoxMQ/MQTT**: Distributed MQTT broker cluster for inter-robot communication
- **Autonomous Exploration**: Frontier-based exploration with A* path planning

## High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Webots Simulation                            │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ...  ┌──────────┐ │
│  │ Robot 1  │  │ Robot 2  │  │ Robot 3  │        │ Robot 5  │ │
│  │ (C Ctrl) │  │ (C Ctrl) │  │ (C Ctrl) │        │ (C Ctrl) │ │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘        └────┬─────┘ │
│       │              │              │                    │        │
│       └──────────────┼──────────────┼────────────────────┘        │
│                     │              │                              │
│              ┌──────▼──────────────▼──────┐                      │
│              │  Supervisor Controller (C)  │                      │
│              └──────┬──────────────────────┘                      │
└─────────────────────┼────────────────────────────────────────────┘
                      │ File Bridge (/tmp/)
┌─────────────────────▼────────────────────────────────────────────┐
│                    ROS 2 Bridge Layer                            │
│  ┌──────────────────┐  ┌──────────────────┐                    │
│  │ supervisor_node   │  │ lidar_bridge_node │                    │
│  │ (1 instance)      │  │ (5 instances)     │                    │
│  └──────────────────┘  └──────────────────┘                    │
└─────────────────────┬────────────────────────────────────────────┘
                      │ ROS 2 Topics
┌─────────────────────▼────────────────────────────────────────────┐
│              Navigation & Exploration Stack                      │
│  ┌──────────────────┐  ┌──────────────────┐                   │
│  │ occupancy_grid_   │  │ frontier_         │                   │
│  │ builder_node      │  │ detector_node     │                   │
│  └──────────────────┘  └──────────────────┘                   │
│  ┌──────────────────┐  ┌──────────────────┐                   │
│  │ goal_selector_    │  │ path_planner_     │                   │
│  │ node              │  │ node              │                   │
│  └──────────────────┘  └──────────────────┘                   │
│  ┌──────────────────┐                                           │
│  │ path_follower_    │                                           │
│  │ node              │                                           │
│  └──────────────────┘                                           │
└─────────────────────┬────────────────────────────────────────────┘
                      │
┌─────────────────────▼────────────────────────────────────────────┐
│              FoxMQ/MQTT Communication Layer                      │
│  ┌──────────────────┐  ┌──────────────────┐                    │
│  │ foxmq_bridge_    │  │ FoxMQ Cluster     │                    │
│  │ node (per robot) │◄─┤ (4-node HA)      │                    │
│  └──────────────────┘  └──────────────────┘                    │
└──────────────────────────────────────────────────────────────────┘
```

## ROS Node Architecture

### Package: `pioneer_ros2`

This package contains the bridge nodes that connect Webots (C controllers) to ROS 2, and the FoxMQ bridge for inter-robot communication.

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
- Graceful shutdown handling

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

#### 3. `foxmq_bridge_node`

**Purpose:** Bridges ROS 2 topics to MQTT topics via FoxMQ broker for robot-to-robot communication.

**Subscribes to (ROS 2 → MQTT):**
- `/{robot_id}/frontiers` - Frontier points (std_msgs/Float32MultiArray)
- `/{robot_id}/object_detection` - Object detections (future)
- `/{robot_id}/mission_status` - Mission status (future)
- `/{robot_id}/territory_assignment` - Territory assignments (future)

**Publishes to (MQTT → ROS 2):**
- `/coordination/frontiers` - Shared frontier information
- `/{robot_id}/object_detection_remote` - Remote detections
- `/{robot_id}/mission_status_remote` - Remote status
- `/consensus/results` - Consensus results (future)

**MQTT Topics:**
- `coordination/frontiers` - Shared frontier information
- `coordination/paths` - Planned paths (future)
- `robots/{robot_id}/detection` - Object detections
- `robots/{robot_id}/status` - Mission status
- `robots/{robot_id}/territory` - Territory assignments
- `robots/{robot_id}/consensus_vote` - Consensus votes (future)

**Key Features:**
- One instance per robot
- JSON message serialization
- QoS 1 (at least once delivery)
- Automatic reconnection on disconnect
- Message routing based on topic structure

#### 4. `spatial_analysis_visualizer_node`

**Purpose:** Visualizes exploration metrics, robot trajectories, and spatial analysis data.

**Subscribes to:**
- `/{robot_id}/odom` - Robot odometry (all robots)
- `/{robot_id}/global_path` - Planned paths
- `/{robot_id}/frontiers` - Frontier points

**Publishes:**
- `/spatial_analysis/heatmap` - Exploration heatmap (visualization_msgs/Marker)
- `/spatial_analysis/trajectories` - Robot trajectories
- `/spatial_analysis/frontiers` - Frontier visualizations

**Key Features:**
- Aggregates data from all robots
- Generates heatmaps of explored areas
- Visualizes robot paths and efficiency metrics
- Configurable update rate and resolution

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
- Logs map statistics (occupied/free/unknown counts)

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
- Configurable detection frequency

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
3. Validates goal against occupancy grid (ensures goal is in free space)
4. If goal is invalid, searches for nearby free cell
5. Selects best frontier and publishes as goal
6. If no frontiers available, generates initial exploration goal

**Key Features:**
- Autonomous goal selection for exploration
- Multi-robot coordination (simple mutex via name hashing, enhanced with FoxMQ)
- Fallback to initial exploration goals when no frontiers detected
- Goal timeout and re-selection logic
- **Goal Validation**: Checks occupancy grid before publishing goals
- **Goal Adjustment**: Finds nearby free cells if selected goal is occupied
- **Re-validation**: Re-checks current goal validity before keeping it

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
- Validates goal is in free space before planning

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

## FoxMQ/MQTT Architecture

### FoxMQ Cluster

**Purpose:** Provides high-availability MQTT broker infrastructure for inter-robot communication.

**Configuration:**
- **Cluster Size**: 4 nodes (high availability)
- **Ports**: 19793-19796 (local testing)
- **Address Book**: `foxmq_cluster/foxmq.d/address-book.toml`
- **User Authentication**: `foxmq_cluster/foxmq.d/users.toml`
- **Default User**: `pioneer_robot`

**Features:**
- Distributed consensus (planned, not yet implemented)
- High availability with automatic failover
- MQTT v5 protocol support
- User authentication and authorization

### MQTT Topic Structure

```
coordination/
  ├── frontiers      # Shared frontier information (active)
  ├── paths          # Planned paths for collision avoidance (planned)
  └── goals          # Shared exploration goals (planned)

robots/
  ├── robot_1/
  │   ├── detection      # Object detection reports (planned)
  │   ├── status         # Mission status updates (planned)
  │   ├── territory      # Territory assignments (planned)
  │   ├── consensus_vote # Consensus voting (planned)
  │   └── consensus_result # Consensus results (planned)
  ├── robot_2/
  │   └── ... (same structure)
  └── robot_5/
      └── ... (same structure)
```

### Message Format

All MQTT messages use JSON format:

```json
{
  "robot_id": "robot_1",
  "timestamp": 1234567890.123,
  "frontiers": [
    {"x": 10.5, "y": 5.2},
    {"x": 12.3, "y": 7.1}
  ]
}
```

### Consensus Status

**Current Status:** FoxMQ consensus features are **not yet implemented**.

**Planned Usage:**
- Territory assignment consensus (4/5 robots required)
- Object detection verification (4/5 robots confirm)
- Mission status coordination
- Collision avoidance priority resolution

**Infrastructure Ready:**
- Consensus topics defined in bridge node
- ConsensusMessage.msg message type exists
- Topic routing implemented
- Actual consensus logic: **TODO**

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
                   ↓ ROS2 TOPICS ↓
┌─────────────────────────────────────────────────────────────┐
│ FOXMQ BRIDGE (5 instances, one per robot)                   │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  foxmq_bridge_node.py                                       │
│  ├─ Subscribes: /robot_X/frontiers                         │
│  ├─ Publishes: /coordination/frontiers (MQTT)              │
│  ├─ Subscribes: coordination/frontiers (MQTT)              │
│  └─ Publishes: /coordination/frontiers (ROS2)               │
└─────────────────────────────────────────────────────────────┘
                   ↓ MQTT PROTOCOL ↓
┌─────────────────────────────────────────────────────────────┐
│ FOXMQ CLUSTER (4-node HA)                                   │
├─────────────────────────────────────────────────────────────┤
│  MQTT Broker Cluster                                        │
│  - Ports: 19793-19796                                       │
│  - Topics: coordination/*, robots/*                        │
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
  - Also bridged to MQTT: `coordination/frontiers`

#### Planning Topics
- `/{robot_id}/goal` - Exploration goal (geometry_msgs/PoseStamped)
  - Published by: `goal_selector_node`
  - Frequency: On-demand (when goal changes)
  - Validated against occupancy grid before publishing
  
- `/{robot_id}/global_path` - Planned path (nav_msgs/Path)
  - Published by: `path_planner_node`
  - Frequency: On-demand (when goal changes or path replanning)

#### Control Topics
- `/{robot_id}/cmd_vel` - Velocity commands (geometry_msgs/Twist)
  - Published by: `path_follower_node`
  - Frequency: ~10-20 Hz

### Coordination Topics (MQTT → ROS 2)

- `/coordination/frontiers` - Shared frontier information (std_msgs/Float32MultiArray)
  - Published by: `foxmq_bridge_node` (from MQTT)
  - Source: MQTT topic `coordination/frontiers`
  - Contains frontiers from all robots

- `/consensus/results` - Consensus results (std_msgs/String, JSON)
  - Published by: `foxmq_bridge_node` (from MQTT)
  - Source: MQTT topic `robots/*/consensus_result`
  - **Status**: Infrastructure ready, not yet used

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

3. **Frontier Sharing (via FoxMQ)**
   - `foxmq_bridge_node` receives frontiers from `frontier_detector_node`
   - Publishes frontiers to MQTT topic `coordination/frontiers`
   - Other robots receive shared frontiers via their bridge nodes

4. **Goal Selection**
   - `goal_selector_node` receives frontier points (local and shared)
   - Evaluates each frontier based on:
     - Distance from robot
     - Information gain (frontier size)
     - Multi-robot coordination (avoid selecting same frontier)
   - **Validates goal against occupancy grid** (ensures goal is in free space)
   - If goal is invalid, searches for nearby free cell
   - Selects best frontier and publishes as goal
   - If no frontiers, generates initial exploration goal

5. **Path Planning**
   - `path_planner_node` receives goal
   - Uses A* algorithm to plan path from current pose to goal
   - Plans path avoiding obstacles in occupancy grid
   - Validates goal is in free space before planning
   - Publishes path as sequence of waypoints

6. **Path Following**
   - `path_follower_node` receives planned path
   - Tracks current waypoint
   - Calculates velocity commands (linear and angular)
   - Publishes `cmd_vel` commands

7. **Command Execution**
   - Robot controller (C) reads velocity commands from file
   - Executes commands in Webots
   - Robot moves toward goal

8. **Loop**
   - Robot moves, updates map, detects new frontiers
   - Process repeats for continuous exploration

### Multi-Robot Coordination

The system uses multiple coordination mechanisms:

1. **Name-based Frontier Assignment**: Each robot's name is hashed to assign frontiers, reducing collisions
2. **FoxMQ Frontier Sharing**: Robots share detected frontiers via MQTT, enabling better coordination
3. **Independent Maps**: Each robot maintains its own occupancy grid
4. **Shared TF Tree**: All robots share the same transform tree for coordinate reference
5. **Goal Validation**: Goals are validated against occupancy grid to prevent invalid selections

### Exploration Phases

1. **Initial Exploration**: When no frontiers are detected, robots use pre-defined exploration goals
2. **Frontier-Based Exploration**: Once frontiers are detected, robots select best frontiers
3. **Continuous Exploration**: As map builds, new frontiers are discovered and explored
4. **Coordination**: Robots share frontiers via FoxMQ to avoid redundant exploration

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
- Written by: `cmd_vel_bridge_node.py` (one per robot) - **Note**: Currently not used, path follower publishes directly
- Read by: Webots robot controllers (C)
- Format: `linear_x:angular_z:timestamp`
- Update rate: ~10-20 Hz

## Node Launch Architecture

Nodes are launched in a specific order:

1. **FoxMQ Cluster** (if using inter-robot communication)
   - Must be running before bridge nodes
   - Provides MQTT broker infrastructure

2. **Supervisor Node** (1 instance)
   - Must be running before other nodes
   - Provides odometry for all robots

3. **Bridge Nodes** (per robot)
   - `lidar_bridge_node` - Must start before occupancy grid builder
   - `foxmq_bridge_node` - Can start anytime (connects to FoxMQ cluster)

4. **Navigation Nodes** (per robot)
   - `occupancy_grid_builder_node` - Depends on lidar_bridge
   - `frontier_detector_node` - Depends on occupancy_grid_builder
   - `goal_selector_node` - Depends on frontier_detector and odom
   - `path_planner_node` - Depends on goal and odom
   - `path_follower_node` - Depends on path_planner and odom

5. **Visualization Nodes** (optional)
   - `spatial_analysis_visualizer_node` - Can start anytime

The `start_full_system.sh` script handles this dependency order automatically.

## Summary

The Pioneer multi-robot system implements a complete autonomous exploration pipeline:
- **Sensing**: LiDAR scans from Webots
- **Mapping**: Occupancy grid building
- **Exploration**: Frontier detection and goal selection
- **Coordination**: Frontier sharing via FoxMQ/MQTT
- **Planning**: A* path planning with goal validation
- **Control**: Path following and velocity commands
- **Execution**: File-based bridge to Webots controllers
- **Visualization**: Spatial analysis and metrics

All nodes run independently per robot, enabling true multi-robot autonomous exploration with coordination through frontier sharing and future consensus mechanisms.
