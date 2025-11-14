#!/usr/bin/env python3
"""
ROS2 Goal Selector Node

Subscribes to:
- /robot_i/frontiers: Detected frontier points (std_msgs/Float32MultiArray)
- /robot_i/odom: Current robot pose (nav_msgs/Odometry)

Publishes:
- /robot_i/goal: Selected exploration goal (geometry_msgs/PoseStamped)

Selects the best frontier as exploration goal based on:
- Distance from robot (closer is better)
- Frontier size/information gain (larger is better)
- Avoids selecting same frontier as other robots (simple mutex via name hashing)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Float32MultiArray, Header
import math
from typing import Optional, List, Tuple

# Import custom message
try:
    from pioneer_msgs.msg import MissionStatus, TerritoryAssignment
    CUSTOM_MSGS_AVAILABLE = True
except ImportError:
    CUSTOM_MSGS_AVAILABLE = False
    print("Warning: MissionStatus message not available")


class GoalSelectorNode(Node):
    """
    Goal selector node for a single robot.
    Selects best frontier for exploration.
    """

    def __init__(self, robot_id: Optional[str] = None):
        """
        Initialize goal selector node.

        Args:
            robot_id: Robot identifier (e.g., "robot_1", "robot_2").
                     If None, will be extracted from node namespace.
        """
        node_name = 'goal_selector_node'
        
        if robot_id:
            super().__init__(node_name, namespace=robot_id)
            self.robot_id = robot_id
        else:
            super().__init__(node_name)
            namespace = self.get_namespace()
            if namespace and namespace != '/':
                self.robot_id = namespace.strip('/') if namespace.startswith('/') else namespace
            else:
                full_name = self.get_fully_qualified_name()
                parts = [p for p in full_name.split('/') if p]
                if len(parts) >= 2:
                    self.robot_id = parts[0]
                else:
                    self.robot_id = "robot_1"

        # Re-extract robot_id after node is fully initialized
        namespace = self.get_namespace()
        if namespace and namespace != '/':
            self.robot_id = namespace.strip('/') if namespace.startswith('/') else namespace
        else:
            full_name = self.get_fully_qualified_name()
            parts = [p for p in full_name.split('/') if p]
            if len(parts) >= 2:
                self.robot_id = parts[0]
            if not hasattr(self, 'robot_id') or not self.robot_id or self.robot_id == "None":
                self.robot_id = "robot_1"

        # Parameters
        self.declare_parameter('distance_weight', 0.5)  # Weight for distance component
        self.declare_parameter('information_gain_weight', 0.5)  # Weight for information gain
        self.declare_parameter('min_frontier_distance', 0.5)  # meters - min distance to consider frontier (lowered)
        self.declare_parameter('selection_frequency', 1.0)  # Hz
        self.declare_parameter('goal_timeout', 30.0)  # seconds - replan goal after timeout
        self.declare_parameter('frontier_separation_distance', 2.0)  # meters - min distance between robot goals
        self.declare_parameter('initial_exploration_distance', 5.0)  # meters - distance for initial exploration (increased from 3.0)
        self.declare_parameter('enable_autonomous_exploration', True)  # Enable autonomous frontier-based exploration
        self.declare_parameter('initial_exploration_phases', 3)  # Number of initial exploration phases before giving up

        self.distance_weight = self.get_parameter('distance_weight').get_parameter_value().double_value
        self.information_gain_weight = self.get_parameter('information_gain_weight').get_parameter_value().double_value
        self.min_frontier_distance = self.get_parameter('min_frontier_distance').get_parameter_value().double_value
        self.goal_timeout = self.get_parameter('goal_timeout').get_parameter_value().double_value
        self.frontier_separation_distance = self.get_parameter('frontier_separation_distance').get_parameter_value().double_value
        self.initial_exploration_distance = self.get_parameter('initial_exploration_distance').get_parameter_value().double_value
        self.enable_autonomous_exploration = self.get_parameter('enable_autonomous_exploration').get_parameter_value().bool_value
        self.initial_exploration_phases = self.get_parameter('initial_exploration_phases').get_parameter_value().integer_value

        # State
        self.current_frontiers: List[Tuple[float, float]] = []
        self.shared_frontiers: List[Tuple[float, float]] = []  # Frontiers from other robots
        self.robot_position: Optional[Tuple[float, float]] = None
        self.robot_yaw: float = 0.0
        self.current_goal: Optional[PoseStamped] = None
        self.goal_set_time: Optional[float] = None
        self.initial_goal_published: bool = False  # Track if we've published initial goal
        self.initial_exploration_phase: int = 0  # Track which phase of initial exploration we're in
        self.occupancy_grid: Optional[OccupancyGrid] = None  # Store occupancy grid for goal validation

        # Subscription
        self.frontiers_subscription = self.create_subscription(
            Float32MultiArray,
            'frontiers',
            self.frontiers_callback,
            10
        )
        self.get_logger().info(f'Subscribed to /{self.robot_id}/frontiers')

        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.get_logger().info(f'Subscribed to /{self.robot_id}/odom')

        # Subscribe to occupancy grid for goal validation
        self.occupancy_grid_subscription = self.create_subscription(
            OccupancyGrid,
            'occupancy_map',
            self.occupancy_grid_callback,
            10
        )
        self.get_logger().info(f'Subscribed to /{self.robot_id}/occupancy_map')

        # Subscribe to shared frontiers from coordination topic (via FoxMQ)
        self.coordination_frontiers_subscription = self.create_subscription(
            Float32MultiArray,
            '/coordination/frontiers',  # Global topic (not namespaced)
            self.coordination_frontiers_callback,
            10
        )
        self.get_logger().info('Subscribed to /coordination/frontiers')

        # Publisher
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            'goal',
            10
        )
        self.get_logger().info(f'Publishing to /{self.robot_id}/goal')

        # Mission status publisher
        if CUSTOM_MSGS_AVAILABLE:
            self.mission_status_publisher = self.create_publisher(
                MissionStatus,
                'mission_status',
                10
            )
            self.get_logger().info(f'Publishing to /{self.robot_id}/mission_status')
        
        # Subscribe to territory assignments
        if CUSTOM_MSGS_AVAILABLE:
            self.territory_subscription = self.create_subscription(
                TerritoryAssignment,
                'territory_assignment',
                self.territory_callback,
                10
            )
            self.get_logger().info(f'Subscribed to /{self.robot_id}/territory_assignment')

        # Subscribe to verification goals
        self.verification_goal_subscription = self.create_subscription(
            PoseStamped,
            'verification_goal',
            self.verification_goal_callback,
            10
        )
        self.get_logger().info(f'Subscribed to /{self.robot_id}/verification_goal')
        
        # Mission status state
        self.current_mission_status = 0  # EXPLORING
        self.object_found = False
        self.object_pose: Optional[PoseStamped] = None
        self.current_territory: Optional[TerritoryAssignment] = None
        self.verification_goal: Optional[PoseStamped] = None

        # Timer for periodic goal selection
        selection_freq = self.get_parameter('selection_frequency').get_parameter_value().double_value
        self.selection_timer = self.create_timer(
            1.0 / selection_freq,
            self.select_goal_callback
        )

        self.get_logger().info(
            f'Goal selector node initialized for {self.robot_id}\n'
            f'  Selection frequency: {selection_freq}Hz\n'
            f'  Distance weight: {self.distance_weight}\n'
            f'  Information gain weight: {self.information_gain_weight}'
        )

    def frontiers_callback(self, msg: Float32MultiArray):
        """Update current frontiers from frontier detector."""
        # Parse Float32MultiArray: data = [x1, y1, x2, y2, ...]
        frontiers = []
        data = msg.data
        
        if len(data) % 2 != 0:
            self.get_logger().warn('Frontiers array has odd number of elements, ignoring')
            return
        
        for i in range(0, len(data), 2):
            x = data[i]
            y = data[i + 1]
            frontiers.append((x, y))
        
        self.current_frontiers = frontiers
        self.get_logger().debug(f'Received {len(frontiers)} frontiers')

    def coordination_frontiers_callback(self, msg: Float32MultiArray):
        """Update shared frontiers from other robots via FoxMQ."""
        # Parse Float32MultiArray: data = [x1, y1, x2, y2, ...]
        shared = []
        data = msg.data
        
        if len(data) % 2 != 0:
            self.get_logger().warn('Coordination frontiers array has odd number of elements, ignoring')
            return
        
        for i in range(0, len(data), 2):
            x = data[i]
            y = data[i + 1]
            shared.append((x, y))
        
        self.shared_frontiers = shared
        self.get_logger().debug(f'Received {len(shared)} shared frontiers from other robots')

    def odom_callback(self, msg: Odometry):
        """Update robot position from odometry."""
        had_position = self.robot_position is not None
        self.robot_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        
        # Extract yaw from quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # If this is the first odometry we've received, immediately try to select a goal
        # This ensures robots start moving as soon as odometry is available
        if not had_position and self.robot_position is not None:
            self.get_logger().info('First odometry received, triggering immediate goal selection')
            # Trigger goal selection immediately (don't wait for timer)
            self.select_goal_callback()

    def occupancy_grid_callback(self, msg: OccupancyGrid):
        """Update occupancy grid for goal validation."""
        self.occupancy_grid = msg
        # Count cells for debugging

    def territory_callback(self, msg: TerritoryAssignment):
        """Update current territory assignment."""
        if CUSTOM_MSGS_AVAILABLE:
            self.current_territory = msg
            self.publish_mission_status()

    def verification_goal_callback(self, msg: PoseStamped):
        """Handle verification goal - set status to VERIFYING and publish goal."""
        self.verification_goal = msg
        self.current_mission_status = 2  # VERIFYING
        self.publish_mission_status()
        
        # Publish verification goal as exploration goal
        self.goal_publisher.publish(msg)
        self.get_logger().info(
            f'Received verification goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )

    def publish_mission_status(self):
        """Publish current mission status."""
        if not CUSTOM_MSGS_AVAILABLE:
            return

        status = MissionStatus()
        status.header = Header()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = f'{self.robot_id}/odom'
        status.robot_id = self.robot_id
        status.status = self.current_mission_status
        status.object_found = self.object_found
        status.status_time = self.get_clock().now().to_msg()

        if self.object_found and self.object_pose:
            status.object_pose = self.object_pose
        else:
            # Create empty pose
            status.object_pose = PoseStamped()
            status.object_pose.header = status.header

        # Set current task description
        status_names = {
            0: "EXPLORING",
            1: "NAVIGATING",
            2: "VERIFYING",
            3: "RETURNING",
            4: "COMPLETE"
        }
        status.current_task = status_names.get(self.current_mission_status, "UNKNOWN")

        self.mission_status_publisher.publish(status)

    def select_goal_callback(self):
        """Periodic callback to select and publish goal."""
        if self.robot_position is None:
            return
        
        if not self.enable_autonomous_exploration:
            return  # Autonomous exploration disabled, skip goal selection
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Merge local and shared frontiers
        all_frontiers = list(self.current_frontiers)  # Start with local
        all_frontiers.extend(self.shared_frontiers)   # Add shared

        # Remove duplicates (frontiers within 0.5m of each other)
        unique_frontiers = []
        for fx, fy in all_frontiers:
            is_duplicate = False
            for ux, uy in unique_frontiers:
                if math.sqrt((fx - ux)**2 + (fy - uy)**2) < 0.5:
                    is_duplicate = True
                    break
            if not is_duplicate:
                unique_frontiers.append((fx, fy))

        # Log when using shared frontiers
        if len(self.shared_frontiers) > 0:
            self.get_logger().info(f'Using {len(self.shared_frontiers)} shared frontiers from other robots')
        
        # If no frontiers and we haven't published initial goal, publish one to start exploration
        if len(unique_frontiers) == 0 and not self.initial_goal_published:
            initial_goal = self.generate_initial_exploration_goal()
            if initial_goal is not None:
                # Validate initial goal before publishing
                if not self.is_goal_valid(initial_goal):
                    # Try to find nearby free cell
                    valid_goal = self.find_nearby_free_cell(initial_goal)
                    if valid_goal is not None:
                        initial_goal = valid_goal
                
                self.publish_goal(initial_goal)
                self.current_goal = self.create_goal_message(initial_goal)
                self.goal_set_time = current_time
                self.initial_goal_published = True
                self.initial_exploration_phase = 1
                # CRITICAL: Log at ERROR level for visibility
                self.get_logger().error(
                    f'[INITIAL_GOAL_PUBLISHED] No frontiers, publishing initial exploration goal phase {self.initial_exploration_phase}: '
                    f'({initial_goal[0]:.2f}, {initial_goal[1]:.2f})'
                )
                return
        
        # If still no frontiers but we have a current goal, check if we should keep it or pick new initial goal
        if len(unique_frontiers) == 0:
            if self.current_goal is not None and self.goal_set_time is not None:
                # Check if we've timed out on current goal
                goal_age = current_time - self.goal_set_time
                if goal_age > self.goal_timeout and self.initial_exploration_phase < self.initial_exploration_phases:
                    # Timeout on initial goal - generate next phase goal
                    self.initial_exploration_phase += 1
                    initial_goal = self.generate_initial_exploration_goal()
                    if initial_goal is not None:
                        # Validate initial goal before publishing
                        if not self.is_goal_valid(initial_goal):
                            # Try to find nearby free cell
                            valid_goal = self.find_nearby_free_cell(initial_goal)
                            if valid_goal is not None:
                                initial_goal = valid_goal
                        
                        self.publish_goal(initial_goal)
                        self.current_goal = self.create_goal_message(initial_goal)
                        self.goal_set_time = current_time
                        self.get_logger().error(
                            f'[INITIAL_GOAL_TIMEOUT] Goal timeout, publishing initial exploration goal phase {self.initial_exploration_phase}: '
                            f'({initial_goal[0]:.2f}, {initial_goal[1]:.2f})'
                        )
                        return
                
                goal_pos = (
                    self.current_goal.pose.position.x,
                    self.current_goal.pose.position.y
                )
                distance_to_goal = self.euclidean_distance(self.robot_position, goal_pos)
                if distance_to_goal < 1.0:  # Close to initial goal, pick new one
                    initial_goal = self.generate_initial_exploration_goal()
                    if initial_goal is not None:
                        # Validate initial goal before publishing
                        if not self.is_goal_valid(initial_goal):
                            # Try to find nearby free cell
                            valid_goal = self.find_nearby_free_cell(initial_goal)
                            if valid_goal is not None:
                                initial_goal = valid_goal
                        
                        self.publish_goal(initial_goal)
                        self.current_goal = self.create_goal_message(initial_goal)
                        self.goal_set_time = current_time
                        return
                else:
                    return  # Keep current goal
            else:
                return  # No frontiers and no goal, wait
        
        # We have frontiers - use frontier-based exploration
        # Check if current goal is still valid
        if self.current_goal is not None and self.goal_set_time is not None:
            time_since_goal = current_time - self.goal_set_time
            if time_since_goal < self.goal_timeout:
                # Goal still valid, check if it's still in free space
                goal_pos = (
                    self.current_goal.pose.position.x,
                    self.current_goal.pose.position.y
                )
                
                # Re-validate goal - if it became occupied, find a new one
                if not self.is_goal_valid(goal_pos):
                    self.get_logger().warn(
                        f'[GOAL_INVALIDATED] Current goal ({goal_pos[0]:.2f}, {goal_pos[1]:.2f}) '
                        f'is no longer in free space. Selecting new goal.'
                    )
                    # Goal became invalid, select a new one below
                else:
                    distance_to_goal = self.euclidean_distance(self.robot_position, goal_pos)
                    if distance_to_goal > 3.0:  # Still far from goal
                        return  # Keep current goal
        
        # Select best frontier from merged frontiers
        best_frontier = self.select_best_frontier(unique_frontiers, self.robot_position)
        
        if best_frontier is None:
            # No valid frontier found, use initial exploration
            initial_goal = self.generate_initial_exploration_goal()
            if initial_goal is not None:
                # Validate initial goal before publishing
                if not self.is_goal_valid(initial_goal):
                    # Try to find nearby free cell
                    valid_goal = self.find_nearby_free_cell(initial_goal)
                    if valid_goal is not None:
                        initial_goal = valid_goal
                
                self.publish_goal(initial_goal)
                self.current_goal = self.create_goal_message(initial_goal)
                self.goal_set_time = current_time
            return
        
        # Check if this frontier is already selected by another robot (simple mutex)
        # Using name-based hashing to assign frontiers
        if self.is_frontier_assigned_by_other_robot(best_frontier):
            self.get_logger().debug('Best frontier appears assigned to another robot, selecting next best')
            # Try second best
            frontiers_without_best = [f for f in unique_frontiers if f != best_frontier]
            if frontiers_without_best:
                best_frontier = self.select_best_frontier(frontiers_without_best, self.robot_position)
            else:
                return
        
        # Validate goal before publishing (check if it's in free space)
        if not self.is_goal_valid(best_frontier):
            # Goal is not valid (occupied or unknown), try to find nearby free cell
            self.get_logger().warn(
                f'[GOAL_VALIDATION_FAILED] Frontier ({best_frontier[0]:.2f}, {best_frontier[1]:.2f}) '
                f'is not in free space. Searching for nearby free cell...'
            )
            valid_goal = self.find_nearby_free_cell(best_frontier, max_search_radius=4.0)
            if valid_goal is None:
                # Couldn't find valid goal nearby, try next best frontier
                self.get_logger().warn(
                    f'[GOAL_VALIDATION_FAILED] No free cell found near frontier '
                    f'({best_frontier[0]:.2f}, {best_frontier[1]:.2f}). Trying next best frontier.'
                )
                # Try second best frontier
                frontiers_without_best = [f for f in unique_frontiers if f != best_frontier]
                if frontiers_without_best:
                    best_frontier = self.select_best_frontier(frontiers_without_best, self.robot_position)
                    if best_frontier is not None and self.is_goal_valid(best_frontier):
                        # Found valid alternative
                        pass
                    elif best_frontier is not None:
                        # Try to find free cell for alternative
                        valid_goal = self.find_nearby_free_cell(best_frontier, max_search_radius=4.0)
                        if valid_goal is not None:
                            best_frontier = valid_goal
                        else:
                            # Still no valid goal, skip
                            return
                    else:
                        return
                else:
                    return
            else:
                best_frontier = valid_goal
                self.get_logger().info(
                    f'[GOAL_ADJUSTED] Adjusted goal from frontier to nearby free cell: '
                    f'({best_frontier[0]:.2f}, {best_frontier[1]:.2f})'
                )
        
        # Final validation check before publishing
        if not self.is_goal_valid(best_frontier):
            # Get cell state for logging
            cell_state = self.get_cell_state(best_frontier)
            self.get_logger().error(
                f'[GOAL_REJECTED] Goal ({best_frontier[0]:.2f}, {best_frontier[1]:.2f}) '
                f'failed final validation (cell state: {cell_state}). Not publishing.'
            )
            return  # Don't publish invalid goals
        
        # Publish goal
        self.publish_goal(best_frontier)
        self.current_goal = self.create_goal_message(best_frontier)
        self.goal_set_time = current_time

    def select_best_frontier(self, frontiers: List[Tuple[float, float]], robot_pos: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """
        Select best frontier based on distance and information gain.

        Args:
            frontiers: List of (x, y) frontier positions
            robot_pos: (x, y) robot position

        Returns:
            Best frontier (x, y) or None if no valid frontier
        """
        if not frontiers:
            return None
        
        best_frontier = None
        best_score = float('-inf')
        
        for frontier in frontiers:
            # Calculate distance
            distance = self.euclidean_distance(robot_pos, frontier)
            
            # Skip if too close (likely already explored)
            if distance < self.min_frontier_distance:
                continue
            
            # Calculate information gain (simplified: assume all frontiers have same size)
            # In practice, could use frontier cluster size from detector
            information_gain = 1.0  # Normalized
            
            # Normalize distance (inverse, closer is better)
            # Max expected distance in 30m x 30m world: ~42m diagonal
            max_distance = 50.0
            normalized_distance = 1.0 - (distance / max_distance)
            
            # Calculate utility score
            score = (
                self.distance_weight * normalized_distance +
                self.information_gain_weight * information_gain
            )
            
            if score > best_score:
                best_score = score
                best_frontier = frontier
        
        return best_frontier

    def is_frontier_assigned_by_other_robot(self, frontier: Tuple[float, float]) -> bool:
        """
        Check if frontier is likely assigned to another robot using simple name-based hashing.

        Args:
            frontier: (x, y) frontier position

        Returns:
            True if frontier should be assigned to another robot
        """
        # Simple mutex: hash frontier coordinates and robot name
        # Robots with lower hash get priority
        frontier_hash = hash((int(frontier[0] * 10), int(frontier[1] * 10)))
        robot_hash = hash(self.robot_id)
        combined_hash = (frontier_hash + robot_hash) % 5  # 5 robots
        
        # Each robot gets assigned frontiers based on hash
        # This is a simple way to distribute frontiers without explicit coordination
        robot_num = int(self.robot_id.split('_')[1]) if '_' in self.robot_id else 1
        robot_hash_val = robot_num % 5
        
        # If combined hash doesn't match robot's hash, another robot might want it
        # But this is just a heuristic, robots can still compete
        # For now, always return False (no explicit mutex, rely on distance)
        return False

    def create_goal_message(self, frontier: Tuple[float, float]) -> PoseStamped:
        """
        Create PoseStamped message for goal.

        Args:
            frontier: (x, y) frontier position

        Returns:
            geometry_msgs/PoseStamped message
        """
        goal = PoseStamped()
        goal.header = Header()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = f'{self.robot_id}/odom'
        
        goal.pose.position.x = float(frontier[0])
        goal.pose.position.y = float(frontier[1])
        goal.pose.position.z = 0.0
        
        # Orient toward frontier
        if self.robot_position is not None:
            dx = frontier[0] - self.robot_position[0]
            dy = frontier[1] - self.robot_position[1]
            yaw = math.atan2(dy, dx)
            
            # Convert yaw to quaternion
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            goal.pose.orientation.z = qz
            goal.pose.orientation.w = qw
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
        else:
            goal.pose.orientation.w = 1.0
        
        return goal

    def publish_goal(self, frontier: Tuple[float, float]):
        """
        Publish goal for selected frontier.

        Args:
            frontier: (x, y) frontier position
        """
        goal = self.create_goal_message(frontier)
        self.goal_publisher.publish(goal)
        
        # Update mission status to NAVIGATING
        self.current_mission_status = 1  # NAVIGATING
        self.publish_mission_status()
        
        # CRITICAL: Log at ERROR level for visibility
        self.get_logger().error(
            f'[GOAL_PUBLISHED] Published goal: ({frontier[0]:.2f}, {frontier[1]:.2f})'
        )
        self.get_logger().info(
            f'Published goal: ({frontier[0]:.2f}, {frontier[1]:.2f})'
        )

    def generate_initial_exploration_goal(self) -> Optional[Tuple[float, float]]:
        """
        Generate initial exploration goal when no frontiers are available.
        Uses robot ID and exploration phase to distribute initial goals across robots.
        Phases increase distance and change direction to explore more area.

        Returns:
            (x, y) goal position or None if robot position unavailable
        """
        if self.robot_position is None:
            return None
        
        # Extract robot number from robot_id (e.g., "robot_1" -> 1)
        robot_num = 1
        if '_' in self.robot_id:
            try:
                robot_num = int(self.robot_id.split('_')[1])
            except:
                robot_num = 1
        
        # Distribute initial exploration goals in different directions based on robot number and phase
        # Phase 1: Radial distribution around starting position
        # Phase 2+: Expand outward and rotate direction
        base_angle = (robot_num - 1) * (2 * math.pi / 5)  # 5 robots, distribute in circle
        phase_angle = (self.initial_exploration_phase - 1) * (math.pi / 4)  # Rotate by 45° each phase
        angle_offset = base_angle + phase_angle
        
        # Distance increases with phase
        distance = self.initial_exploration_distance * self.initial_exploration_phase
        
        # Calculate goal position at distance in the assigned direction
        goal_x = self.robot_position[0] + distance * math.cos(self.robot_yaw + angle_offset)
        goal_y = self.robot_position[1] + distance * math.sin(self.robot_yaw + angle_offset)
        
        # Clamp to world bounds (roughly -15 to 15)
        goal_x = max(-14.0, min(14.0, goal_x))
        goal_y = max(-14.0, min(14.0, goal_y))
        
        return (goal_x, goal_y)

    def is_goal_valid(self, goal: Tuple[float, float]) -> bool:
        """
        Check if goal position is in free space according to occupancy grid.
        
        Args:
            goal: (x, y) goal position in world coordinates
            
        Returns:
            True if goal is in free space, False if occupied or unknown
        """
        if self.occupancy_grid is None:
            # No occupancy grid yet, assume goal is valid (will be validated by path planner)
            return True
        
        # Convert world coordinates to grid coordinates
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y
        resolution = self.occupancy_grid.info.resolution
        
        grid_x = int((goal[0] - origin_x) / resolution)
        grid_y = int((goal[1] - origin_y) / resolution)
        
        # Check bounds
        width = self.occupancy_grid.info.width
        height = self.occupancy_grid.info.height
        
        if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
            # Out of bounds - treat as invalid
            return False
        
        # Get cell value (0-100, where 0=free, 100=occupied, -1=unknown)
        index = grid_y * width + grid_x
        if index < 0 or index >= len(self.occupancy_grid.data):
            return False
        
        cell_value = self.occupancy_grid.data[index]
        
        # Free space: 0 (or low values < 50 for probabilistic grids)
        # Occupied: 100 (or high values > 50)
        # Unknown: -1
        if cell_value == -1:
            # Unknown - treat as potentially valid for exploration
            return True
        elif cell_value >= 0 and cell_value < 50:
            # Free space
            return True
        else:
            # Occupied
            return False
    
    def get_cell_state(self, goal: Tuple[float, float]) -> str:
        """
        Get the state of a cell in the occupancy grid for logging.
        
        Args:
            goal: (x, y) goal position in world coordinates
            
        Returns:
            String describing the cell state
        """
        if self.occupancy_grid is None:
            return "no_grid"
        
        # Convert world coordinates to grid coordinates
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y
        resolution = self.occupancy_grid.info.resolution
        
        grid_x = int((goal[0] - origin_x) / resolution)
        grid_y = int((goal[1] - origin_y) / resolution)
        
        # Check bounds
        width = self.occupancy_grid.info.width
        height = self.occupancy_grid.info.height
        
        if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
            return "out_of_bounds"
        
        # Get cell value
        index = grid_y * width + grid_x
        if index < 0 or index >= len(self.occupancy_grid.data):
            return "invalid_index"
        
        cell_value = self.occupancy_grid.data[index]
        
        if cell_value == -1:
            return "unknown"
        elif cell_value >= 0 and cell_value < 50:
            return f"free({cell_value})"
        else:
            return f"occupied({cell_value})"
    
    def find_nearby_free_cell(self, goal: Tuple[float, float], max_search_radius: float = 3.0) -> Optional[Tuple[float, float]]:
        """
        Find a nearby free cell if the goal is occupied.
        Searches in expanding circles around the goal.
        
        Args:
            goal: (x, y) original goal position
            max_search_radius: Maximum distance to search from goal (meters)
            
        Returns:
            (x, y) position of nearby free cell, or None if none found
        """
        if self.occupancy_grid is None:
            return None
        
        resolution = self.occupancy_grid.info.resolution
        search_step = resolution * 2  # Search every 2 cells
        
        # Search in expanding circles with more radii for better coverage
        radii = [0.3, 0.5, 0.8, 1.2, 1.8, 2.5, max_search_radius]
        
        for radius in radii:
            num_points = max(8, int(2 * math.pi * radius / search_step))
            
            for i in range(num_points):
                angle = 2 * math.pi * i / num_points
                test_x = goal[0] + radius * math.cos(angle)
                test_y = goal[1] + radius * math.sin(angle)
                
                if self.is_goal_valid((test_x, test_y)):
                    self.get_logger().info(
                        f'[FOUND_FREE_CELL] Found free cell at ({test_x:.2f}, {test_y:.2f}) '
                        f'distance {radius:.2f}m from invalid goal ({goal[0]:.2f}, {goal[1]:.2f})'
                    )
                    return (test_x, test_y)
        
        self.get_logger().warn(
            f'[NO_FREE_CELL_FOUND] Could not find free cell within {max_search_radius}m '
            f'of goal ({goal[0]:.2f}, {goal[1]:.2f})'
        )
        return None

    @staticmethod
    def euclidean_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """Calculate Euclidean distance between two points."""
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return math.sqrt(dx * dx + dy * dy)


def main(args=None):
    """Main entry point for goal selector node."""
    rclpy.init(args=args)

    node = GoalSelectorNode(robot_id=None)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

