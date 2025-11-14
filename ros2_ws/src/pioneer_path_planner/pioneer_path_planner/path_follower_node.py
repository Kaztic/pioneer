#!/usr/bin/env python3
"""
ROS2 Path Follower Node

Subscribes to:
- /robot_i/global_path: Planned path (nav_msgs/Path)
- /robot_i/odom: Current robot pose (nav_msgs/Odometry)

Publishes:
- /robot_i/cmd_vel: Velocity commands (geometry_msgs/Twist)

Implements Pure Pursuit algorithm to follow waypoints along a path.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import math
from typing import Optional, Tuple

# Import custom message
try:
    from pioneer_msgs.msg import MissionStatus
    CUSTOM_MSGS_AVAILABLE = True
except ImportError:
    CUSTOM_MSGS_AVAILABLE = False
    print("Warning: MissionStatus message not available")


class PathFollowerNode(Node):
    """
    Path follower node for a single robot.
    Implements Pure Pursuit algorithm to follow waypoints.
    """

    def __init__(self, robot_id: Optional[str] = None):
        """
        Initialize path follower node.

        Args:
            robot_id: Robot identifier (e.g., "robot_1", "robot_2").
                     If None, will be extracted from node namespace.
        """
        # If robot_id not provided, we'll get it from namespace after initialization
        node_name = 'path_follower_node'
        
        if robot_id:
            super().__init__(node_name, namespace=robot_id)
            self.robot_id = robot_id
        else:
            super().__init__(node_name)
            # Extract robot_id from node namespace
            # Try multiple methods to get namespace
            namespace = self.get_namespace()
            if namespace and namespace != '/':
                self.robot_id = namespace.strip('/') if namespace.startswith('/') else namespace
            else:
                # Use fully qualified name to extract namespace
                full_name = self.get_fully_qualified_name()
                # Format: /namespace/node_name or /node_name
                # Example: /robot_1/path_follower_node or /path_follower_node
                parts = [p for p in full_name.split('/') if p]  # Remove empty strings
                if len(parts) >= 2:
                    # First part is namespace, second is node name
                    self.robot_id = parts[0]
                else:
                    self.robot_id = "robot_1"  # Default

        # Parameters
        self.declare_parameter('lookahead_distance', 0.5)  # meters
        self.declare_parameter('max_linear_speed', 2.0)  # m/s
        self.declare_parameter('min_linear_speed', 0.1)  # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        self.declare_parameter('waypoint_tolerance', 0.3)  # meters
        self.declare_parameter('goal_tolerance', 0.2)  # meters
        self.declare_parameter('control_frequency', 20.0)  # Hz
        self.declare_parameter('path_timeout', 5.0)  # seconds
        self.declare_parameter('max_path_deviation', 2.0)  # meters
        self.declare_parameter('k_p', 1.5)  # Proportional gain for heading control
        self.declare_parameter('replan_blocked_distance', 0.5)  # meters - trigger replan if path blocked
        self.declare_parameter('stop_on_goal', True)  # Stop robot when goal reached

        # Get parameter values
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.min_linear_speed = self.get_parameter('min_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        control_freq = self.get_parameter('control_frequency').get_parameter_value().double_value
        self.path_timeout = self.get_parameter('path_timeout').get_parameter_value().double_value
        self.max_path_deviation = self.get_parameter('max_path_deviation').get_parameter_value().double_value
        self.k_p = self.get_parameter('k_p').get_parameter_value().double_value
        self.replan_blocked_distance = self.get_parameter('replan_blocked_distance').get_parameter_value().double_value
        self.stop_on_goal = self.get_parameter('stop_on_goal').get_parameter_value().bool_value

        # State variables
        self.current_path: Optional[Path] = None
        self.current_waypoint_idx: int = 0
        self.robot_position: Optional[Tuple[float, float]] = None
        self.robot_yaw: float = 0.0
        self.path_active: bool = False
        self.last_path_time: Optional[float] = None
        self.last_odom_time: Optional[float] = None  # Track last odometry update time
        self.current_map: Optional[OccupancyGrid] = None  # For checking path blockage
        self.path_blocked: bool = False  # Track if path is blocked

        # Re-extract robot_id after node is fully initialized
        # (namespace info is only available after full init)
        namespace = self.get_namespace()
        if namespace and namespace != '/':
            self.robot_id = namespace.strip('/') if namespace.startswith('/') else namespace
        else:
            full_name = self.get_fully_qualified_name()
            parts = [p for p in full_name.split('/') if p]
            if len(parts) >= 2:
                self.robot_id = parts[0]
            # If still not found, default
            if not self.robot_id or self.robot_id == "None":
                self.robot_id = "robot_1"
        
        robot_id = self.robot_id  # Use the properly extracted robot_id

        # Subscriptions (use relative names, ROS2 will auto-add namespace)
        self.path_subscription = self.create_subscription(
            Path,
            'global_path',
            self.path_callback,
            10
        )
        self.get_logger().info(f'Subscribed to /{robot_id}/global_path')

        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.get_logger().info(f'Subscribed to /{robot_id}/odom')

        # Subscribe to occupancy map for replan triggers
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'occupancy_map',
            self.map_callback,
            10
        )
        self.get_logger().info(f'Subscribed to /{robot_id}/occupancy_map')

        # Publisher
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        self.get_logger().info(f'Publishing to /{robot_id}/cmd_vel')

        # Mission status publisher
        if CUSTOM_MSGS_AVAILABLE:
            self.mission_status_publisher = self.create_publisher(
                MissionStatus,
                'mission_status',
                10
            )
            self.get_logger().info(f'Publishing to /{robot_id}/mission_status')
        
        # Mission status state
        self.current_mission_status = 1  # NAVIGATING (default when following path)

        # Timer for periodic mission status publishing
        if CUSTOM_MSGS_AVAILABLE:
            self.status_timer = self.create_timer(1.0, self.publish_mission_status)  # 1 Hz

        # Control timer
        self.control_timer = self.create_timer(
            1.0 / control_freq,
            self.control_timer_callback
        )

        self.get_logger().info(
            f'Path follower node initialized for {robot_id}\n'
            f'  Lookahead distance: {self.lookahead_distance}m\n'
            f'  Max linear speed: {self.max_linear_speed}m/s\n'
            f'  Control frequency: {control_freq}Hz\n'
            f'  Waypoint tolerance: {self.waypoint_tolerance}m\n'
            f'  Goal tolerance: {self.goal_tolerance}m'
        )

    def path_callback(self, msg: Path):
        """
        Handle new path received from path planner.

        Args:
            msg: nav_msgs/Path message
        """
        # CRITICAL: Log EVERY path received - use ERROR level to ensure we see it even with default logging
        self.get_logger().error(f'[PATH_RECEIVED] Path callback triggered: {len(msg.poses) if msg.poses else 0} poses, frame_id={msg.header.frame_id}, robot_id={self.robot_id}')
        
        # Validate path - but DON'T return on empty, log it
        if not msg.poses:
            self.get_logger().error('[PATH_RECEIVED] Received empty path - this should not happen!')
            # Don't return - try to activate anyway if we don't have a path
            if not self.path_active:
                self.get_logger().error('[PATH_RECEIVED] No active path but received empty path - this is a bug!')
            return

        # Check frame_id matches (warn but don't reject - frame might be different initially)
        expected_frame = f'{self.robot_id}/odom'
        if msg.header.frame_id != expected_frame:
            self.get_logger().debug(
                f'Path frame_id mismatch: expected {expected_frame}, got {msg.header.frame_id} (proceeding anyway)'
            )

        # Check if this is actually a new/different path
        # Compare goal position AND path length to avoid resetting on identical paths
        is_new_path = True
        if self.current_path is not None and len(self.current_path.poses) > 0 and len(msg.poses) > 0:
            # Compare final waypoint (goal)
            old_goal = self.current_path.poses[-1].pose.position
            new_goal = msg.poses[-1].pose.position
            
            goal_distance = math.sqrt(
                (new_goal.x - old_goal.x)**2 + 
                (new_goal.y - old_goal.y)**2
            )
            
            # Also check path length to catch cases where goal is same but path changed
            old_path_len = len(self.current_path.poses)
            new_path_len = len(msg.poses)
            
            # If goal is the same AND path length is the same AND we have an active path, 
            # it's likely the same path (don't reset)
            # BUT: Always accept if path_active is False (no current path active)
            if goal_distance < 0.1 and old_path_len == new_path_len and self.path_active:
                self.get_logger().debug(f'Duplicate path detected (goal distance: {goal_distance:.3f}m, same length: {old_path_len}), ignoring')
                is_new_path = False

        # CRITICAL FIX: Always accept path if we don't have an active path yet
        # This ensures we start following even if first path arrives before odometry
        if not self.path_active:
            # No active path - always accept this one
            self.current_path = msg
            self.current_waypoint_idx = 0
            self.path_active = True
            now = self.get_clock().now()
            self.last_path_time = now.nanoseconds / 1e9
            
            goal_pos = (msg.poses[-1].pose.position.x, msg.poses[-1].pose.position.y) if msg.poses else (0, 0)
            self.get_logger().info(
                f'FIRST PATH ACTIVATED: {len(msg.poses)} waypoints, '
                f'goal: ({goal_pos[0]:.2f}, {goal_pos[1]:.2f}), '
                f'frame_id={msg.header.frame_id}'
            )
        elif is_new_path:
            # We have an active path AND this is a genuinely new path - update it
            self.current_path = msg
            self.current_waypoint_idx = 0
            self.path_active = True
            now = self.get_clock().now()
            self.last_path_time = now.nanoseconds / 1e9

            goal_pos = (msg.poses[-1].pose.position.x, msg.poses[-1].pose.position.y) if msg.poses else (0, 0)
            self.get_logger().info(
                f'New path received: {len(msg.poses)} waypoints, '
                f'starting at index {self.current_waypoint_idx}, '
                f'goal: ({goal_pos[0]:.2f}, {goal_pos[1]:.2f})'
            )
        else:
            # Duplicate path - log but don't update
            self.get_logger().debug(f'Duplicate path ignored (goal distance < 0.1m, same length: {old_path_len})')

    def odom_callback(self, msg: Odometry):
        """
        Update robot position and orientation from odometry.

        Args:
            msg: nav_msgs/Odometry message
        """
        # Update odometry timestamp
        now = self.get_clock().now()
        self.last_odom_time = now.nanoseconds / 1e9
        
        # Extract position
        self.robot_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

        # Extract yaw from quaternion
        self.robot_yaw = self.quaternion_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

    def map_callback(self, msg: OccupancyGrid):
        """
        Update occupancy map and check if path is blocked.

        Args:
            msg: nav_msgs/OccupancyGrid message
        """
        self.current_map = msg
        
        # Check if path ahead is blocked
        if self.path_active and self.current_path is not None and self.robot_position is not None:
            # Check a few waypoints ahead
            check_distance = self.replan_blocked_distance
            blocked = self.is_path_blocked(check_distance)
            
            if blocked and not self.path_blocked:
                self.path_blocked = True
                self.get_logger().warn(
                    f'Path appears blocked within {check_distance}m. '
                    f'Consider triggering replan from path planner.'
                )
            elif not blocked:
                self.path_blocked = False

    def control_timer_callback(self):
        """
        Main control loop: compute and publish cmd_vel.
        """
        if not self.path_active or self.current_path is None:
            # Log why we're not publishing (but only occasionally to avoid spam)
            if not hasattr(self, '_last_warn_time'):
                self._last_warn_time = 0.0
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_warn_time > 5.0:  # Warn every 5 seconds
                if not self.path_active:
                    # CRITICAL: Log at ERROR level for visibility
                    self.get_logger().error('[PATH_FOLLOWER_WAITING] Path not active - waiting for path from planner')
                elif self.current_path is None:
                    self.get_logger().error('[PATH_FOLLOWER_WAITING] No current path - waiting for path from planner')
                self._last_warn_time = now
            return

        if self.robot_position is None:
            if not hasattr(self, '_odom_warn_time'):
                self._odom_warn_time = 0.0
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._odom_warn_time > 5.0:
                self.get_logger().warn('Robot position unknown - waiting for odometry')
                self._odom_warn_time = now
            return

        # Check for stale odometry (robot might be stuck or odometry not updating)
        # Only stop if odometry hasn't been updated for the timeout period
        now = self.get_clock().now()
        current_time = now.nanoseconds / 1e9
        if self.last_odom_time is not None:
            odom_age = current_time - self.last_odom_time
            if odom_age > self.path_timeout:
                self.get_logger().warn(
                    f'Odometry is stale ({odom_age:.2f}s), robot may be stuck. Stopping.'
                )
                self.publish_stop_command()
                self.path_active = False
                return

        # Check if path is completed
        # Only consider path completed if we've passed all intermediate waypoints
        # AND reached the goal (final waypoint) within goal_tolerance
        # The goal check happens in compute_cmd_vel, so if we get here with
        # waypoint_idx >= len, it means we've passed all waypoints including goal
        # But we should have stopped in compute_cmd_vel when goal was reached.
        # This is a safety check for edge cases.
        if self.current_waypoint_idx >= len(self.current_path.poses):
            # Double-check: are we actually at the goal?
            if self.current_path and len(self.current_path.poses) > 0:
                final_waypoint = self.current_path.poses[-1]
                goal_pos = (final_waypoint.pose.position.x, final_waypoint.pose.position.y)
                distance_to_goal = self.euclidean_distance(self.robot_position, goal_pos)
                
                if distance_to_goal < self.goal_tolerance:
                    self.get_logger().info(
                        f'Path completed: reached goal at ({goal_pos[0]:.2f}, {goal_pos[1]:.2f}), '
                        f'distance: {distance_to_goal:.2f}m'
                    )
                    # Update mission status based on context
                    if self.current_mission_status == 2:  # VERIFYING
                        self.current_mission_status = 0  # EXPLORING after verification
                    self.publish_mission_status()
                    self.publish_stop_command()
                    self.path_active = False
                    return
                else:
                    # Not at goal yet, reset waypoint index to final waypoint
                    self.get_logger().warn(
                        f'Waypoint index out of bounds but not at goal. '
                        f'Distance to goal: {distance_to_goal:.2f}m. Resetting to final waypoint.'
                    )
                    self.current_waypoint_idx = len(self.current_path.poses) - 1
            else:
                self.get_logger().info('Path completed, stopping')
                # Update mission status to EXPLORING (ready for next goal)
                self.current_mission_status = 0  # EXPLORING
                self.publish_mission_status()
                self.publish_stop_command()
                self.path_active = False
                return

        # Compute and publish cmd_vel
        cmd_vel = self.compute_cmd_vel()
        if cmd_vel is not None:
            self.cmd_vel_publisher.publish(cmd_vel)
            # Log first few cmd_vel publications at ERROR level
            if not hasattr(self, '_cmd_vel_publish_count'):
                self._cmd_vel_publish_count = 0
            self._cmd_vel_publish_count += 1
            if self._cmd_vel_publish_count <= 5:
                self.get_logger().error(
                    f'[CMD_VEL_PUBLISHED] Published cmd_vel #{self._cmd_vel_publish_count}: '
                    f'linear.x={cmd_vel.linear.x:.2f}, angular.z={cmd_vel.angular.z:.2f}'
                )

    def compute_cmd_vel(self) -> Optional[Twist]:
        """
        Compute velocity commands using Pure Pursuit algorithm.

        Returns:
            geometry_msgs/Twist command or None if path invalid
        """
        if not self.path_active or self.current_path is None:
            return None

        if self.robot_position is None:
            return None

        if self.current_waypoint_idx >= len(self.current_path.poses):
            return None

        # Find lookahead point on path
        lookahead_point = self.find_lookahead_point()
        if lookahead_point is None:
            return None

        # Calculate desired heading to lookahead point
        dx = lookahead_point[0] - self.robot_position[0]
        dy = lookahead_point[1] - self.robot_position[1]
        desired_yaw = math.atan2(dy, dx)

        # Calculate yaw error (normalized to [-π, π])
        yaw_error = self.normalize_angle(desired_yaw - self.robot_yaw)

        # Calculate distance to goal (final waypoint)
        final_waypoint = self.current_path.poses[-1]
        goal_pos = (final_waypoint.pose.position.x, final_waypoint.pose.position.y)
        distance_to_goal = self.euclidean_distance(self.robot_position, goal_pos)

        # Check if goal reached
        if distance_to_goal < self.goal_tolerance:
            self.get_logger().info(f'Goal reached! Distance: {distance_to_goal:.3f}m')
            if self.stop_on_goal:
                self.publish_stop_command()
                self.path_active = False
            else:
                # Continue to maintain position
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                return twist
            return None

        # Calculate linear velocity (distance-based)
        if distance_to_goal < 1.0:
            # Slow down near goal
            linear_speed = max(self.min_linear_speed, distance_to_goal * self.max_linear_speed)
        else:
            linear_speed = self.max_linear_speed

        linear_speed = max(self.min_linear_speed, min(self.max_linear_speed, linear_speed))

        # Improved angular velocity: combine Pure Pursuit with proportional control
        # Pure Pursuit component
        pure_pursuit_angular = (2.0 * linear_speed * math.sin(yaw_error)) / self.lookahead_distance
        
        # Proportional control component for smoother heading
        proportional_angular = self.k_p * yaw_error
        
        # Combine both (weighted towards Pure Pursuit)
        angular_speed = 0.7 * pure_pursuit_angular + 0.3 * proportional_angular

        # Clamp angular velocity
        angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angular_speed))

        # Create Twist message
        twist = Twist()
        twist.linear.x = linear_speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_speed

        # Check waypoint arrival and advance if needed
        self.check_and_advance_waypoint()

        return twist

    def find_lookahead_point(self) -> Optional[Tuple[float, float]]:
        """
        Find lookahead point on path using Pure Pursuit algorithm.

        Returns:
            (x, y) coordinates of lookahead point, or None if not found
        """
        if not self.path_active or self.current_path is None:
            return None

        if self.robot_position is None:
            return None

        # Start searching from current waypoint index
        robot_x, robot_y = self.robot_position

        # Iterate through path segments starting from current waypoint
        for i in range(self.current_waypoint_idx, len(self.current_path.poses)):
            waypoint = self.current_path.poses[i]
            wp_x = waypoint.pose.position.x
            wp_y = waypoint.pose.position.y

            # Calculate distance from robot to this waypoint
            distance = self.euclidean_distance(self.robot_position, (wp_x, wp_y))

            # If waypoint is beyond lookahead distance, interpolate on previous segment
            if distance >= self.lookahead_distance:
                if i > self.current_waypoint_idx:
                    # Interpolate between previous and current waypoint
                    prev_waypoint = self.current_path.poses[i - 1]
                    prev_x = prev_waypoint.pose.position.x
                    prev_y = prev_waypoint.pose.position.y

                    # Find point on segment that is exactly lookahead_distance away
                    return self.point_on_line_segment(
                        (prev_x, prev_y),
                        (wp_x, wp_y),
                        self.robot_position,
                        self.lookahead_distance
                    )
                else:
                    # First waypoint is beyond lookahead, use it directly
                    return (wp_x, wp_y)

        # If all waypoints are within lookahead distance, use final waypoint
        final_waypoint = self.current_path.poses[-1]
        return (final_waypoint.pose.position.x, final_waypoint.pose.position.y)

    def point_on_line_segment(
        self,
        p1: Tuple[float, float],
        p2: Tuple[float, float],
        robot_pos: Tuple[float, float],
        target_distance: float
    ) -> Tuple[float, float]:
        """
        Find point on line segment that is target_distance from robot.

        Args:
            p1: Start point of line segment
            p2: End point of line segment
            robot_pos: Robot position
            target_distance: Desired distance from robot

        Returns:
            (x, y) coordinates of point on segment
        """
        # Vector from robot to p1 and p2
        dx1 = p1[0] - robot_pos[0]
        dy1 = p1[1] - robot_pos[1]
        dx2 = p2[0] - robot_pos[0]
        dy2 = p2[1] - robot_pos[1]

        # Distance from robot to both points
        dist1 = math.sqrt(dx1 * dx1 + dy1 * dy1)
        dist2 = math.sqrt(dx2 * dx2 + dy2 * dy2)

        # If target distance is between dist1 and dist2, interpolate
        if dist1 <= target_distance <= dist2 or dist2 <= target_distance <= dist1:
            # Line segment vector
            seg_dx = p2[0] - p1[0]
            seg_dy = p2[1] - p1[1]
            seg_length = math.sqrt(seg_dx * seg_dx + seg_dy * seg_dy)

            if seg_length < 0.001:  # Very short segment
                return p2

            # Find parameter t such that distance from robot to point is target_distance
            # Use binary search or analytical solution
            t = 0.5  # Start with midpoint
            for _ in range(10):  # Iterative refinement
                point_x = p1[0] + t * seg_dx
                point_y = p1[1] + t * seg_dy
                dist = self.euclidean_distance(robot_pos, (point_x, point_y))

                if abs(dist - target_distance) < 0.01:
                    return (point_x, point_y)

                # Adjust t based on distance
                if dist < target_distance:
                    t += 0.1
                else:
                    t -= 0.1
                t = max(0.0, min(1.0, t))

            return (point_x, point_y)

        # If target distance is beyond segment, return p2
        return p2

    def check_and_advance_waypoint(self):
        """Check if current waypoint is reached and advance if needed."""
        if not self.path_active or self.current_path is None:
            return

        if self.robot_position is None:
            return

        if self.current_waypoint_idx >= len(self.current_path.poses):
            return

        # CRITICAL: Never advance past the final waypoint (goal)
        # The final waypoint should only be "reached" via goal_tolerance check, not waypoint_tolerance
        if self.current_waypoint_idx >= len(self.current_path.poses) - 1:
            # We're already at the final waypoint (goal), don't advance further
            return

        # Get current target waypoint (not the final one)
        target_waypoint = self.current_path.poses[self.current_waypoint_idx]
        target_pos = (target_waypoint.pose.position.x, target_waypoint.pose.position.y)

        # Calculate distance to waypoint
        distance = self.euclidean_distance(self.robot_position, target_pos)

        # If close enough to intermediate waypoint, advance to next waypoint
        if distance < self.waypoint_tolerance:
            self.current_waypoint_idx += 1
            # Only log if not at final waypoint
            if self.current_waypoint_idx < len(self.current_path.poses) - 1:
                next_wp = self.current_path.poses[self.current_waypoint_idx]
                self.get_logger().debug(
                    f'Waypoint {self.current_waypoint_idx - 1} reached, '
                    f'advancing to waypoint {self.current_waypoint_idx}: '
                    f'({next_wp.pose.position.x:.2f}, {next_wp.pose.position.y:.2f})'
                )
            else:
                # Reached final waypoint (goal), will be checked by goal_tolerance
                self.get_logger().debug(
                    f'Reached final waypoint (goal), checking goal_tolerance...'
                )

    def publish_stop_command(self):
        """Publish zero velocity command to stop robot."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    @staticmethod
    def euclidean_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """Calculate Euclidean distance between two points."""
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return math.sqrt(dx * dx + dy * dy)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        Normalize angle to [-π, π] range.

        Args:
            angle: Angle in radians

        Returns:
            Normalized angle in [-π, π]
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    @staticmethod
    def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
        """
        Convert quaternion to yaw angle (rotation around z-axis).

        Args:
            qx, qy, qz, qw: Quaternion components

        Returns:
            Yaw angle in radians [-π, π]
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw

    def is_path_blocked(self, check_distance: float) -> bool:
        """
        Check if path ahead is blocked by obstacles in occupancy map.

        Args:
            check_distance: Distance ahead to check (meters)

        Returns:
            True if path is blocked, False otherwise
        """
        if self.current_map is None or not self.path_active or self.current_path is None:
            return False

        if self.robot_position is None:
            return False

        # Get current waypoint index
        if self.current_waypoint_idx >= len(self.current_path.poses):
            return False

        # Check waypoints within check_distance
        robot_x, robot_y = self.robot_position
        blocked_count = 0
        checked_points = 0

        # Check current and next few waypoints
        for i in range(self.current_waypoint_idx, min(self.current_waypoint_idx + 5, len(self.current_path.poses))):
            waypoint = self.current_path.poses[i]
            wp_x = waypoint.pose.position.x
            wp_y = waypoint.pose.position.y

            # Check distance from robot
            distance = self.euclidean_distance(self.robot_position, (wp_x, wp_y))
            if distance > check_distance:
                break

            # Check if this point is in an occupied cell
            # Convert world coordinates to grid coordinates
            map_info = self.current_map.info
            grid_x = int((wp_x - map_info.origin.position.x) / map_info.resolution)
            grid_y = int((wp_y - map_info.origin.position.y) / map_info.resolution)

            # Check bounds
            if grid_x < 0 or grid_x >= map_info.width or grid_y < 0 or grid_y >= map_info.height:
                continue

            # Get occupancy value (row-major: data[i] = grid[y * width + x])
            idx = grid_y * map_info.width + grid_x
            if idx < len(self.current_map.data):
                occupancy_value = self.current_map.data[idx]
                # Occupied if value > 50
                if occupancy_value > 50:
                    blocked_count += 1
                checked_points += 1

        # Path is blocked if significant portion is occupied
        if checked_points > 0:
            blocked_ratio = blocked_count / checked_points
            return blocked_ratio > 0.3  # More than 30% of checked path is blocked

        return False

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
        status.object_found = False  # Path follower doesn't detect objects
        status.status_time = self.get_clock().now().to_msg()

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


def main(args=None):
    """Main entry point for path follower node."""
    rclpy.init(args=args)

    # Let ROS2 handle namespace via --ros-args -r __ns:=/robot_1
    # The node will extract robot_id from its namespace
    node = PathFollowerNode(robot_id=None)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

