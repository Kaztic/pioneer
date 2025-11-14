#!/usr/bin/env python3
"""
ROS2 Path Planner Node

Subscribes to:
- /robot_i/odom: Current robot pose (nav_msgs/Odometry)
- /robot_i/goal: Goal pose (geometry_msgs/PoseStamped)

Publishes:
- /robot_i/global_path: Planned path (nav_msgs/Path)

Implements A* path planning on a 2D occupancy grid.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header, Float32, String, Float32MultiArray
import math
import time
import json
from typing import Optional

from pioneer_path_planner.a_star_planner import AStarPlanner, CellState


class PathPlannerNode(Node):
    """
    Path planner node for a single robot.
    Should be instantiated once per robot (with namespace).
    """

    def __init__(self, robot_id: Optional[str] = None):
        """
        Initialize path planner node.

        Args:
            robot_id: Robot identifier (e.g., "robot_1", "robot_2"). 
                     If None, will be extracted from node namespace.
        """
        # If robot_id not provided, we'll get it from namespace after initialization
        node_name = 'path_planner_node'
        
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
                # Example: /robot_1/path_planner_node or /path_planner_node
                parts = [p for p in full_name.split('/') if p]  # Remove empty strings
                if len(parts) >= 2:
                    # First part is namespace, second is node name
                    self.robot_id = parts[0]
                else:
                    self.robot_id = "robot_1"  # Default

        # Get parameters
        self.declare_parameter('grid_resolution', 0.2)  # meters
        self.declare_parameter('inflation_radius', 0.5)  # meters
        self.declare_parameter('planner_frequency', 5.0)  # Hz (debounced from 10 to 5)
        self.declare_parameter('replan_debounce_interval', 0.5)  # seconds - minimum time between replans
        self.declare_parameter('world_bounds_x_min', -15.0)
        self.declare_parameter('world_bounds_x_max', 15.0)
        self.declare_parameter('world_bounds_y_min', -15.0)
        self.declare_parameter('world_bounds_y_max', 15.0)
        self.declare_parameter('allow_unknown', True)  # Allow planning through unknown cells

        grid_resolution = self.get_parameter('grid_resolution').get_parameter_value().double_value
        inflation_radius = self.get_parameter('inflation_radius').get_parameter_value().double_value
        allow_unknown = self.get_parameter('allow_unknown').get_parameter_value().bool_value
        world_bounds = (
            self.get_parameter('world_bounds_x_min').get_parameter_value().double_value,
            self.get_parameter('world_bounds_x_max').get_parameter_value().double_value,
            self.get_parameter('world_bounds_y_min').get_parameter_value().double_value,
            self.get_parameter('world_bounds_y_max').get_parameter_value().double_value,
        )

        # Initialize A* planner
        self.planner = AStarPlanner(
            grid_resolution=grid_resolution,
            inflation_radius=inflation_radius,
            world_bounds=world_bounds,
            allow_unknown=allow_unknown
        )

        # Current robot pose (from odometry)
        self.current_pose: Optional[Pose] = None
        self.current_position: Optional[tuple] = None  # (x, y) in world coordinates

        # Current goal (from goal topic)
        self.current_goal: Optional[PoseStamped] = None
        
        # Track last published goal to avoid unnecessary replanning
        self.last_published_goal: Optional[tuple] = None
        self.goal_tolerance_for_replan = 0.1  # meters - only replan if goal moved this much
        
        # Debouncing: track last replan time
        self.last_replan_time: float = 0.0
        self.replan_debounce_interval = self.get_parameter('replan_debounce_interval').get_parameter_value().double_value
        self.first_plan_done: bool = False  # Track if we've done at least one plan
        
        # Diagnostics: track planning metrics
        self.plan_count: int = 0
        self.total_plan_time: float = 0.0
        self.last_path_length: float = 0.0

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
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.get_logger().info(f'Subscribed to /{robot_id}/odom')

        self.goal_subscription = self.create_subscription(
            PoseStamped,
            'goal',
            self.goal_callback,
            10
        )
        self.get_logger().info(f'Subscribed to /{robot_id}/goal')

        # Subscribe to occupancy map (for LiDAR integration)
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'occupancy_map',
            self.map_callback,
            10
        )
        self.get_logger().info(f'Subscribed to /{robot_id}/occupancy_map')

        # Publisher
        self.path_publisher = self.create_publisher(
            Path,
            'global_path',
            10
        )
        self.get_logger().info(f'Publishing to /{robot_id}/global_path')

        # Path coordination publisher (for collision avoidance)
        self.path_coord_publisher = self.create_publisher(
            Float32MultiArray,
            'global_path_coord',
            10
        )
        self.get_logger().info(f'Publishing to /{robot_id}/global_path_coord')

        # Subscribe to replan requests
        self.replan_subscription = self.create_subscription(
            String,
            'replan_request',
            self.replan_callback,
            10
        )
        self.get_logger().info(f'Subscribed to /{robot_id}/replan_request')
        self.replan_requested = False
        
        # Diagnostics publishers
        self.path_length_publisher = self.create_publisher(
            Float32,
            'path_length',
            10
        )
        self.plan_latency_publisher = self.create_publisher(
            Float32,
            'plan_latency',
            10
        )
        self.diagnostics_publisher = self.create_publisher(
            String,
            'diagnostics',
            10
        )

        # Timer for periodic planning
        planner_freq = self.get_parameter('planner_frequency').get_parameter_value().double_value
        self.planner_timer = self.create_timer(
            1.0 / planner_freq,
            self.plan_path_callback
        )

        self.get_logger().info(
            f'Path planner node initialized for {robot_id}\n'
            f'  Grid resolution: {grid_resolution}m\n'
            f'  Inflation radius: {inflation_radius}m\n'
            f'  World bounds: {world_bounds}\n'
            f'  Planning frequency: {planner_freq}Hz'
        )

    def odom_callback(self, msg: Odometry):
        """Update current robot pose from odometry."""
        self.current_pose = msg.pose.pose
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def goal_callback(self, msg: PoseStamped):
        """Update current goal from goal topic."""
        goal_pos = (msg.pose.position.x, msg.pose.position.y)
        
        # CRITICAL: Log EVERY goal received at ERROR level for visibility
        self.get_logger().error(
            f'[GOAL_RECEIVED] New goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}), '
            f'frame_id={msg.header.frame_id}'
        )
        
        # Check if goal has changed significantly
        if self.last_published_goal is not None:
            goal_distance = math.sqrt(
                (goal_pos[0] - self.last_published_goal[0])**2 +
                (goal_pos[1] - self.last_published_goal[1])**2
            )
            if goal_distance < self.goal_tolerance_for_replan:
                # Goal hasn't changed significantly, don't replan
                self.get_logger().debug(f'Goal unchanged (distance: {goal_distance:.3f}m), skipping replan')
                return
        
        self.current_goal = msg
        self.get_logger().info(
            f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )
        # Trigger immediate planning (will be debounced in plan_path_callback except for first plan)
        self.plan_path_callback()

    def map_callback(self, msg: OccupancyGrid):
        """Update occupancy grid from occupancy map message."""
        self.planner.update_grid_from_occupancy_map(msg)
        self.get_logger().debug(
            f'Updated occupancy grid from map: {msg.info.width}x{msg.info.height} cells'
        )
        # Trigger replanning if we have a goal (will be debounced in plan_path_callback)
        if self.current_goal is not None:
            self.plan_path_callback()

    def plan_path_callback(self):
        """
        Periodic callback to plan and publish path.
        Only plans if both current pose and goal are available.
        Implements debouncing to avoid excessive replanning.
        """
        if self.current_position is None:
            return

        if self.current_goal is None:
            return

        # Debounce: check if enough time has passed since last replan
        # BUT: don't debounce the first plan - allow immediate planning when we get first goal/pose
        current_time = time.time()
        time_since_last_replan = current_time - self.last_replan_time
        if not self.first_plan_done:
            # First plan: allow immediately (no debounce)
            self.get_logger().info('First plan attempt - allowing immediate planning (no debounce)')
        elif time_since_last_replan < self.replan_debounce_interval:
            # Too soon since last replan, skip
            self.get_logger().debug(f'Skipping replan - only {time_since_last_replan:.2f}s since last plan (debounce: {self.replan_debounce_interval}s)')
            return

        # Extract goal position
        goal_pos = (
            self.current_goal.pose.position.x,
            self.current_goal.pose.position.y
        )
        
        # Check if goal has changed significantly (for periodic replanning)
        if self.last_published_goal is not None:
            goal_distance = math.sqrt(
                (goal_pos[0] - self.last_published_goal[0])**2 +
                (goal_pos[1] - self.last_published_goal[1])**2
            )
            if goal_distance < self.goal_tolerance_for_replan:
                # Goal hasn't changed, skip replanning
                return

        # Measure planning time for diagnostics
        plan_start_time = time.time()
        
        # Plan path
        path_world = self.planner.plan_path(
            start_world=self.current_position,
            goal_world=goal_pos
        )

        plan_end_time = time.time()
        plan_latency = (plan_end_time - plan_start_time) * 1000.0  # Convert to milliseconds

        if path_world is None:
            # CRITICAL: Log failure with reason at ERROR level for visibility
            failure_reason = getattr(self.planner, 'last_failure_reason', 'Unknown reason')
            self.get_logger().error(
                f'[PATH_PLANNING_FAILED] No path found from '
                f'({self.current_position[0]:.2f}, {self.current_position[1]:.2f}) to '
                f'({goal_pos[0]:.2f}, {goal_pos[1]:.2f}). Reason: {failure_reason}'
            )
            return

        # Calculate path length for diagnostics
        path_length = 0.0
        for i in range(len(path_world) - 1):
            dx = path_world[i + 1][0] - path_world[i][0]
            dy = path_world[i + 1][1] - path_world[i][1]
            path_length += math.sqrt(dx * dx + dy * dy)

        # Create and publish Path message
        path_msg = self.create_path_message(path_world)
        self.path_publisher.publish(path_msg)

        # Publish path for coordination (collision avoidance)
        self.publish_path_coordination(path_msg)
        
        # CRITICAL: Log successful path publication at ERROR level for visibility
        self.get_logger().error(
            f'[PATH_PUBLISHED] Published path: {len(path_world)} waypoints, {path_length:.2f}m, '
            f'{plan_latency:.1f}ms'
        )
        
        # Update last published goal and replan time
        self.last_published_goal = goal_pos
        self.last_replan_time = current_time
        self.first_plan_done = True  # Mark that we've done at least one plan
        
        # Update diagnostics
        self.plan_count += 1
        self.total_plan_time += plan_latency
        self.last_path_length = path_length

        # Publish diagnostics
        path_length_msg = Float32()
        path_length_msg.data = path_length
        self.path_length_publisher.publish(path_length_msg)

        plan_latency_msg = Float32()
        plan_latency_msg.data = plan_latency
        self.plan_latency_publisher.publish(plan_latency_msg)

        # Publish diagnostic string (averaged metrics)
        if self.plan_count > 0:
            avg_plan_time = self.total_plan_time / self.plan_count
            diagnostics_msg = String()
            diagnostics_msg.data = (
                f'Plan #{self.plan_count}: {len(path_world)} waypoints, '
                f'length={path_length:.2f}m, latency={plan_latency:.1f}ms, '
                f'avg_latency={avg_plan_time:.1f}ms'
            )
            self.diagnostics_publisher.publish(diagnostics_msg)

        self.get_logger().info(
            f'Path planned: {len(path_world)} waypoints, {path_length:.2f}m, '
            f'{plan_latency:.1f}ms from ({self.current_position[0]:.2f}, {self.current_position[1]:.2f}) '
            f'to ({goal_pos[0]:.2f}, {goal_pos[1]:.2f})'
        )

    def create_path_message(self, waypoints: list) -> Path:
        """
        Convert list of waypoints to nav_msgs/Path message.

        Args:
            waypoints: List of (x, y) tuples in world coordinates

        Returns:
            nav_msgs/Path message
        """
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = f'{self.robot_id}/odom'

        for waypoint in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = waypoint[0]
            pose_stamped.pose.position.y = waypoint[1]
            pose_stamped.pose.position.z = 0.0

            # Set orientation to face direction of travel (except last point)
            if waypoints.index(waypoint) < len(waypoints) - 1:
                next_waypoint = waypoints[waypoints.index(waypoint) + 1]
                dx = next_waypoint[0] - waypoint[0]
                dy = next_waypoint[1] - waypoint[1]
                yaw = math.atan2(dy, dx)
            else:
                # Last waypoint: use previous orientation
                if len(waypoints) > 1:
                    prev_waypoint = waypoints[waypoints.index(waypoint) - 1]
                    dx = waypoint[0] - prev_waypoint[0]
                    dy = waypoint[1] - prev_waypoint[1]
                    yaw = math.atan2(dy, dx)
                else:
                    yaw = 0.0

            # Convert yaw to quaternion
            qx, qy, qz, qw = self.yaw_to_quaternion(yaw)
            pose_stamped.pose.orientation.x = qx
            pose_stamped.pose.orientation.y = qy
            pose_stamped.pose.orientation.z = qz
            pose_stamped.pose.orientation.w = qw

            path_msg.poses.append(pose_stamped)

        return path_msg

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> tuple:
        """Convert yaw angle to quaternion."""
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qx, qy, qz, qw

    def publish_path_coordination(self, path_msg: Path):
        """Publish path waypoints for coordination via MQTT."""
        coord_msg = Float32MultiArray()
        
        # Serialize path waypoints: [robot_id_hash, x1, y1, x2, y2, ...]
        # Use robot_id hash for identification
        robot_id_hash = hash(self.robot_id) % 10000
        coord_msg.data.append(float(robot_id_hash))
        
        for pose in path_msg.poses:
            coord_msg.data.append(pose.pose.position.x)
            coord_msg.data.append(pose.pose.position.y)
        
        self.path_coord_publisher.publish(coord_msg)

    def replan_callback(self, msg: String):
        """Handle replan request from collision checker."""
        try:
            request_data = json.loads(msg.data)
            if request_data.get('robot_id') == self.robot_id:
                self.replan_requested = True
                self.get_logger().info('Replan requested by collision checker')
                # Clear current path to trigger replanning
                self.current_goal = None
        except Exception as e:
            self.get_logger().error(f'Error processing replan request: {e}')


def main(args=None):
    """Main entry point for path planner node."""
    rclpy.init(args=args)

    # Let ROS2 handle namespace via --ros-args -r __ns:=/robot_1
    # The node will extract robot_id from its namespace
    node = PathPlannerNode(robot_id=None)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

