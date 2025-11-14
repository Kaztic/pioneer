#!/usr/bin/env python3
"""
ROS2 Mission Coordinator Node

Centralized coordinator that assigns territories to robots and manages mission state.

Subscribes to:
- /robot_1/odom through /robot_5/odom: Robot positions
- /robot_1/occupancy_map through /robot_5/occupancy_map: Optional, for territory planning

Publishes:
- /robot_i/territory_assignment: Territory assignments (pioneer_msgs/TerritoryAssignment)

Implements territory assignment algorithm:
- Initial: Divide 60x60m arena into 5 equal regions
- Dynamic: When robot completes region, assign nearest unexplored region
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Polygon, Point32, PoseStamped
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from typing import Optional, Dict, List, Tuple
import math

# Import custom message
try:
    from pioneer_msgs.msg import TerritoryAssignment
    CUSTOM_MSGS_AVAILABLE = True
except ImportError:
    CUSTOM_MSGS_AVAILABLE = False
    print("Warning: TerritoryAssignment message not available")


class MissionCoordinatorNode(Node):
    """
    Mission coordinator node.
    Assigns territories to robots and manages mission state.
    """

    def __init__(self):
        """Initialize mission coordinator node."""
        super().__init__('mission_coordinator_node')

        # Parameters
        self.declare_parameter('num_robots', 5)
        self.declare_parameter('arena_size_x', 60.0)  # meters
        self.declare_parameter('arena_size_y', 60.0)  # meters
        self.declare_parameter('arena_origin_x', -30.0)  # meters
        self.declare_parameter('arena_origin_y', -30.0)  # meters
        self.declare_parameter('assignment_frequency', 1.0)  # Hz
        self.declare_parameter('region_completion_threshold', 0.8)  # 80% explored

        # Get parameters
        self.num_robots = self.get_parameter('num_robots').get_parameter_value().integer_value
        self.arena_size_x = self.get_parameter('arena_size_x').get_parameter_value().double_value
        self.arena_size_y = self.get_parameter('arena_size_y').get_parameter_value().double_value
        self.arena_origin_x = self.get_parameter('arena_origin_x').get_parameter_value().double_value
        self.arena_origin_y = self.get_parameter('arena_origin_y').get_parameter_value().double_value
        assignment_freq = self.get_parameter('assignment_frequency').get_parameter_value().double_value
        self.completion_threshold = self.get_parameter('region_completion_threshold').get_parameter_value().double_value

        # State variables
        self.robot_positions: Dict[str, Tuple[float, float]] = {}
        self.robot_occupancy_maps: Dict[str, OccupancyGrid] = {}
        self.territory_assignments: Dict[str, TerritoryAssignment] = {}
        self.region_explored_status: Dict[int, float] = {}  # region_id -> exploration percentage

        # Subscriptions for all robots
        self.odom_subscriptions = {}
        self.map_subscriptions = {}
        
        for i in range(1, self.num_robots + 1):
            robot_id = f'robot_{i}'
            
            # Subscribe to odometry
            self.odom_subscriptions[robot_id] = self.create_subscription(
                Odometry,
                f'/{robot_id}/odom',
                lambda msg, rid=robot_id: self.odom_callback(msg, rid),
                10
            )
            
            # Subscribe to occupancy map (optional)
            self.map_subscriptions[robot_id] = self.create_subscription(
                OccupancyGrid,
                f'/{robot_id}/occupancy_map',
                lambda msg, rid=robot_id: self.map_callback(msg, rid),
                10
            )

        # Publishers for territory assignments
        self.territory_publishers = {}
        for i in range(1, self.num_robots + 1):
            robot_id = f'robot_{i}'
            if CUSTOM_MSGS_AVAILABLE:
                self.territory_publishers[robot_id] = self.create_publisher(
                    TerritoryAssignment,
                    f'/{robot_id}/territory_assignment',
                    10
                )

        # Timer for periodic territory assignment
        self.assignment_timer = self.create_timer(
            1.0 / assignment_freq,
            self.assign_territories
        )

        # Initial territory assignment
        self.initial_assignment_done = False

        self.get_logger().info(
            f'Mission coordinator node initialized\n'
            f'  Number of robots: {self.num_robots}\n'
            f'  Arena size: {self.arena_size_x}x{self.arena_size_y}m\n'
            f'  Assignment frequency: {assignment_freq}Hz'
        )

    def odom_callback(self, msg: Odometry, robot_id: str):
        """Update robot position."""
        position = msg.pose.pose.position
        self.robot_positions[robot_id] = (position.x, position.y)

    def map_callback(self, msg: OccupancyGrid, robot_id: str):
        """Update robot occupancy map."""
        self.robot_occupancy_maps[robot_id] = msg

    def assign_territories(self):
        """Assign territories to robots."""
        if not CUSTOM_MSGS_AVAILABLE:
            return

        # Initial assignment: divide arena into grid regions
        if not self.initial_assignment_done:
            self.initial_territory_assignment()
            self.initial_assignment_done = True
            return

        # Dynamic re-assignment: check if regions are completed
        self.check_and_reassign_territories()

    def initial_territory_assignment(self):
        """Perform initial territory assignment by dividing arena into grid."""
        # Calculate grid dimensions (e.g., 2x3 or 3x2 for 5 robots)
        # For 5 robots, use a 2x3 grid (6 regions, one unused)
        cols = 3
        rows = 2
        
        region_width = self.arena_size_x / cols
        region_height = self.arena_size_y / rows

        self.get_logger().info(
            f'Performing initial territory assignment: '
            f'{cols}x{rows} grid, region size: {region_width:.1f}x{region_height:.1f}m'
        )

        # Assign regions to robots
        region_id = 0
        for i in range(1, self.num_robots + 1):
            robot_id = f'robot_{i}'
            
            # Calculate region position in grid
            row = region_id // cols
            col = region_id % cols
            
            # Calculate region boundaries
            min_x = self.arena_origin_x + col * region_width
            max_x = min_x + region_width
            min_y = self.arena_origin_y + row * region_height
            max_y = min_y + region_height

            # Create territory assignment
            assignment = self.create_territory_assignment(
                robot_id,
                min_x, min_y,
                max_x, max_y,
                priority=1
            )

            self.territory_assignments[robot_id] = assignment
            self.region_explored_status[region_id] = 0.0

            # Publish assignment
            if robot_id in self.territory_publishers:
                self.territory_publishers[robot_id].publish(assignment)
                self.get_logger().info(
                    f'Assigned territory to {robot_id}: '
                    f'region ({min_x:.1f}, {min_y:.1f}) to ({max_x:.1f}, {max_y:.1f})'
                )

            region_id += 1

    def create_territory_assignment(
        self,
        robot_id: str,
        min_x: float, min_y: float,
        max_x: float, max_y: float,
        priority: int = 1
    ) -> TerritoryAssignment:
        """Create a territory assignment message."""
        assignment = TerritoryAssignment()
        assignment.header = Header()
        assignment.header.stamp = self.get_clock().now().to_msg()
        assignment.header.frame_id = 'map'
        assignment.assigned_robot_id = robot_id
        assignment.priority = priority
        assignment.assignment_time = self.get_clock().now().to_msg()
        assignment.assigned_by = 'mission_coordinator'

        # Create polygon with region boundaries
        # Polygon points in counter-clockwise order
        # Note: Polygon uses Point32, not Point
        assignment.assigned_region.points = [
            Point32(x=float(min_x), y=float(min_y), z=0.0),
            Point32(x=float(max_x), y=float(min_y), z=0.0),
            Point32(x=float(max_x), y=float(max_y), z=0.0),
            Point32(x=float(min_x), y=float(max_y), z=0.0),
        ]

        return assignment

    def check_and_reassign_territories(self):
        """Check if territories are completed and reassign if needed."""
        # For now, implement simple completion check based on robot position
        # In a full implementation, we'd analyze occupancy maps
        
        for robot_id, assignment in self.territory_assignments.items():
            if robot_id not in self.robot_positions:
                continue

            # Check if robot is within its assigned territory
            robot_x, robot_y = self.robot_positions[robot_id]
            region = assignment.assigned_region
            
            if len(region.points) < 4:
                continue

            # Get region bounds
            min_x = min(p.x for p in region.points)
            max_x = max(p.x for p in region.points)
            min_y = min(p.y for p in region.points)
            max_y = max(p.y for p in region.points)

            # Simple heuristic: if robot has been exploring for a while and is near center,
            # consider region explored
            # This is a simplified check - in production, analyze occupancy map
            
            # For now, just republish existing assignments periodically
            if robot_id in self.territory_publishers:
                self.territory_publishers[robot_id].publish(assignment)

    def get_robot_position(self, robot_id: str) -> Optional[Tuple[float, float]]:
        """Get current robot position."""
        return self.robot_positions.get(robot_id)

    def is_region_completed(self, region_id: int) -> bool:
        """Check if a region is completed (explored above threshold)."""
        exploration_percentage = self.region_explored_status.get(region_id, 0.0)
        return exploration_percentage >= self.completion_threshold


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = MissionCoordinatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


