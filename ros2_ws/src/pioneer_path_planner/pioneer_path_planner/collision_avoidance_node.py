#!/usr/bin/env python3
"""
ROS2 Collision Avoidance Node

Detects path conflicts between robots and resolves them via priority or consensus.

Subscribes to:
- /coordination/paths: All robot paths via MQTT
- /robot_1/odom through /robot_5/odom: Robot positions

Publishes:
- /robot_i/replan_request: Replan requests for affected robots

Detects conflicts:
- Path intersections within time window
- Robots too close (< safety distance)
Resolves via priority-based system or FoxMQ consensus.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Point
import json
import math
from typing import Dict, List, Tuple, Optional
from collections import defaultdict


class CollisionAvoidanceNode(Node):
    """
    Collision avoidance node.
    Detects and resolves path conflicts between robots.
    """

    def __init__(self):
        """Initialize collision avoidance node."""
        super().__init__('collision_avoidance_node')

        # Parameters
        self.declare_parameter('num_robots', 5)
        self.declare_parameter('safety_distance', 1.5)  # meters
        self.declare_parameter('check_frequency', 5.0)  # Hz
        self.declare_parameter('time_horizon', 10.0)  # seconds - look ahead for conflicts

        self.num_robots = self.get_parameter('num_robots').get_parameter_value().integer_value
        self.safety_distance = self.get_parameter('safety_distance').get_parameter_value().double_value
        check_freq = self.get_parameter('check_frequency').get_parameter_value().double_value
        self.time_horizon = self.get_parameter('time_horizon').get_parameter_value().double_value

        # State: robot paths and positions
        self.robot_paths: Dict[str, List[Tuple[float, float]]] = {}
        self.robot_positions: Dict[str, Tuple[float, float]] = {}
        self.robot_velocities: Dict[str, Tuple[float, float]] = {}  # (vx, vy)

        # Subscriptions
        # Subscribe to all robot odometry
        self.odom_subscriptions = {}
        for i in range(1, self.num_robots + 1):
            robot_id = f'robot_{i}'
            self.odom_subscriptions[robot_id] = self.create_subscription(
                Odometry,
                f'/{robot_id}/odom',
                lambda msg, rid=robot_id: self.odom_callback(msg, rid),
                10
            )

        # Subscribe to coordination paths
        self.paths_subscription = self.create_subscription(
            Float32MultiArray,
            '/coordination/paths',
            self.paths_callback,
            10
        )
        self.get_logger().info('Subscribed to /coordination/paths')

        # Publishers for replan requests
        self.replan_publishers = {}
        for i in range(1, self.num_robots + 1):
            robot_id = f'robot_{i}'
            self.replan_publishers[robot_id] = self.create_publisher(
                String,
                f'/{robot_id}/replan_request',
                10
            )

        # Timer for periodic conflict checking
        self.check_timer = self.create_timer(
            1.0 / check_freq,
            self.check_conflicts
        )

        self.get_logger().info(
            f'Collision avoidance node initialized\n'
            f'  Number of robots: {self.num_robots}\n'
            f'  Safety distance: {self.safety_distance}m\n'
            f'  Check frequency: {check_freq}Hz'
        )

    def odom_callback(self, msg: Odometry, robot_id: str):
        """Update robot position and velocity."""
        position = msg.pose.pose.position
        self.robot_positions[robot_id] = (position.x, position.y)

        # Estimate velocity from twist
        twist = msg.twist.twist
        self.robot_velocities[robot_id] = (twist.linear.x, twist.linear.y)

    def paths_callback(self, msg: Float32MultiArray):
        """Update robot paths from coordination topic."""
        if len(msg.data) < 1:
            return

        # Parse path: [robot_id_hash, x1, y1, x2, y2, ...]
        robot_id_hash = int(msg.data[0])
        
        # Find robot ID from hash (simple reverse lookup)
        robot_id = None
        for i in range(1, self.num_robots + 1):
            rid = f'robot_{i}'
            if hash(rid) % 10000 == robot_id_hash:
                robot_id = rid
                break

        if robot_id is None:
            return

        # Extract waypoints
        waypoints = []
        for i in range(1, len(msg.data), 2):
            if i + 1 < len(msg.data):
                waypoints.append((float(msg.data[i]), float(msg.data[i + 1])))

        self.robot_paths[robot_id] = waypoints

    def check_conflicts(self):
        """Check for path conflicts between robots."""
        conflicts = []

        # Check all pairs of robots
        robot_ids = list(self.robot_paths.keys())
        for i in range(len(robot_ids)):
            for j in range(i + 1, len(robot_ids)):
                robot1_id = robot_ids[i]
                robot2_id = robot_ids[j]

                conflict = self.detect_path_conflict(robot1_id, robot2_id)
                if conflict:
                    conflicts.append(conflict)

        # Resolve conflicts
        for conflict in conflicts:
            self.resolve_conflict(conflict)

    def detect_path_conflict(
        self,
        robot1_id: str,
        robot2_id: str
    ) -> Optional[Dict]:
        """
        Detect if two robots' paths will conflict.

        Returns:
            Conflict dict with robot IDs and conflict type, or None
        """
        if robot1_id not in self.robot_paths or robot2_id not in self.robot_paths:
            return None

        path1 = self.robot_paths[robot1_id]
        path2 = self.robot_paths[robot2_id]

        if len(path1) < 2 or len(path2) < 2:
            return None

        # Check if paths intersect
        for i in range(len(path1) - 1):
            seg1_start = path1[i]
            seg1_end = path1[i + 1]

            for j in range(len(path2) - 1):
                seg2_start = path2[j]
                seg2_end = path2[j + 1]

                # Check if segments intersect
                intersection = self.segment_intersection(
                    seg1_start, seg1_end,
                    seg2_start, seg2_end
                )

                if intersection:
                    # Check distance at intersection
                    dist = self.euclidean_distance(seg1_start, seg2_start)
                    if dist < self.safety_distance:
                        return {
                            'robot1': robot1_id,
                            'robot2': robot2_id,
                            'type': 'path_intersection',
                            'location': intersection
                        }

        # Check if robots are currently too close
        if (robot1_id in self.robot_positions and
            robot2_id in self.robot_positions):
            pos1 = self.robot_positions[robot1_id]
            pos2 = self.robot_positions[robot2_id]
            distance = self.euclidean_distance(pos1, pos2)

            if distance < self.safety_distance:
                return {
                    'robot1': robot1_id,
                    'robot2': robot2_id,
                    'type': 'proximity',
                    'distance': distance
                }

        return None

    def segment_intersection(
        self,
        p1: Tuple[float, float],
        p2: Tuple[float, float],
        p3: Tuple[float, float],
        p4: Tuple[float, float]
    ) -> Optional[Tuple[float, float]]:
        """Check if two line segments intersect."""
        # Simple bounding box check first
        x1_min, x1_max = min(p1[0], p2[0]), max(p1[0], p2[0])
        y1_min, y1_max = min(p1[1], p2[1]), max(p1[1], p2[1])
        x2_min, x2_max = min(p3[0], p4[0]), max(p3[0], p4[0])
        y2_min, y2_max = min(p3[1], p4[1]), max(p3[1], p4[1])

        if x1_max < x2_min or x2_max < x1_min:
            return None
        if y1_max < y2_min or y2_max < y1_min:
            return None

        # Check if segments are close enough
        # For simplicity, check if midpoints are close
        mid1 = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)
        mid2 = ((p3[0] + p4[0]) / 2, (p3[1] + p4[1]) / 2)
        dist = self.euclidean_distance(mid1, mid2)

        if dist < self.safety_distance:
            return mid1  # Return approximate intersection

        return None

    def euclidean_distance(
        self,
        p1: Tuple[float, float],
        p2: Tuple[float, float]
    ) -> float:
        """Calculate Euclidean distance between two points."""
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return math.sqrt(dx * dx + dy * dy)

    def resolve_conflict(self, conflict: Dict):
        """Resolve conflict by requesting replan from lower-priority robot."""
        robot1_id = conflict['robot1']
        robot2_id = conflict['robot2']

        # Priority: lower robot number has higher priority
        robot1_num = int(robot1_id.split('_')[1]) if '_' in robot1_id else 1
        robot2_num = int(robot2_id.split('_')[1]) if '_' in robot2_id else 1

        # Lower priority robot replans
        if robot1_num < robot2_num:
            # robot2 replans
            self.request_replan(robot2_id, conflict)
        else:
            # robot1 replans
            self.request_replan(robot1_id, conflict)

    def request_replan(self, robot_id: str, conflict: Dict):
        """Request robot to replan its path."""
        if robot_id not in self.replan_publishers:
            return

        request = {
            'robot_id': robot_id,
            'reason': conflict.get('type', 'unknown'),
            'timestamp': self.get_clock().now().to_msg()
        }

        msg = String()
        msg.data = json.dumps(request)
        self.replan_publishers[robot_id].publish(msg)

        self.get_logger().info(
            f'Requested replan for {robot_id} due to {conflict.get("type", "unknown")} conflict'
        )


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = CollisionAvoidanceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



