#!/usr/bin/env python3
"""
ROS2 Verification Coordinator Node

Coordinates multi-robot object verification when detections are received.

Subscribes to:
- /robot_i/object_detection_remote: Detections from other robots
- /robot_1/odom through /robot_5/odom: Robot positions
- /consensus/results: Consensus results

Publishes:
- /robot_i/verification_goal: Goals for robots to verify detections

When object detection received:
- If confidence > threshold, initiate verification
- Select nearest available robot to verify
- Publish verification goal to selected robot
- Wait for verification result
- Aggregate votes via consensus (4/5 required)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import String, Header
import json
import math
import time
from typing import Dict, List, Tuple, Optional

# Import custom messages
try:
    from pioneer_msgs.msg import DetectionReport
    CUSTOM_MSGS_AVAILABLE = True
except ImportError:
    CUSTOM_MSGS_AVAILABLE = False
    print("Warning: DetectionReport message not available")


class VerificationCoordinatorNode(Node):
    """
    Verification coordinator node.
    Coordinates multi-robot object verification.
    """

    def __init__(self):
        """Initialize verification coordinator node."""
        super().__init__('verification_coordinator_node')

        # Parameters
        self.declare_parameter('num_robots', 5)
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('verification_timeout', 30.0)  # seconds

        self.num_robots = self.get_parameter('num_robots').get_parameter_value().integer_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.verification_timeout = self.get_parameter('verification_timeout').get_parameter_value().double_value

        # State
        self.robot_positions: Dict[str, Tuple[float, float]] = {}
        self.active_verifications: Dict[str, Dict] = {}  # detection_id -> verification info
        self.verification_results: Dict[str, List[bool]] = {}  # detection_id -> [votes]

        # Subscriptions
        # Subscribe to detections from all robots
        self.detection_subscriptions = {}
        for i in range(1, self.num_robots + 1):
            robot_id = f'robot_{i}'
            if CUSTOM_MSGS_AVAILABLE:
                self.detection_subscriptions[robot_id] = self.create_subscription(
                    DetectionReport,
                    f'/{robot_id}/object_detection_remote',
                    lambda msg, rid=robot_id: self.detection_callback(msg, rid),
                    10
                )

        # Subscribe to robot odometry
        self.odom_subscriptions = {}
        for i in range(1, self.num_robots + 1):
            robot_id = f'robot_{i}'
            self.odom_subscriptions[robot_id] = self.create_subscription(
                Odometry,
                f'/{robot_id}/odom',
                lambda msg, rid=robot_id: self.odom_callback(msg, rid),
                10
            )

        # Subscribe to consensus results
        self.consensus_subscription = self.create_subscription(
            String,
            '/consensus/results',
            self.consensus_callback,
            10
        )

        # Publishers for verification goals
        self.verification_goal_publishers = {}
        for i in range(1, self.num_robots + 1):
            robot_id = f'robot_{i}'
            self.verification_goal_publishers[robot_id] = self.create_publisher(
                PoseStamped,
                f'/{robot_id}/verification_goal',
                10
            )

        # Timer to check for expired verifications
        self.check_timer = self.create_timer(1.0, self.check_expired_verifications)

        self.get_logger().info(
            f'Verification coordinator node initialized\n'
            f'  Number of robots: {self.num_robots}\n'
            f'  Confidence threshold: {self.confidence_threshold}\n'
            f'  Verification timeout: {self.verification_timeout}s'
        )

    def odom_callback(self, msg: Odometry, robot_id: str):
        """Update robot position."""
        position = msg.pose.pose.position
        self.robot_positions[robot_id] = (position.x, position.y)

    def detection_callback(self, msg: DetectionReport, robot_id: str):
        """Handle detection from another robot - initiate verification if needed."""
        if not CUSTOM_MSGS_AVAILABLE:
            return

        if msg.confidence < self.confidence_threshold:
            return  # Low confidence, skip verification

        # Create detection ID
        detection_id = f"det_{int(time.time() * 1000)}_{robot_id}"

        # Check if already verifying this detection
        if detection_id in self.active_verifications:
            return

        # Select nearest available robot to verify
        verifier_id = self.select_verifier_robot(robot_id, msg.object_pose.pose.position)

        if verifier_id is None:
            self.get_logger().warn('No available robot for verification')
            return

        # Create verification request
        self.active_verifications[detection_id] = {
            'detector_id': robot_id,
            'verifier_id': verifier_id,
            'object_pose': msg.object_pose,
            'confidence': msg.confidence,
            'start_time': time.time(),
            'votes': [True]  # Detector votes yes
        }
        self.verification_results[detection_id] = [True]

        # Publish verification goal
        self.publish_verification_goal(verifier_id, msg.object_pose)

        self.get_logger().info(
            f'Initiated verification: {detection_id}, verifier: {verifier_id}'
        )

    def select_verifier_robot(
        self,
        detector_id: str,
        object_position: Point
    ) -> Optional[str]:
        """Select nearest available robot to verify detection."""
        if detector_id not in self.robot_positions:
            return None

        object_pos = (object_position.x, object_position.y)
        detector_pos = self.robot_positions[detector_id]

        # Find nearest robot (excluding detector)
        best_robot = None
        best_distance = float('inf')

        for robot_id, robot_pos in self.robot_positions.items():
            if robot_id == detector_id:
                continue

            # Check if robot is already verifying something
            is_busy = any(
                v['verifier_id'] == robot_id
                for v in self.active_verifications.values()
            )

            if is_busy:
                continue

            distance = self.euclidean_distance(robot_pos, object_pos)
            if distance < best_distance:
                best_distance = distance
                best_robot = robot_id

        return best_robot

    def euclidean_distance(
        self,
        p1: Tuple[float, float],
        p2: Tuple[float, float]
    ) -> float:
        """Calculate Euclidean distance."""
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return math.sqrt(dx * dx + dy * dy)

    def publish_verification_goal(self, robot_id: str, object_pose: PoseStamped):
        """Publish verification goal to robot."""
        if robot_id not in self.verification_goal_publishers:
            return

        goal = PoseStamped()
        goal.header = Header()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = f'{robot_id}/odom'
        goal.pose = object_pose.pose

        self.verification_goal_publishers[robot_id].publish(goal)

    def consensus_callback(self, msg: String):
        """Handle consensus result for verification."""
        try:
            result = json.loads(msg.data)
            if result.get('type') != 'detection':
                return

            detection_id = result.get('request_id', '')
            if detection_id in self.active_verifications:
                consensus_reached = result.get('consensus_reached', False)
                votes = result.get('votes', 0)

                self.get_logger().info(
                    f'Verification consensus for {detection_id}: '
                    f'reached={consensus_reached}, votes={votes}'
                )

                # Remove from active verifications
                if detection_id in self.active_verifications:
                    del self.active_verifications[detection_id]

        except Exception as e:
            self.get_logger().error(f'Error processing consensus result: {e}')

    def check_expired_verifications(self):
        """Check for expired verification requests."""
        current_time = time.time()
        expired = []

        for detection_id, verification in self.active_verifications.items():
            elapsed = current_time - verification['start_time']
            if elapsed > self.verification_timeout:
                expired.append(detection_id)

        for detection_id in expired:
            self.get_logger().warn(
                f'Verification {detection_id} expired after {self.verification_timeout}s'
            )
            if detection_id in self.active_verifications:
                del self.active_verifications[detection_id]


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = VerificationCoordinatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



