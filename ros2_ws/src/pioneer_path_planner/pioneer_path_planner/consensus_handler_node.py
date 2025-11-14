#!/usr/bin/env python3
"""
ROS2 Consensus Handler Node

Handles consensus voting for critical decisions (object detections, territory assignments).

Subscribes to:
- /robot_i/object_detection_remote: Detections from other robots
- /robot_i/territory_assignment_remote: Territory proposals from coordinator
- /consensus/votes: Votes from other robots (via FoxMQ)

Publishes:
- /consensus/results: Consensus results when threshold reached
- /robot_i/consensus_vote: Votes to be sent via FoxMQ

Implements consensus logic:
- When detection received, initiate consensus vote
- When territory proposal received, initiate consensus vote
- Collect votes from other robots via MQTT
- Require 4/5 consensus for critical decisions
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from builtin_interfaces.msg import Time
import json
import time
from typing import Optional, Dict, List
from collections import defaultdict

# Import custom messages
try:
    from pioneer_msgs.msg import DetectionReport, TerritoryAssignment, ConsensusMessage
    CUSTOM_MSGS_AVAILABLE = True
except ImportError:
    CUSTOM_MSGS_AVAILABLE = False
    print("Warning: Custom messages not available")


class ConsensusHandlerNode(Node):
    """
    Consensus handler node for a single robot.
    Handles consensus voting for critical decisions.
    """

    def __init__(self, robot_id: Optional[str] = None):
        """
        Initialize consensus handler node.

        Args:
            robot_id: Robot identifier (e.g., "robot_1", "robot_2").
                     If None, will be extracted from node namespace.
        """
        node_name = 'consensus_handler_node'
        
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
        self.declare_parameter('num_robots', 5)
        self.declare_parameter('consensus_threshold', 4)  # 4/5 required
        self.declare_parameter('vote_timeout', 10.0)  # seconds

        self.num_robots = self.get_parameter('num_robots').get_parameter_value().integer_value
        self.consensus_threshold = self.get_parameter('consensus_threshold').get_parameter_value().integer_value
        self.vote_timeout = self.get_parameter('vote_timeout').get_parameter_value().double_value

        # State: active consensus requests
        # Key: request_id, Value: {type, data, votes, start_time, initiator}
        self.active_requests: Dict[str, Dict] = {}

        # Subscriptions
        if CUSTOM_MSGS_AVAILABLE:
            self.detection_subscription = self.create_subscription(
                DetectionReport,
                'object_detection_remote',
                self.detection_callback,
                10
            )
            self.get_logger().info(f'Subscribed to /{self.robot_id}/object_detection_remote')

            self.territory_subscription = self.create_subscription(
                TerritoryAssignment,
                'territory_assignment_remote',
                self.territory_callback,
                10
            )
            self.get_logger().info(f'Subscribed to /{self.robot_id}/territory_assignment_remote')

        # Subscribe to votes from other robots (via FoxMQ)
        self.votes_subscription = self.create_subscription(
            String,
            '/consensus/votes',  # Global topic
            self.vote_callback,
            10
        )
        self.get_logger().info('Subscribed to /consensus/votes')

        # Publishers
        self.vote_publisher = self.create_publisher(
            String,
            'consensus_vote',
            10
        )
        self.get_logger().info(f'Publishing to /{self.robot_id}/consensus_vote')

        self.results_publisher = self.create_publisher(
            String,
            '/consensus/results',  # Global topic
            10
        )
        self.get_logger().info('Publishing to /consensus/results')

        # Timer to check for expired requests
        self.check_timer = self.create_timer(1.0, self.check_expired_requests)

        self.get_logger().info(
            f'Consensus handler node initialized for {self.robot_id}\n'
            f'  Consensus threshold: {self.consensus_threshold}/{self.num_robots}\n'
            f'  Vote timeout: {self.vote_timeout}s'
        )

    def detection_callback(self, msg: DetectionReport):
        """Handle detection from another robot - initiate consensus."""
        if not CUSTOM_MSGS_AVAILABLE:
            return

        # Create request ID
        request_id = f"detection_{int(time.time() * 1000)}"

        # Serialize detection data
        detection_data = {
            'robot_id': msg.header.frame_id.split('/')[0] if '/' in msg.header.frame_id else 'unknown',
            'object_pose': {
                'x': msg.object_pose.pose.position.x,
                'y': msg.object_pose.pose.position.y,
                'z': msg.object_pose.pose.position.z,
            },
            'confidence': msg.confidence,
            'object_class': msg.object_class,
        }

        # Create consensus request
        self.create_consensus_request(
            request_id,
            'detection',
            detection_data,
            initiator=self.robot_id
        )

    def territory_callback(self, msg: TerritoryAssignment):
        """Handle territory assignment proposal - initiate consensus."""
        if not CUSTOM_MSGS_AVAILABLE:
            return

        # Create request ID
        request_id = f"territory_{int(time.time() * 1000)}"

        # Serialize territory data
        territory_data = {
            'assigned_robot_id': msg.assigned_robot_id,
            'priority': msg.priority,
            'region': [
                {'x': p.x, 'y': p.y, 'z': p.z}
                for p in msg.assigned_region.points
            ],
        }

        # Create consensus request
        self.create_consensus_request(
            request_id,
            'territory',
            territory_data,
            initiator=self.robot_id
        )

    def create_consensus_request(
        self,
        request_id: str,
        request_type: str,
        data: Dict,
        initiator: str
    ):
        """Create a new consensus request."""
        self.active_requests[request_id] = {
            'type': request_type,
            'data': data,
            'votes': {initiator: True},  # Initiator votes yes
            'start_time': time.time(),
            'initiator': initiator
        }

        # Publish vote
        self.publish_vote(request_id, request_type, data, True)

        self.get_logger().info(
            f'Created consensus request: {request_id} (type: {request_type}, initiator: {initiator})'
        )

    def vote_callback(self, msg: String):
        """Handle vote from another robot."""
        try:
            vote_data = json.loads(msg.data)
            request_id = vote_data.get('request_id')
            robot_id = vote_data.get('robot_id')
            vote_value = vote_data.get('vote', False)
            request_type = vote_data.get('type')
            data = vote_data.get('data', {})

            if request_id not in self.active_requests:
                # New request from another robot - add it
                self.active_requests[request_id] = {
                    'type': request_type,
                    'data': data,
                    'votes': {robot_id: vote_value},
                    'start_time': time.time(),
                    'initiator': robot_id
                }
                self.get_logger().info(
                    f'Received new consensus request: {request_id} from {robot_id}'
                )
            else:
                # Add vote to existing request
                self.active_requests[request_id]['votes'][robot_id] = vote_value
                self.get_logger().info(
                    f'Received vote from {robot_id} for request {request_id}: {vote_value}'
                )

            # Check if consensus reached
            self.check_consensus(request_id)

        except Exception as e:
            self.get_logger().error(f'Error processing vote: {e}')

    def publish_vote(self, request_id: str, request_type: str, data: Dict, vote: bool):
        """Publish vote to consensus topic."""
        vote_msg = {
            'request_id': request_id,
            'robot_id': self.robot_id,
            'type': request_type,
            'data': data,
            'vote': vote,
            'timestamp': time.time()
        }

        msg = String()
        msg.data = json.dumps(vote_msg)
        self.vote_publisher.publish(msg)

    def check_consensus(self, request_id: str):
        """Check if consensus threshold is reached for a request."""
        if request_id not in self.active_requests:
            return

        request = self.active_requests[request_id]
        votes = request['votes']
        
        # Count yes votes
        yes_votes = sum(1 for v in votes.values() if v)
        total_votes = len(votes)

        if yes_votes >= self.consensus_threshold:
            # Consensus reached!
            self.get_logger().info(
                f'Consensus reached for {request_id}: {yes_votes}/{total_votes} votes'
            )

            # Publish result
            result = {
                'request_id': request_id,
                'type': request['type'],
                'data': request['data'],
                'consensus_reached': True,
                'votes': yes_votes,
                'total_votes': total_votes,
                'timestamp': time.time()
            }

            result_msg = String()
            result_msg.data = json.dumps(result)
            self.results_publisher.publish(result_msg)

            # Remove request
            del self.active_requests[request_id]

    def check_expired_requests(self):
        """Check for expired consensus requests."""
        current_time = time.time()
        expired = []

        for request_id, request in self.active_requests.items():
            elapsed = current_time - request['start_time']
            if elapsed > self.vote_timeout:
                expired.append(request_id)

        for request_id in expired:
            self.get_logger().warn(
                f'Consensus request {request_id} expired after {self.vote_timeout}s'
            )
            del self.active_requests[request_id]


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = ConsensusHandlerNode(robot_id=None)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



