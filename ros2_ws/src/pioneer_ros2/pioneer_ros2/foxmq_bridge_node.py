#!/usr/bin/env python3
"""
ROS2-FoxMQ Bridge Node

Bridges ROS2 topics to MQTT topics via FoxMQ broker for robot-to-robot communication.

Subscribes to ROS2 topics (outgoing):
- /robot_i/object_detection → MQTT robots/robot_i/detection
- /robot_i/mission_status → MQTT robots/robot_i/status
- /robot_i/territory_assignment → MQTT robots/robot_i/territory
- /robot_i/consensus_vote → MQTT robots/robot_i/consensus_vote
- /robot_i/frontiers → MQTT coordination/frontiers (with robot_id in payload)
- /robot_i/global_path → MQTT coordination/paths (with robot_id in payload)

Publishes to ROS2 topics (incoming):
- MQTT robots/*/detection → /robot_i/object_detection_remote
- MQTT robots/*/status → /robot_i/mission_status_remote
- MQTT robots/*/territory → /robot_i/territory_assignment_remote
- MQTT robots/*/consensus_result → /consensus/results
- MQTT coordination/frontiers → /coordination/frontiers
- MQTT coordination/paths → /coordination/paths
"""

import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
import threading
from typing import Optional, Dict, Any
import time

# ROS2 message imports
from std_msgs.msg import Header, String, Float32MultiArray
from geometry_msgs.msg import PoseStamped, Polygon, Point
from builtin_interfaces.msg import Time

# Custom message imports (if available)
try:
    from pioneer_msgs.msg import DetectionReport, TerritoryAssignment, MissionStatus, ConsensusMessage
    CUSTOM_MSGS_AVAILABLE = True
except ImportError:
    CUSTOM_MSGS_AVAILABLE = False
    print("Warning: Custom messages not available, using generic String messages")


class FoxMQBridgeNode(Node):
    """
    ROS2-MQTT bridge node for robot-to-robot communication via FoxMQ.
    """

    def __init__(self, robot_id: Optional[str] = None):
        """
        Initialize FoxMQ bridge node.

        Args:
            robot_id: Robot identifier (e.g., "robot_1", "robot_2").
                     If None, will be extracted from node namespace.
        """
        node_name = 'foxmq_bridge_node'
        
        if robot_id:
            super().__init__(node_name, namespace=robot_id)
            self.robot_id = robot_id
        else:
            super().__init__(node_name)
            # Extract robot_id from namespace
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

        # Configuration parameters
        self.declare_parameter('mqtt_broker_host', '127.0.0.1')
        self.declare_parameter('mqtt_broker_port', 1883)
        self.declare_parameter('mqtt_username', 'pioneer_robot')
        self.declare_parameter('mqtt_password', '')
        self.declare_parameter('mqtt_client_id', f'{self.robot_id}_bridge')
        self.declare_parameter('enable_object_detection', True)
        self.declare_parameter('enable_mission_status', True)
        self.declare_parameter('enable_territory', True)
        self.declare_parameter('enable_consensus', True)
        self.declare_parameter('enable_coordination', True)

        # Get parameters
        self.mqtt_host = self.get_parameter('mqtt_broker_host').get_parameter_value().string_value
        self.mqtt_port = self.get_parameter('mqtt_broker_port').get_parameter_value().integer_value
        self.mqtt_username = self.get_parameter('mqtt_username').get_parameter_value().string_value
        self.mqtt_password = self.get_parameter('mqtt_password').get_parameter_value().string_value
        self.mqtt_client_id = self.get_parameter('mqtt_client_id').get_parameter_value().string_value

        # MQTT client
        self.mqtt_client: Optional[mqtt.Client] = None
        self.mqtt_connected = False
        self.mqtt_lock = threading.Lock()

        # ROS2 subscriptions (outgoing to MQTT)
        self.ros2_subscriptions = {}

        # ROS2 publishers (incoming from MQTT)
        self.ros2_publishers = {}

        # Setup MQTT client
        self.setup_mqtt_client()

        # Setup ROS2 subscriptions and publishers
        self.setup_ros2_interfaces()

        # Connection timer
        self.connection_timer = self.create_timer(5.0, self.check_mqtt_connection)

        self.get_logger().info(
            f'FoxMQ Bridge Node initialized for {self.robot_id}\n'
            f'  MQTT Broker: {self.mqtt_host}:{self.mqtt_port}\n'
            f'  Client ID: {self.mqtt_client_id}'
        )

    def setup_mqtt_client(self):
        """Initialize and connect MQTT client"""
        self.mqtt_client = mqtt.Client(
            client_id=self.mqtt_client_id,
            protocol=mqtt.MQTTv5
        )

        # Set callbacks
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        self.mqtt_client.on_message = self.on_mqtt_message

        # Set credentials - FoxMQ requires authentication
        # If username is empty, don't set credentials (will fail, but that's expected)
        # Otherwise, use provided credentials
        if self.mqtt_username and self.mqtt_username.strip():
            self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)
        # If no username provided, try connecting without auth (may fail if FoxMQ requires it)

        # Connect in background thread
        self.connect_mqtt()

    def connect_mqtt(self):
        """Connect to MQTT broker"""
        try:
            self.get_logger().info(f'Connecting to MQTT broker at {self.mqtt_host}:{self.mqtt_port}...')
            self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MQTT broker: {e}')

    def on_mqtt_connect(self, client, userdata, flags, rc, properties=None):
        """Callback when MQTT client connects"""
        if rc == 0:
            self.mqtt_connected = True
            self.get_logger().info('Connected to MQTT broker')

            # Subscribe to MQTT topics
            self.subscribe_mqtt_topics()
        else:
            self.mqtt_connected = False
            self.get_logger().error(f'Failed to connect to MQTT broker, return code: {rc}')

    def on_mqtt_disconnect(self, client, userdata, rc, properties=None):
        """Callback when MQTT client disconnects"""
        self.mqtt_connected = False
        self.get_logger().warn('Disconnected from MQTT broker')

    def subscribe_mqtt_topics(self):
        """Subscribe to MQTT topics for incoming messages"""
        # Subscribe to all robot status topics
        for i in range(1, 6):
            robot_id = f'robot_{i}'
            if robot_id != self.robot_id:
                # Status updates from other robots
                topic = f'robots/{robot_id}/status'
                self.mqtt_client.subscribe(topic, qos=1)
                self.get_logger().info(f'Subscribed to MQTT: {topic}')

                # Object detections from other robots
                topic = f'robots/{robot_id}/detection'
                self.mqtt_client.subscribe(topic, qos=1)
                self.get_logger().info(f'Subscribed to MQTT: {topic}')

                # Territory assignments
                topic = f'robots/{robot_id}/territory'
                self.mqtt_client.subscribe(topic, qos=1)

                # Consensus results
                topic = f'robots/{robot_id}/consensus_result'
                self.mqtt_client.subscribe(topic, qos=1)

        # Coordination topics (shared)
        self.mqtt_client.subscribe('coordination/frontiers', qos=1)
        self.get_logger().info('Subscribed to MQTT: coordination/frontiers')
        self.mqtt_client.subscribe('coordination/paths', qos=1)
        self.get_logger().info('Subscribed to MQTT: coordination/paths')
        self.mqtt_client.subscribe('coordination/goals', qos=1)
        self.get_logger().info('Subscribed to MQTT: coordination/goals')

        self.get_logger().info('Subscribed to all MQTT topics')

    def on_mqtt_message(self, client, userdata, msg):
        """Callback when MQTT message is received"""
        try:
            topic = msg.topic
            payload = msg.payload.decode('utf-8')
            data = json.loads(payload)

            self.get_logger().info(f'Received MQTT message on {topic}: {len(payload)} bytes')

            # Route message to appropriate ROS2 topic
            self.route_mqtt_to_ros2(topic, data)

        except Exception as e:
            self.get_logger().error(f'Error processing MQTT message: {e}')

    def route_mqtt_to_ros2(self, mqtt_topic: str, data: Dict[str, Any]):
        """Route MQTT message to ROS2 topic"""
        # Parse topic structure
        parts = mqtt_topic.split('/')
        
        if len(parts) >= 2 and parts[0] == 'robots':
            robot_id = parts[1]
            message_type = parts[2] if len(parts) > 2 else 'status'

            if robot_id == self.robot_id:
                return  # Don't echo our own messages

            # Route based on message type
            if message_type == 'detection':
                self.publish_object_detection_remote(data)
            elif message_type == 'status':
                self.publish_mission_status_remote(data)
            elif message_type == 'territory':
                self.publish_territory_assignment_remote(data)
            elif message_type == 'consensus_result':
                self.publish_consensus_result(data)

        elif parts[0] == 'coordination':
            coord_type = parts[1]
            if coord_type == 'frontiers':
                self.get_logger().info(f'Routing coordination/frontiers message to ROS2')
                self.publish_coordination_frontiers(data)
            elif coord_type == 'paths':
                self.publish_coordination_paths(data)
            elif coord_type == 'goals':
                self.publish_coordination_goals(data)

    def setup_ros2_interfaces(self):
        """Setup ROS2 subscriptions and publishers"""
        # Outgoing subscriptions (ROS2 → MQTT)
        if self.get_parameter('enable_object_detection').get_parameter_value().bool_value:
            if CUSTOM_MSGS_AVAILABLE:
                self.create_subscription(
                    DetectionReport,
                    'object_detection',
                    self.object_detection_callback,
                    10
                )
            else:
                self.create_subscription(
                    String,
                    'object_detection',
                    self.object_detection_callback_string,
                    10
                )

        if self.get_parameter('enable_mission_status').get_parameter_value().bool_value:
            if CUSTOM_MSGS_AVAILABLE:
                self.create_subscription(
                    MissionStatus,
                    'mission_status',
                    self.mission_status_callback,
                    10
                )
            else:
                self.create_subscription(
                    String,
                    'mission_status',
                    self.mission_status_callback_string,
                    10
                )

        if self.get_parameter('enable_territory').get_parameter_value().bool_value:
            if CUSTOM_MSGS_AVAILABLE:
                self.create_subscription(
                    TerritoryAssignment,
                    'territory_assignment',
                    self.territory_assignment_callback,
                    10
                )
            else:
                self.create_subscription(
                    String,
                    'territory_assignment',
                    self.territory_assignment_callback_string,
                    10
                )

        # Frontiers (coordination)
        if self.get_parameter('enable_coordination').get_parameter_value().bool_value:
            self.create_subscription(
                Float32MultiArray,
                'frontiers',
                self.frontiers_callback,
                10
            )

        # Incoming publishers (MQTT → ROS2)
        if CUSTOM_MSGS_AVAILABLE:
            self.ros2_publishers['object_detection_remote'] = self.create_publisher(
                DetectionReport,
                'object_detection_remote',
                10
            )
            self.ros2_publishers['mission_status_remote'] = self.create_publisher(
                MissionStatus,
                'mission_status_remote',
                10
            )
            self.ros2_publishers['territory_assignment_remote'] = self.create_publisher(
                TerritoryAssignment,
                'territory_assignment_remote',
                10
            )
        else:
            self.ros2_publishers['object_detection_remote'] = self.create_publisher(
                String,
                'object_detection_remote',
                10
            )
            self.ros2_publishers['mission_status_remote'] = self.create_publisher(
                String,
                'mission_status_remote',
                10
            )
            self.ros2_publishers['territory_assignment_remote'] = self.create_publisher(
                String,
                'territory_assignment_remote',
                10
            )

        # Coordination topics
        self.ros2_publishers['coordination_frontiers'] = self.create_publisher(
            Float32MultiArray,
            '/coordination/frontiers',
            10
        )

        self.ros2_publishers['consensus_result'] = self.create_publisher(
            String,
            '/consensus/results',
            10
        )

    # ROS2 → MQTT callbacks
    def object_detection_callback(self, msg: DetectionReport):
        """Publish object detection to MQTT"""
        if not self.mqtt_connected:
            return

        data = {
            'robot_id': self.robot_id,
            'timestamp': time.time(),
            'object_pose': {
                'x': msg.object_pose.pose.position.x,
                'y': msg.object_pose.pose.position.y,
                'z': msg.object_pose.pose.position.z,
            },
            'confidence': msg.confidence if hasattr(msg, 'confidence') else 0.0,
        }

        mqtt_topic = f'robots/{self.robot_id}/detection'
        self.publish_mqtt(mqtt_topic, data)

    def object_detection_callback_string(self, msg: String):
        """Publish object detection (String) to MQTT"""
        if not self.mqtt_connected:
            return

        data = {
            'robot_id': self.robot_id,
            'timestamp': time.time(),
            'data': msg.data,
        }

        mqtt_topic = f'robots/{self.robot_id}/detection'
        self.publish_mqtt(mqtt_topic, data)

    def mission_status_callback(self, msg: MissionStatus):
        """Publish mission status to MQTT"""
        if not self.mqtt_connected:
            return

        data = {
            'robot_id': self.robot_id,
            'timestamp': time.time(),
            'status': int(msg.status),
            'object_found': msg.object_found,
            'current_task': msg.current_task if hasattr(msg, 'current_task') else '',
        }

        mqtt_topic = f'robots/{self.robot_id}/status'
        self.publish_mqtt(mqtt_topic, data)

    def mission_status_callback_string(self, msg: String):
        """Publish mission status (String) to MQTT"""
        if not self.mqtt_connected:
            return

        data = {
            'robot_id': self.robot_id,
            'timestamp': time.time(),
            'data': msg.data,
        }

        mqtt_topic = f'robots/{self.robot_id}/status'
        self.publish_mqtt(mqtt_topic, data)

    def territory_assignment_callback(self, msg: TerritoryAssignment):
        """Publish territory assignment to MQTT"""
        if not self.mqtt_connected:
            return

        # Extract polygon points
        points = []
        for point in msg.assigned_region.points:
            points.append({'x': point.x, 'y': point.y, 'z': point.z})

        data = {
            'robot_id': self.robot_id,
            'assigned_robot_id': msg.assigned_robot_id,
            'priority': msg.priority,
            'region': points,
            'timestamp': time.time(),
        }

        mqtt_topic = f'robots/{self.robot_id}/territory'
        self.publish_mqtt(mqtt_topic, data)

    def territory_assignment_callback_string(self, msg: String):
        """Publish territory assignment (String) to MQTT"""
        if not self.mqtt_connected:
            return

        data = {
            'robot_id': self.robot_id,
            'timestamp': time.time(),
            'data': msg.data,
        }

        mqtt_topic = f'robots/{self.robot_id}/territory'
        self.publish_mqtt(mqtt_topic, data)

    def frontiers_callback(self, msg: Float32MultiArray):
        """Publish frontiers to MQTT coordination topic"""
        if not self.mqtt_connected:
            self.get_logger().warn('MQTT not connected, cannot publish frontiers')
            return

        # Extract frontier points
        frontiers = []
        data = msg.data
        self.get_logger().info(f'Received frontiers callback: {len(data)} data points')
        
        if len(data) % 2 == 0:
            for i in range(0, len(data), 2):
                frontiers.append({'x': data[i], 'y': data[i + 1]})

        # Only publish if we have frontiers (avoid spamming with empty messages)
        if len(frontiers) == 0:
            self.get_logger().debug('No frontiers to publish (empty message)')
            return

        mqtt_data = {
            'robot_id': self.robot_id,
            'timestamp': time.time(),
            'frontiers': frontiers,
        }

        mqtt_topic = 'coordination/frontiers'
        self.publish_mqtt(mqtt_topic, mqtt_data)
        self.get_logger().info(f'Published {len(frontiers)} frontiers to MQTT topic {mqtt_topic}')

    # MQTT → ROS2 publishers
    def publish_object_detection_remote(self, data: Dict[str, Any]):
        """Publish object detection from MQTT to ROS2"""
        if 'object_detection_remote' not in self.ros2_publishers:
            return

        if CUSTOM_MSGS_AVAILABLE:
            msg = DetectionReport()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f'{self.robot_id}/odom'
            if 'object_pose' in data:
                msg.object_pose.pose.position.x = data['object_pose']['x']
                msg.object_pose.pose.position.y = data['object_pose']['y']
                msg.object_pose.pose.position.z = data['object_pose']['z']
            if 'confidence' in data:
                msg.confidence = data['confidence']
            self.ros2_publishers['object_detection_remote'].publish(msg)
        else:
            msg = String()
            msg.data = json.dumps(data)
            self.ros2_publishers['object_detection_remote'].publish(msg)

    def publish_mission_status_remote(self, data: Dict[str, Any]):
        """Publish mission status from MQTT to ROS2"""
        if 'mission_status_remote' not in self.ros2_publishers:
            return

        if CUSTOM_MSGS_AVAILABLE:
            msg = MissionStatus()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.robot_id = data.get('robot_id', '')
            msg.status = data.get('status', 0)
            msg.object_found = data.get('object_found', False)
            if 'current_task' in data:
                msg.current_task = data['current_task']
            self.ros2_publishers['mission_status_remote'].publish(msg)
        else:
            msg = String()
            msg.data = json.dumps(data)
            self.ros2_publishers['mission_status_remote'].publish(msg)

    def publish_territory_assignment_remote(self, data: Dict[str, Any]):
        """Publish territory assignment from MQTT to ROS2"""
        if 'territory_assignment_remote' not in self.ros2_publishers:
            return

        if CUSTOM_MSGS_AVAILABLE:
            msg = TerritoryAssignment()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.assigned_robot_id = data.get('assigned_robot_id', '')
            msg.priority = data.get('priority', 0)
            if 'region' in data:
                for point_data in data['region']:
                    point = Point()
                    point.x = point_data.get('x', 0.0)
                    point.y = point_data.get('y', 0.0)
                    point.z = point_data.get('z', 0.0)
                    msg.assigned_region.points.append(point)
            self.ros2_publishers['territory_assignment_remote'].publish(msg)
        else:
            msg = String()
            msg.data = json.dumps(data)
            self.ros2_publishers['territory_assignment_remote'].publish(msg)

    def publish_consensus_result(self, data: Dict[str, Any]):
        """Publish consensus result from MQTT to ROS2"""
        if 'consensus_result' not in self.ros2_publishers:
            return

        msg = String()
        msg.data = json.dumps(data)
        self.ros2_publishers['consensus_result'].publish(msg)

    def publish_coordination_frontiers(self, data: Dict[str, Any]):
        """Publish coordination frontiers from MQTT to ROS2"""
        if 'coordination_frontiers' not in self.ros2_publishers:
            self.get_logger().warn('coordination_frontiers publisher not initialized')
            return

        msg = Float32MultiArray()
        if 'frontiers' in data:
            for frontier in data['frontiers']:
                msg.data.append(frontier.get('x', 0.0))
                msg.data.append(frontier.get('y', 0.0))
        
        num_frontiers = len(data.get('frontiers', []))
        self.get_logger().info(f'Publishing {num_frontiers} coordination frontiers to ROS2 topic /coordination/frontiers')
        self.ros2_publishers['coordination_frontiers'].publish(msg)

    def publish_coordination_paths(self, data: Dict[str, Any]):
        """Publish coordination paths from MQTT to ROS2"""
        # TODO: Implement when path coordination is needed
        pass

    def publish_coordination_goals(self, data: Dict[str, Any]):
        """Publish coordination goals from MQTT to ROS2"""
        # TODO: Implement when goal coordination is needed
        pass

    def publish_mqtt(self, topic: str, data: Dict[str, Any]):
        """Publish data to MQTT topic"""
        if not self.mqtt_connected:
            self.get_logger().warn(f'MQTT not connected, cannot publish to {topic}')
            return

        try:
            payload = json.dumps(data)
            result = self.mqtt_client.publish(topic, payload, qos=1)
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                self.get_logger().warn(f'Failed to publish to MQTT topic {topic}, return code: {result.rc}')
            else:
                self.get_logger().debug(f'Successfully published to MQTT topic {topic}')
        except Exception as e:
            self.get_logger().error(f'Error publishing to MQTT topic {topic}: {e}')

    def check_mqtt_connection(self):
        """Periodically check MQTT connection and reconnect if needed"""
        if not self.mqtt_connected and self.mqtt_client:
            self.get_logger().warn('MQTT disconnected, attempting to reconnect...')
            try:
                self.mqtt_client.reconnect()
            except Exception as e:
                self.get_logger().error(f'Failed to reconnect to MQTT: {e}')


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    node = FoxMQBridgeNode(robot_id=None)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        # ROS2 is already shutting down, don't call shutdown again
        pass
    finally:
        try:
            if node.mqtt_client:
                node.mqtt_client.loop_stop()
                node.mqtt_client.disconnect()
        except:
            pass
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()

