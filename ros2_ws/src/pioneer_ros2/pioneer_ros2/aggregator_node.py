#!/usr/bin/env python3
"""
ROS 2 Aggregator Node for Pioneer Multi-Robot Simulation

This optional node subscribes to all robot odometry topics and publishes
a combined message with all robot positions.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import math

NUM_ROBOTS = 5
ROBOT_NAMES = [f'robot_{i}' for i in range(1, NUM_ROBOTS + 1)]


class AggregatorNode(Node):
    def __init__(self):
        super().__init__('aggregator_node')
        
        self.get_logger().info('Starting Pioneer ROS 2 Aggregator Node')
        
        # Subscribers for each robot's odometry
        self.odom_subscribers = {}
        self.robot_poses = {}
        
        for robot_name in ROBOT_NAMES:
            topic_name = f'/{robot_name}/odom'
            self.robot_poses[robot_name] = None
            
            # Create subscriber
            self.odom_subscribers[robot_name] = self.create_subscription(
                Odometry,
                topic_name,
                lambda msg, name=robot_name: self.odom_callback(msg, name),
                10
            )
            self.get_logger().info(f'Subscribed to: {topic_name}')
        
        # Publisher for aggregated poses
        self.poses_publisher = self.create_publisher(
            PoseStamped,
            '/robots/poses',
            10
        )
        
        # Timer to publish aggregated data
        self.publish_timer = self.create_timer(0.1, self.publish_aggregated_poses)  # 10 Hz
        
    def odom_callback(self, msg, robot_name):
        """Callback when odometry is received"""
        self.robot_poses[robot_name] = msg
        
        # Optionally print robot positions
        if msg.pose.pose.position is not None:
            self.get_logger().debug(
                f'{robot_name}: x={msg.pose.pose.position.x:.2f}, '
                f'y={msg.pose.pose.position.y:.2f}, '
                f'yaw={self.quaternion_to_yaw(msg.pose.pose.orientation):.2f}'
            )
    
    def quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def publish_aggregated_poses(self):
        """Publish aggregated poses for all robots"""
        # For simplicity, publish first robot's pose
        # In a full implementation, create a custom message with all poses
        if self.robot_poses[ROBOT_NAMES[0]] is not None:
            msg = PoseStamped()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose = self.robot_poses[ROBOT_NAMES[0]].pose.pose
            
            self.poses_publisher.publish(msg)
            
            # Log summary
            active_robots = sum(1 for pose in self.robot_poses.values() if pose is not None)
            self.get_logger().info(f'Active robots: {active_robots}/{NUM_ROBOTS}')


def main(args=None):
    rclpy.init(args=args)
    
    node = AggregatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


