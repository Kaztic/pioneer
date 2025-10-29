#!/usr/bin/env python3
"""
ROS 2 Supervisor Node for Pioneer Multi-Robot Simulation

This node:
1. Reads robot position data from Webots supervisor controller (via stdout)
2. Publishes odometry for each robot to /robotN/odom topics
3. Publishes TF transforms for each robot
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Header
import subprocess
import threading
import re
import math

# Number of robots in the simulation
NUM_ROBOTS = 5
ROBOT_NAMES = [f'robot_{i}' for i in range(1, NUM_ROBOTS + 1)]


class SupervisorNode(Node):
    def __init__(self):
        super().__init__('supervisor_node')
        
        self.get_logger().info('Starting Pioneer ROS 2 Supervisor Node')
        
        # Publishers for each robot's odometry
        self.odom_publishers = {}
        for robot_name in ROBOT_NAMES:
            topic_name = f'/{robot_name}/odom'
            self.odom_publishers[robot_name] = self.create_publisher(
                Odometry,
                topic_name,
                10
            )
            self.get_logger().info(f'Created publisher: {topic_name}')
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Store latest poses
        self.robot_poses = {}
        for robot_name in ROBOT_NAMES:
            self.robot_poses[robot_name] = {
                'x': 0.0, 'y': 0.0, 'z': 0.0,
                'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0,
                'yaw': 0.0
            }
        
        # Thread for reading supervisor output
        self.supervisor_process = None
        self.reading = False
        self.read_thread = None
        
        # Start reading from supervisor
        self.start_supervisor_reader()
        
        # Timer to publish odometry and TF
        self.publish_timer = self.create_timer(0.032, self.publish_odometry)  # ~31 Hz (Webots timestep)
        
    def start_supervisor_reader(self):
        """Start reading from supervisor controller output"""
        # Note: This assumes supervisor controller is run separately
        # In practice, Webots runs the supervisor, and we read its output
        # For now, we'll read from a pipe or file
        self.reading = True
        
        # Start thread to read supervisor output
        self.read_thread = threading.Thread(target=self.read_supervisor_output, daemon=True)
        self.read_thread.start()
        
        self.get_logger().info('Started supervisor reader thread')
    
    def read_supervisor_output(self):
        """
        Read robot pose data from supervisor controller output file.
        
        Expected format: POSE:robot_name:x:y:z:qx:qy:qz:qw:roll:pitch:yaw
        Reads from /tmp/pioneer_poses.txt file written by supervisor controller
        """
        import time
        pose_file_path = "/tmp/pioneer_poses.txt"
        consecutive_errors = 0
        
        while self.reading and rclpy.ok():
            try:
                # Read from file (supervisor controller writes to this file)
                with open(pose_file_path, 'r') as f:
                    lines = f.readlines()
                    if lines:
                        for line in lines:
                            line = line.strip()
                            if line.startswith('POSE:'):
                                self.parse_pose_line(line)
                                consecutive_errors = 0
            except FileNotFoundError:
                # File doesn't exist yet, supervisor hasn't started
                consecutive_errors += 1
                if consecutive_errors == 100:  # Log after 1 second (100 * 10ms)
                    self.get_logger().warn('Pose file not found. Is supervisor controller running?')
                    consecutive_errors = 0
            except Exception as e:
                consecutive_errors += 1
                if consecutive_errors % 100 == 0:
                    self.get_logger().debug(f'Reading pose file error: {e}')
            
            # Small delay to avoid busy-waiting
            time.sleep(0.01)  # 10ms
    
    def parse_pose_line(self, line):
        """Parse a pose line from supervisor output"""
        try:
            # Format: POSE:robot_name:x:y:z:qx:qy:qz:qw:roll:pitch:yaw
            parts = line.split(':')
            if len(parts) != 12:
                self.get_logger().debug(f'Invalid line format: {len(parts)} parts instead of 12')
                return
            
            robot_name = parts[1]
            if robot_name not in ROBOT_NAMES:
                self.get_logger().debug(f'Unknown robot name: {robot_name}')
                return
            
            # Parse pose values
            x = float(parts[2])
            y = float(parts[3])
            z = float(parts[4])
            qx = float(parts[5])
            qy = float(parts[6])
            qz = float(parts[7])
            qw = float(parts[8])
            roll = float(parts[9])
            pitch = float(parts[10])
            yaw = float(parts[11])
            
            # Store pose
            self.robot_poses[robot_name] = {
                'x': x, 'y': y, 'z': z,
                'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
                'yaw': yaw
            }
            
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Failed to parse pose line: {line[:50]}..., error: {e}')
    
    def publish_odometry(self):
        """Publish odometry and TF for all robots"""
        now = self.get_clock().now()
        
        for robot_name in ROBOT_NAMES:
            pose = self.robot_poses[robot_name]
            
            # Skip if pose data hasn't been loaded yet (still at initialized zeros)
            # Note: We can't use (0,0,0) check because robots might actually be at origin
            # So we check if it's the initial zero state by checking if quaternion is also default
            if pose is None:
                continue
            # If pose is still at initialized default (all zeros including quaternion w=1.0, qx=qy=qz=0)
            if (pose['x'] == 0.0 and pose['y'] == 0.0 and 
                pose['qw'] == 1.0 and pose['qx'] == 0.0 and pose['qy'] == 0.0 and pose['qz'] == 0.0):
                continue  # Skip publishing if no valid data has been read yet
            
            # Publish Odometry message
            odom_msg = Odometry()
            odom_msg.header = Header()
            odom_msg.header.stamp = now.to_msg()
            odom_msg.header.frame_id = f'{robot_name}/odom'
            odom_msg.child_frame_id = f'{robot_name}/base_link'
            
            # Set pose
            odom_msg.pose.pose.position.x = pose['x']
            odom_msg.pose.pose.position.y = pose['y']
            odom_msg.pose.pose.position.z = pose['z']
            
            odom_msg.pose.pose.orientation.x = pose['qx']
            odom_msg.pose.pose.orientation.y = pose['qy']
            odom_msg.pose.pose.orientation.z = pose['qz']
            odom_msg.pose.pose.orientation.w = pose['qw']
            
            # Set covariance (small values for simulated ground truth)
            odom_msg.pose.covariance = [
                0.01, 0.0, 0.0, 0.0, 0.0, 0.0,  # x
                0.0, 0.01, 0.0, 0.0, 0.0, 0.0,  # y
                0.0, 0.0, 0.01, 0.0, 0.0, 0.0,  # z
                0.0, 0.0, 0.0, 0.01, 0.0, 0.0,  # roll
                0.0, 0.0, 0.0, 0.0, 0.01, 0.0,  # pitch
                0.0, 0.0, 0.0, 0.0, 0.0, 0.01   # yaw
            ]
            
            # Velocity (zero for now, could be computed from pose differences)
            odom_msg.twist.twist.linear.x = 0.0
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.angular.z = 0.0
            
            # Publish odometry
            self.odom_publishers[robot_name].publish(odom_msg)
            
            # Publish TF transform
            transform = TransformStamped()
            transform.header.stamp = now.to_msg()
            transform.header.frame_id = f'{robot_name}/odom'
            transform.child_frame_id = f'{robot_name}/base_link'
            
            transform.transform.translation.x = pose['x']
            transform.transform.translation.y = pose['y']
            transform.transform.translation.z = pose['z']
            
            transform.transform.rotation.x = pose['qx']
            transform.transform.rotation.y = pose['qy']
            transform.transform.rotation.z = pose['qz']
            transform.transform.rotation.w = pose['qw']
            
            self.tf_broadcaster.sendTransform(transform)
    
    def destroy_node(self):
        self.reading = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = SupervisorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

