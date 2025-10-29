#!/usr/bin/env python3
"""
Print odometry for all 5 Pioneer robots every 10 seconds

Usage:
    python3 print_robot_odom.py
    
    Or make it executable:
    chmod +x print_robot_odom.py
    ./print_robot_odom.py
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import time

NUM_ROBOTS = 5
ROBOT_NAMES = [f'robot_{i}' for i in range(1, NUM_ROBOTS + 1)]


class OdometryPrinter(Node):
    def __init__(self):
        super().__init__('odom_printer')
        
        # Store latest odometry for each robot
        self.robot_odoms = {name: None for name in ROBOT_NAMES}
        
        # Subscribers for each robot
        self.subscribers = {}
        for robot_name in ROBOT_NAMES:
            topic_name = f'/{robot_name}/odom'
            self.subscribers[robot_name] = self.create_subscription(
                Odometry,
                topic_name,
                lambda msg, name=robot_name: self.odom_callback(msg, name),
                10
            )
            self.get_logger().info(f'Subscribed to {topic_name}')
        
        # Timer to print every 10 seconds
        self.print_timer = self.create_timer(10.0, self.print_all_odoms)
        
        self.get_logger().info('Odometry printer started. Will print every 10 seconds...')
    
    def odom_callback(self, msg, robot_name):
        """Store latest odometry"""
        self.robot_odoms[robot_name] = msg
    
    def quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle (in radians)"""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def print_all_odoms(self):
        """Print odometry for all robots"""
        print("\n" + "="*80)
        print(f"Robot Odometry Update - {time.strftime('%Y-%m-%d %H:%M:%S')}")
        print("="*80)
        
        for robot_name in ROBOT_NAMES:
            odom = self.robot_odoms[robot_name]
            
            if odom is None:
                print(f"\n{robot_name:12} | NO DATA (topic not publishing)")
                continue
            
            pos = odom.pose.pose.position
            ori = odom.pose.pose.orientation
            yaw_rad = self.quaternion_to_yaw(ori)
            yaw_deg = math.degrees(yaw_rad)
            
            # Calculate speed (magnitude of linear velocity)
            speed = math.sqrt(odom.twist.twist.linear.x**2 + odom.twist.twist.linear.y**2)
            angular_speed = odom.twist.twist.angular.z
            
            print(f"\n{robot_name:12} |")
            print(f"  Position:    x = {pos.x:8.3f} m,  y = {pos.y:8.3f} m,  z = {pos.z:6.3f} m")
            print(f"  Orientation: yaw = {yaw_deg:7.2f}° ({yaw_rad:.4f} rad)")
            print(f"  Velocity:    linear = {speed:.3f} m/s,  angular = {angular_speed:.3f} rad/s")
            
            # Show quaternion if requested
            # print(f"  Quaternion:  x={ori.x:.4f}, y={ori.y:.4f}, z={ori.z:.4f}, w={ori.w:.4f}")
        
        print("\n" + "="*80)
        print("Next update in 10 seconds...\n")


def main(args=None):
    rclpy.init(args=args)
    
    node = OdometryPrinter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nShutting down odometry printer...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

