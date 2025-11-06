#!/usr/bin/env python3
"""
ROS2 cmd_vel Bridge Node

Bridges ROS2 geometry_msgs/Twist commands from /robot_i/cmd_vel topics
to file-based communication for the Webots controller.c to read.

This node:
1. Subscribes to /robot_i/cmd_vel (geometry_msgs/Twist)
2. Converts Twist (linear.x, angular.z) to wheel velocities
3. Writes velocity commands to /tmp/pioneer_cmd_vel_robot_i.txt
4. Handles timeouts (if no cmd_vel received, writes stop command)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import time
import os
from typing import Optional, Dict


class CmdVelBridgeNode(Node):
    """
    Bridge node for a single robot.
    Should be instantiated once per robot.
    """

    def __init__(self, robot_id: Optional[str] = None, num_robots: int = 5):
        """
        Initialize cmd_vel bridge node.

        Args:
            robot_id: Robot identifier (e.g., "robot_1", "robot_2").
                     If None, will be extracted from node namespace.
            num_robots: Total number of robots (for creating all bridges)
        """
        # If robot_id not provided, we'll get it from namespace after initialization
        node_name = 'cmd_vel_bridge_node'
        
        if robot_id:
            super().__init__(node_name, namespace=robot_id)
            self.robot_id = robot_id
        else:
            super().__init__(node_name)
            # Extract robot_id from node namespace
            namespace = self.get_namespace()
            if namespace and namespace != '/':
                self.robot_id = namespace.strip('/') if namespace.startswith('/') else namespace
            else:
                # Use fully qualified name to extract namespace
                full_name = self.get_fully_qualified_name()
                parts = [p for p in full_name.split('/') if p]
                if len(parts) >= 2:
                    self.robot_id = parts[0]
                else:
                    self.robot_id = "robot_1"  # Default

        # Parameters
        self.declare_parameter('timeout_seconds', 0.5)  # Timeout if no cmd_vel received
        self.declare_parameter('wheel_base', 0.5)  # Distance between wheels (meters)
        self.declare_parameter('max_linear_speed', 5.0)  # m/s
        self.declare_parameter('max_angular_speed', 2.0)  # rad/s

        self.timeout_seconds = self.get_parameter('timeout_seconds').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        
        # Re-extract robot_id after node is fully initialized (if needed)
        if not hasattr(self, 'robot_id') or not self.robot_id:
            namespace = self.get_namespace()
            if namespace and namespace != '/':
                self.robot_id = namespace.strip('/') if namespace.startswith('/') else namespace
            else:
                full_name = self.get_fully_qualified_name()
                parts = [p for p in full_name.split('/') if p]
                if len(parts) >= 2:
                    self.robot_id = parts[0]
                else:
                    self.robot_id = "robot_1"  # Default

        # File path for this robot's cmd_vel commands
        self.cmd_vel_file = f'/tmp/pioneer_cmd_vel_{self.robot_id}.txt'

        # Track last received cmd_vel time
        self.last_cmd_vel_time: Optional[float] = None
        self.last_cmd_vel: Optional[Twist] = None

        # Subscription (use relative name, ROS2 will auto-add namespace)
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info(f'Subscribed to /{self.robot_id}/cmd_vel')

        # Timer to periodically write commands and check timeout
        self.write_timer = self.create_timer(0.032, self.write_cmd_vel_file)  # 31.25 Hz (Webots timestep)

        self.get_logger().info(
            f'cmd_vel bridge node initialized for {self.robot_id}\n'
            f'  Output file: {self.cmd_vel_file}\n'
            f'  Timeout: {self.timeout_seconds}s\n'
            f'  Wheel base: {self.wheel_base}m'
        )

    def cmd_vel_callback(self, msg: Twist):
        """Update last received cmd_vel command and write immediately."""
        self.last_cmd_vel_time = time.time()
        self.last_cmd_vel = msg
        # Debug: log first few messages to verify callback is working
        if not hasattr(self, '_callback_count'):
            self._callback_count = 0
        self._callback_count += 1
        if self._callback_count <= 10:
            # CRITICAL: Log at ERROR level for visibility
            self.get_logger().error(
                f'[CMD_VEL_BRIDGE_RECEIVED] Received cmd_vel #{self._callback_count}: '
                f'linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}'
            )
        
        # Write command immediately (don't wait for timer)
        # This ensures file is updated as soon as cmd_vel is received
        self.write_twist_command(msg)

    def write_cmd_vel_file(self):
        """
        Periodic check to handle timeout if no cmd_vel received.
        Commands are written immediately in callback, but we check timeout here.
        """
        current_time = time.time()

        # Check for timeout
        if (self.last_cmd_vel_time is None or
            (current_time - self.last_cmd_vel_time) > self.timeout_seconds):
            # Timeout: write stop command
            if not hasattr(self, '_timeout_logged'):
                self.get_logger().warn(
                    f'cmd_vel timeout ({self.timeout_seconds}s), writing stop command'
                )
                self._timeout_logged = True
            self.write_stop_command()
            return

        # Reset timeout warning flag if we receive a new message
        if hasattr(self, '_timeout_logged'):
            self._timeout_logged = False

        # Note: cmd_vel is written immediately in callback, so we don't need to write here
        # This timer just handles timeout checking

    def write_twist_command(self, twist: Twist):
        """
        Convert Twist to wheel velocities and write to file.

        Format: linear_x:angular_z:left_wheel:right_wheel:front_left:front_right:back_left:back_right

        For 4-wheel differential drive:
        - v = linear.x (forward velocity)
        - ω = angular.z (angular velocity)
        - Left wheel speed = v - (ω * wheel_base / 2)
        - Right wheel speed = v + (ω * wheel_base / 2)
        - Front and back wheels same as left/right respectively
        """
        # Clamp velocities to max limits
        linear_x = max(-self.max_linear_speed, min(self.max_linear_speed, twist.linear.x))
        angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, twist.angular.z))

        # Convert to wheel velocities for 4-wheel differential drive
        # For differential drive: v_left = v - (ω * L/2), v_right = v + (ω * L/2)
        # Where L is wheel base (distance between left and right wheels)
        left_speed = linear_x - (angular_z * self.wheel_base / 2.0)
        right_speed = linear_x + (angular_z * self.wheel_base / 2.0)

        # For 4-wheel robot: front and back wheels match left/right speeds
        front_left = left_speed
        front_right = right_speed
        back_left = left_speed
        back_right = right_speed

        # Write to file (atomic write)
        try:
            # Write to temp file first, then rename (atomic operation)
            temp_file = self.cmd_vel_file + '.tmp'
            file_content = f'{linear_x:.6f}:{angular_z:.6f}:'
            file_content += f'{left_speed:.6f}:{right_speed:.6f}:'
            file_content += f'{front_left:.6f}:{front_right:.6f}:'
            file_content += f'{back_left:.6f}:{back_right:.6f}\n'
            
            with open(temp_file, 'w') as f:
                f.write(file_content)
                f.flush()
                os.fsync(f.fileno())  # Force write to disk

            # Atomic rename
            os.replace(temp_file, self.cmd_vel_file)
            
            # Debug: log first few writes to verify file writing works
            if not hasattr(self, '_write_count'):
                self._write_count = 0
            self._write_count += 1
            if self._write_count <= 10:
                # CRITICAL: Log at ERROR level for visibility
                self.get_logger().error(
                    f'[CMD_VEL_FILE_WRITTEN] Written to file #{self._write_count}: {file_content.strip()}'
                )
        except Exception as e:
            self.get_logger().error(f'Failed to write cmd_vel file: {e}', exc_info=True)

    def write_stop_command(self):
        """Write stop command (all velocities zero)."""
        try:
            temp_file = self.cmd_vel_file + '.tmp'
            with open(temp_file, 'w') as f:
                f.write('0.0:0.0:0.0:0.0:0.0:0.0:0.0:0.0\n')
                f.flush()
                os.fsync(f.fileno())

            os.replace(temp_file, self.cmd_vel_file)
        except Exception as e:
            self.get_logger().error(f'Failed to write stop command: {e}')


class MultiRobotCmdVelBridge:
    """
    Manages cmd_vel bridges for all robots.
    Creates one node per robot.
    """

    def __init__(self, num_robots: int = 5):
        """
        Initialize bridges for all robots.

        Args:
            num_robots: Number of robots (default: 5)
        """
        self.num_robots = num_robots
        self.nodes: Dict[str, CmdVelBridgeNode] = {}

        for i in range(1, num_robots + 1):
            robot_id = f'robot_{i}'
            node = CmdVelBridgeNode(robot_id=robot_id, num_robots=num_robots)
            self.nodes[robot_id] = node

    def get_nodes(self):
        """Get all bridge nodes."""
        return list(self.nodes.values())


def main(args=None):
    """Main entry point for cmd_vel bridge node."""
    rclpy.init(args=args)

    # Let ROS2 handle namespace via --ros-args -r __ns:=/robot_1
    # The node will extract robot_id from its namespace
    node = CmdVelBridgeNode(robot_id=None)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error in bridge node: {e}')
    finally:
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

