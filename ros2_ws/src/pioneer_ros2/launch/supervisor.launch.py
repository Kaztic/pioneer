from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pioneer_ros2',
            executable='supervisor_node',
            name='supervisor_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }]
        ),
    ])


