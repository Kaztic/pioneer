from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pioneer_path_planner',
            executable='spatial_analysis_visualizer_node',
            name='spatial_analysis_visualizer_node',
            output='screen',
            parameters=[{
                'num_robots': 5,
                'heatmap_resolution': 0.5,
                'heatmap_update_rate': 1.0,
                'trajectory_window_seconds': 30.0,
                'frontier_window_seconds': 30.0,
                'path_window_seconds': 10.0,
                'world_bounds_x_min': -15.0,
                'world_bounds_x_max': 15.0,
                'world_bounds_y_min': -15.0,
                'world_bounds_y_max': 15.0,
                'fixed_frame': 'robot_1/odom',
            }]
        ),
    ])








