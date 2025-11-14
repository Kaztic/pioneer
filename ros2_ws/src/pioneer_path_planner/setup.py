from setuptools import setup
import os
from glob import glob

package_name = 'pioneer_path_planner'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'opencv-python', 'cv-bridge'],
    zip_safe=True,
    maintainer='Pioneer Team',
    maintainer_email='user@example.com',
    description='A* path planner node for Pioneer multi-robot simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planner_node = pioneer_path_planner.path_planner_node:main',
            'path_follower_node = pioneer_path_planner.path_follower_node:main',
            'occupancy_grid_builder_node = pioneer_path_planner.occupancy_grid_builder_node:main',
            'frontier_detector_node = pioneer_path_planner.frontier_detector_node:main',
            'goal_selector_node = pioneer_path_planner.goal_selector_node:main',
            'object_detection_node = pioneer_path_planner.object_detection_node:main',
            'mission_coordinator_node = pioneer_path_planner.mission_coordinator_node:main',
            'consensus_handler_node = pioneer_path_planner.consensus_handler_node:main',
            'collision_avoidance_node = pioneer_path_planner.collision_avoidance_node:main',
            'verification_coordinator_node = pioneer_path_planner.verification_coordinator_node:main',
            'spatial_analysis_visualizer_node = pioneer_path_planner.spatial_analysis_visualizer_node:main',
        ],
    },
)

