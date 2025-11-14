from setuptools import setup
import os
from glob import glob

package_name = 'pioneer_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'paho-mqtt'],
    zip_safe=True,
    maintainer='Pioneer Team',
    maintainer_email='user@example.com',
    description='ROS 2 integration for Pioneer multi-robot simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'supervisor_node = pioneer_ros2.supervisor_node:main',
            'aggregator_node = pioneer_ros2.aggregator_node:main',
            'cmd_vel_bridge_node = pioneer_ros2.cmd_vel_bridge_node:main',
            'lidar_bridge_node = pioneer_ros2.lidar_bridge_node:main',
            'camera_bridge_node = pioneer_ros2.camera_bridge_node:main',
            'foxmq_bridge_node = pioneer_ros2.foxmq_bridge_node:main',
        ],
    },
)


