from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pioneer_ros2',
            executable='foxmq_bridge_node',
            name='foxmq_bridge_node',
            namespace='robot_1',
            output='screen',
            parameters=[{
                'mqtt_broker_host': '127.0.0.1',
                'mqtt_broker_port': 1883,
                'mqtt_username': 'pioneer_robot',
                'mqtt_password': 'test123',
                'enable_object_detection': True,
                'enable_mission_status': True,
                'enable_territory': True,
                'enable_consensus': True,
                'enable_coordination': True,
            }]
        ),
        Node(
            package='pioneer_ros2',
            executable='foxmq_bridge_node',
            name='foxmq_bridge_node',
            namespace='robot_2',
            output='screen',
            parameters=[{
                'mqtt_broker_host': '127.0.0.1',
                'mqtt_broker_port': 1883,
                'mqtt_username': 'pioneer_robot',
                'mqtt_password': 'test123',
            }]
        ),
        Node(
            package='pioneer_ros2',
            executable='foxmq_bridge_node',
            name='foxmq_bridge_node',
            namespace='robot_3',
            output='screen',
            parameters=[{
                'mqtt_broker_host': '127.0.0.1',
                'mqtt_broker_port': 1883,
                'mqtt_username': 'pioneer_robot',
                'mqtt_password': 'test123',
            }]
        ),
        Node(
            package='pioneer_ros2',
            executable='foxmq_bridge_node',
            name='foxmq_bridge_node',
            namespace='robot_4',
            output='screen',
            parameters=[{
                'mqtt_broker_host': '127.0.0.1',
                'mqtt_broker_port': 1883,
                'mqtt_username': 'pioneer_robot',
                'mqtt_password': 'test123',
            }]
        ),
        Node(
            package='pioneer_ros2',
            executable='foxmq_bridge_node',
            name='foxmq_bridge_node',
            namespace='robot_5',
            output='screen',
            parameters=[{
                'mqtt_broker_host': '127.0.0.1',
                'mqtt_broker_port': 1883,
                'mqtt_username': 'pioneer_robot',
                'mqtt_password': 'test123',
            }]
        ),
    ])

