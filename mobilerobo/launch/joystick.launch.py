from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            remappings=[
                ('/cmd_vel', '/cmd_vel_joy'),
                ('/joy', '/joy')
            ],
            parameters=[{
                'axis_linear': {'x': 1},
                'axis_angular': {'yaw': 0},
                'scale_linear': {'x': 1.0},
                'scale_angular': {'yaw': 1.0},
                'use_stamped_vel': True,
                'enable_button': 10
            }]
        ),
    ])