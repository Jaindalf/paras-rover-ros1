import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Define the Motor Bridge Node
    motor_bridge_node = Node(
        package='rover_hardware',
        executable='motor_bridge',  # <--- Change this to match your entry_point in setup.py
        name='motor_bridge',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyMega',
            'baud_rate': 115200,
            'use_sim_time': False
        }],
        remappings=[
            # This connects your hardware to the Nav2 velocity smoother output
            ('/cmd_vel', '/cmd_vel')
        ]
    )

    return LaunchDescription([
        motor_bridge_node
    ])

