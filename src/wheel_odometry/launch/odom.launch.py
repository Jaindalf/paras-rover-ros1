from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='wheel_odometry',
            executable='odom_node',
            name='wheel_odometry',
            output='screen',
            parameters=[
                {
                    'port': '/dev/ttyNano',
                    'baudrate': 115200
                }
            ]
        )
    ])

