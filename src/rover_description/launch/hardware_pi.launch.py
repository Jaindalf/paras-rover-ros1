import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    rover_desc_dir = get_package_share_directory('rover_description')

    # 1. LiDAR
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py'
        )])
    )

    # 2. Robot State Publisher (Pi needs this to broadcast the laser/base_link TF)
    robot_state_pub_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            rover_desc_dir, 'launch', 'robot_state_publisher.launch.py'
        )])
    )

    # 3. Motor Bridge (Talking to Arduino)
    rover_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rover_hardware'), 'launch', 'motor_bridge.launch.py'
        )])
    )

    # 4. Wheel Odometry
    wheel_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('wheel_odometry'), 'launch', 'odom.launch.py'
        )])
    )

    return LaunchDescription([
        rplidar_launch,
        robot_state_pub_launch,
        rover_hardware_launch,
        wheel_odom_launch
    ])
