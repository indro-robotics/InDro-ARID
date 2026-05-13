from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rslidar_coordinator',
            executable='rslidar_coordinator_node',
            name='rslidar_coordinator',
            output='screen',
        ),
    ])
