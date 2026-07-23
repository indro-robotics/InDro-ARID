from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arid_supervisor',
            executable='arid_supervisor_node',
            name='arid_supervisor',
            output='screen',
        ),
    ])
