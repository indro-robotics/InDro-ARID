from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vslam_supervisor',
            executable='vslam_supervisor_node',
            name='vslam_supervisor',
            output='screen',
        ),
    ])
