# Loaded by gst_camera_manager.service.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gst_camera_manager',
            executable='gst_camera_manager',
            name='gst_camera_manager',
            output='screen',
        ),
    ])
