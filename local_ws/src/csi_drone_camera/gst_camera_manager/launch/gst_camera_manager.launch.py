# ros2 launch gst_camera_manager gst_camera_manager.launch.py

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
