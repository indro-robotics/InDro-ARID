# Run at boot by gst_camera_manager.service, which supplies ROS_DOMAIN_ID and the log redirect.

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
