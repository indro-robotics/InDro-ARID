import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
        pkg_share = get_package_share_directory('apriltag_ros')
        shelf_tag_config = os.path.join(pkg_share, 'cfg', 'shelf_tags.yaml')
        amr_tag_config = os.path.join(pkg_share, 'cfg', 'amr_tags.yaml')

        shelf_tracker = Node(
                package='apriltag_ros',
                executable='apriltag_node',
                name='shelf_tracker',
                namespace='cam_front',
                parameters=[shelf_tag_config],
                remappings=[
                        ('image_rect', 'synced_image'),
                        ('camera_info', 'synced_camera_info')
                ],
                arguments=['--ros-args', '--log-level', 'error']
        )

        amr_tracker = Node(
                package='apriltag_ros',
                executable='apriltag_node',
                name='amr_tracker',
                namespace='cam_down',
                parameters=[amr_tag_config],
                remappings=[
                        ('image_rect', 'synced_image'),
                        ('camera_info', 'synced_camera_info')
                ],
                arguments=['--ros-args', '--log-level', 'error']
        )

        april_tracker = Node(
            package='april_targeting',
            executable='april_tracker_node',
            name='april_tracker'
        )

        return LaunchDescription([amr_tracker, shelf_tracker, april_tracker])