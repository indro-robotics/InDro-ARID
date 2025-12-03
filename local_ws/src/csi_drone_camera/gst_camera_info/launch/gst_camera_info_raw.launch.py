import yaml
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    vid_src = int(LaunchConfiguration('vid_src').perform(context))
    camera_topic = LaunchConfiguration('camera_topic').perform(context)
    framerate = int(LaunchConfiguration('framerate').perform(context))
    calib_file = LaunchConfiguration('calib_file').perform(context)
    visual_link = LaunchConfiguration('visual_link').perform(context)

    # Raw filesystem path for Python/YAML
    camera_info_config_path = f"{get_package_share_directory('gst_camera_info')}/config/{calib_file}"
    # URI for camera_info_manager
    camera_info_path = f"file://{camera_info_config_path}"

    # Open config file to get width/height for camera node parameters
    with open(camera_info_config_path, 'r') as f:
        cam_config = yaml.safe_load(f)

    composable_nodes = [
        ComposableNode(
            package='gst_cam_node',
            plugin='GstCamNode',
            name='gst_cam_node',
            parameters=[{
                'vid_src': vid_src,
                'width': cam_config['image_width'],
                'height': cam_config['image_height'],
                'framerate': framerate,
                'frame_id': visual_link,
                'camera_topic': camera_topic,
                'camera_info_path': camera_info_path,  # Pass URI to node
            }],
        )
    ]

    container = ComposableNodeContainer(
        name='vision_pipeline_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=composable_nodes,
        output='screen',
        emulate_tty=True,
    )

    return [container]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vid_src', default_value='0'),
        DeclareLaunchArgument('camera_topic', default_value='cam_down'),
        DeclareLaunchArgument('framerate', default_value='10'),
        DeclareLaunchArgument('calib_file', default_value='IMX219_2K.yaml'),
        DeclareLaunchArgument('visual_link', default_value='bottom_visual_link'),
        OpaqueFunction(function=launch_setup),
    ])