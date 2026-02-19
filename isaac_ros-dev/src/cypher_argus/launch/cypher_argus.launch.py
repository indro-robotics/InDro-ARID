import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_nodes(context, *args, **kwargs):
    package_name = 'cypher_argus'
    pkg_share = get_package_share_directory(package_name)
    config_dir = Path(pkg_share) / 'config'

    video_device = int(LaunchConfiguration('video_device').perform(context))
    camera_mode = int(LaunchConfiguration('camera_mode').perform(context))
    image_ns = LaunchConfiguration('image_namespace').perform(context)

    # Pick calibration file: IMX477_<camera_mode>.yaml
    calib_filename = f'IMX477_{camera_mode}.yaml'
    calib_path = config_dir / calib_filename

    camera_info_url = ''
    frame_name = None

    if calib_path.is_file():
        camera_info_url = f'file://{calib_path}'
        try:
            with calib_path.open('r') as f:
                calib_data = yaml.safe_load(f)
            cam_name = calib_data.get('camera_name', None)
            if cam_name:
                frame_name = cam_name
        except Exception:
            pass

    if frame_name is None:
        raise RuntimeError(f"Could not read camera_name from calibration file: {calib_path}")

# argus_node = ComposableNode(
#     package='isaac_ros_argus_camera',
#     plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
#     name='argus_mono',
#     namespace=image_ns,
#     parameters=[{
#         'video_device': video_device,
#         'mode': camera_mode,
#         'camera_info_url': camera_info_url,
#         'optical_frame_name': frame_name,
#     }],
#     remappings=[
#         # Argus internally uses left/image_raw + left/camera_info.
#         # Remap them to image_raw + camera_info in this namespace.
#         ('left/image_raw', 'image_raw'),
#         ('left/camera_info', 'camera_info'),
#         ('left/image_raw/nitros', 'image_raw/nitros'),
#         ('left/camera_info/nitros', 'camera_info/nitros'),
#     ],
# )

# rectify_node = ComposableNode(
#     package='isaac_ros_image_proc',
#     plugin='nvidia::isaac_ros::image_proc::RectifyNode',
#     name='image_rectify',
#     namespace=image_ns,
#     # Rectify subscribes to image + camera_info in its namespace,
#     # which resolve to /<image_ns>/image_raw and /<image_ns>/camera_info
# )

    argus_node = ComposableNode(
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        name='argus_mono',
        namespace=image_ns,
        parameters=[{
            'video_device': video_device,
            'mode': camera_mode,
            'camera_info_url': camera_info_url,
            'optical_frame_name': frame_name,
        }],
        remappings=[
            # Argus internally uses left/image_raw + left/camera_info.
            # Remap them to image_raw + camera_info in this namespace.
            ('left/image_raw', 'image_raw'),
            ('left/camera_info', 'camera_info'),
            ('left/image_raw/nitros', 'image_raw/nitros'),
            ('left/camera_info/nitros', 'camera_info/nitros'),
        ],
    )

    rectify_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        name='image_rectify',
        namespace=image_ns,
        # Rectify subscribes to image + camera_info in its namespace,
        # which resolve to /<image_ns>/image_raw and /<image_ns>/camera_info
    )

    container = ComposableNodeContainer(
        name='cypher_argus_container',
        namespace='',  # container itself at root; nodes live under image_ns
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[argus_node, rectify_node],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return [container]


def generate_launch_description():
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='0',
        description='Index of the video device (e.g. 0 for /dev/video0)',
    )

    camera_mode_arg = DeclareLaunchArgument(
        'camera_mode',
        default_value='0',
        description='Camera mode index (0 -> IMX477_0.yaml, 1 -> IMX477_1.yaml, etc.)',
    )

    image_ns_arg = DeclareLaunchArgument(
        'image_namespace',
        default_value='camera',
        description='Base namespace for image topics',
    )

    setup = OpaqueFunction(function=generate_nodes)

    return LaunchDescription([
        video_device_arg,
        camera_mode_arg,
        image_ns_arg,
        setup,
    ])
