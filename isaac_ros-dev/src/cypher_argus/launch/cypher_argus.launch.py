import os
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


# LAUNCH
# ros2 launch cypher_argus cypher_argus.launch.py \
#   video_device:=0 camera_mode:=0 image_namespace:=cam_down \
#   frame_name:=down_cv_link framerate:=10.0


def generate_nodes(context, *args, **kwargs):
    package_name = 'cypher_argus'
    pkg_share = get_package_share_directory(package_name)
    config_dir = Path(pkg_share) / 'config'

    video_device = int(LaunchConfiguration('video_device').perform(context))
    camera_mode = int(LaunchConfiguration('camera_mode').perform(context))
    image_ns_arg = LaunchConfiguration('image_namespace').perform(context)
    frame_name = LaunchConfiguration('frame_name').perform(context)
    framerate = float(LaunchConfiguration('framerate').perform(context))

    # Pick calibration file: IMX477_<camera_mode>.yaml
    calib_filename = f'IMX477_{camera_mode}.yaml'
    calib_path = config_dir / calib_filename

    camera_info_url = ''
    yaml_cam_name = None

    if calib_path.is_file():
        camera_info_url = f'file://{calib_path}'
        try:
            with calib_path.open('r') as f:
                calib_data = yaml.safe_load(f)
            yaml_cam_name = calib_data.get('camera_name', None)
        except Exception:
            pass

    # Namespace priority: launch arg > YAML camera_name > default 'cam_down'
    image_ns = image_ns_arg or yaml_cam_name or 'cam_down'

    env = dict(os.environ)
    for bad in ['DISPLAY', 'WAYLAND_DISPLAY']:
        env.pop(bad, None)

    ld = env.get('LD_LIBRARY_PATH', '')
    env['LD_LIBRARY_PATH'] = f"/opt/ros/humble/lib:{ld}" if ld else "/opt/ros/humble/lib"

    pkg_april_share = get_package_share_directory('apriltag_ros')
    tag_config_map = {
        'cam_down': {
            'config': os.path.join(pkg_april_share, 'cfg', 'amr_tags.yaml')
        },
        'cam_front': {
            'config': os.path.join(pkg_april_share, 'cfg', 'shelf_tags.yaml')
        },
    }

    april_name = 'april_' + image_ns

    argus_node = ComposableNode(
        package='isaac_ros_argus_camera',
        plugin='nvidia::isaac_ros::argus::ArgusMonoNode',
        name='argus_mono',
        namespace=image_ns,
        parameters=[{
            'video_device': video_device,
            'mode': camera_mode,
            'camera_info_url': camera_info_url,
            'camera_link_frame_name': frame_name,
            'optical_frame_name': frame_name,
            'framerate': framerate,
        }],
        remappings=[
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
    )

    composable_nodes = [argus_node, rectify_node]

    if image_ns in tag_config_map:
        config_info = tag_config_map[image_ns]
        apriltagger = ComposableNode(
            package='apriltag_ros',
            plugin='apriltag_ros::AprilTagNode',
            name=april_name,
            namespace=image_ns,
            parameters=[config_info['config']],
            extra_arguments=[{'use_intra_process_comms': True}]
            # 1:1 remappings removed
        )
        composable_nodes.append(apriltagger)

    container = ComposableNodeContainer(
        name='cypher_argus_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=composable_nodes,
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        env=env,
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
        default_value='1',
        description='Camera mode index (0 -> IMX477_0.yaml, 1 -> IMX477_1.yaml, etc.)',
    )

    image_ns_arg = DeclareLaunchArgument(
        'image_namespace',
        default_value='',
        description='Namespace for image topics (if empty, use camera_name from YAML or "cam_down")',
    )

    frame_arg = DeclareLaunchArgument(
        'frame_name',
        default_value='down_cv_link',
        description='Frame ID / frame name for Argus camera',
    )

    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='10.0',
        description='Camera framerate (Hz)',
    )

    setup = OpaqueFunction(function=generate_nodes)

    return LaunchDescription([
        video_device_arg,
        camera_mode_arg,
        image_ns_arg,
        frame_arg,
        framerate_arg,
        setup,
    ])
