import os
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


# LAUNCH
# Two containers: [argus] --NITROS DDS--> [flip → rectify → converter → apriltag]
# The container split at argus→flip forces a serialized CPU copy, preventing the CUDA
# async race condition that caused random black frames from the rectify node.
#
# DEFAULT (mono8 rectified pipeline + AprilTag detection, 180° flip, no compression):
# ros2 launch cypher_argus cypher_argus.launch.py \
#   video_device:=0 camera_mode:=1 image_namespace:=cam_down \
#   frame_name:=down_cv_link framerate:=10.0 output_encoding:=mono8
#
# WITHOUT FLIP (e.g. camera not physically rotated 180°):
# ros2 launch cypher_argus cypher_argus.launch.py \
#   video_device:=0 camera_mode:=1 image_namespace:=cam_down \
#   frame_name:=down_cv_link framerate:=10.0 output_encoding:=mono8 flip_image:=false
#
# WITH COMPRESSION (publishes /cam_down/image_rect/compressed for Foxglove):
# NOTE: CPU JPEG via image_transport — suitable for low-frequency use (e.g. calibration checks,
#       one-off monitoring). NOT efficient for continuous streaming; use an Argus GPU compression
#       package for that instead.
# ros2 launch cypher_argus cypher_argus.launch.py \
#   video_device:=0 camera_mode:=1 image_namespace:=cam_down \
#   frame_name:=down_cv_link framerate:=10.0 output_encoding:=mono8 compress:=true


def generate_nodes(context, *args, **kwargs):
    package_name = 'cypher_argus'
    pkg_share = get_package_share_directory(package_name)
    config_dir = Path(pkg_share) / 'config'

    video_device = int(LaunchConfiguration('video_device').perform(context))
    camera_mode = int(LaunchConfiguration('camera_mode').perform(context))
    image_ns_arg = LaunchConfiguration('image_namespace').perform(context)
    frame_name = LaunchConfiguration('frame_name').perform(context)
    framerate = float(LaunchConfiguration('framerate').perform(context))
    output_encoding = LaunchConfiguration('output_encoding').perform(context)
    compress = LaunchConfiguration('compress').perform(context).lower() in ('true', '1', 'yes')
    flip_image = LaunchConfiguration('flip_image').perform(context).lower() in ('true', '1', 'yes')

    # Pick calibration file: IMX477_<camera_mode>.yaml
    calib_filename = f'IMX477_{camera_mode}.yaml'
    calib_path = config_dir / calib_filename

    camera_info_url = ''
    yaml_cam_name = None

    image_width = None
    image_height = None

    if calib_path.is_file():
        camera_info_url = f'file://{calib_path}'
        try:
            with calib_path.open('r') as f:
                calib_data = yaml.safe_load(f)
            yaml_cam_name = calib_data.get('camera_name', None)
            image_width = calib_data.get('image_width', None)
            image_height = calib_data.get('image_height', None)
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

    raw_image_topic = 'image_raw_color' if output_encoding != 'rgb8' else 'image_raw'

    # When flipping, argus publishes to a pre-flip topic; the flip node outputs to raw_image_topic.
    # This keeps all downstream nodes (rectify, compress, calibrate.sh) unchanged.
    argus_raw_topic = f'{raw_image_topic}_pre_flip' if flip_image else raw_image_topic

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
            ('left/image_raw', argus_raw_topic),
            ('left/camera_info', 'camera_info'),
            ('left/image_raw/nitros', f'{argus_raw_topic}/nitros'),
            ('left/camera_info/nitros', 'camera_info/nitros'),
        ],
    )

    # Pipeline order: argus → rectify → format_converter → apriltag
    # For non-rgb8: argus outputs image_raw_color (rgb8), rectify reads that and
    # outputs image_rect_color (bgr8), format_converter converts to image_rect (target encoding).
    # apriltag reads image_rect (default) regardless of encoding.
    # For rgb8: argus outputs image_raw directly, rectify uses default topics, no converter needed.
    rectify_remaps = [
        ('image_raw', 'image_raw_color'),
        ('image_rect', 'image_rect_color'),
    ] if output_encoding != 'rgb8' else []

    rectify_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        name='image_rectify',
        namespace=image_ns,
        parameters=[{k: v for k, v in {
            'output_width': image_width,
            'output_height': image_height,
        }.items() if v is not None}],
        remappings=rectify_remaps,
    )

    # Container 1: argus only.
    # Separating argus into its own container means the argus→flip boundary is a NITROS
    # DDS hop (serialized CPU copy). This guarantees the GPU buffer is fully written before
    # the downstream pipeline ever reads it, preventing the CUDA async race that causes
    # random black frames from the rectify node.
    argus_container = ComposableNodeContainer(
        name='cypher_argus_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[argus_node],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        env=env,
    )

    # Container 2: flip (optional) → rectify → format_converter → apriltag.
    # All nodes here communicate via NITROS zero-copy GPU buffers.
    pipeline_nodes = []

    if flip_image:
        pipeline_nodes.append(ComposableNode(
            package='isaac_ros_image_proc',
            plugin='nvidia::isaac_ros::image_proc::ImageFlipNode',
            name='image_flip',
            namespace=image_ns,
            parameters=[{'flip_mode': 'BOTH'}],
            remappings=[
                ('image', argus_raw_topic),
                ('image_flipped', raw_image_topic),
            ],
        ))

    pipeline_nodes.append(rectify_node)

    if output_encoding != 'rgb8':
        pipeline_nodes.append(ComposableNode(
            package='isaac_ros_image_proc',
            plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
            name='image_format_converter',
            namespace=image_ns,
            parameters=[{k: v for k, v in {
                'encoding_desired': output_encoding,
                'image_width': image_width,
                'image_height': image_height,
            }.items() if v is not None}],
            remappings=[
                ('image_raw', 'image_rect_color'),
                ('image', 'image_rect'),
            ],
        ))

    if image_ns in tag_config_map:
        config_info = tag_config_map[image_ns]
        apriltagger = ComposableNode(
            package='apriltag_ros',
            plugin='apriltag_ros::AprilTagNode',
            name=april_name,
            namespace=image_ns,
            parameters=[config_info['config']],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        pipeline_nodes.append(apriltagger)

    pipeline_container = ComposableNodeContainer(
        name='cypher_argus_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=pipeline_nodes,
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        env=env,
    )

    launch_actions = [argus_container, pipeline_container]

    if compress:
        # Republish image_rect as JPEG-compressed for low-bandwidth monitoring (e.g. Foxglove).
        # Taps the same topic apriltag reads; pipeline nodes are unaffected.
        # Output topic: <namespace>/image_rect/compressed
        launch_actions.append(Node(
            package='image_transport',
            executable='republish',
            name='image_compressor',
            namespace=image_ns,
            arguments=['raw', 'compressed'],
            remappings=[
                ('in', 'image_rect'),
                ('out/compressed', 'image_rect/compressed'),
            ],
            parameters=[{'compressed.jpeg_quality': 80}],
        ))

    return launch_actions


def generate_launch_description():
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='0',
        description='Index of the video device (e.g. 0 for /dev/video0)',
    )

    camera_mode_arg = DeclareLaunchArgument(
        'camera_mode',
        default_value='1',
        description='Camera mode index (0 -> IMX477_0.yaml, etc.)',
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

    output_encoding_arg = DeclareLaunchArgument(
        'output_encoding',
        default_value='mono8',
        description='Output encoding before rectification. Use "mono8" (default) or any '
                    'encoding supported by isaac_ros_image_proc (e.g. "rgb8").',
    )

    compress_arg = DeclareLaunchArgument(
        'compress',
        default_value='false',
        description='Publish a JPEG-compressed image_rect/compressed topic for low-bandwidth '
                    'monitoring (e.g. Foxglove). Does not affect the main pipeline.',
    )

    flip_image_arg = DeclareLaunchArgument(
        'flip_image',
        default_value='true',
        description='Rotate image 180° (GPU-accelerated) immediately after argus, before all '
                    'downstream processing. Default true for cam_down on new drone (camera '
                    'physically mounted 180° rotated about Z).',
    )

    setup = OpaqueFunction(function=generate_nodes)

    return LaunchDescription([
        video_device_arg,
        camera_mode_arg,
        image_ns_arg,
        frame_arg,
        framerate_arg,
        output_encoding_arg,
        compress_arg,
        flip_image_arg,
        setup,
    ])
