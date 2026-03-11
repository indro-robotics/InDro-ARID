import os

# LAUNCH
# ros2 launch realsense_cv_pipe realsense_cv_pipe.launch.py \
#   input_namespace:=front_realsense output_namespace:=cam_front \
#   output_encoding:=mono8 image_width:=1280 image_height:=800

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_nodes(context, *args, **kwargs):
    input_ns  = LaunchConfiguration('input_namespace').perform(context)
    output_ns = LaunchConfiguration('output_namespace').perform(context)
    output_encoding = LaunchConfiguration('output_encoding').perform(context)
    image_width  = int(LaunchConfiguration('image_width').perform(context))
    image_height = int(LaunchConfiguration('image_height').perform(context))

    if not input_ns:
        input_ns = 'front_realsense'
    if not output_ns:
        output_ns = 'cam_front'

    env = dict(os.environ)
    for bad in ['DISPLAY', 'WAYLAND_DISPLAY']:
        env.pop(bad, None)

    ld = env.get('LD_LIBRARY_PATH', '')
    env['LD_LIBRARY_PATH'] = f"/opt/ros/humble/lib:{ld}" if ld else "/opt/ros/humble/lib"

    pkg_april_share = get_package_share_directory('apriltag_ros')
    tag_config_map = {
        'cam_down': {
            'config': os.path.join(pkg_april_share, 'cfg', 'amr_tags.yaml'),
        },
        'cam_front': {
            'config': os.path.join(pkg_april_share, 'cfg', 'shelf_tags.yaml'),
        },
    }

    # Pipeline order: rectify → format_converter → apriltag
    # Rectify reads color/image_raw from RealSense driver, outputs image_rect_color (intermediate).
    # Format converter converts image_rect_color to target encoding on /{output_ns}/image_rect.
    # For rgb8: rectify outputs directly to /{output_ns}/image_rect, no converter needed.

    rect_image_out = 'image_rect_color' if output_encoding != 'rgb8' else f'/{output_ns}/image_rect'

    rectify_node = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        name='realsense_image_rectify',
        namespace=input_ns,
        parameters=[{
            'output_width': image_width,
            'output_height': image_height,
            'input_qos': 'SENSOR_DATA',
        }],
        remappings=[
            ('image_raw',         'image_raw'),
            ('camera_info',       'camera_info'),
            ('image_rect',        rect_image_out),
            ('camera_info_rect',  f'/{output_ns}/camera_info_rect'),
        ],
    )

    composable_nodes = [rectify_node]

    if output_encoding != 'rgb8':
        composable_nodes.append(ComposableNode(
            package='isaac_ros_image_proc',
            plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
            name='image_format_converter',
            namespace=input_ns,
            parameters=[{
                'encoding_desired': output_encoding,
                'image_width':  image_width,
                'image_height': image_height,
            }],
            remappings=[
                ('image_raw', 'image_rect_color'),
                ('image',     f'/{output_ns}/image_rect'),
            ],
        ))

    if output_ns in tag_config_map:
        config_info = tag_config_map[output_ns]
        april_name = 'april_' + output_ns

        apriltagger = ComposableNode(
            package='apriltag_ros',
            plugin='apriltag_ros::AprilTagNode',
            name=april_name,
            namespace=output_ns,
            parameters=[config_info['config']],
            remappings=[
                ('image_rect', 'image_rect'),
                ('camera_info', 'camera_info_rect'),
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        )
        composable_nodes.append(apriltagger)

    # Heartbeat: subscribes to image_rect, publishes std_msgs/Empty on
    # /<output_ns>/heartbeat at the same rate — allows node_manager to confirm the full
    # NITROS chain is healthy without subscribing to the full image topic cross-process.
    composable_nodes.append(ComposableNode(
        package='pipeline_health',
        plugin='PipelineHeartbeatNode',
        name='pipeline_heartbeat',
        namespace=output_ns,
        parameters=[{'watch_topic': 'image_rect'}],
    ))

    container = ComposableNodeContainer(
        name='realsense_cv_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=composable_nodes,
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
        env=env,
    )

    return [container]


def generate_launch_description():
    input_ns_arg = DeclareLaunchArgument(
        'input_namespace',
        default_value='front_realsense',
        description='Namespace where the RealSense camera publishes topics.',
    )

    output_ns_arg = DeclareLaunchArgument(
        'output_namespace',
        default_value='cam_front',
        description='Namespace for rectified image topics and AprilTag processing.',
    )

    output_encoding_arg = DeclareLaunchArgument(
        'output_encoding',
        default_value='mono8',
        description='Output encoding. Use "mono8" (default) or "rgb8" to skip conversion.',
    )

    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='1280',
        description='Width of the RealSense color image.',
    )

    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='800',
        description='Height of the RealSense color image.',
    )

    setup = OpaqueFunction(function=generate_nodes)

    return LaunchDescription([
        input_ns_arg,
        output_ns_arg,
        output_encoding_arg,
        image_width_arg,
        image_height_arg,
        setup,
    ])
