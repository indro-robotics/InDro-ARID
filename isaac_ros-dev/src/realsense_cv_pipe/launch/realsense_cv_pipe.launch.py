import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_nodes(context, *args, **kwargs):
    input_ns  = LaunchConfiguration('input_namespace').perform(context)
    output_ns = LaunchConfiguration('output_namespace').perform(context)

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

    composable_nodes = []

    rectify_node = ComposableNode(
        package="isaac_ros_image_proc",
        plugin="nvidia::isaac_ros::image_proc::RectifyNode",
        name="realsense_image_rectify",
        namespace=input_ns,
        remappings=[
            ('image_rect', f'/{output_ns}/image_rect'),
            ('camera_info_rect', f'/{output_ns}/camera_info_rect'),
        ],
    )
    composable_nodes.append(rectify_node)

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

    container = ComposableNodeContainer(
        name="realsense_cv_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=composable_nodes,
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],
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

    setup = OpaqueFunction(function=generate_nodes)

    return LaunchDescription([
        input_ns_arg,
        output_ns_arg,
        setup,
    ])
