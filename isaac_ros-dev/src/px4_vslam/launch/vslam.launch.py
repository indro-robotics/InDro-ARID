import os

import launch
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():

    # Config file (vslam tuning + realsense params) ------------------------------
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    config = DeclareLaunchArgument(
        'camera_config_file',
        default_value=os.path.join(launch_dir, '..', 'config', 'vslam_config.yaml'),
        description='Path to config file'
    )

    config_file = LaunchConfiguration('camera_config_file')
    param_file = ParameterFile(config_file, allow_substs=True)

    env = dict(os.environ)
    for bad in ['DISPLAY', 'WAYLAND_DISPLAY']:
        env.pop(bad, None)
    ld = env.get('LD_LIBRARY_PATH', '')
    env['LD_LIBRARY_PATH'] = f"/opt/ros/humble/lib:{ld}" if ld else "/opt/ros/humble/lib"

    # Wait for the host-side arid_description to be running ----------------------
    # The host runs arid_description.service (robot_state_publisher) which latches
    # /robot_description and /tf_static. VSLAM needs those TF frames. We block the
    # rest of this launch until the latched /robot_description message is visible
    # on the DDS graph — `ros2 topic echo --once` with matching TRANSIENT_LOCAL QoS
    # exits immediately once the publisher is up.
    wait_for_description = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo',
             '/robot_description', 'std_msgs/msg/String',
             '--once',
             '--qos-durability', 'transient_local',
             '--qos-reliability', 'reliable'],
        output='log',   # URDF content is huge; don't spam the console
        name='wait_for_robot_description',
    )

    # Converts VIO solution to PX4 topic -----------------------------------------
    vio_transform_node = Node(
        name='vio_transform',
        namespace='vio_transform',
        package='px4_vslam',
        executable='vio_transform'
    )

    vslam_reactor_node = Node(
        package='px4_vslam_reactor',
        executable='vslam_reactor_node',
        name='vslam_reactor',
        output='screen'
    )

    vslam_container = ComposableNodeContainer(
        name='vslam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        env=env,
        composable_node_descriptions=[
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='left_realsense_link',
                namespace='left_realsense',
                parameters=[param_file],
            ),
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='front_realsense_link',
                namespace='front_realsense',
                parameters=[param_file],
                remappings=[
                    ('color/image_raw',  'image_raw'),
                    ('color/camera_info','camera_info'),
                ],
            ),
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='right_realsense_link',
                namespace='right_realsense',
                parameters=[param_file],
            ),
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam_node',
                parameters=[param_file],
                remappings=[('visual_slam/image_0', 'front_realsense/infra1/image_rect_raw'),
                    ('visual_slam/camera_info_0', 'front_realsense/infra1/camera_info'),
                    ('visual_slam/image_1', 'front_realsense/infra2/image_rect_raw'),
                    ('visual_slam/camera_info_1', 'front_realsense/infra2/camera_info'),
                    ('visual_slam/image_2', 'left_realsense/infra1/image_rect_raw'),
                    ('visual_slam/camera_info_2', 'left_realsense/infra1/camera_info'),
                    ('visual_slam/image_3', 'left_realsense/infra2/image_rect_raw'),
                    ('visual_slam/camera_info_3', 'left_realsense/infra2/camera_info'),
                    ('visual_slam/image_4', 'right_realsense/infra1/image_rect_raw'),
                    ('visual_slam/camera_info_4', 'right_realsense/infra1/camera_info'),
                    ('visual_slam/image_5', 'right_realsense/infra2/image_rect_raw'),
                    ('visual_slam/camera_info_5', 'right_realsense/infra2/camera_info'),
                    ('visual_slam/imu', 'vio_transform/imu')]
            )
        ]
    )

    return launch.LaunchDescription([
        config,
        LogInfo(msg='[vslam] Waiting for /robot_description from host-side arid_description...'),
        wait_for_description,
        RegisterEventHandler(OnProcessExit(
            target_action=wait_for_description,
            on_exit=[
                LogInfo(msg='[vslam] /robot_description detected — starting VSLAM stack'),
                vslam_container,
                vslam_reactor_node,
                vio_transform_node,
            ],
        )),
    ])
