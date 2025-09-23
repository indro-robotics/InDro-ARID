import os
import launch
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.actions import ComposableNodeContainer, Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Launch the URDF TF publisher
    package_name = 'cypher_drone_description'
    launch_file_name = 'cypher.launch.py'
    package_share_path = get_package_share_directory(package_name)
    launch_file_path = os.path.join(package_share_path, 'launch', launch_file_name)
    cypher_URDF_TFs = IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file_path))

    # Declare the config file parameter
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    config = DeclareLaunchArgument(
        'camera_config_file',
        default_value=os.path.join(launch_dir, '..', 'config', 'vslam_config.yaml'),
        description='Path to config file'
    )

    # Load parameters from the YAML file
    config_file = LaunchConfiguration('camera_config_file')
    param_file = ParameterFile(config_file, allow_substs=True)

    # Converts VIO solution to PX4 topic
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
        executable='component_container',
        output='screen',
        composable_node_descriptions=[
            # RealSense Nodes
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='left_realsense_link',
                namespace='left_realsense',
                parameters=[param_file]
            ),
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='front_realsense_link',
                namespace='front_realsense',
                parameters=[param_file]
            ),
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='right_realsense_link',
                namespace='right_realsense',
                parameters=[param_file]
            ),
            # Visual SLAM Node
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
        cypher_URDF_TFs,
        config,
        vslam_container,
        vslam_reactor_node,
        vio_transform_node,
        #LogInfo(msg=["Using camera configuration from: ", LaunchConfiguration('camera_config_file')])
    ])