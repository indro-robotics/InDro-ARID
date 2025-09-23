import yaml
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch.actions import OpaqueFunction

def launch_setup(context, *args, **kwargs):

    resolved_calib_path = PathJoinSubstitution([
        get_package_share_directory('gst_camera_info'),
        'config',
        context.launch_configurations['calib_file']
    ]).perform(context)

    with open(resolved_calib_path, 'r') as f:
        cam_config = yaml.safe_load(f)

    width = cam_config['image_width']
    height = cam_config['image_height']

    gst_pipeline = (
        f"unset DISPLAY && gst-launch-1.0 --gst-plugin-path="
        f"{get_package_share_directory('gst_camera_info')}/../../../gst_bridge/lib/gst_bridge "
        f"nvarguscamerasrc sensor-id={context.launch_configurations['vid_src']} "
        f"wbmode=1 aelock=false ee-mode=1 tnr-mode=1 tnr-strength=0.5 ! "
        f"'video/x-raw(memory:NVMM),width={width},height={height},"
        f"framerate={context.launch_configurations['framerate']}/1,format=NV12' ! "
        f"queue leaky=downstream max-size-buffers=3 ! nvvidconv flip-method=0 "
        f"interpolation-method=1 ! 'video/x-raw,format=GRAY8' ! "
        f"queue leaky=downstream max-size-buffers=3 ! "
        f"rosimagesink sync=false enable-last-sample=false "
        f"ros-topic='/{context.launch_configurations['camera_topic']}/image_raw'"
    )

    synchron = Node(
        package='synchron',
        executable='synchron_node',
        namespace=context.launch_configurations['camera_topic'],
        parameters=[{'frame_id': context.launch_configurations['visual_link']}]
    )
    
    'bottom_visual_link'
    'top_visual_link'

    return [
        ExecuteProcess(
            cmd=['/bin/bash', '-c', gst_pipeline],
            output='screen'
        ),
        Node(
            package='gst_camera_info',
            executable='gst_camera_info',
            parameters=[{
                'calibration_file': resolved_calib_path,
                'camera_topic': context.launch_configurations['camera_topic'],
                'update_rate': float(context.launch_configurations['framerate'])
            }],
            output='screen'
        ),
        synchron  
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vid_src', default_value='0'),
        DeclareLaunchArgument('camera_topic', default_value='cam_down'),
        DeclareLaunchArgument('framerate', default_value='10'),
        DeclareLaunchArgument('calib_file', default_value='IMX219_2K.yaml'),
        DeclareLaunchArgument('visual_link', default_value='bottom_visual_link'),
        OpaqueFunction(function=launch_setup)
    ])
