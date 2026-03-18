from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    overlap = LaunchConfiguration('overlap')
    distanceToWall = LaunchConfiguration('distanceToWall')
    vfov_deg = LaunchConfiguration('vfov_deg')

    rc_trigger_channel = LaunchConfiguration('rc_trigger_channel')
    rc_trigger_threshold_us = LaunchConfiguration('rc_trigger_threshold_us')

    return LaunchDescription([
        DeclareLaunchArgument('overlap', default_value='0.2'),
        DeclareLaunchArgument('distanceToWall', default_value='0.70'),
        DeclareLaunchArgument('vfov_deg', default_value='75.0'),

        DeclareLaunchArgument('rc_trigger_channel', default_value='7'),
        DeclareLaunchArgument('rc_trigger_threshold_us', default_value='2000'),

        Node(
            package='rc_trigger',
            executable='rc_input_listener',
            name='rc_input_listener',
            output='screen',
            parameters=[
                {'overlap': overlap},
                {'distanceToWall': distanceToWall},
                {'vfov_deg': vfov_deg},
                {'rc_trigger_channel': rc_trigger_channel},
                {'rc_trigger_threshold_us': rc_trigger_threshold_us},
            ],
        ),
    ])

