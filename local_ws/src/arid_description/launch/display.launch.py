# arid_description.service runs this at boot with no arguments.
# The container's px4_vslam launch blocks on the latched /robot_description published here, so
# VSLAM bringup stalls while this unit is down.

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('arid_description')
    xacro_path = os.path.join(pkg_share, 'xacro', 'arid.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'arid.rviz')

    robot_desc = xacro.process_file(xacro_path).toxml()

    use_gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false',
                              description='Use joint_state_publisher_gui'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Launch RViz'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),

        Node(
            condition=IfCondition(use_gui),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
