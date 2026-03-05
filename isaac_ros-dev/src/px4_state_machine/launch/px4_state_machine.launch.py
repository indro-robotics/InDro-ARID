#!/usr/bin/env python
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
        april_tracker = Node(
            package='april_targeting',
            executable='april_tracker_node',
            name='april_tracker',
            output='screen',
            emulate_tty=True 
        )

        px4_controller = Node(
            package='px4_state_machine',
            namespace='px4_state_machine',
            executable='px4_state_control',
        )

        node_manager = Node(
            package='node_manager',
            executable='node_manager',
            name='node_manager',
            output='screen',
            emulate_tty=True
        )

        return LaunchDescription([april_tracker,
                                  px4_controller,
                                  node_manager])
