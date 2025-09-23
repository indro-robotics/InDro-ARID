import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node
import xacro


def generate_launch_description():
    
    # this name has to match the robot name in the Xacro file
    robotXacroName='arid'
    
    # this is the name of our package, at the same time this is the name of the 
    # folder that will be used to define the paths
    namePackage = 'arid_drone_description'
    
    # this is a relative path to the xacro file defining the model
    modelFileRelativePath = 'models/arid/xacro/arid.xacro'
    
    # this is the absolute path to the model
    pathModelFile = os.path.join(get_package_share_directory(namePackage),modelFileRelativePath)
    robotDescription = xacro.process_file(pathModelFile).toxml()

    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription,
        'use_sim_time': True}] 
    )

    # here we create an empty launch description object
    ld = LaunchDescription()
     
    # we add gazeboLaunch 
    # launchDescriptionObject.add_action(gazeboLaunch)
    
    # # we add the two nodes
    # launchDescriptionObject.add_action(spawnModelNode)
    ld.add_action(robot_state_publisher_node)
    
    return ld