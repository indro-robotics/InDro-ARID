from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    rectify_node = ComposableNode(package="isaac_ros_image_proc",
                                  plugin="nvidia::isaac_ros::image_proc::RectifyNode",
                                  name="realsense_image_rectify",
                                  namespace= 'front_realsense')

    container = ComposableNodeContainer(name="realsense_cv_container",
                                        namespace="",
                                        package="rclcpp_components",
                                        executable="component_container_mt",
                                        composable_node_descriptions=[rectify_node],
                                        output="screen",
                                        arguments=["--ros-args", "--log-level", "INFO"])

    return LaunchDescription([container])