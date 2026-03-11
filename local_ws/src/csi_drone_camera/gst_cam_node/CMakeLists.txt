cmake_minimum_required(VERSION 3.5)
project(gst_cam_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(camera_info_manager REQUIRED)

# Build shared library for composable node
add_library(gst_cam_node SHARED
  src/gst_cam_node.cpp
)

target_include_directories(gst_cam_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(gst_cam_node
  rclcpp
  ament_index_cpp
  rclcpp_components
  sensor_msgs
  OpenCV
  camera_info_manager  
)

rclcpp_components_register_nodes(gst_cam_node "GstCamNode")

install(TARGETS gst_cam_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_package()
