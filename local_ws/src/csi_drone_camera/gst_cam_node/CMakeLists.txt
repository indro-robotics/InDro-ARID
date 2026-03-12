cmake_minimum_required(VERSION 3.5)
project(gst_cam_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)

add_executable(gst_cam_node
  src/gst_cam_node.cpp
)

target_include_directories(gst_cam_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(gst_cam_node
  rclcpp
  rclcpp_components
  sensor_msgs
  OpenCV
  camera_info_manager
  image_transport
)

install(TARGETS gst_cam_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
