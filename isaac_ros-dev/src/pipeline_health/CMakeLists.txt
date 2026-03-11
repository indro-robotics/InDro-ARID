cmake_minimum_required(VERSION 3.5)
project(pipeline_health)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(pipeline_heartbeat_node SHARED
  src/pipeline_heartbeat_node.cpp
)

ament_target_dependencies(pipeline_heartbeat_node
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
)

rclcpp_components_register_nodes(pipeline_heartbeat_node "PipelineHeartbeatNode")

install(TARGETS pipeline_heartbeat_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
