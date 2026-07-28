cmake_minimum_required(VERSION 3.5)
project(vslam_sentry)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# realsense2: resolved to the RSUSB build at /usr/local via the colcon_isaac
# -Drealsense2_DIR pin (same linkage rule as the camera driver).
find_package(realsense2 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(vslam_sentry_node src/vslam_sentry_node.cpp)
ament_target_dependencies(vslam_sentry_node
  rclcpp std_msgs std_srvs sensor_msgs nav_msgs)
target_link_libraries(vslam_sentry_node ${realsense2_LIBRARY} yaml-cpp)
target_include_directories(vslam_sentry_node PRIVATE ${realsense2_INCLUDE_DIR})

install(TARGETS vslam_sentry_node DESTINATION lib/${PROJECT_NAME})

ament_package()
