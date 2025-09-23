cmake_minimum_required(VERSION 3.8)
project(gst_camera_info)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(yaml-cpp REQUIRED)

# Declare the executable
add_executable(gst_camera_info src/gst_camera_info.cpp)

# Specify include directories
target_include_directories(gst_camera_info PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Link the executable to the required libraries
ament_target_dependencies(gst_camera_info
  rclcpp
  sensor_msgs
  camera_info_manager
  yaml-cpp
)

# Install the executable
install(TARGETS gst_camera_info
  DESTINATION lib/${PROJECT_NAME})

# Install launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
