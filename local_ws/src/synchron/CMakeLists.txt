# cmake_minimum_required(VERSION 3.8)
# project(synchron)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(message_filters REQUIRED)
# find_package(rclcpp_components REQUIRED)

# # Change this from add_executable to add_library
# add_library(synchron_node SHARED
#   src/synchron_node.cpp)

# # Add this line to properly export the node as a plugin
# target_compile_definitions(synchron_node PRIVATE "COMPOSITION_BUILDING_DLL")

# ament_target_dependencies(synchron_node
#   rclcpp
#   rclcpp_components
#   sensor_msgs
#   message_filters
# )

# # Install the library instead of the executable
# install(TARGETS
#   synchron_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)

# # Register the node
# rclcpp_components_register_nodes(synchron_node "synchron::SynchronNode")

# ament_package()
cmake_minimum_required(VERSION 3.8)
project(synchron)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp_components REQUIRED)

# Component library (for container usage)
add_library(synchron_component SHARED
  src/synchron_node.cpp
)
target_compile_definitions(synchron_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(synchron_component
  rclcpp
  message_filters
  sensor_msgs
  rclcpp_components
)

# Standalone executable (for direct execution)
add_executable(synchron_node
  src/synchron_node.cpp
)
ament_target_dependencies(synchron_node
  rclcpp
  message_filters
  sensor_msgs
)

# Installation
install(TARGETS 
  synchron_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS synchron_node
  DESTINATION lib/${PROJECT_NAME}
)

# Component registration
rclcpp_components_register_nodes(synchron_component "synchron::SynchronNode")

ament_package()
