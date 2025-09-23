cmake_minimum_required(VERSION 3.8)
project(gst_camera_interfaces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define interface files
set(interface_files
  "srv/ControlService.srv"
)

# Generate code for the interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${interface_files}
)

# Export runtime dependency
ament_export_dependencies(rosidl_default_runtime)
ament_package()
