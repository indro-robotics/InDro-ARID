cmake_minimum_required(VERSION 3.8)
project(px4_state_machine)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Specify the .srv files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Launch.srv"
  "srv/Land.srv"
  "srv/Cycle.srv"
  "srv/Force_Land.srv"
  "srv/Halt.srv"
  "srv/PX4_Reboot.srv"
  "srv/Panic.srv"
)

ament_package()
