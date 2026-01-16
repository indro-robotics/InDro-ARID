cmake_minimum_required(VERSION 3.8)
project(state_machine_interfaces)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Launch.srv"
  "srv/Land.srv"
  "srv/Cycle.srv"
  "srv/Panic.srv"
  "srv/Halt.srv"
  "srv/Forceland.srv"
  "srv/FMUreboot.srv"
)

ament_package()
