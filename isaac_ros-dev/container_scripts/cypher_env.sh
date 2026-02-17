#!/bin/bash

export ROS_DOMAIN_ID=23
export ISAAC_ROS_WS=/workspaces/isaac_ros-dev
export ROS_PACKAGE_PATH=${ISAAC_ROS_WS}/src:$ROS_PACKAGE_PATH

source /opt/ros/humble/setup.bash && source "${ISAAC_ROS_WS}/install/setup.bash"

alias takeoff='ros2 service call px4_state_machine/launch state_machine_interfaces/srv/Launch "{loiter_altitude: 1.5}"'
alias land='ros2 service call px4_state_machine/land state_machine_interfaces/srv/Land'
alias cycle='ros2 service call px4_state_machine/cycle state_machine_interfaces/srv/Cycle "{shelf_height: 3.5, scan_velocity: 0.20, shelf_distance: 1.0, amr_orientation: 45.0}"'
alias halt='ros2 service call px4_state_machine/halt state_machine_interfaces/srv/Halt'
alias fland='ros2 service call px4_state_machine/force_land state_machine_interfaces/srv/Forceland'
alias fmu_reboot="ros2 service call px4_state_machine/fmu_reboot state_machine_interfaces/srv/FMUreboot"
alias fkill='ros2 service call px4_state_machine/panic state_machine_interfaces/srv/Panic'
alias reset_usb='ros2 service call /reset_usb std_srvs/srv/Trigger "{}"'
alias rosdep_isaac='sudo apt update && rosdep install --from-paths ${ISAAC_ROS_WS}/src/ --ignore-src -y'
alias colcon_isaac='cd ${ISAAC_ROS_WS} && colcon build --symlink-install --base-paths src --cmake-args -DBUILD_TESTING=OFF && source ./install/setup.bash'
alias vslam='ros2 launch px4_vslam vslam.launch.py'
alias state_machine='ros2 launch px4_state_machine px4_state_machine.launch.py'
alias foxglove_bridge='ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765'