#!/bin/bash

export ROS_DOMAIN_ID=23
export ISAAC_ROS_WS=/workspaces/isaac_ros-dev
export ROS_PACKAGE_PATH=${ISAAC_ROS_WS}/src:$ROS_PACKAGE_PATH

source /opt/ros/humble/setup.bash && source "${ISAAC_ROS_WS}/install/setup.bash"

alias reset_usb='ros2 service call /reset_usb std_srvs/srv/Trigger "{}"'
alias rosdep_isaac='sudo apt update && rosdep install --from-paths ${ISAAC_ROS_WS}/src/ --ignore-src -y'
alias colcon_isaac='cd ${ISAAC_ROS_WS} && colcon build --symlink-install --base-paths src --cmake-args -DBUILD_TESTING=OFF && source ./install/setup.bash'
alias vslam='ros2 launch px4_vslam vslam.launch.py'
alias foxglove_bridge='ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765'