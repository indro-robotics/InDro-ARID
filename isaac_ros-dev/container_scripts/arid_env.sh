#!/bin/bash

export ROS_DOMAIN_ID=23
export ROS_LOCALHOST_ONLY=1
export ISAAC_ROS_WS=/workspaces/isaac_ros-dev
export ROS_PACKAGE_PATH=${ISAAC_ROS_WS}/src:$ROS_PACKAGE_PATH

source /opt/ros/humble/setup.bash && source "${ISAAC_ROS_WS}/install/setup.bash"

_reset_usb_up() {
    ros2 service list 2>/dev/null | grep -q '^/reset_usb$' && return 0
    echo "ERROR: /reset_usb service not found (host-side reset_usb.service may be down)." >&2
    return 1
}
_supervisor_up() {
    ros2 service list 2>/dev/null | grep -q '^/arid_supervisor/status$' && return 0
    echo "ERROR: arid_supervisor not running. Check 'sudo systemctl status arid_supervisor.service' on the host." >&2
    return 1
}

alias reset_usb='_reset_usb_up && ros2 service call /reset_usb std_srvs/srv/Trigger "{}"'
# skip-keys librealsense2: the source realsense2_camera declares it, and rosdep resolving it
# apt-installs ros-humble-librealsense2 (v4l2, no HW metadata), which then links into overlay
# builds. The workspace uses the RSUSB build at /usr/local instead.
alias rosdep_isaac='{ sudo apt update || true; } && rosdep install --from-paths ${ISAAC_ROS_WS}/src/ --ignore-src -y --skip-keys librealsense2'
# -Drealsense2_DIR pin: forces find_package(realsense2) to the RSUSB 2.55.1 build at
# /usr/local even when apt ros-humble-librealsense2 (v4l2, no metadata) is installed.
alias colcon_isaac='cd ${ISAAC_ROS_WS} && colcon build --symlink-install --base-paths src --cmake-args -DBUILD_TESTING=OFF -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 && source ./install/setup.bash'
alias clean_isaac='cd ${ISAAC_ROS_WS} && colcon clean workspace --base-select build install log'
alias vslam='ros2 launch px4_vslam vslam.launch.py'
alias initialize='_supervisor_up && /bin/bash ${ISAAC_ROS_WS}/container_scripts/initialize.sh'
alias status='_supervisor_up && ros2 service call /arid_supervisor/status std_srvs/srv/Trigger "{}"'
alias deinitialize='_supervisor_up && /bin/bash ${ISAAC_ROS_WS}/container_scripts/deinitialize.sh'
alias foxglove_bridge='ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765'

help() {
    cat <<'ARIDHELP'
ARID container commands:

  System
    initialize       Enable VSLAM via the supervisor (camera-proven, SetBool true)
    deinitialize     Disable VSLAM via the supervisor (refused unless drone is landed)
    status           Supervisor status - vslam running (true/false) + land state
    vslam            Direct VSLAM launch - bypasses the supervisor (development use only)
    rosdep_isaac     Install rosdep deps for isaac_ros-dev
    colcon_isaac     Build isaac_ros-dev
    clean_isaac      Clean isaac_ros-dev
    foxglove_bridge  Foxglove bridge on port 8765
    reset_usb        USB reset (ROS service)
ARIDHELP
}
