#!/bin/bash

# Set ROS domain for this container/session
export ROS_DOMAIN_ID=23

# Source setup so we can ros2 immediately (only if it exists)
if [ -f install/setup.bash ]; then
  source install/setup.bash
fi

# Append aliases to the per-user .bashrc
BASHRC="/home/${USERNAME}/.bashrc"

# Only add block once: check for an alias that should exist
if ! grep -q 'alias takeoff=' "$BASHRC" 2>/dev/null; then
cat >> "$BASHRC" << 'EOF'
alias takeoff='ros2 service call px4_state_machine/launch state_machine_interfaces/srv/Launch "{loiter_altitude: 1.5}"'
alias land='ros2 service call px4_state_machine/land state_machine_interfaces/srv/Land'
alias cycle='ros2 service call px4_state_machine/cycle state_machine_interfaces/srv/Cycle "{shelf_height: 3.5, scan_velocity: 0.20, shelf_distance: 1.0, amr_orientation: 45.0}"'
alias halt='ros2 service call px4_state_machine/halt state_machine_interfaces/srv/Halt'
alias fland='ros2 service call px4_state_machine/force_land state_machine_interfaces/srv/Forceland'
alias fmu_reboot='ros2 service call px4_state_machine/fmu_reboot state_machine_interfaces/srv/FMUreboot'
alias fkill='ros2 service call px4_state_machine/panic state_machine_interfaces/srv/Panic'
alias reset_usb='ros2 service call /reset_usb std_srvs/srv/Trigger "{}"'
alias rosdep_isaac='rosdep install --from-paths ${ISAAC_ROS_WS}/src/ --ignore-src -y'
alias colcon_isaac='cd ${ISAAC_ROS_WS} && colcon build --symlink-install --base-paths src && source ./install/setup.bash'
alias clean_isaac='cd ${ISAAC_ROS_WS} && colcon clean workspace --base-select build install log'
alias vslam='ros2 launch px4_vslam vslam.launch.py'
alias state_machine='ros2 launch px4_state_machine px4_state_machine.launch.py'
alias foxglove_bridge='ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765'
EOF
fi