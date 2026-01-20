#!/bin/bash

BASHRC_FILE="${HOME}/.bashrc"
ROS_SOURCE="source /opt/ros/humble/setup.bash"
ISAAC_SOURCE="source /workspaces/isaac_ros-dev/install/setup.bash"
ROSDEP_EXPORT="export ROS_PACKAGE_PATH=/workspaces/isaac_ros-dev/src:\$ROS_PACKAGE_PATH"

# Function to append to .bashrc if not already present
append_if_not_exists() {
    local line="$1"
    if ! grep -qF "$line" "$BASHRC_FILE"; then
        echo "$line" | sudo tee -a "$BASHRC_FILE" > /dev/null
    fi
}

append_if_not_exists "$ROS_SOURCE"
append_if_not_exists "$ISAAC_SOURCE"
append_if_not_exists "$ROSDEP_EXPORT"
