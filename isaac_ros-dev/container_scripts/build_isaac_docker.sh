#!/bin/bash
# Builds the container image and starts the container, behind the `build_isaac` alias.
# An empty ISAAC_ROS_WS passes `set -u`; `:?` rejects it before the path resolves from /.
set -eu

: "${ISAAC_ROS_WS:?ISAAC_ROS_WS must be exported - re-login or source ~/.bashrc first}"

"${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/run_dev.sh"
