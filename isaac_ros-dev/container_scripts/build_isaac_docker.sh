#!/bin/bash
# build_isaac_docker.sh - builds and enters the Isaac container via run_dev.sh.
# Without the guard an unset ISAAC_ROS_WS collapses the path to
# /src/isaac_ros_common/scripts/run_dev.sh, which exits 127 having built nothing.
set -eu

: "${ISAAC_ROS_WS:?ISAAC_ROS_WS must be exported - re-login or source ~/.bashrc first}"

"${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/run_dev.sh"
