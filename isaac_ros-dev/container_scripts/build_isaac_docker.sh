#!/bin/bash
# Builds (and enters) the Isaac container via run_dev.sh.
# Fail fast on empty ISAAC_ROS_WS: it would silently resolve run_dev.sh from /.
set -eu

: "${ISAAC_ROS_WS:?ISAAC_ROS_WS must be exported - re-login or source ~/.bashrc first}"

"${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/run_dev.sh"
