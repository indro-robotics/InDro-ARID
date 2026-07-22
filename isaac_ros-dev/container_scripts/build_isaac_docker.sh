#!/bin/bash
# Builds (and enters) the Isaac container via run_dev.sh. Must run from a shell that has
# ${ISAAC_ROS_WS} exported. An empty value would silently invoke /src/isaac_ros_common/scripts/run_dev.sh
# from / and either no-op or exit 127. Fail fast instead.
set -eu

: "${ISAAC_ROS_WS:?ISAAC_ROS_WS must be exported - re-login or source ~/.bashrc first}"

"${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/run_dev.sh"
