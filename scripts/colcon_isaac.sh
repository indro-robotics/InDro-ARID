#!/bin/bash
# colcon_isaac - deinitialize VSLAM, rebuild the in-container workspace, restart the supervisor.
# deinitialize is safety-gated (refused unless landed), so it also blocks a build while airborne.
# VSLAM is deliberately NOT auto-restarted: `initialize` is the operator's explicit action.
set -uo pipefail

WORKSPACES="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IN_ISAAC="${WORKSPACES}/scripts/in_isaac.sh"

echo "==> tearing down the VSLAM stack (deinitialize)"
if ! "${IN_ISAAC}" deinitialize; then
    echo "colcon_isaac: aborting - could not confirm the VSLAM stack is stopped (see above)." >&2
    echo "  drone not landed? land first.   supervisor down? sudo systemctl start arid_supervisor.service" >&2
    exit 1
fi

echo "==> building isaac_ros-dev in the container"
"${IN_ISAAC}" colcon_isaac
rc=$?
if [[ ${rc} -ne 0 ]]; then
    echo "colcon_isaac: BUILD FAILED (rc=${rc}); supervisor left running old code, stack stays down." >&2
    exit ${rc}
fi

echo "==> restarting arid_supervisor.service on the fresh build"
sudo systemctl restart arid_supervisor.service

echo "colcon_isaac: done. Run 'initialize' when ready to bring up VSLAM."
