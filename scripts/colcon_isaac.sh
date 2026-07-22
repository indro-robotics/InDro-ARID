#!/bin/bash
# colcon_isaac - rebuild the in-container isaac_ros-dev workspace with the VSLAM stack torn
# down first, then bring the supervisor back on the fresh build.
#
# Sequence:
#   1) deinitialize - stop the VSLAM stack via the supervisor. Safety-gated (refused unless the
#      drone is landed), so it also blocks a build while airborne. Succeeds idempotently if the
#      stack was never initialized (supervisor up, stack already down).
#   2) colcon_isaac - build inside the container (the pure-build container alias).
#   3) restart arid_supervisor.service - relaunch the supervisor on the freshly-built code so
#      its enable-services are ready for the next `initialize`.
#
# The VSLAM stack is deliberately NOT auto-started: `initialize` is the operator's explicit,
# safety-gated action. Assumes a bench/ground context.
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
