#!/bin/bash
# colcon_local - rebuild local_ws with its dependent host services stopped, then restart them
# (a running service holds the old install until restarted). jetson-clocks, usbfs-memory, and
# reset_usb do not depend on local_ws and are left running.
# The `colcon_local` alias sources the fresh install into the caller's shell on success.
set -uo pipefail

LOCAL_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/local_ws"

SERVICES=(
    gst_camera_manager
    arid_description
    rslidar_coordinator
    usb_ros_reset
)

echo "==> stopping local_ws services"
for s in "${SERVICES[@]}"; do
    echo "    stop ${s}.service"
    sudo systemctl stop "${s}.service" || true
done

echo "==> building local_ws"
cd "${LOCAL_WS}" && colcon build --symlink-install --base-paths src
rc=$?

echo "==> restarting local_ws services"
for s in "${SERVICES[@]}"; do
    echo "    start ${s}.service"
    sudo systemctl start "${s}.service" || echo "    WARN: ${s}.service did not start (check journalctl -u ${s}.service)"
done

if [[ ${rc} -ne 0 ]]; then
    echo "colcon_local: BUILD FAILED (rc=${rc}); services restarted on the previous install." >&2
fi
exit ${rc}
