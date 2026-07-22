#!/bin/bash
# colcon_local - rebuild local_ws with its dependent host services stopped, then restart them.
#
# Building while the nodes run leaves half-updated processes: a running service holds the old
# install until it is restarted. Every service below sources local_ws/install and runs a
# local_ws node (arid_description, the camera manager, the USB reset service), so each is stopped
# for the build and restarted afterwards to pick up the new install. The sysctl-style units
# (jetson-clocks, usbfs-memory) and reset_usb (runs scripts/usb_reset.sh, not local_ws code)
# do NOT depend on local_ws and are left running.
#
# Invoked by the `colcon_local` alias, which sources the fresh install into the caller's shell
# on success. Assumes a bench/ground context (do not rebuild while flying).
set -uo pipefail

LOCAL_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/local_ws"

SERVICES=(
    gst_camera_manager
    arid_description
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
