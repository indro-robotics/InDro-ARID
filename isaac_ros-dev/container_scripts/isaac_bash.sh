#!/bin/bash
# Interactive shell inside the Isaac container, behind the `isaac_bash` alias. DISPLAY is
# forwarded only when a real X socket exists: a bare ":" value makes every Qt/GTK tool fail.
set -u

sock=$(ls /tmp/.X11-unix/X* 2>/dev/null | head -n1)
if [[ -n "${sock}" ]]; then
    num=${sock##*/X}
    # DISPLAY takes digits only; an X<n>-lock entry matched by the glob otherwise carries
    # its suffix into the value.
    num="${num%%[!0-9]*}"
fi

if [[ -n "${num:-}" ]]; then
    exec docker exec -it \
        -u admin \
        -e DISPLAY=":${num}" \
        isaac_ros_dev-aarch64-container \
        bash
else
    exec docker exec -it \
        -u admin \
        isaac_ros_dev-aarch64-container \
        bash
fi
