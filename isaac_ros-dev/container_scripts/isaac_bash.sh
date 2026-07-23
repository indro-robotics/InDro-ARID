#!/bin/bash
# Bash shell inside the Isaac container. Forward DISPLAY only when a real X socket exists:
# a malformed ":" value makes every Qt/gtk tool fail.
set -u

sock=$(ls /tmp/.X11-unix/X* 2>/dev/null | head -n1)
if [[ -n "${sock}" ]]; then
    num=${sock##*/X}
    # Strip everything after the digits so an `X1005-lock` sibling cannot corrupt DISPLAY.
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
