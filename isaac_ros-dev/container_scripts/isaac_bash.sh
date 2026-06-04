#!/bin/bash
# Drop into a bash shell inside the Isaac container. If the host has an X socket (NoMachine
# or local console), forward DISPLAY so GUI tools work; otherwise omit -e DISPLAY entirely
# so the container does not inherit a malformed ":" value that makes every Qt/gtk tool fail
# with "Can't open display ':'".
set -u

sock=$(ls /tmp/.X11-unix/X* 2>/dev/null | head -n1)
if [[ -n "${sock}" ]]; then
    num=${sock##*/X}
    # Strip everything after the digits so an `X1005-lock` sibling cannot poison DISPLAY.
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
