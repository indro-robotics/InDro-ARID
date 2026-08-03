#!/bin/bash
# isaac_bash.sh - opens a bash shell inside the Isaac container.
# DISPLAY is passed only when the host has an X socket. Passing it otherwise gives the
# container a malformed ":" value and every Qt/gtk tool fails with "Can't open display ':'".
set -u

sock=$(ls /tmp/.X11-unix/X* 2>/dev/null | head -n1)
if [[ -n "${sock}" ]]; then
    num=${sock##*/X}
    # Strip everything after the digits (so an `X1005-lock` sibling can't poison DISPLAY).
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
