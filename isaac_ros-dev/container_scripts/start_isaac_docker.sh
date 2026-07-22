#!/bin/bash
set -e

CONTAINER=isaac_ros_dev-aarch64-container

# If the container record does not exist (image never built, container manually
# removed), `docker start` exits non-zero and systemd loops Restart=on-failure
# forever. Probe first and exit cleanly with a clear log line so the systemd unit
# lands in "active (exited)" rather than churning.
if ! docker container inspect "${CONTAINER}" >/dev/null 2>&1; then
    echo "/// Container ${CONTAINER} does not exist - run build_isaac to create it. Exiting cleanly. ///" >&2
    exit 0
fi

echo "/// Starting container ${CONTAINER}... ///"
docker start "${CONTAINER}"