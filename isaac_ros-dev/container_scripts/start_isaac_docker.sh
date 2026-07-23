#!/bin/bash
set -e

CONTAINER=isaac_ros_dev-aarch64-container

# A missing container record makes `docker start` fail and systemd restart-loop indefinitely;
# probe first and exit cleanly.
if ! docker container inspect "${CONTAINER}" >/dev/null 2>&1; then
    echo "/// Container ${CONTAINER} does not exist - run build_isaac to create it. Exiting cleanly. ///" >&2
    exit 0
fi

echo "/// Starting container ${CONTAINER}... ///"
docker start "${CONTAINER}"