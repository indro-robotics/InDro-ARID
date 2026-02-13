#!/bin/bash
set -e

CONTAINER=isaac_ros_dev-aarch64-container

echo "/// Starting container ${CONTAINER}... ///"
docker start "${CONTAINER}" >/dev/null 2>&1 || true
