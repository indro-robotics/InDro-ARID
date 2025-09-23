#!/bin/bash
echo "/// Starting container... ///"
docker start isaac_ros_dev-aarch64-container
echo "/// Sourcing dependencies... ///"
docker exec isaac_ros_dev-aarch64-container "/workspaces/isaac_ros-dev/scripts/isaac_source.sh"
echo "/// Restarting services... ///"
docker exec isaac_ros_dev-aarch64-container /bin/bash -c "sudo service udev restart"

