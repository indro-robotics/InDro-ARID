#!/bin/bash
ISAAC_ROS_WS="${HOME}/workspaces/isaac_ros-dev"
echo "enable system service for max performance + fans"
sudo cp -f ${ISAAC_ROS_WS}/scripts/jetson-clocks.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl restart jetson-clocks.service
