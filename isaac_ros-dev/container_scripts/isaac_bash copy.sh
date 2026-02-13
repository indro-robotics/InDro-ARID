sock=$(ls /tmp/.X11-unix/X* 2>/dev/null | head -n1)
num=${sock##*/X}
DISPLAY=":${num}"

docker exec -it \
  -u admin \
  -e DISPLAY="$DISPLAY" \
  isaac_ros_dev-aarch64-container \
  bash -ic 'source "$ISAAC_ROS_WS/install/setup.bash" 2>/dev/null || true; source "$ISAAC_ROS_WS/isaac_aliases.sh" 2>/dev/null || true; exec bash'