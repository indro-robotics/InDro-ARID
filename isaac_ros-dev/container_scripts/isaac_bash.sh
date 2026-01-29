sock=$(ls /tmp/.X11-unix/X* 2>/dev/null | head -n1)
num=${sock##*/X}
DISPLAY=":${num}"

docker exec -it \
  -e DISPLAY="$DISPLAY" \
  isaac_ros_dev-aarch64-container \
  /bin/bash -c "source ~/.bashrc && exec /bin/bash"