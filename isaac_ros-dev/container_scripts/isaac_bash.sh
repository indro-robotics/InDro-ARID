sock=$(ls /tmp/.X11-unix/X* 2>/dev/null | head -n1)
num=${sock##*/X}
DISPLAY=":${num}"

docker exec -it \
  -u admin \
  -e DISPLAY="$DISPLAY" \
  isaac_ros_dev-aarch64-container \
  bash -i -c "source /etc/profile.d/cypher_env.sh && exec bash"