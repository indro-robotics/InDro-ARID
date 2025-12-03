#!/bin/bash

# Restart udev daemon
sudo service udev restart

# Source setup so we can ros2 immediately
source install/setup.bash

# Append aliases to the per-user .bashrc (works as admin user)
echo "
alias reset_usb='ros2 service call /reset_usb std_srvs/srv/Trigger \"{}\"'
alias camera_down='ros2 service call /start_camera gst_camera_interfaces/srv/ControlService \"{service_name: '\''cam_down_2k_20'\''}\"'
alias camera_front='ros2 service call /start_camera gst_camera_interfaces/srv/ControlService \"{service_name: '\''cam_front_4k_10'\''}\"'
alias camera_stop='ros2 service call /stop_camera gst_camera_interfaces/srv/ControlService \"{service_name: '\'''\''}\"'
alias rosdep_isaac='rosdep install --from-paths \${ISAAC_ROS_WS}/src/ --ignore-src -y'
alias colcon_isaac='cd \${ISAAC_ROS_WS} && colcon build --symlink-install --base-paths src && source ./install/setup.bash'
alias clean_isaac='cd \${ISAAC_ROS_WS} && colcon clean workspace --base-select build install log'
alias vslam='ros2 launch px4_vslam vslam.launch.py'
alias foxglove_bridge='ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765'
" >> /home/admin/.bashrc

$@