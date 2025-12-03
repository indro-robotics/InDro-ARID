#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/jetson/workspaces/local_ws/install/setup.bash
unset DISPLAY && ros2 launch gst_camera_info gst_camera_info_raw.launch.py vid_src:=0 calib_file:='IMX219_2K.yaml' camera_topic:='cam_down' framerate:=30 'visual_link':='bottom_visual_link'