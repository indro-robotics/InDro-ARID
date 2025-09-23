#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/jetson/workspaces/local_ws/install/setup.bash
unset DISPLAY && ros2 launch gst_camera_info gst_camera_info_raw.launch.py vid_src:=1 calib_file:='IMX219_4K.yaml' camera_topic:='cam_front' framerate:=5 'visual_link':='top_visual_link'
