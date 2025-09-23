# CYPHER DRONE LOCAL WORKSPACE
**NOTE:** Interaction with the local_ws software can only be performed in ROS DOMAIN 23.

## Functionality
**Camera Manager Service -** Manages a single GST pipeline for either the front-facing or down-facing cameras and publishes the raw image as a ROS2 topic with a synchronized camera_info topic. Automatically starts on boot.

**USB Reset Service -** Enables the power cycling of all ARK PAB Carrier board USB ports via ROS2 service call. Automatically power cycles USB ports on system reboot.

**UWB Drone Node -** Enables health-monitoring and reset of the local and remote-AMR UWB nodes in the case of ranging quality issues. Interfaces with PX4 rangefinder for redundancy. Automatically starts on boot.

## Installation
Clone the repository:
```
LOCAL_WS="${HOME}/workspaces/local_ws"
git clone --recurse-submodules https://github.com/indro-robotics/cypher_drone_local_ws.git ${LOCAL_WS}
```

Set permissions and run the setup script:
```
chmod u+x ${LOCAL_WS}/scripts/setup.sh
. ${LOCAL_WS}/scripts/setup.sh
```

Replace default calibration files with your own .yaml calibration files at:
```
${LOCAL_WS}/src/csi_drone_cmamera/gst_camera_info/config
```

## Camera Manager
### Direct Use

To start a camera via ros2 launch:
```
ros2 launch gst_camera_info gst_camera_info_raw.launch.py camera_topic:=<image-topic-name> vid_src:=<device-number> calib_file:=<calibration-filename> framerate:=<desired-fps>
```
e.g.
```
ros2 launch gst_camera_info gst_camera_info_raw.launch.py camera_topic:='cam_front' vid_src:=0 calib_file:='IMX219_2K.yaml' framerate:=30
```

### Camera Manager Service Service-Based Use

The **cam_manager** service allows you to start a a single camera at fixed resolution+framerate. If you have run the setup script, this should be running as a system service on boot.

Manual Camera Manager start via systemctl (standard systemctl controls):
```
sudo systemctl start cam_manager.service
```

The **/start-camera** service call can be used to start a pre-defined capture pipeline:
```
ros2 service call /start_camera gst_camera_interfaces/srv/ControlService "{service_name: 'cam_front_4k_10'}"
```

The **/stop_camera** service call will attempt to stop any of pre-defined pipelines if they are running (the 'service_name' field is arbitrary but must exist).
```
ros2 service call /stop_camera gst_camera_interfaces/srv/ControlService "{service_name: ''}"
```

Predefined services include:  
'cam_front_4k_10' -- front-facing camera, 4K @ 10FPS  
'cam_front_4k_5' -- front-facing camera, 4K @ 5FPS  
'cam_down_2k_20' -- downwards-facing camera, 2K @ 20FPS  
'cam_down_2k_30' -- downwards-facing camera, 2K @ 30FPS  

## USB Reset
To reset all USB ports at any time:
```
ros2 service call /reset_usb std_srvs/srv/Trigger
```
