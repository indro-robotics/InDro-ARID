## SYNCHRON: Synchronizes 'camera_info' messages with 'image_raw' images

| Subscribed Topics | Interface |
| --------- | --------- |
| `image_raw` | [`sensor_msgs/Image`] (https://docs.ros.org/en/ros2_packages/humble/api/sensor_msgs/msg/Image.html) | 
| `camera_info` | [`sensor_msgs/CameraInfo`] (https://docs.ros.org/en/ros2_packages/humble/api/sensor_msgs/msg/CameraInfo.html) |

| Published Topics | Interface |
| --------- | --------- |
| `synced_camera_info` | [`sensor_msgs/Image`] (https://docs.ros.org/en/ros2_packages/humble/api/sensor_msgs/msg/Image.html) |

| Parameters | Interface |
| --------- | --------- |
| ` slop_time_sec` | [`seconds`] | 
| `max_interval_sec` | [`seconds`] |
