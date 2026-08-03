# ros_gst_cameras

This directory holds two packages that publish the CSI camera as a ROS 2 camera stream.

- `gst_cam_node`: wraps one GStreamer pipeline ending in `appsink` and publishes image plus `camera_info`.
- `gst_camera_manager`: starts and stops one `gst_cam_node` subprocess per entry in `pipelines.yaml`.

## Startup

`gst_camera_manager.service` runs the manager from boot and reads the installed `config/pipelines.yaml` at startup. No frames flow until a pipeline is started.

## Pipelines

| Pipeline | Sensor | Topic root | `frame_id` | Stream |
|---|---|---|---|---|
| `cam_down` | IMX477, `sensor-id=0` | `/cam_down` | `bottom_visual_link` | 1080x1080 GRAY8 15 fps, centre-cropped from 1920x1080 |

## Operator commands

| Command | Invokes | Result |
|---|---|---|
| `cam_down_start` | `/gst_camera_manager/cam_down` `SetBool{true}` | forks `gst_cam_node`; `/cam_down` topics appear; `alive` true |
| `cam_down_stop` | `/gst_camera_manager/cam_down` `SetBool{false}` | subprocess terminated; topics stop; `alive` false |
| `cam_down_status` | `/gst_camera_manager/cam_down/status` `Trigger` | `cam_down RUNNING (pid=N)` or `cam_down STOPPED` |
| `cam_down_alive` | echoes `/gst_camera_manager/cam_down/alive` | latched `data: true` or `data: false` |
| `cam_refresh` | `/gst_camera_manager/refresh` `Trigger` | pipelines stopped, `pipelines.yaml` re-read, services and `alive` topics rebuilt |
| `ver_cv_cams` | `/gst_camera_manager/cam_down` `SetBool{true}`, then subscribes `/cam_down/image_raw/compressed` | live window; a pipeline it started is stopped on exit |

## Services hosted

`<name>` is a pipeline key in `pipelines.yaml`.

| Service | Type | Effect |
|---|---|---|
| `/gst_camera_manager/<name>` | `std_srvs/SetBool` | `true` forks `gst_cam_node` for `<name>`; `false` terminates it |
| `/gst_camera_manager/<name>/status` | `std_srvs/Trigger` | `<name> RUNNING (pid=N)`, or `success=False` with `<name> STOPPED` |
| `/gst_camera_manager/status_all` | `std_srvs/Trigger` | one `[RUNNING]` or `[STOPPED]` line per pipeline |
| `/gst_camera_manager/stop_all` | `std_srvs/Trigger` | terminates every running pipeline |
| `/gst_camera_manager/refresh` | `std_srvs/Trigger` | stops pipelines, re-reads `pipelines.yaml`, rebuilds per-pipeline services and `alive` topics |

```bash
ros2 service call /gst_camera_manager/<name> std_srvs/srv/SetBool "{data: true}"
ros2 service call /gst_camera_manager/status_all std_srvs/srv/Trigger "{}"
ros2 service call /gst_camera_manager/stop_all std_srvs/srv/Trigger "{}"
```

## Published topics

| Topic | Type | Published | QoS |
|---|---|---|---|
| `/<topic>/image_raw` | `sensor_msgs/Image` | every frame while the pipeline runs | depth 5, volatile, best-effort; `reliable: true` makes it reliable |
| `/<topic>/image_raw/compressed` | `sensor_msgs/CompressedImage` | every frame, with `compress: true` | as above |
| `/<topic>/camera_info` | `sensor_msgs/CameraInfo` | every frame, stamp and `frame_id` equal to the image | as above |
| `/gst_camera_manager/<name>/alive` | `std_msgs/Bool` | manager start, pipeline start and stop, every watchdog transition | reliable, transient-local, depth 1 |

## Subscribed topics

| Topic | Type | Used for |
|---|---|---|
| `/<topic>/camera_info` | `sensor_msgs/CameraInfo` | the manager's evidence that frames are flowing, one subscription per pipeline |

The subscription QoS follows the pipeline's `reliable` field, so it matches the publisher.

## Liveness

The watchdog evaluates every running pipeline at 2 Hz.

| Event | `alive` | Manager log |
|---|---|---|
| manager start | `false` | |
| pipeline started | `true`, stall timer seeded so the first `alive_threshold` seconds are startup grace | `Started <name>` |
| no `camera_info` for `alive_threshold` | `false` | `Pipeline <name>: stalled, Ns since last frame` |
| frames resume | `true` | `Pipeline <name>: frames resumed` |
| subprocess exits | `false` | `Pipeline crashed: <name> (exit=N)` |

> The topic is transient-local. `ros2 topic echo` without `--qos-durability transient_local` returns nothing until the next state change.

## Configuration

`gst_camera_manager` declares no parameters and reads the installed copy of `config/pipelines.yaml`, so an edit needs `colcon_local` before `cam_refresh`.

### pipelines.yaml fields

| Field | Default | Effect |
|---|---|---|
| `gst_pipeline` | required | GStreamer pipeline string ending in `appsink`; newlines are flattened |
| `calibration` | pipeline name | basename of a YAML in `config/calibrations/` |
| `topic` | pipeline name | root of the three image topics |
| `frame_id` | `<name>_frame` | frame stamped on every image and `camera_info` |
| `encoding` | `""` | empty auto-detects; a ROS encoding name overrides |
| `compress` | `true` | `true` adds `/<topic>/image_raw/compressed` |
| `alive_threshold` | `5.0` | seconds without `camera_info` before `alive` goes false |
| `reliable` | `false` | `true` selects reliable QoS for that pipeline's image and `camera_info` |

### gst_cam_node parameters

The manager sets these on the forked node from the pipeline's `pipelines.yaml` entry.

| Parameter | Default | Controls |
|---|---|---|
| `gst_pipeline` | `""` | pipeline string; empty logs an error and opens nothing |
| `camera_topic` | `cam_down` | root of `image_raw`, `image_raw/compressed` and `camera_info` |
| `frame_id` | `camera_frame` | `header.frame_id` on every image and `camera_info` |
| `camera_info_path` | `""` | `file://` URL of a calibration YAML |
| `encoding` | `""` | `sensor_msgs/Image.encoding`; empty auto-detects on the first frame |
| `compress` | `true` | `true` publishes raw and compressed through `image_transport` |
| `reliable` | `false` | `true` selects reliable QoS on both publishers |

`gst_cam_node` is also registered as an `rclcpp_components` node.

### Encoding

The encoding resolves on the first frame and is logged as `Image encoding: <enc>`.

| `cv::Mat` type | ROS encoding |
|---|---|
| `CV_8UC1` | `mono8` |
| `CV_8UC3` | `bgr8` |
| `CV_8UC4` | `bgra8` |
| `CV_16UC1` | `mono16` |
| `CV_16UC3` | `bgr16` |
| `CV_16UC4` | `bgra16` |
| anything else | `""`, with a warning to set `encoding` |

### Calibration

Calibration files live in `config/calibrations/` in `camera_calibration_parsers` format. `cam_down` ships `calibration: "IMX477_1080sq"`, the intrinsics of the 1080x1080 crop. When the named file is unreadable the node logs `Invalid calibration path` and publishes placeholder intrinsics from the first frame: zero distortion, `fx = fy = width`, centre principal point.

`cam_calibrate` writes `config/calibrations/cam_down.yaml` in the source tree. Set `calibration: "cam_down"` in `pipelines.yaml`, then install and reload.

```bash
cam_calibrate
colcon_local
cam_refresh
cam_down_start
```

### Adding a pipeline

Append an entry to `pipelines.yaml` with the fields above, run `colcon_local`, then `cam_refresh`. The `/gst_camera_manager/<name>` service and the `<name>/alive` topic appear with it.

## Logs

| Log | Path |
|---|---|
| manager | `~/workspaces/isaac_ros-dev/run_logs/gst_camera_manager/gst_camera_manager.log`, previous run as `gst_camera_manager.prev.log` |
| per pipeline | `~/workspaces/local_ws/install/gst_camera_manager/share/gst_camera_manager/logs/<name>/`, 20 runs kept |

## Troubleshooting

| Symptom | Cause | Action |
|---|---|---|
| `stalled` in the manager log | no `camera_info` for `alive_threshold` seconds; `status` still reports `RUNNING` | read the per-pipeline log |
| `Failed to open GStreamer pipeline` | bad pipeline string, or the sensor is already in use | run the string under `gst-launch-1.0` with `appsink` replaced by `fakesink` |
| repeated `Frame read failed` | the camera stopped delivering | stop and start the pipeline |
| no `Image encoding:` line and no encoding warning | no frame reached the node | stop and start the pipeline |
| `Pipeline crashed` | the subprocess exited with the logged code | read the per-pipeline log |
| a subscriber receives nothing while `alive` is true | subscriber QoS does not match the publisher | match the subscriber, or set `reliable: true` |

Raise `alive_threshold` only for a pipeline whose frame interval exceeds it.
