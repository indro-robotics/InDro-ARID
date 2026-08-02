# ros_gst_cameras

This directory holds the ROS 2 GStreamer camera stack in two packages.

- `gst_cam_node`: a C++ node that wraps any GStreamer pipeline ending in `appsink` and publishes it as a ROS camera stream.
- `gst_camera_manager`: a Python supervisor that starts and stops each pipeline in `pipelines.yaml` as a subprocess, under a liveness watchdog.

`gst_camera_manager.service` runs the manager at boot. It reads `gst_camera_manager/config/pipelines.yaml` at startup, and no frames flow until a pipeline is started.

## Services

The manager exposes two services per pipeline and three manager-level services.

| Service | Type | Function |
|---|---|---|
| `/gst_camera_manager/<name>` | `std_srvs/SetBool` | Start (`data: true`) or stop (`data: false`) one pipeline. |
| `/gst_camera_manager/<name>/status` | `std_srvs/Trigger` | `RUNNING (pid=N)` or `STOPPED`. |
| `/gst_camera_manager/status_all` | `std_srvs/Trigger` | One state line per pipeline. |
| `/gst_camera_manager/stop_all` | `std_srvs/Trigger` | Stops every running pipeline. |
| `/gst_camera_manager/refresh` | `std_srvs/Trigger` | Stops running pipelines, re-reads `pipelines.yaml`, rebuilds the per-pipeline services. |

`<name>` matches the key in `pipelines.yaml`: `cam_down`.

```bash
ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool "{data: true}"
ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool "{data: false}"
ros2 service call /gst_camera_manager/cam_down/status std_srvs/srv/Trigger "{}"
ros2 service call /gst_camera_manager/status_all std_srvs/srv/Trigger "{}"
ros2 service call /gst_camera_manager/stop_all std_srvs/srv/Trigger "{}"
ros2 service call /gst_camera_manager/refresh std_srvs/srv/Trigger "{}"
```

## Topics

A running pipeline publishes under its `topic` root, and the manager publishes one liveness topic per pipeline.

| Topic | Type | Published |
|---|---|---|
| `/<topic>/image_raw` | `sensor_msgs/Image` | while the pipeline runs |
| `/<topic>/image_raw/compressed` | `sensor_msgs/CompressedImage` | while the pipeline runs, with `compress: true` |
| `/<topic>/camera_info` | `sensor_msgs/CameraInfo` | while the pipeline runs |
| `/gst_camera_manager/<name>/alive` | `std_msgs/Bool` | latched, from manager start |

Each image and its `camera_info` carry the same stamp. Both use sensor_data QoS unless the pipeline sets `reliable: true`.

```bash
ros2 topic hz /cam_down/image_raw
ros2 topic hz /cam_down/image_raw/compressed
ros2 topic echo /cam_down/camera_info --once
```

## Liveness

The watchdog checks every running pipeline at 2 Hz and reports on `/gst_camera_manager/<name>/alive`. It publishes false and logs `Pipeline crashed` with the exit code when the subprocess exits, false and logs `stalled` when no `camera_info` has arrived for `alive_threshold` seconds, and true again when frames resume. The timer is seeded when the pipeline starts, so the first `alive_threshold` seconds act as a startup grace period.

> The topic is latched. An echo whose QoS does not match returns nothing until the next state change.

```bash
ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable /gst_camera_manager/cam_down/alive
```

## Defined pipelines

One pipeline ships in `pipelines.yaml`, an IMX477 CSI sensor at 1920x1080 centre-cropped to 1080x1080, 15 fps, GRAY8, unrotated.

| Pipeline | Sensor | Frame ID | Topic root |
|---|---|---|---|
| `cam_down` | `sensor-id=0` | `bottom_visual_link` | `/cam_down` |

## Configuration

`gst_camera_manager/config/pipelines.yaml` defines every pipeline. Each entry takes the fields below.

| Field | Default | Behaviour |
|---|---|---|
| `gst_pipeline` | required | GStreamer pipeline string ending in `appsink`. |
| `calibration` | pipeline name | Basename of a file in `config/calibrations/`. |
| `topic` | pipeline name | Root of the three image topics. |
| `frame_id` | `<name>_frame` | TF frame stamped onto every image and `camera_info`. |
| `encoding` | `""` | `""` auto-detects; set explicitly (`"rgb8"`, `"bayer_rggb8"`) to override. |
| `compress` | `true` | `true` adds the `compressed` topic; the encoder runs only with a subscriber. |
| `alive_threshold` | `5.0` | Seconds without `camera_info` before not-alive. |
| `reliable` | `false` | `true` selects RELIABLE QoS; `false` selects sensor_data, which drops frames instead of stalling. |

### Encoding auto-detect

The encoding is resolved on the first frame and logged as `Image encoding: <enc> (auto-detected|override)`.

| cv::Mat type | ROS encoding |
|---|---|
| `CV_8UC1` | `mono8` |
| `CV_8UC3` | `bgr8` |
| `CV_8UC4` | `bgra8` |
| `CV_16UC1` | `mono16` |
| `CV_16UC3` | `bgr16` |
| `CV_16UC4` | `bgra16` |
| anything else | `""`, with a warning to set `encoding:` |

### Calibration

Calibration YAMLs live in `config/calibrations/` in `camera_calibration_parsers` format. `cam_calibrate` writes `config/calibrations/cam_down.yaml`. The entry ships with `calibration: "IMX477_1080sq"`, the intrinsics for the 1080x1080 crop; set `calibration: "cam_down"` and restart the pipeline to load a fresh calibration. When the field names no readable file the node logs `Invalid calibration path` and publishes a default `CameraInfo` built from the first frame (zero distortion, `fx = fy = width`, principal point at the image centre).

> A calibration file created since the last `local_ws` build is not installed. Run `colcon_local` before naming it in `pipelines.yaml`.

## Adding a pipeline

Append an entry to `pipelines.yaml`:

```yaml
  my_cam:
    gst_pipeline: >-
      <any gstreamer pipeline ending in appsink>
    calibration: "my_cam"
    topic: "my_cam"
    frame_id: "my_cam_frame"
    encoding: ""
    compress: true
    alive_threshold: 5.0
```

Call `refresh`, or the `cam_refresh` alias, and the `/gst_camera_manager/my_cam` service appears with it.

## Logs

The manager log is at `~/workspaces/isaac_ros-dev/run_logs/gst_camera_manager/gst_camera_manager.log`, with the previous run kept beside it as `gst_camera_manager.prev.log`.

Each pipeline writes its own GStreamer log under `~/workspaces/local_ws/install/gst_camera_manager/share/gst_camera_manager/logs/<name>/`, where the 20 most recent runs are kept per pipeline.

## Troubleshooting

### `stalled` in the manager log

No `camera_info` arrived for `alive_threshold` seconds. The subprocess stays up, so a pipeline that failed to open also reports `RUNNING` while delivering nothing. Read the per-pipeline log: `Failed to open GStreamer pipeline` is a bad pipeline string or a sensor already in use, repeated `Frame read failed` is a camera that stopped delivering, and a missing `Image encoding:` line means no frame ever reached the node. Restart the pipeline with `SetBool false` then `true`. Raise `alive_threshold` only for a legitimately slow pipeline such as a long exposure.

Test the pipeline string outside ROS with `appsink` replaced by `fakesink`. A failure at this point is the camera or driver, not the ROS node.

```bash
gst-launch-1.0 <pipeline-string>
```

### `Pipeline crashed` in the manager log

The subprocess exited. The message carries the exit code and the per-pipeline log carries its output.

### A subscriber receives nothing while `/alive` is true

The subscriber QoS does not match the publisher, which also logs `Incompatible QoS`. Match the subscriber to the pipeline, or set `reliable: true` on the pipeline.
