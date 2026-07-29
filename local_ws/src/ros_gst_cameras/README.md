# ros_gst_cameras

This directory holds the ROS 2 GStreamer camera stack in two packages.

- `gst_cam_node`: a C++ node that wraps any GStreamer pipeline ending in `appsink` and publishes `image_raw`, `image_raw/compressed` when `compress: true`, and `camera_info`.
- `gst_camera_manager`: a Python supervisor that starts and stops each pipeline in `pipelines.yaml` as a subprocess, under a liveness watchdog.

`gst_camera_manager.service` runs the manager at boot. It reads `gst_camera_manager/config/pipelines.yaml` at startup, and no frames flow until a pipeline is started.

## Services

The manager exposes one service per pipeline plus four global ones.

| Service | Type | Function |
|---|---|---|
| `/gst_camera_manager/<name>` | `std_srvs/SetBool` | Start (`data: true`) or stop (`data: false`) one pipeline. |
| `/gst_camera_manager/<name>/status` | `std_srvs/Trigger` | `RUNNING (pid=N)` or `STOPPED`. |
| `/gst_camera_manager/status_all` | `std_srvs/Trigger` | One state line per pipeline. |
| `/gst_camera_manager/stop_all` | `std_srvs/Trigger` | Stops every running pipeline. |
| `/gst_camera_manager/refresh` | `std_srvs/Trigger` | Stops running pipelines, re-reads `pipelines.yaml`, rebuilds the services. |

`<name>` matches the keys in `pipelines.yaml`: `cam_front` and `cam_down`.

```bash
ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool "{data: true}"
ros2 service call /gst_camera_manager/cam_down/status std_srvs/srv/Trigger "{}"
ros2 service call /gst_camera_manager/status_all std_srvs/srv/Trigger "{}"
ros2 service call /gst_camera_manager/stop_all std_srvs/srv/Trigger "{}"
ros2 service call /gst_camera_manager/refresh std_srvs/srv/Trigger "{}"
```

## Verify frames

Check a running pipeline against its three topics.

```bash
ros2 topic hz /cam_down/image_raw
ros2 topic hz /cam_down/image_raw/compressed
ros2 topic echo /cam_down/camera_info --once
```

The same topics exist under `/cam_front`. The `compressed` topic exists only when the pipeline sets `compress: true`.

## Liveness

Each pipeline publishes `/gst_camera_manager/<name>/alive` (`std_msgs/Bool`, latched). The 2 Hz watchdog sets it false on subprocess exit, logging `Pipeline crashed` with the exit code, or when no `camera_info` arrives within `alive_threshold` seconds, logging `stalled`. It sets it true again when frames resume. The first `alive_threshold` seconds after launch are a startup grace period.

## Configuration: `pipelines.yaml`

Each pipeline entry takes the fields below.

| Field | Behavior |
|---|---|
| `gst_pipeline` | GStreamer pipeline string ending in `appsink`. |
| `calibration` | Basename of a file in `config/calibrations/`; missing or empty falls back to default `camera_info`. |
| `topic` | Root of `/<topic>/image_raw`, `/<topic>/image_raw/compressed` and `/<topic>/camera_info`. |
| `frame_id` | TF frame stamped onto every message. |
| `encoding` | `""` auto-detects; set explicitly (`"rgb8"`, `"bayer_rggb8"`) to override. |
| `compress` | `true` adds JPEG through `image_transport`; the encoder runs only with a subscriber. |
| `alive_threshold` | Seconds without `camera_info` before not-alive. Default `5.0`. |
| `reliable` | `true` selects RELIABLE QoS; omitted or `false` selects sensor_data, which drops frames instead of stalling. |

### Encoding auto-detect

The encoding is resolved once on the first frame, and the log shows `Image encoding: <enc> (auto-detected|override)`.

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

Calibration YAMLs live in `config/calibrations/` in `camera_calibration_parsers` format. Recalibrate with `cam_calibrate front|down`, which writes `config/calibrations/<cam>.yaml`. Without a calibration the node publishes a default `CameraInfo` built from the first frame (zero distortion, `fx = fy = width`, centered principal point), so `image_raw` and `camera_info` subscribers keep working before calibration.

## Defined pipelines

Two pipelines ship in `pipelines.yaml`.

- `cam_front`: CSI sensor-id 0, 1920x1080 at 15 fps, GRAY8, frame ID `top_visual_link`.
- `cam_down`: CSI sensor-id 1, the same format, frame ID `bottom_visual_link`.

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
    alive_threshold: 2.0
```

Then call `refresh`, or `cam_refresh` from the host alias set. The `/gst_camera_manager/my_cam` service appears with it.

## Logs

The manager writes to `~/workspaces/isaac_ros-dev/run_logs/gst_camera_manager/gst_camera_manager.log`.

Each pipeline writes its own GStreamer log under `~/workspaces/local_ws/install/gst_camera_manager/share/gst_camera_manager/logs/<pipeline_name>/`, and the last 20 are kept per pipeline.

## Troubleshooting

### A pipeline runs but `ros2 topic hz` shows nothing

1. Test the pipeline standalone with `appsink` replaced by `fakesink`. A failure at this point is the camera or driver, not the ROS node.
   ```bash
   gst-launch-1.0 <pipeline-string>
   ```
2. Read the per-pipeline log for GStreamer errors.
3. Check liveness:
   ```bash
   ros2 topic echo /gst_camera_manager/<name>/alive --once
   ```
   `false` means crashed or stalled, so read the log. `true` with zero hz is a subscriber QoS mismatch.
4. No `Image encoding:` log line means no frame ever reached the node.

### A pipeline crashes immediately

The per-pipeline log carries the GStreamer error. The usual causes are a missing plugin, another process already holding the camera, and device permissions.

### `stalled` warnings

The camera is disconnected or the driver is unresponsive: restart the pipeline with `SetBool false` then `true`. For a legitimately slow pipeline such as a long exposure, raise `alive_threshold`.

### `Incompatible QoS` warnings

The subscriber reliability differs from the publisher. Match the subscriber, or set `reliable: true` on the pipeline.
