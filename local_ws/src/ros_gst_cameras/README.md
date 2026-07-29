# ros_gst_cameras

This repository directory holds the ROS 2 GStreamer camera stack, split into two packages.

- **`gst_cam_node`**: a C++ node wrapping any GStreamer pipeline that ends in `appsink`. It
  publishes `image_raw`, `camera_info`, and `image_raw/compressed` when `compress: true`.
- **`gst_camera_manager`**: a Python supervisor that starts and stops each pipeline in
  `pipelines.yaml` as a subprocess, with a liveness watchdog.

`gst_camera_manager.service` launches the manager at boot. It reads `pipelines.yaml` at startup and
no frames flow until a pipeline is started.

## Services

| Service | Type | Function |
|---|---|---|
| `/gst_camera_manager/<name>` | `std_srvs/SetBool` | Start (`data: true`) or stop (`data: false`) one pipeline. |
| `/gst_camera_manager/<name>/status` | `std_srvs/Trigger` | `RUNNING (pid=N)` or `STOPPED`. |
| `/gst_camera_manager/status_all` | `std_srvs/Trigger` | One state line per pipeline. |
| `/gst_camera_manager/stop_all` | `std_srvs/Trigger` | Stops every running pipeline. |
| `/gst_camera_manager/refresh` | `std_srvs/Trigger` | Stops running pipelines, re-reads `pipelines.yaml`, rebuilds the services. |

`<name>` matches the keys in `pipelines.yaml`, which on this drone is `cam_down` alone.

```bash
ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool "{data: true}"
ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool "{data: false}"
ros2 service call /gst_camera_manager/cam_down/status std_srvs/srv/Trigger "{}"
ros2 service call /gst_camera_manager/status_all std_srvs/srv/Trigger "{}"
ros2 service call /gst_camera_manager/stop_all std_srvs/srv/Trigger "{}"
ros2 service call /gst_camera_manager/refresh std_srvs/srv/Trigger "{}"
```

### Verify frames

```bash
ros2 topic hz /cam_down/image_raw
ros2 topic hz /cam_down/image_raw/compressed
ros2 topic echo /cam_down/camera_info --once
```

The `compressed` topic exists only when the pipeline sets `compress: true`.

### Liveness

Each pipeline publishes a latched `/gst_camera_manager/<name>/alive`. The 2 Hz watchdog sets it
false when the subprocess exits (logged as `Pipeline crashed` with the exit code) or when no
`camera_info` arrives within `alive_threshold` seconds (logged as `stalled`), and true when frames
resume. The first `alive_threshold` seconds after launch are a startup grace period.

## Configuration

`gst_camera_manager/config/pipelines.yaml` defines every pipeline.

| Field | Behavior |
|---|---|
| `gst_pipeline` | GStreamer pipeline string ending in `appsink`. |
| `calibration` | Basename of a file in `config/calibrations/`; missing or empty falls back to a default `camera_info`. |
| `topic` | Root of `/<topic>/image_raw`, `/<topic>/image_raw/compressed`, `/<topic>/camera_info`. |
| `frame_id` | TF frame stamped onto every message. |
| `encoding` | `""` auto-detects; set explicitly (`"rgb8"`, `"bayer_rggb8"`) to override. |
| `compress` | `true` adds JPEG through `image_transport`; the encoder runs only with a subscriber. |
| `alive_threshold` | Seconds without `camera_info` before not-alive. Default `5.0`. |
| `reliable` | `true` selects RELIABLE QoS; omitted or `false` selects sensor_data, which drops frames instead of stalling. |

### Encoding auto-detect

The encoding is resolved once on the first frame, and the log shows
`Image encoding: <enc> (auto-detected|override)`.

| cv::Mat type | ROS encoding |
|---|---|
| `CV_8UC1` | `mono8` |
| `CV_8UC3` | `bgr8` |
| `CV_8UC4` | `bgra8` |
| `CV_16UC1` | `mono16` |
| `CV_16UC3` | `bgr16` |
| `CV_16UC4` | `bgra16` |
| anything else | `""`, with a warning; set `encoding:` to override |

### Calibration

Calibration YAMLs live in `config/calibrations/` in `camera_calibration_parsers` format.
`cam_calibrate` writes `config/calibrations/cam_down.yaml`. Without a calibration the node
publishes a default `CameraInfo` derived from the first frame (zero distortion, `fx = fy = width`,
centered principal point), so subscribers keep working before calibration.

### Defined pipelines

`cam_down` runs CSI sensor 0 at 1920x1080, 15 fps, GRAY8, with frame id `bottom_visual_link`.

## Adding a pipeline

Append an entry to `pipelines.yaml` using the fields above:

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

Call `refresh` to pick it up. The `/gst_camera_manager/my_cam` service appears automatically.

## Logs

The manager log is at `~/workspaces/isaac_ros-dev/run_logs/gst_camera_manager/`. Per-pipeline
GStreamer logs are at
`~/workspaces/local_ws/install/gst_camera_manager/share/gst_camera_manager/logs/<pipeline>/`, where
the last 20 are kept per pipeline.

## Troubleshooting

**The pipeline runs but `ros2 topic hz` shows nothing.** Test the pipeline standalone with
`appsink` replaced by `fakesink`; a failure there is a camera or driver problem rather than the ROS
node.

```bash
gst-launch-1.0 <pipeline-string>
```

Then check the per-pipeline log for GStreamer errors and read the liveness topic:

```bash
ros2 topic echo /gst_camera_manager/<name>/alive --once
```

`false` means crashed or stalled, and the log says which. `true` with zero hz is a subscriber QoS
mismatch. A missing `Image encoding:` log line means no frame ever reached the node.

**The pipeline crashes immediately.** The per-pipeline log carries the GStreamer error. The usual
causes are a missing plugin, a camera already in use (`sudo fuser /dev/video0`), or device
permissions.

**`stalled` warnings.** The camera disconnected or the driver stopped responding; restart the
pipeline with `SetBool false` then `true`. A legitimately slow pipeline needs a higher
`alive_threshold`.

**`Incompatible QoS` warnings.** The subscriber reliability differs from the publisher. Match the
subscriber, or set `reliable: true` on the pipeline.
