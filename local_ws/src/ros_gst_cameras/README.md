# ros_gst_cameras

ROS 2 GStreamer camera stack. Two packages:

- **`gst_cam_node`**: C++ node; wraps any GStreamer pipeline ending in `appsink`, publishes `image_raw` (plus `image_raw/compressed` when `compress: true`) and `camera_info`.
- **`gst_camera_manager`**: Python supervisor; starts/stops each pipeline in `pipelines.yaml` as a subprocess, with a liveness watchdog.

## Quick start

Build:

```bash
cd ~/workspaces/local_ws
colcon build --packages-select gst_cam_node gst_camera_manager --symlink-install
source install/setup.bash
```

Launch the manager:

```bash
ros2 launch gst_camera_manager gst_camera_manager.launch.py
```

Reads `gst_camera_manager/config/pipelines.yaml` at startup. No frames flow until a pipeline is started.

### Services

| Service | Type | Function |
|---|---|---|
| `/gst_camera_manager/<name>` | `std_srvs/SetBool` | Start (`data: true`) or stop (`data: false`) one pipeline. |
| `/gst_camera_manager/<name>/status` | `std_srvs/Trigger` | `RUNNING (pid=N)` or `STOPPED`. |
| `/gst_camera_manager/status_all` | `std_srvs/Trigger` | One state line per pipeline. |
| `/gst_camera_manager/stop_all` | `std_srvs/Trigger` | Stops every running pipeline. |
| `/gst_camera_manager/refresh` | `std_srvs/Trigger` | Stops running pipelines, re-reads `pipelines.yaml`, rebuilds services. |

`<name>` matches the keys in `pipelines.yaml`: `cam_down`.

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

Each pipeline publishes `/gst_camera_manager/<name>/alive` (`std_msgs/Bool`, latched). The 2 Hz watchdog sets it `false` on subprocess exit (`Pipeline crashed`, exit code logged) or when no `camera_info` arrives within `alive_threshold` seconds (`stalled`), and `true` when frames resume. The first `alive_threshold` seconds after launch are a startup grace period.

## Configuration: `pipelines.yaml`

| Field | Behavior |
|---|---|
| `gst_pipeline` | GStreamer pipeline string ending in `appsink`. |
| `calibration` | Basename of a file in `config/calibrations/`; missing or empty falls back to default `camera_info`. |
| `topic` | Root of `/<topic>/image_raw`, `/<topic>/image_raw/compressed`, `/<topic>/camera_info`. |
| `frame_id` | TF frame stamped onto every message. |
| `encoding` | `""` auto-detects (table below); set explicitly (`"rgb8"`, `"bayer_rggb8"`) to override. |
| `compress` | `true` adds JPEG via `image_transport` (encoder runs only with a subscriber). |
| `alive_threshold` | Seconds without `camera_info` before not-alive. Default `5.0`. |
| `reliable` | `true` = RELIABLE QoS; omitted/`false` = sensor_data (BEST_EFFORT), drops frames instead of stalling. |

### Encoding auto-detect

Resolved once on the first frame; the log shows `Image encoding: <enc> (auto-detected|override)`.

| cv::Mat type | ROS encoding |
|---|---|
| `CV_8UC1` | `mono8` |
| `CV_8UC3` | `bgr8` |
| `CV_8UC4` | `bgra8` |
| `CV_16UC1` | `mono16` |
| `CV_16UC3` | `bgr16` |
| `CV_16UC4` | `bgra16` |
| anything else | `""` (warns; set `encoding:` to override) |

### Calibration

Calibration YAMLs live in `config/calibrations/` (`camera_calibration_parsers` format). Recalibrate with `camera_calibrate.sh` (`local_ws/auxiliary/camera_calibration/`); it writes `config/calibrations/cam_down.yaml`. Without a calibration the node publishes a default `CameraInfo` from the first frame (zero distortion, `fx = fy = width`, centered principal point), so `image_raw`/`camera_info` subscribers keep working before calibration.

## Adding a new pipeline

Append to `pipelines.yaml`:

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

Field semantics: table above. Rebuild (`colcon build --packages-select gst_camera_manager`) or call `refresh`. The `/gst_camera_manager/my_cam` service appears automatically.

### Defined pipelines

- **`cam_down`**: CSI sensor-id 0, 1920x1080 at 15 fps, GRAY8 (IR-sensitive mono). Frame ID `bottom_visual_link`.

## Auto-start on boot

`gst_camera_manager.service` (installed by `setup.sh`) launches the manager at boot. Pipelines stay off until started via SetBool.

## Logs

Manager log:

```
~/workspaces/isaac_ros-dev/run_logs/gst_camera_manager/gst_camera_manager.log
```

Per-pipeline GStreamer log (last 20 kept per pipeline):

```
~/workspaces/local_ws/install/gst_camera_manager/share/gst_camera_manager/logs/<pipeline_name>/<pipeline_name>_<timestamp>.log
```

## Troubleshooting

### Pipeline runs but `ros2 topic hz` shows nothing

1. Test the pipeline standalone (replace `appsink` with `fakesink`); failure here means camera/driver, not the ROS node:
   ```bash
   gst-launch-1.0 <pipeline-string>
   ```
2. Check the per-pipeline log for GStreamer errors.
3. Check liveness:
   ```bash
   ros2 topic echo /gst_camera_manager/<name>/alive --once
   ```
   `false`: crashed or stalled, see the log. `true` with zero hz: subscriber QoS mismatch.
4. No `Image encoding:` log line means no frames ever reached the node.

### Pipeline crashes immediately

Per-pipeline log has the GStreamer error. Usual causes: missing plugin, camera busy (`sudo fuser /dev/video0`), device permissions.

### `stalled` warnings

Camera disconnected or driver unresponsive: restart the pipeline (`SetBool false` then `true`). Legitimately slow pipelines (long exposure): raise `alive_threshold`.

### `Incompatible QoS` warnings

Subscriber reliability differs from the publisher. Match the subscriber, or set `reliable: true` on the pipeline.
