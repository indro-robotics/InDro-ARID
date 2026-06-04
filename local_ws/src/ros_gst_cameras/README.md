# ros_gst_cameras

ROS 2 GStreamer-based camera stack. Two packages:

- **`gst_cam_node`**: C++ node that wraps an arbitrary GStreamer pipeline and publishes `image_raw` (plus `image_raw/compressed` when `compress: true`) and `camera_info`. The pipeline is opaque to the node: anything that produces frames into an `appsink` works.
- **`gst_camera_manager`**: Python supervisor that loads a YAML of named pipelines and exposes ROS 2 services to start and stop each one as a managed subprocess, with a watchdog on liveness.

---

## Quick start

### Build
```bash
cd ~/workspaces/local_ws
colcon build --packages-select gst_cam_node gst_camera_manager --symlink-install
source install/setup.bash
```

### Launch the manager
```bash
ros2 launch gst_camera_manager gst_camera_manager.launch.py
```

At startup the manager reads `gst_camera_manager/config/pipelines.yaml` and creates a set of services per pipeline. No frames flow until a pipeline is explicitly started.

### Service interface

| Service | Type | What it does |
|---|---|---|
| `/gst_camera_manager/<name>` | `std_srvs/SetBool` | Per-pipeline start (`data: true`) or stop (`data: false`). |
| `/gst_camera_manager/<name>/status` | `std_srvs/Trigger` | Per-pipeline state. Returns `RUNNING (pid=…)` or `STOPPED`. |
| `/gst_camera_manager/status_all` | `std_srvs/Trigger` | Multi-line summary of every pipeline's state, one line per pipeline. |
| `/gst_camera_manager/stop_all` | `std_srvs/Trigger` | Kills every currently-running pipeline. No-op for stopped pipelines. |

`<name>` matches the keys in `pipelines.yaml`. Currently: `cam_down`.

#### Examples

Start or stop the `cam_down` pipeline by setting `data: true` to start or `data: false` to stop:

```bash
ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool "{data: true}"
ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool "{data: false}"
```

Query the state of one pipeline:

```bash
ros2 service call /gst_camera_manager/cam_down/status std_srvs/srv/Trigger "{}"
```

The response returns `success=True, message='cam_down RUNNING (pid=N)'` when the pipeline is up, and `success=False, message='cam_down STOPPED'` when it isn't.

Summary across every pipeline (handy for ground-station UIs that render a panel of camera states without polling each `/<name>/status` individually):

```bash
ros2 service call /gst_camera_manager/status_all std_srvs/srv/Trigger "{}"
```

Kill every running pipeline in one call. Equivalent to `SetBool false` on each one individually:

```bash
ros2 service call /gst_camera_manager/stop_all std_srvs/srv/Trigger "{}"
```

### Verify frames

The `compressed` topic is only published when the pipeline's `compress: true` field is set.

```bash
ros2 topic hz /cam_down/image_raw
ros2 topic hz /cam_down/image_raw/compressed
ros2 topic echo /cam_down/camera_info --once
```

### Liveness (latched)
Each pipeline publishes `/gst_camera_manager/<name>/alive` (`std_msgs/Bool`, TRANSIENT_LOCAL). The watchdog runs at 2 Hz and flips alive to `false` in either of these cases:

1. **Process died**: the subprocess exited. Logs "Pipeline crashed" with the exit code.
2. **Stalled**: the subprocess is still running but no `camera_info` message has arrived within the pipeline's configured `alive_threshold` seconds. Logs "Pipeline stalled" with the observed gap.

Once frames resume, alive flips back to `true` automatically (logs "frames resumed"). The timer starts when the subprocess is launched, so the first `alive_threshold` seconds after start act as a startup grace period.

---

## Configuration: `pipelines.yaml`

Header comments in [`config/pipelines.yaml`](gst_camera_manager/config/pipelines.yaml) document every field. Summary:

| Field | Behavior |
|---|---|
| `gst_pipeline` | Full GStreamer pipeline string ending in `appsink`. |
| `calibration` | Basename (no extension) of a file in `config/calibrations/`. If the file is missing or the field is omitted, default `camera_info` is generated from the first frame (zero distortion, `fx = fy = width`, principal point at image centre). |
| `topic` | Root of the published topics: `/<topic>/image_raw`, `/<topic>/image_raw/compressed`, `/<topic>/camera_info`. |
| `frame_id` | TF frame stamped onto every `Image` and `CameraInfo`. |
| `encoding` | Override for `sensor_msgs/Image.encoding`. Leave `""` to auto-detect from the `cv::Mat::type()` returned by OpenCV (see table below). Set explicitly (e.g. `"rgb8"`, `"bayer_rggb8"`) to override. |
| `compress` | `true` publishes raw plus JPEG compressed via `image_transport` (lazy: the compressed encoder only runs when a subscriber exists). `false` publishes raw only. |
| `alive_threshold` | Seconds (float) without a `camera_info` message before the pipeline is marked not-alive. Starts ticking when the subprocess launches. Omitted defaults to `5.0`. |
| `reliable` | QoS selector for `image_raw` and `camera_info`. Omitted, `""`, or `false` selects **sensor_data QoS** (BEST_EFFORT, VOLATILE, depth 5). This is the ROS 2 convention for image streams; drops frames on lossy links rather than stalling the publisher. `true` selects RELIABLE. Use that for low-rate or frame-critical streams where loss is unacceptable. |

### Encoding auto-detect

`gst_cam_node` maps these `cv::Mat` types directly to unambiguous ROS encodings:

| cv::Mat type | ROS encoding |
|---|---|
| `CV_8UC1` | `mono8` |
| `CV_8UC3` | `bgr8` |
| `CV_8UC4` | `bgra8` |
| `CV_16UC1` | `mono16` |
| `CV_16UC3` | `bgr16` |
| `CV_16UC4` | `bgra16` |
| anything else | `""` (warns and publishes unlabeled; set `encoding:` in YAML to override) |

Resolution happens once on the first frame and is cached. The log line tells you which path was taken:
```
[INFO] Image encoding: mono8 (auto-detected)
[INFO] Image encoding: bgr8 (override)
```

### Calibration

Calibration YAMLs in `config/calibrations/` use the standard `camera_calibration_parsers` format produced by `ros2 run camera_calibration cameracalibrator`. If `camera_info_path` is empty or the file is missing, the node still publishes a sensible default `CameraInfo`:

- `width`, `height` from the first frame
- `distortion_model: "plumb_bob"`, `d = [0, 0, 0, 0, 0]`
- `fx = fy = width`, `cx = width/2`, `cy = height/2`
- Identity rectification, intrinsic padded into projection matrix

This keeps subscribers that expect synchronized `image_raw` and `camera_info` pairs functional even before calibration is done.

---

## Adding a new pipeline

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

Field semantics are in the *Configuration: `pipelines.yaml`* table above. `calibration` is optional and refers to a file in `config/calibrations/`. `encoding: ""` selects auto-detect; set explicitly to override. Add `reliable: true` to opt out of the default BEST_EFFORT (sensor_data) QoS.

Rebuild (`colcon build --packages-select gst_camera_manager`) or restart the manager to pick it up. The pipeline gets its own `/gst_camera_manager/my_cam` service automatically. No code changes needed.

### Currently defined pipelines

- **`cam_down`**: CSI camera (sensor-id 0) via `nvarguscamerasrc` at 1920×1080 at 20 fps (delivered ~16 Hz), NV12 to GRAY8 via `nvvidconv`. The sensor is IR-sensitive; the pipeline drops to mono (GRAY8) so the stream is directly usable for IR-aware computer-vision tasks (feature tracking, motion detection, fiducial decoding) without per-channel filtering. Frame ID: `bottom_visual_link`.

---

## Auto-start on boot

`gst_camera_manager.service` (installed by `setup.sh`) launches the manager node on boot under the `multi-user.target`. Pipelines remain off until explicitly started via SetBool. The service runs only the supervisor; pipeline subprocesses are spawned on demand.

---

## Logs

Per-pipeline stdout/stderr goes to:
```
~/workspaces/local_ws/install/gst_camera_manager/share/gst_camera_manager/logs/<pipeline_name>/<pipeline_name>_<timestamp>.log
```
The manager keeps the last 20 log files per pipeline and rotates older ones.

---

## Troubleshooting

### Pipeline starts but `ros2 topic hz /<topic>/image_raw` shows nothing

1. **Test the GStreamer pipeline standalone** (bypasses ROS, proves the pipeline itself works):
   ```bash
   gst-launch-1.0 <pipeline string, but replace `appsink` with `fakesink`>
   ```
   If this fails, the problem is in the pipeline, camera, or drivers, not the ROS node.

2. **Check the per-pipeline log** at the path above. The `gst_cam_node` subprocess writes every GStreamer error there.

3. **Check the liveness topic:**
   ```bash
   ros2 topic echo /gst_camera_manager/<name>/alive --once
   ```
   - `data: false`: subprocess died or hasn't produced a frame within `alive_threshold`. Look at the log file.
   - `data: true` but `hz` still zero: QoS mismatch on the subscriber side (consumer expects RELIABLE but the pipeline is BEST_EFFORT, or vice versa).

4. **Encoding auto-detect log line**. On first frame the log shows:
   ```
   [INFO] Image encoding: mono8 (auto-detected)
   ```
   If this never appears, no frames are reaching the OpenCV read loop (GStreamer negotiation or hardware problem).

### Pipeline crashes immediately on start

Check the log file. Almost always one of:
- GStreamer element missing (plugin not installed).
- Camera busy or in use by another process. Use `sudo fuser /dev/video0` to see owners.
- Permission issue on the camera device.

### "Pipeline stalled" warnings in the manager log

Frames arrived but stopped. Could be:
- Camera disconnected or driver wedged. Restart the pipeline: `SetBool false` then `true`.
- Exposure auto-adjusted to a very long value (check `exposure=` in the pipeline string).
- GPU or ISP overloaded (other pipelines or models contending).

Increase `alive_threshold` if the pipeline is legitimately slow (e.g. long-exposure scanner at <1 fps).

### QoS mismatch errors in the ROS log

If `Incompatible QoS` warnings appear, the subscriber is using a different reliability than the publisher. Either change the subscriber's QoS to match, or set `reliable: true` on the pipeline in `pipelines.yaml` to make the publisher RELIABLE.
