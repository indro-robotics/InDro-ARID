# ros_gst_cameras

ROS 2 GStreamer-based camera stack. Two packages:

- **`gst_cam_node`**: C++ node that wraps an arbitrary GStreamer pipeline and publishes `image_raw` (plus `image_raw/compressed` when `compress: true`) and `camera_info`. The pipeline is opaque to the node. Anything that produces frames into an `appsink` works: CSI via `nvarguscamerasrc`, V4L2, RTSP, file source, test pattern, etc.
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

`<name>` matches the keys in `pipelines.yaml`. Currently just `cam_down`.

#### Examples

**Start or stop a single pipeline:**
```bash
ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool "{data: true}"   # start
ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool "{data: false}"  # stop
```

**Query state of one pipeline:**
```bash
ros2 service call /gst_camera_manager/cam_down/status std_srvs/srv/Trigger "{}"
# response:
# success=True,  message='cam_down RUNNING (pid=12345)'
# or
# success=False, message='cam_down STOPPED'
```

**See all pipelines at once:**
```bash
ros2 service call /gst_camera_manager/status_all std_srvs/srv/Trigger "{}"
# response message (multi-line):
#   [RUNNING] cam_down  (pid=12345)
```
Useful for a quick "what's running" check. Also handy for ground-station UIs that want to render a panel of every camera's state without polling each `/<name>/status` individually.

**Kill everything in one call:**
```bash
ros2 service call /gst_camera_manager/stop_all std_srvs/srv/Trigger "{}"
# response message: 'stopped: cam_down'  (or 'nothing running')
```
Use when shutting down or before reconfiguring. Equivalent to calling SetBool `false` on each running pipeline.

### Verify frames
```bash
ros2 topic hz /cam_down/image_raw
ros2 topic hz /cam_down/image_raw/compressed   # only when compress: true
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
    calibration: "my_cam"          # optional: file in config/calibrations/
    topic: "my_cam"
    frame_id: "my_cam_frame"
    encoding: ""                   # "" auto-detect, or force e.g. "bgr8"
    compress: true
    alive_threshold: 2.0           # seconds (float) without camera_info before alive flips false
    # reliable: true               # optional; default is BEST_EFFORT (sensor_data QoS)
```

Rebuild (`colcon build --packages-select gst_camera_manager`) or restart the manager to pick it up. The pipeline gets its own `/gst_camera_manager/my_cam` service automatically. No code changes needed.

### Currently defined pipelines

- **`cam_down`**: CSI camera (sensor-id 0) via `nvarguscamerasrc` at 1920×1080 at 20 fps (delivered ~16 Hz), NV12 to GRAY8 via `nvvidconv`. The sensor is IR-sensitive; the pipeline drops to mono (GRAY8) so the stream is directly usable for IR-aware computer-vision tasks (feature tracking, motion detection, fiducial decoding) without per-channel filtering. Frame ID: `bottom_visual_link`.

---

## Example: CSI camera via `nvarguscamerasrc`

The currently used sensor is a Sony IMX219. Typical mode table for IMX219 on Jetson (mode numbers and max framerates are defined by the sensor driver in the device-tree overlay; confirm against the specific overlay in use):

| Mode | Resolution | Max FPS |
|---|---|---|
| 0 | 3280 × 2464 | 21 |
| 1 | 3280 × 1848 | 28 |
| 2 | 1920 × 1080 | 30 |
| 3 | 1640 × 1232 | 30 |
| 4 | 1280 × 720  | 60 |

All modes are 10-bit Bayer RGGB. `nvarguscamerasrc`'s ISP produces NV12, which `nvvidconv` converts to GRAY8 (mono) or BGRx (colour) downstream. Set `sensor-mode=N` on `nvarguscamerasrc` to pick a mode explicitly; otherwise it auto-selects based on the width, height, and framerate in the capsfilter.

Example pipeline string (mono, 1080p at 30 fps):
```
nvarguscamerasrc sensor-id=0 wbmode=1 aelock=false ee-mode=2 tnr-mode=2
  ! video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1,format=NV12
  ! nvvidconv flip-method=0 interpolation-method=1
  ! video/x-raw,format=GRAY8
  ! appsink sync=false
```

Other sensor families (IMX477, IMX219 variants, OV5693, custom modules) follow the same pattern. Consult the board's device-tree overlay for the specific mode table.

## Other example sources

| Source | Pipeline skeleton |
|---|---|
| USB / V4L2 camera | `v4l2src device=/dev/video0 ! image/jpeg,width=1280,height=720 ! jpegdec ! videoconvert ! appsink` |
| RTSP stream | `rtspsrc location=rtsp://host/stream latency=100 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink` |
| Test pattern | `videotestsrc ! video/x-raw,format=BGR,width=640,height=480 ! appsink` |

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

4. **Encoding auto-detect log line**. On first frame you should see:
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
