# csi_drone_camera

GStreamer-based camera stack for the Cypher drone. Two packages:

- **`gst_cam_node`** — C++ node that wraps any GStreamer pipeline and publishes `image_raw` + `camera_info`
- **`gst_camera_manager`** — Python manager that launches/stops named pipelines via ROS2 services

---

## Quick Start

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

### Start a pipeline
```bash
ros2 service call /gst_camera_manager/phoenix_4k std_srvs/srv/SetBool "{data: true}"
```

### Stop a pipeline
```bash
ros2 service call /gst_camera_manager/phoenix_4k std_srvs/srv/SetBool "{data: false}"
```

### Stop all pipelines
```bash
ros2 service call /gst_camera_manager/stop_all std_srvs/srv/Trigger "{}"
```

### Check pipeline status
```bash
ros2 service call /gst_camera_manager/phoenix_4k/status std_srvs/srv/Trigger "{}"
ros2 service call /gst_camera_manager/status_all std_srvs/srv/Trigger "{}"
```

### Check frames arriving
```bash
ros2 topic hz /scan_cam/image_raw
```

---

## LUCID Phoenix PHX124S-M (GigE Vision, 4096×3000 Mono8 @ 5fps)

Configured in `gst_camera_manager/config/pipelines.yaml` as `phoenix_4k`.

**Prerequisites:**
- Aravis built from source (`arv-tool-0.10` available) — run `setup.sh`
- Ethernet interface configured for camera subnet — run `setup.sh`
- `GST_PLUGIN_PATH=/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0` (set by setup.sh in `.bashrc`, injected automatically by the manager node)

**Verify camera detected:**
```bash
arv-tool-0.10
# → Lucid Vision Labs-PHX124S-M-XXXXXXX (192.168.10.10)
```

**Test pipeline directly:**
```bash
gst-launch-1.0 aravissrc \
  features="AcquisitionMode=Continuous PixelFormat=Mono8 AcquisitionFrameRateEnable=true AcquisitionFrameRate=5.0" \
  ! video/x-raw,format=GRAY8,width=4096,height=3000,framerate=5/1 \
  ! fakesink sync=false
```

**Published topics:**
- `/scan_cam/image_raw` (`sensor_msgs/Image`, mono8)
- `/scan_cam/camera_info` (`sensor_msgs/CameraInfo`)

---

## Adding a CSI Camera (Argus / IMX477)

Add a new entry to `pipelines.yaml`:

```yaml
pipelines:
  csi_down:
    gst_pipeline: >-
      nvarguscamerasrc sensor-id=0 !
      video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1 !
      nvvidconv !
      video/x-raw,format=BGRx !
      videoconvert !
      video/x-raw,format=BGR !
      appsink sync=false
    calibration: "csi_down"
    topic: "csi_down"
    frame_id: "csi_down_frame"
    encoding: "bgr8"
```

Then add a matching calibration YAML in `config/calibrations/csi_down.yaml`.

Start/stop it the same way:
```bash
ros2 service call /gst_camera_manager/csi_down std_srvs/srv/SetBool "{data: true}"
```

**Published topics:**
- `/csi_down/image_raw`
- `/csi_down/camera_info`

---

## Adding a New Pipeline Generally

Each pipeline entry in `pipelines.yaml` requires:

| Field | Description |
|---|---|
| `gst_pipeline` | Full GStreamer pipeline string ending with `appsink sync=false` |
| `calibration` | Filename (without `.yaml`) in `config/calibrations/` |
| `topic` | ROS topic prefix — publishes to `/<topic>/image_raw` |
| `frame_id` | TF frame ID stamped on each image |
| `encoding` | OpenCV encoding string: `mono8`, `bgr8`, `rgb8` |

The pipeline is passed to OpenCV's `VideoCapture` with `CAP_GSTREAMER`. Any pipeline that ends with `appsink sync=false` and produces frames compatible with the specified encoding will work.

Logs are written to:
```
~/workspaces/local_ws/install/gst_camera_manager/share/gst_camera_manager/logs/<pipeline_name>/
```
