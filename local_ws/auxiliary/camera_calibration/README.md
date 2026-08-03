# camera_calibration

The camera_calibration package produces intrinsics for the `cam_front` and `cam_down` CSI pipelines. `cam_calibrate` runs the ROS 2 `cameracalibrator` GUI against a live pipeline and writes the result into the calibration store.

## Board

`calibration_pattern/calib_pattern.pdf` is a 10x7-square chessboard with 50 mm squares, matching the `SIZE` and `SQUARE` defaults. The custom-board prompt takes squares and millimetres, not corners and metres.

## Running a calibration

`cam_calibrate` takes `front` or `down` and requires an interactive terminal with a connected NoMachine session. Menu option 6 of `setup.sh` prompts for the camera and runs the same script.

```bash
cam_calibrate front
cam_calibrate down
```

| Argument | Pipeline `<cam>` | Sensor | Frame |
| --- | --- | --- | --- |
| `front` | `cam_front` | `sensor-id=0` | `top_visual_link` |
| `down` | `cam_down` | `sensor-id=1` | `bottom_visual_link` |

Move the board through the frame in the calibrator window, then click Calibrate and Save.

> Commit calls `/camera/set_camera_info`, which no node hosts. Save writes `/tmp/calibrationdata.tar.gz`, which the run unpacks.

## Services called

The run starts `gst_camera_manager.service` when no `/gst_camera_manager/` service is registered.

| Service | Type | When |
| --- | --- | --- |
| `/gst_camera_manager/<cam>` | `std_srvs/srv/SetBool` | `false` before the run and at exit, `true` to start the pipeline. |
| `/camera/set_camera_info` | `sensor_msgs/srv/SetCameraInfo` | GUI Commit. No node hosts it. |

## Topics

Both pipelines stream 1920x1080 `mono8` frames at 15 fps.

| Subscribed | Type | Use |
| --- | --- | --- |
| `/<cam>/image_raw` | `sensor_msgs/msg/Image` | Frames the calibrator detects the board in; QoS matched to the publisher. |

| Published by the pipeline | Type | Content |
| --- | --- | --- |
| `/<cam>/image_raw` | `sensor_msgs/msg/Image` | Pipeline frames, BEST_EFFORT depth 5. |
| `/<cam>/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | Same frames through `image_transport`, encoded only while subscribed. |
| `/<cam>/camera_info` | `sensor_msgs/msg/CameraInfo` | Intrinsics on the image stamp and frame, BEST_EFFORT depth 5. |

| Published by the manager | Type | Content |
| --- | --- | --- |
| `/gst_camera_manager/<cam>/alive` | `std_msgs/msg/Bool` | `true` on start, `false` after 5 s without `camera_info`; RELIABLE TRANSIENT_LOCAL depth 1. |

Without a calibration, `camera_info` carries zero distortion, `fx` = `fy` = width and a centred principal point.

## Applying the result

Every run writes all three paths.

| Path | Content |
| --- | --- |
| `camera_calibrations/<cam>/<cam>.yaml` | Latest result. |
| `camera_calibrations/<cam>/<cam>_<timestamp>.yaml` | Per-run copy, gitignored. |
| `local_ws/src/ros_gst_cameras/gst_camera_manager/config/calibrations/<cam>.yaml` | Copy the build installs. |

The manager reads calibrations from the installed share, so a new file needs a build. Set `calibration: "<cam>"` in `pipelines.yaml`, which ships empty, run `colcon_local`, then `<cam>_stop` and `<cam>_start`. A later `pipelines.yaml` edit takes effect on `cam_refresh`.

## Environment

Setting `SIZE` and `SQUARE` skips the board prompt.

| Variable | Default | Effect |
| --- | --- | --- |
| `SIZE` | `9x6` | Interior corners, one less than squares per side. |
| `SQUARE` | `0.050` | Square side in metres; the included pattern printed at full scale. |
| `NM_WAIT_S` | `180` | Seconds waited for a NoMachine display before the run exits. |
| `ROS_DOMAIN_ID` | `23` | Domain carrying the pipeline services and topics. |
| `DISPLAY` | session value | Probed first; otherwise `/tmp/.X11-unix` is scanned, highest number first. |
| `XAUTHORITY` | `~/.Xauthority` | Source of the display cookie, copied to a temporary authority file. |
| `WORKSPACES` | repo root | Root the store and the manager calibration are written under. |

Bash does not expand the `cam_calibrate` alias after a variable assignment, so prefix the assignments to the script path.

```bash
SIZE=<corners> SQUARE=<metres> bash "$WORKSPACES"/local_ws/auxiliary/camera_calibration/camera_calibration_auto/camera_calibrate.sh front
```

## Exit codes

| Code | Meaning |
| --- | --- |
| 0 | Calibration written, or the display wait cancelled or timed out. |
| 1 | OpenCV, ROS 2 Humble, or the pipeline service unavailable. |
| 2 | Argument was not `front` or `down`. |
| 3 | No frames on `/<cam>/image_raw`; check whether the Isaac container holds the sensor. |
| 4 | GUI closed without a calibration. |
| 5 | No display and no terminal to prompt on. |
| 6 | Not an interactive terminal. |
