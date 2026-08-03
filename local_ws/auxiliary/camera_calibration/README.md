# camera_calibration

`camera_calibration` produces intrinsics for the CSI pipelines. `cam_calibrate` starts a pipeline, runs the ROS 2 `cameracalibrator` GUI against it, and writes the result to the calibration store and to the manager's calibration file.

## Running a calibration

A run needs a NoMachine session for the GUI and an interactive terminal for the board prompt. The script is menu option 6 of `setup.sh` and the `cam_calibrate` alias.

```bash
cam_calibrate front
```

The argument selects the pipeline.

| Argument | Pipeline `<cam>` | Sensor | Frame |
| --- | --- | --- | --- |
| `front` | `cam_front` | IMX219, `sensor-id=0` | `top_visual_link` |
| `down` | `cam_down` | IMX219, `sensor-id=1` | `bottom_visual_link` |

The first prompt offers a custom board, taking columns and rows in squares and the square size in millimetres. Enter or `n` uses `calibration_pattern/calib_pattern.pdf`, 10x7 squares at 50 mm.

## The calibrator window

Move the board through the frame until CALIBRATE activates, click it, then click COMMIT or SAVE.

> COMMIT writes `/tmp/ost.yaml`; SAVE writes `/tmp/calibrationdata.tar.gz`, which the script unpacks. Either produces a calibration.

## Applying the result

The manager loads calibrations from the installed share, so a new file reaches the pipeline only after a build.

| Path | Content |
| --- | --- |
| `camera_calibrations/<cam>/<cam>.yaml` | Latest result, tracked. |
| `camera_calibrations/<cam>/<cam>_<timestamp>.yaml` | Per-run copy, gitignored. |
| `local_ws/src/ros_gst_cameras/gst_camera_manager/config/calibrations/<cam>.yaml` | Copy the build installs. |

`pipelines.yaml` ships both pipelines with `calibration: ""`. Set it to `"<cam>"`, run `colcon_local`, then `<cam>_stop` and `<cam>_start`.

## Services called

The script starts `gst_camera_manager.service` when no `/gst_camera_manager/` service is registered.

| Service | Type | When |
| --- | --- | --- |
| `/gst_camera_manager/<cam>` | `std_srvs/srv/SetBool` | `data: true` starts the pipeline; `data: false` stops it before the run and at exit. |
| `/camera/set_camera_info` | `sensor_msgs/srv/SetCameraInfo` | GUI COMMIT. No server exists. |

## Topics

| Subscribed | Type | Use |
| --- | --- | --- |
| `/<cam>/image_raw` | `sensor_msgs/msg/Image` | Frames the calibrator detects the board in; QoS matched to the publisher. |

| Published by the pipeline | Type | Content |
| --- | --- | --- |
| `/<cam>/image_raw` | `sensor_msgs/msg/Image` | 1920x1080 `mono8` at 15 fps, BEST_EFFORT depth 5. |
| `/<cam>/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | Same frames through `image_transport`, encoded only while subscribed. |
| `/<cam>/camera_info` | `sensor_msgs/msg/CameraInfo` | Intrinsics, stamped with the image time and the same frame. |

## Environment

Setting `SIZE` and `SQUARE` skips the board prompt. Bash does not expand the `cam_calibrate` alias after a variable assignment, so prefix the assignments to the script path.

| Variable | Default | Effect |
| --- | --- | --- |
| `SIZE` | `9x6` | Interior corners, one less than squares per side. |
| `SQUARE` | `0.050` | Square side in metres. |
| `NM_WAIT_S` | `180` | Seconds to wait for a NoMachine display before exiting. |
| `ROS_DOMAIN_ID` | `23` | Domain carrying the pipeline services and topics. |
| `DISPLAY` | session value | Probed first; otherwise `/tmp/.X11-unix` is scanned, highest number first. |
| `XAUTHORITY` | `~/.Xauthority` | Source of the display cookie, copied to a temporary authority file. |
| `WORKSPACES` | repo root | Root the store and the manager calibration are written under. |

```bash
SIZE=<corners> SQUARE=<metres> bash "$WORKSPACES"/local_ws/auxiliary/camera_calibration/camera_calibration_auto/camera_calibrate.sh front
```

## Exit codes

| Code | Meaning |
| --- | --- |
| 0 | Calibration written, or the display wait cancelled or timed out. |
| 1 | OpenCV, ROS 2 Humble, or `gst_camera_manager` unavailable. |
| 2 | Argument was not `front` or `down`. |
| 3 | No frames on `/<cam>/image_raw`. |
| 4 | GUI closed without a calibration. |
| 5 | No display and no terminal to prompt on. |
| 6 | Not an interactive terminal. |
