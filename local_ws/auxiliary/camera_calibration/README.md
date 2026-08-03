# camera_calibration

The CSI camera calibrator produces intrinsics for the `cam_front` and `cam_down` pipelines. It runs the ROS 2 `cameracalibrator` GUI against a live pipeline on a NoMachine display.

## Running a calibration

`cam_calibrate` takes `front` or `down` and requires an interactive terminal and a connected NoMachine session. Menu option 6 of `setup.sh` prompts for the camera and runs the same script.

```bash
cam_calibrate front
cam_calibrate down
```

The run prompts for the board, starts the pipeline, and waits for frames before opening the calibrator window. In the GUI, move the board through the frame, then click Calibrate and Save.

> Commit calls `/camera/set_camera_info`, which no node hosts. Save writes the calibration into `/tmp/calibrationdata.tar.gz`, which the script unpacks.

## Board

The script asks whether to use a custom board. Answering `y` prompts for columns, rows and square size in millimetres; `n` uses `calibration_pattern/calib_pattern.pdf`, 10x7 squares at 50 mm.

## Loading a result

Each run writes the result into the calibration store and into the manager's source tree. The manager loads calibrations from the installed share, so a new file reaches a pipeline only after a build.

| Path | Content |
| --- | --- |
| `camera_calibrations/<cam>/<cam>.yaml` | Latest result. |
| `camera_calibrations/<cam>/<cam>_<timestamp>.yaml` | Per-run copy, gitignored. |
| `local_ws/src/ros_gst_cameras/gst_camera_manager/config/calibrations/<cam>.yaml` | Copy the build installs. |

Set `calibration: "<cam>"` in the manager's `pipelines.yaml`, which ships empty for both pipelines, then run `colcon_local` and start the pipeline with `cam_front_start` or `cam_down_start`. A `pipelines.yaml` edit made after that build takes effect on `cam_refresh`.

## Services called

The script starts `gst_camera_manager.service` when no `/gst_camera_manager/` service is registered.

| Service | Type | When |
| --- | --- | --- |
| `/gst_camera_manager/<cam>` | `std_srvs/srv/SetBool` | `true` starts the pipeline; `false` stops it before the run and at exit. |
| `/camera/set_camera_info` | `sensor_msgs/srv/SetCameraInfo` | GUI Commit. No server exists. |

## Topics

`<cam>` is `cam_front` or `cam_down`; their headers carry frame `top_visual_link` and `bottom_visual_link`.

| Subscribed | Type | Use |
| --- | --- | --- |
| `/<cam>/image_raw` | `sensor_msgs/msg/Image` | Frames the calibrator detects the board in. |

| Published by the pipeline | Type | Content |
| --- | --- | --- |
| `/<cam>/image_raw` | `sensor_msgs/msg/Image` | Pipeline frames, BEST_EFFORT depth 5. |
| `/<cam>/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | Same frames through `image_transport`. |
| `/<cam>/camera_info` | `sensor_msgs/msg/CameraInfo` | Intrinsics, stamped with the image time. |

## Environment

| Variable | Default | Effect |
| --- | --- | --- |
| `SIZE` | `9x6` | Interior corners, one less than squares per side. |
| `SQUARE` | `0.050` | Square side in metres. |
| `NM_WAIT_S` | `180` | Seconds to wait for a NoMachine display before exiting. |
| `ROS_DOMAIN_ID` | `23` | Domain carrying the pipeline services and topics. |
| `WORKSPACES` | repo root | Root the store and the manager calibration are written under. |

Setting `SIZE` and `SQUARE` skips the board prompt. Bash does not expand the `cam_calibrate` alias after a variable assignment, so prefix the assignments to the script path.

```bash
SIZE=<corners> SQUARE=<metres> bash "$WORKSPACES"/local_ws/auxiliary/camera_calibration/camera_calibration_auto/camera_calibrate.sh <camera>
```

## Exit codes

| Code | Meaning |
| --- | --- |
| 0 | Calibration written, or the display wait cancelled or timed out. |
| 1 | ROS 2 Humble, OpenCV, or `gst_camera_manager` unavailable. |
| 2 | Argument was not `front` or `down`. |
| 3 | No frames on `/<cam>/image_raw`. |
| 4 | GUI closed without a calibration. |
| 5 | No display and no terminal to prompt on. |
| 6 | Not an interactive terminal. |
