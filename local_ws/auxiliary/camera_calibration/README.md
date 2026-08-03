# camera_calibration

The `cam_down` calibration wraps the ROS 2 `camera_calibration` GUI, which renders over NoMachine. Each run starts the pipeline, calibrates it, and writes the intrinsics to the store and to the manager's calibration file.

## Running a calibration

A run needs a NoMachine session for the GUI and an interactive terminal for the board prompt. The script is menu option 9 of `setup.sh` and the `cam_calibrate` alias.

```bash
cam_calibrate
```

The first prompt offers a custom board, taking columns and rows in squares and the square size in millimetres. Enter or `n` uses `calibration_pattern/calib_pattern.pdf`, 10x7 squares at 50 mm.

## The calibrator window

Move the board through the frame until CALIBRATE activates, click it, then click SAVE.

> COMMIT uploads to `/camera/set_camera_info`, which no node hosts. SAVE writes `/tmp/calibrationdata.tar.gz`, which the script unpacks.

## Applying the result

The manager loads calibrations from the installed share, so a new file reaches the pipeline only after a build.

| Path | Content |
| --- | --- |
| `camera_calibrations/cam_down/cam_down.yaml` | Latest result, tracked. |
| `camera_calibrations/cam_down/cam_down_<timestamp>.yaml` | Per-run copy, gitignored. |
| `local_ws/src/ros_gst_cameras/gst_camera_manager/config/calibrations/cam_down.yaml` | Copy the build installs. |

`pipelines.yaml` ships `cam_down` with `calibration: "IMX477_1080sq"`. Set it to `"cam_down"`, run `colcon_local`, then `cam_down_start`.

## Services called

The script starts `gst_camera_manager.service` when no `/gst_camera_manager/` service is registered.

| Service | Type | When |
| --- | --- | --- |
| `/gst_camera_manager/cam_down` | `std_srvs/srv/SetBool` | `data: true` starts the pipeline; `data: false` stops it before the run and at exit. |
| `/camera/set_camera_info` | `sensor_msgs/srv/SetCameraInfo` | GUI COMMIT. No server exists. |

## Topics

| Subscribed | Type | Use |
| --- | --- | --- |
| `/cam_down/image_raw` | `sensor_msgs/msg/Image` | Frames the calibrator detects the board in. |

| Published by the pipeline | Type | Content |
| --- | --- | --- |
| `/cam_down/image_raw` | `sensor_msgs/msg/Image` | 1080x1080 GRAY8 at 15 fps, frame `bottom_visual_link`, BEST_EFFORT depth 5. |
| `/cam_down/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | Same frames through `image_transport`. |
| `/cam_down/camera_info` | `sensor_msgs/msg/CameraInfo` | Intrinsics, stamped with the image time and the same frame. |

## Environment

Setting `SIZE` and `SQUARE` skips the board prompt. Bash does not expand the `cam_calibrate` alias after a variable assignment, so prefix the assignments to the script path.

| Variable | Default | Effect |
| --- | --- | --- |
| `SIZE` | `9x6` | Interior corners, one less than squares per side. |
| `SQUARE` | `0.050` | Square side in metres. |
| `NM_WAIT_S` | `180` | Seconds to wait for a NoMachine display before exiting. |
| `ROS_DOMAIN_ID` | `23` | Domain carrying the pipeline services and topics. |
| `WORKSPACES` | repo root | Root the store and the manager calibration are written under. |

```bash
SIZE=<corners> SQUARE=<metres> bash "$WORKSPACES"/local_ws/auxiliary/camera_calibration/camera_calibration_auto/camera_calibrate.sh
```

## Exit codes

| Code | Meaning |
| --- | --- |
| 0 | Calibration written, or the display wait cancelled or timed out. |
| 1 | OpenCV, ROS 2 Humble, or `gst_camera_manager` unavailable. |
| 3 | No frames on `/cam_down/image_raw`. |
| 4 | GUI closed without a calibration. |
| 5 | No display and no terminal to prompt on. |
| 6 | Not an interactive terminal. |
