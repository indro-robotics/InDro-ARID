# camera_calibration

This directory holds the CSI camera calibrator and the checkerboard it defaults to. The calibrator runs the ROS 2 `cameracalibrator` GUI against a live `cam_front` or `cam_down` pipeline, and that GUI renders on a NoMachine display.

## Calibrating

`cam_calibrate` takes `front` or `down` and must be run from an interactive terminal; `setup.sh` menu option **6** prompts for the camera.

```bash
cam_calibrate front
cam_calibrate down
```

The run starts `gst_camera_manager.service` if that service is not already active, stops and restarts the selected pipeline, and waits for frames on `/<cam>/image_raw` before opening the calibrator window. In the GUI, move the board through the frame, then click Calibrate followed by Commit or Save; without one of those the run ends with no calibration written. The pipeline is stopped again when the script exits.

If no NoMachine display is ready, the run waits up to `NM_WAIT_S` seconds, 180 by default, and then exits without calibrating. Enter cancels the wait.

## Board

The script asks whether to use a custom board. Answering `y` prompts for columns and rows in squares plus the square size in millimetres; Enter or `n` uses the included `calibration_pattern/calib_pattern.pdf`, 10x7 squares at 50 mm.

Setting both `SIZE` and `SQUARE` skips that prompt. `SIZE` is interior corners, one less per side than the square count, and `SQUARE` is in metres. `NM_WAIT_S` overrides the display wait. Prefix the assignments to the script rather than to `cam_calibrate`, which bash does not expand after an assignment.

```bash
SIZE=7x5 SQUARE=0.030 NM_WAIT_S=60 "$WORKSPACES"/local_ws/auxiliary/camera_calibration/camera_calibration_auto/camera_calibrate.sh down
```

## Loading a result

Each successful run writes `camera_calibrations/<cam>/<cam>.yaml` plus a timestamped copy, and puts the same file at `local_ws/src/ros_gst_cameras/gst_camera_manager/config/calibrations/<cam>.yaml`.

The manager reads calibrations from the built workspace, so a file created since the last build is not yet installed. Set `calibration: "<cam>"` in the manager's `pipelines.yaml`, which ships empty for both pipelines, then run `colcon_local` and start the pipeline with `cam_front_start` or `cam_down_start`. Editing `pipelines.yaml` after that build needs `cam_refresh`, since the manager holds the config it read at startup.

## Files

The directory holds the calibrator, the pattern and two trees created on first use.

- `camera_calibration_auto/camera_calibrate.sh`: the `front|down` calibrator.
- `calibration_pattern/calib_pattern.pdf`: the included 10x7-square, 50 mm checkerboard.
- `camera_calibrations/<cam>/`: the per-camera store; timestamped copies are gitignored.
- `camera_calibration_auto/calib_env/`: the calibrator's virtualenv, gitignored.
