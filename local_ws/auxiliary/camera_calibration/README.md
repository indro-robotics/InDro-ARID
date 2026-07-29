# camera_calibration

This directory holds the CSI camera calibration launcher, which wraps the ROS 2 `camera_calibration` GUI. The GUI renders over a NoMachine remote display, so connect a session first.

## `camera_calibration_auto/camera_calibrate.sh`

The script calibrates `cam_front` or `cam_down` and is wired into `setup.sh` as menu option 6. The camera argument is required:

```bash
./camera_calibration_auto/camera_calibrate.sh front
./camera_calibration_auto/camera_calibrate.sh down
```

It starts the pipeline through the `gst_camera_manager` service, starting the systemd unit if needed, waits for frames, runs the calibrator, and stops the camera on exit. It waits up to `NM_WAIT_S` seconds, 180 by default, for a NoMachine viewer, then skips cleanly. The result is saved to the store, `camera_calibrations/cam_front/` or `cam_down/`, as a timestamped record plus the current `<cam>.yaml`, and applied to the live pipeline file at `ros_gst_cameras/gst_camera_manager/config/calibrations/<cam>.yaml`, taking effect on the next pipeline restart.

The script prompts for a board, taking columns and rows in squares plus square size in millimetres. Enter or `n` uses the included 10x7-square, 50 mm board at `../calibration_pattern/calib_pattern.pdf`.

Environment overrides skip the prompt:

```bash
SIZE=9x6 SQUARE=0.050 ROS_DOMAIN_ID=23 NM_WAIT_S=180 \
    ./camera_calibration_auto/camera_calibrate.sh down
```

After the first successful run, set `calibration: "cam_front"` or `calibration: "cam_down"` in `pipelines.yaml`, since both default to empty, so the pipeline loads the new intrinsics.

## Files

- `camera_calibration_auto/camera_calibrate.sh`: the `front|down` calibrator.
- `calibration_pattern/calib_pattern.pdf`: the included 10x7-square, 50 mm checkerboard.
- `camera_calibrations/`: the per-camera store; the current `<name>.yaml` is tracked and timestamped records are gitignored.
- `camera_calibration_auto/calib_env/`: an auto-created venv, gitignored.
