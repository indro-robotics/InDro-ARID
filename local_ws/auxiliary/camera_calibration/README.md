# camera_calibration

CSI camera calibration launcher wrapping ROS 2's `camera_calibration` GUI. The GUI renders over a **NoMachine remote display**: connect a session first.

## `camera_calibration_auto/camera_calibrate.sh`

Calibrates `cam_front` or `cam_down`; wired into `setup.sh` (menu option 6). Required argument picks the camera:

```bash
./camera_calibration_auto/camera_calibrate.sh front
./camera_calibration_auto/camera_calibrate.sh down
```

Flow:

- starts the pipeline via the `gst_camera_manager` service (starting the systemd unit if needed), waits for frames, runs the calibrator, stops the camera on exit
- waits up to `NM_WAIT_S` seconds (default 180) for a NoMachine viewer, then skips cleanly
- saves the result to the store (`camera_calibrations/cam_front/` or `cam_down/`: timestamped record + current `<cam>.yaml`) and applies it to the live pipeline file (`ros_gst_cameras/gst_camera_manager/config/calibrations/<cam>.yaml`); takes effect on the next pipeline restart

Board: prompts for a custom board (columns/rows in squares + square size in mm), or Enter/`n` uses the included 10x7-square / 50 mm board at `../calibration_pattern/calib_pattern.pdf`.

Env overrides skip the prompt:

```bash
SIZE=9x6 SQUARE=0.050 ROS_DOMAIN_ID=23 NM_WAIT_S=180 \
    ./camera_calibration_auto/camera_calibrate.sh down
```

After the first successful run, set `calibration: "cam_front"` / `calibration: "cam_down"` in `pipelines.yaml` (both default to empty) so the pipeline loads the new intrinsics.

## Files

- `camera_calibration_auto/camera_calibrate.sh`: the `front|down` calibrator
- `calibration_pattern/calib_pattern.pdf`: included 10x7-square / 50 mm checkerboard
- `camera_calibrations/`: per-camera store (current `<name>.yaml` tracked, timestamped records gitignored)
- `camera_calibration_auto/calib_env/`: auto-created venv (gitignored)
