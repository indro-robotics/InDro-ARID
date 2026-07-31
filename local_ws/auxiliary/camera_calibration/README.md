# camera_calibration

This directory holds the `cam_down` calibration launcher, which wraps the ROS 2
`camera_calibration` GUI in an isolated virtualenv. The GUI renders on the NoMachine remote
desktop, so connect a session before starting.

## Running

`camera_calibration_auto/camera_calibrate.sh` takes no arguments and is also reachable as the
`cam_calibrate` alias and as setup menu option **9**.

```bash
./camera_calibration_auto/camera_calibrate.sh
```

The script starts `cam_down` through the `gst_camera_manager` service, starting the systemd unit
first if it is not running, waits for frames, runs the calibrator, and stops the camera on exit. If
no NoMachine viewer appears within `NM_WAIT_S` seconds it exits cleanly without calibrating.

A successful run saves a timestamped record and the current `cam_down.yaml` under
`camera_calibrations/cam_down/`, and copies the result into
`ros_gst_cameras/gst_camera_manager/config/calibrations/cam_down.yaml`. It takes effect on the
next pipeline restart.

## Board

The script prompts for a custom board (columns and rows in squares, plus square size in
millimetres). Pressing Enter or answering `n` uses the included 10x7-square, 50 mm board at
`calibration_pattern/calib_pattern.pdf`.

Setting the environment variables skips the prompt:

```bash
SIZE=9x6 SQUARE=0.050 NM_WAIT_S=180 ./camera_calibration_auto/camera_calibrate.sh
```

`SIZE` is interior corners (one less than squares in each dimension), `SQUARE` is the side length
in metres, and `NM_WAIT_S` is the NoMachine wait in seconds, defaulting to 180.

## Applying the result

`pipelines.yaml` ships with an empty `calibration` field for `cam_down`. After the first
successful run, set `calibration: "cam_down"` on that entry so the pipeline loads the new
intrinsics.

## Files

- `camera_calibration_auto/camera_calibrate.sh`: the calibrator.
- `calibration_pattern/calib_pattern.pdf`: the included checkerboard.
- `camera_calibrations/`: the per-camera store, created on the first run.
