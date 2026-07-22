# camera_calibration

The CSI camera calibration launcher, wrapping ROS 2's `camera_calibration` GUI tool and isolating itself in a local `calib_env/` venv pinned to `numpy<2` (ROS 2 Humble's `cv_bridge` is compiled against NumPy 1.x and breaks with NumPy 2.x). The GUI renders over a **NoMachine remote display** to the workstation.

## `camera_calibration_auto/camera_calibrate.sh`

This is the calibrator wired into `setup.sh` (menu option 6). It targets the two CSI pipelines, `cam_front` and `cam_down`, driving the selected one through the live `gst_camera_manager` service. Pick the camera with a `front|down` argument:

```bash
./camera_calibration_auto/camera_calibrate.sh front
./camera_calibration_auto/camera_calibrate.sh down
```

For the selected camera the script:

- brings the pipeline up via the `gst_camera_manager` service (starting the systemd unit if needed), waits for frames, runs the interactive calibrator, then stops the camera on exit;
- on completion, saves the result to the calibration store (`camera_calibrations/cam_front/` or `camera_calibrations/cam_down/`, timestamped record + current `cam_front.yaml` / `cam_down.yaml`) **and** applies it to the live pipeline calibration file (`ros_gst_cameras/gst_camera_manager/config/calibrations/cam_front.yaml` or `cam_down.yaml`), so it takes effect on the next pipeline restart;
- gates on an attached NoMachine viewer before starting the camera: waits up to `NM_WAIT_S` seconds (default 180), then skips calibration cleanly.

The `front|down` argument is required; a missing or unknown argument prints usage and exits.

Interactive: prompts for a custom board (columns / rows in squares + square size in mm) and converts to interior corners = squares - 1. Enter or `n` uses the included default board at `../calibration_pattern/calib_pattern.pdf` (10x7 squares / 50 mm).

Env overrides (skip the prompt; useful for automation):

```bash
SIZE=9x6 SQUARE=0.050 ROS_DOMAIN_ID=23 NM_WAIT_S=180 \
    ./camera_calibration_auto/camera_calibrate.sh down
```

After the first successful run, set `calibration: "cam_front"` / `calibration: "cam_down"` in the matching entry of `pipelines.yaml` (both ship empty) so the pipeline loads the new intrinsics.

Prompts are driven through the controlling terminal (`/dev/tty`) rather than plain `read -p`, because `setup.sh` runs everything under `exec > >(tee -a ...)`, where a `read -p` prompt block-buffers in the tee pipe and never reaches the operator.

## Files

- `camera_calibration_auto/camera_calibrate.sh`: the `front|down` CSI calibrator wired into `setup.sh`; applies to the live gst store.
- `calibration_pattern/calib_pattern.pdf`: the included 10x7-square / 50 mm checkerboard.
- `camera_calibrations/`: per-camera calibration store (the current `<name>.yaml` stays tracked, timestamped records are gitignored).
- `calib_env/`: auto-created venv (gitignored).
