# camera_calibration

Wrapper around ROS 2's `camera_calibration` tool for running the GUI calibrator against any image topic on the drone.

Built to work over a **NoMachine remote display**. The calibrator GUI renders on the Jetson and streams back to the workstation. The script auto-detects the `DISPLAY` socket under `/tmp/.X11-unix/` so no manual `DISPLAY` export is needed after SSH or NoMachine connects.

It also isolates itself in a local venv (`calib_env/`) pinned to `numpy<2`, because ROS 2 Humble's `cv_bridge` is compiled against NumPy 1.x and breaks with NumPy 2.x.

## Usage

With no arguments, the script queries the ROS graph for every live `sensor_msgs/msg/Image` topic and presents an interactive numbered menu. Pick the camera to calibrate:

```bash
./camera_calibrate.sh
```

Example output:

```
[calib] Select an image topic to calibrate (or Ctrl-C to abort):
1) /cam_down/image_raw
[calib] > 1
```

`CAMERA_NS` is auto-derived from the selected topic (strips the trailing segment, e.g. `/cam_down/image_raw` becomes `/cam_down`).

**Scripted (non-interactive):** set `IMAGE_TOPIC` explicitly and the menu is skipped:

```bash
SIZE=9x7 SQUARE=0.030 IMAGE_TOPIC=/cam_down/image_raw \
    ./camera_calibrate.sh
```

Click **Save** in the GUI before closing. The script extracts `ost.yaml` and writes it next to itself as `<topic_slug>_calibration.yaml` (e.g. `cam_down_image_raw_calibration.yaml`). Re-calibrating the same topic overwrites the previous file.

## Files

- `camera_calibrate.sh`: the launcher.
- `calib_env/`: auto-created venv (gitignored).
- `*_calibration.yaml`: saved calibrations, one per topic slug (gitignored).
