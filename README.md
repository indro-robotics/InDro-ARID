# ARID

ARID is a quadrotor on an NVIDIA Jetson Orin with an ARK PAB carrier, running ROS 2 Humble, Isaac ROS and PX4. This repository provisions the drone and runs its camera and visual-odometry stacks. ROS 2 traffic stays on the drone at `ROS_DOMAIN_ID=23`.

- [`setup.sh`](setup.sh): provisioning, from a fresh flash to both workspaces built.
- [`arid_description`](local_ws/src/arid_description/): robot description on `/robot_description` and `/tf_static`.
- [`ros_gst_cameras`](local_ws/src/ros_gst_cameras/): the two IMX219 CSI pipelines.
- [`reset_ark_usb`](local_ws/src/reset_ark_usb/): the `/reset_usb` service.
- [`px4_vslam`](isaac_ros-dev/src/px4_vslam/), [`px4_vslam_reactor`](isaac_ros-dev/src/px4_vslam_reactor/), [`arid_supervisor`](isaac_ros-dev/src/arid_supervisor/): RealSense visual odometry into PX4.
- [`camera_calibration`](local_ws/auxiliary/camera_calibration/): the CSI calibrator and its pattern.
- [`PX4-Autopilot`](local_ws/auxiliary/PX4-Autopilot/): the InDro PX4 fork carrying the ARID airframe.

## Sensors

| Sensor | Fit |
|---|---|
| 3x RealSense | Front, left, right; IR stereo into cuVSLAM. |
| 2x IMX219 CSI | `cam_front` forward, `cam_down` downward. |
| Optical flow, rangefinder | Underside. |

## After boot

A login shell carries the alias set with `local_ws` sourced. The units below run at boot, with both CSI pipelines idle and VSLAM idle until `initialize`.

| Unit | Function |
|---|---|
| `usbfs-memory` | usbfs buffer at 1000 MB. |
| `jetson-clocks` | Clocks and fan at maximum. |
| `start_isaac_docker` | The Isaac container. |
| `arid_description` | `robot_state_publisher`. |
| `gst_camera_manager` | The CSI camera manager. |
| `usb_ros_reset` | Hosts `/reset_usb`. |
| `arid_supervisor` | VSLAM lifecycle, inside the container. |

## Aliases

`help` prints this set in a host shell.

| Alias | Action |
|---|---|
| `setup` | Run `setup.sh`. |
| `run_isaac` / `start_isaac` / `stop_isaac` / `isaac_bash` | Container: run, start, stop, shell. |
| `build_isaac` | Rebuild the container image. |
| `colcon_isaac` | Deinitialize, build the container workspace, restart the supervisor. |
| `clean_isaac` / `rosdep_isaac` | Clean or rosdep the container workspace. |
| `colcon_local` | Build `local_ws`, stopping and restarting its host services. |
| `clean_local` / `rosdep_local` | Clean or rosdep `local_ws`. |
| `reset_usb` | USB hub reset. |
| `cam_front_*` / `cam_down_*` | Per pipeline: `start`, `stop`, `status`, `alive`. |
| `cam_stop` | Stop both pipelines. |
| `cam_refresh` | Re-read `pipelines.yaml`, stopping running pipelines first. |
| `cam_calibrate front\|down` | Calibrate a CSI camera. |
| `config_realsense` | Assign the three RealSense serials. |
| `local_test` | Host-stack smoke test. |
| `ver_cv_cams` | Live CSI camera feeds. |
| `wifi` / `zt_join` / `update_submods` | Wi-Fi picker, ZeroTier, submodule sync. |
| `foxglove_bridge` | Bridge on port 8765. |
| `initialize` / `deinitialize` / `status` | VSLAM through the supervisor. |

Inside the container `help` prints its own smaller set, in which `colcon_isaac` runs the colcon build alone, without the deinitialize and the supervisor restart the host alias adds.

## CSI video

Both pipelines run at 1920x1080, 15 fps, GRAY8, uncalibrated. No node in either workspace subscribes to their image topics.

| Pipeline | Sensor | Frame | Topic root |
|---|---|---|---|
| `cam_front` | `sensor-id=0` | `top_visual_link` | `/cam_front` |
| `cam_down` | `sensor-id=1` | `bottom_visual_link` | `/cam_down` |

A running pipeline publishes `image_raw`, `image_raw/compressed` and `camera_info` under its topic root, and reports frame flow on the latched `/gst_camera_manager/<name>/alive`. The manager tees its log to `isaac_ros-dev/run_logs/gst_camera_manager/`. Pipeline fields and troubleshooting are in [`ros_gst_cameras/README.md`](local_ws/src/ros_gst_cameras/README.md).

## Visual odometry

Isaac cuVSLAM runs six-stream stereo over the three RealSense, one IR pair per camera at 640x360x60. The reactor gates jumps and re-seats that solution against the PX4 pose. `vio_transform` publishes it on `/fmu/in/vehicle_visual_odometry`.

> Never select a 90 fps RealSense profile. That USB service interval stalls the bus.

`initialize` blocks while the supervisor proves all three cameras up: about 15 s on a healthy bringup, up to about 3 minutes when its one recovery cycle runs, over at most three attempts. `deinitialize` is refused unless a landed sample no older than 3.5 s proves the drone is down. Each launch writes `run_logs/vslam/vslam.log`.

Bringup and interlock detail is in [`arid_supervisor/README.md`](isaac_ros-dev/src/arid_supervisor/README.md), filter tunables in [`px4_vslam_reactor/README.md`](isaac_ros-dev/src/px4_vslam_reactor/README.md).

### RealSense serials

`config_realsense` identifies each camera from a live feed and pins its serial to the front, left or right mount in `vslam_config.yaml`. Run it after a camera swap.

## USB reset

`reset_usb` power-cycles the ARK PAB USB hub and the standalone USB3 port. The `/reset_usb` service runs the same script and answers callers on the host and inside the container.

> Never reset USB in flight. The power cycle reboots the flight controller and drops all three RealSense off the bus.

Failure causes are in [`reset_ark_usb/README.md`](local_ws/src/reset_ark_usb/README.md).

## Foxglove

`foxglove_bridge` starts the bridge on port 8765 from a host or container shell. Connect Foxglove Studio to `ws://<device-ip>:8765`. The URDF is on `/robot_description` with panel frame `base_link`.

## Health checks

`local_test` runs the host-stack smoke test over the units, the aliases and both CSI pipelines. It leaves the pipelines stopped.

`ver_cv_cams` opens the live CSI feeds in a window on the NoMachine desktop, where any key advances and `q` quits. Pass `front` or `down` for a single feed.

## setup.sh

Running `./setup.sh` with no arguments opens the menu below. Every step detects what is in place and does only what is missing.

| Option | Action |
|---|---|
| **1** | Full setup |
| **2** | Smoke test |
| **3** | RealSense serial assignment |
| **4** | Camera feed check |
| **5** | Wi-Fi connect |
| **6** | Camera calibration |
| **7** | Camera focus |
| **8** | Build the Isaac container image |
| **9** | Build the Isaac workspace and restart `arid_supervisor.service` |
| **10** | Build `local_ws` |
| **11** | Install or reinstall ARK-OS |
| **12** | Install or reinstall ROS 2 |
| **13** | ZeroTier join or switch |
| **14** | Uninstall, keeping the repo, the OS and the Docker engine |

Option **9** needs the container running, and option **12** needs the ARK-OS clone that option **11** or a full setup creates. The host `colcon_isaac` alias calls `deinitialize` first and needs the supervisor on the graph, so a container workspace that has never been built is built through option **9**. A ZeroTier join reporting `ACCESS_DENIED` completes once the node is authorized in ZeroTier Central. Each run logs to `log/setup_log_*.log`.

### Camera calibration

Menu **6** and `cam_calibrate front|down` run the ROS 2 calibrator against the live pipeline, on the NoMachine desktop. Writing and loading a result is covered in [`camera_calibration/README.md`](local_ws/auxiliary/camera_calibration/README.md).

### Camera focus

Menu **7** starts the selected CSI pipelines with the Foxglove bridge and confirms each stream is sustained. Adjust the lens while watching the image panel in Foxglove Studio. Any key stops the pipelines and the bridge.

## Full setup

`./setup.sh --full` walks the questionnaire once, then provisions unattended. Every build prompt defaults to skip, so holding Enter through the questionnaire starts no build.

```bash
./setup.sh --full
```

| Stage | Covers |
|---|---|
| Questionnaire | Every prompt, answered once and persisted. |
| Host | Hostname, password, power mode, Wi-Fi, NoMachine. |
| ARK-OS | ARK-OS, ROS 2, JetPack. |
| Phase A | Repos, apt, permissions, units, `local_ws`. Checkpointed to `~/.arid_progress`. |
| Phase B | Isaac image, container workspace, opted-in camera steps. |
| Finish | Summary, then the smoke test. |

The run reboots at most twice: at the end of Phase A when ARK-OS, ROS 2, JetPack or the CSI overlay was applied, and before the smoke test when a build ran. Open a bash terminal after each reboot and answer the prompt to resume.

`--resume` invokes that continuation by hand. `--continue` re-enters the tail after an abort, skipping the sections already checkpointed.

The run does not calibrate the CSI cameras. Calibrate both once it finishes.

## PX4 firmware

The PX4 fork is at [`local_ws/auxiliary/PX4-Autopilot/`](local_ws/auxiliary/PX4-Autopilot/) on branch `PX4-InDro` and carries airframe `4026_arid_quad_v1_2`. A built image ships in [`px4_compiled/`](local_ws/auxiliary/px4_compiled/) as `arid.px4`.

```bash
cd local_ws/auxiliary/PX4-Autopilot
make ark_fmu-v6x_default
```

The build lands at `build/ark_fmu-v6x_default/ark_fmu-v6x_default.px4`. Flash a `.px4` file from the ARK-OS web interface, then select the airframe from a MAVLink shell.

```
param set SYS_AUTOSTART 4026
param save
reboot
```

## Submodules

[`update_submods.sh`](scripts/update_submods.sh) enforces two classes: LIVE entries sit on their tracked branch at or behind its tip, PINNED entries on an exact tag. `update_submods.sh --update` advances both.

| Submodule | Class |
|---|---|
| [`PX4-Autopilot`](https://github.com/indro-robotics/PX4-Autopilot/tree/PX4-InDro) | LIVE `PX4-InDro` |
| [`realsense-ros`](https://github.com/indro-robotics/realsense-ros/tree/v4.51.1) | LIVE `v4.51.1` |
| [`isaac_ros_visual_slam`](https://github.com/indro-robotics/isaac_ros_visual_slam/tree/v3.2-14) | LIVE `v3.2-14` |
| [`px4_msgs`](https://github.com/PX4/px4_msgs/tree/release/1.15) | LIVE `release/1.15` |
| [`isaac_ros_common`](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/tree/v3.2-14) | PINNED `v3.2-14` |
| [`isaac_ros_nitros`](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros/tree/v3.2-14) | PINNED `v3.2-14` |
| [`isaac_ros_image_pipeline`](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline/tree/v3.2-14) | PINNED `v3.2-14` |
| [`px4-ros2-interface-lib`](https://github.com/Auterion/px4-ros2-interface-lib/tree/1.4.0) | PINNED `1.4.0` |

## Development

All code, builds and the live runtime stay on the drone, edited over VSCode Remote-SSH to `jetson@<device-ip>`. The host workspace bind-mounts into the container at `/workspaces/isaac_ros-dev`, so `colcon_isaac` builds the same tree from either side.
