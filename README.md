# ARID

ARID is an indoor quadrotor built on an NVIDIA Jetson Orin with an ARK PAB carrier, running ROS 2 Humble, Isaac ROS and PX4. This repository provisions the drone and operates its flight stack. ROS 2 traffic is restricted to the drone on domain 23.

## Sensors

| Sensor | Fit |
|---|---|
| RealSense D435 | Front; IR stereo into VSLAM. |
| RoboSense RSAIRY LiDAR | Ethernet on `enP8p1s0`; point cloud only. |
| IMX477 CSI | Down; the `cam_down` pipeline. |
| ARK optical flow + rangefinder | Bottom pod. |

---

## After boot

A login shell has `local_ws` sourced and the alias set installed. These units are running; the CSI pipeline, the LiDAR and VSLAM stay idle until started.

| Unit | Function |
|---|---|
| `jetson-clocks` | Clocks and fan at maximum. |
| `usbfs-memory` | usbfs buffer at 1000 MB. |
| `start_isaac_docker` | Isaac container. |
| `arid_supervisor` | VSLAM lifecycle service, in the container. |
| `arid_description` | `robot_state_publisher` and the TF tree. |
| `gst_camera_manager` | CSI pipeline manager. |
| `rslidar_coordinator` | LiDAR supervisor. |
| `usb_ros_reset` | Hosts `/reset_usb`. |

---

## Aliases

The alias set is installed in every login shell, and `help` prints it with descriptions.

| Alias | Action |
|---|---|
| `setup` | Run `setup.sh`. |
| `start_isaac` / `stop_isaac` / `isaac_bash` / `run_isaac` | Container start, stop, shell, run and enter. |
| `build_isaac` | Rebuild the container image. |
| `colcon_isaac` / `clean_isaac` / `rosdep_isaac` | Container workspace build, clean, rosdep. |
| `colcon_local` / `clean_local` / `rosdep_local` | `local_ws` build, clean, rosdep. |
| `initialize` / `deinitialize` / `status` | VSLAM enable, disable, state. |
| `cam_down_start` / `_stop` / `_status` / `_alive` | CSI pipeline control and liveness. |
| `cam_refresh` / `cam_calibrate` | Re-read `pipelines.yaml`; calibrate `cam_down`. |
| `rslidar_start` / `_stop` / `_status` / `_alive` / `_restart` | LiDAR driver control and liveness. |
| `config_lidar` / `lidar_diag` | LiDAR address auto-detect; network diagnostic. |
| `config_realsense` | Write the RealSense serial into the VSLAM config. |
| `reset_usb` | USB hub reset. |
| `foxglove_bridge` | Bridge on port 8765. |
| `local_test` / `ver_cv_cams` | Host smoke test; live CSI feed. |
| `wifi` / `zt_join` / `update_submods` | Wi-Fi picker; ZeroTier join; submodule sync. |

---

## CSI video

The `cam_down` pipeline drives the IMX477 on `sensor-id=0` at 1080x1080 GRAY8, 15 fps, centre-cropped from the 1920x1080 sensor mode. It publishes `image_raw`, `image_raw/compressed` and `camera_info` under `/cam_down` in frame `bottom_visual_link`. Pipeline fields and troubleshooting are in [`ros_gst_cameras/README.md`](local_ws/src/ros_gst_cameras/README.md).

---

## LiDAR

`rslidar_coordinator` runs the RoboSense SDK node as a managed subprocess and publishes `/rslidar_points` in frame `rslidar_link`. `/rslidar_coordinator/alive` goes false after 5 s without frames. There is no auto-restart; recover with `rslidar_restart`.

> The subprocess starts whether or not the LiDAR is reachable. Without a configured link and a powered LiDAR, no frames arrive.

Parameters and SDK keys are in [`rslidar_coordinator/README.md`](local_ws/src/rslidar_coordinator/README.md).

---

## Visual odometry

Isaac cuVSLAM runs on the front RealSense IR pair. The reactor filters and re-seats its solution, and `vio_transform` publishes it to PX4 on `/fmu/in/vehicle_visual_odometry`.

`initialize` enables the stack through [`arid_supervisor`](isaac_ros-dev/src/arid_supervisor/README.md) and allows 300 s for the camera-proven bringup. `deinitialize` is refused unless the drone is provably landed. Each launch writes `run_logs/vslam/vslam.log`.

> Never select a 90 fps RealSense profile. That USB service interval stalls the bus.

`config_realsense` writes the installed camera's serial into `vslam_config.yaml`; run it after a camera swap. The launch graph and the tunables are in [`px4_vslam`](isaac_ros-dev/src/px4_vslam/README.md) and [`px4_vslam_reactor`](isaac_ros-dev/src/px4_vslam_reactor/README.md). In the container, `vslam` launches the same stack without the gate or the interlock.

---

## USB reset

`reset_usb` power-cycles the ARK PAB USB hub and the standalone USB3 port. The `/reset_usb` service runs the same script.

> `/reset_usb` also resets the flight controller. Never call it in flight.

---

## Foxglove

`foxglove_bridge` starts the bridge on port 8765, on the host or in the container. Connect Foxglove Studio to `ws://<device-ip>:8765`; the URDF is on `/robot_description` with panel frame `base_link`.

---

## Health checks

`local_test` exercises the host services, the aliases, and a full `cam_down` and LiDAR lifecycle, and leaves both pipelines stopped. `ver_cv_cams` opens the live `cam_down` feed on the NoMachine desktop, where `q` quits.

---

## setup.sh

`./setup.sh` opens the menu. Every step detects its current state and configures only what is missing, and each run logs to `log/setup_log_*.log`.

| Option | Action |
|---|---|
| **1** | Full setup |
| **2** | Smoke test |
| **3** | RealSense serial assignment |
| **4** | Camera feed check |
| **5** | CSI overlay for the down camera |
| **6** | LiDAR network diagnostic |
| **7** | LiDAR address auto-detect |
| **8** | Wi-Fi connect |
| **9** | Camera calibration |
| **10** | Camera focus |
| **11** | Build the Isaac container image |
| **12** | Build the Isaac workspace; needs the container running |
| **13** | Build `local_ws` |
| **14** | Install ARK-OS |
| **15** | Install ROS 2 |
| **16** | ZeroTier join or switch |
| **17** | Uninstall, keeping the repo, the OS and the Docker engine |

The IMX477 reaches `sensor-id=0` only after the CSI overlay of option **5** and a reboot.

### LiDAR link

`config_lidar` sniffs `enP8p1s0` and rewrites the `rslidar` NetworkManager profile and its dispatcher to the addresses the LiDAR carries in firmware. Run it after a LiDAR swap or a reconfiguration. With no LiDAR detected, the static fallback stays at `192.168.1.102/24` to `192.168.1.200`.

On link-up the dispatcher ARP-probes the LiDAR for 8 s and falls back to the DHCP `dev` profile without a response.

`lidar_diag` walks link state, profile, address and ARP probe, then coordinator and cloud rate. Cached sudo credentials add a `tcpdump` sniff and an `arp-scan` sweep.

### Camera calibration

`cam_calibrate` calibrates `cam_down` against the live pipeline and waits for a NoMachine session. The default board is the included 10x7-square, 50 mm PDF. It writes `config/calibrations/cam_down.yaml`; set `calibration: "cam_down"` in `pipelines.yaml` and restart the pipeline to load it.

### Camera focus

Menu option **10** starts `cam_down` and the Foxglove bridge. Watch `/cam_down/image_raw/compressed` while adjusting the lens; `q` stops both.

---

## Full setup

`./setup.sh --full` walks the questionnaire once, then provisions unattended. `--resume` continues manually after a reboot, and `--continue` re-enters the tail with finished steps skipped.

```bash
./setup.sh --full
```

Every prompt defaults to skip. The run reboots at most twice: after the install steps when ARK-OS, ROS 2 or JetPack installed, and before the smoke test when anything built. Open a terminal after each reboot and answer the resume prompt.

> Setup run over the wired port defers the LiDAR network step. Finish it over Wi-Fi with `config_lidar`.

Calibration is not part of the run. Calibrate the down camera once setup finishes.

---

## PX4 firmware

The PX4 fork is at [`local_ws/auxiliary/PX4-Autopilot/`](local_ws/auxiliary/PX4-Autopilot/) on branch `PX4-InDro` and carries the `4026_arid_quad_v1_2` airframe. A built image is kept at [`px4_compiled/arid.px4`](local_ws/auxiliary/px4_compiled/).

```bash
cd local_ws/auxiliary/PX4-Autopilot
make ark_fmu-v6x_default
```

The build lands at `build/ark_fmu-v6x_default/ark_fmu-v6x_default.px4`. Flash it from the ARK-OS web interface, then select the airframe from a MAVLink shell.

```
param set SYS_AUTOSTART 4026
param save
reboot
```

---

## Development

Code, builds and runtime stay on the drone over VSCode Remote-SSH. `.vscode/` carries the extension list and the lint, IntelliSense and search settings.

```bash
ssh jetson@<device-ip>
```

The host workspace bind-mounts into the container at `/workspaces/isaac_ros-dev`. Build it with `colcon_isaac` and enter it with `start_isaac` then `isaac_bash`; `local_ws` builds on the host with `colcon_local`. The camera feed and the calibration GUI render on a NoMachine desktop at `<device-ip>`.

Remote-SSH fails offline with `Failed to download VS Code Server` when the laptop's VSCode commit has no server staged on the drone. Set `update.mode` to `none`, `extensions.autoUpdate` to false and `remote.SSH.localServerDownload` to `off` in the laptop's user settings.

### Submodules

`update_submods.sh` syncs every submodule to its pin and verifies it: LIVE entries must sit on their branch, PINNED entries on their exact tag. `--update` advances both.

| Submodule | Class | Role |
|---|---|---|
| [`PX4-Autopilot`](https://github.com/indro-robotics/PX4-Autopilot/tree/PX4-InDro) | LIVE `PX4-InDro` | PX4 fork with the ARID airframe. |
| [`realsense-ros`](https://github.com/indro-robotics/realsense-ros/tree/v4.51.1) | LIVE `v4.51.1` | RealSense driver. |
| [`isaac_ros_visual_slam`](https://github.com/indro-robotics/isaac_ros_visual_slam/tree/v3.2-14) | LIVE `v3.2-14` | cuVSLAM backend. |
| [`px4_msgs`](https://github.com/PX4/px4_msgs/tree/release/1.15) | LIVE `release/1.15` | PX4 messages; `local_ws/src/px4_msgs` symlinks to it. |
| [`isaac_ros_common`](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/tree/v3.2-14) | PINNED `v3.2-14` | Isaac base image and Dockerfile chain. |
| [`isaac_ros_nitros`](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros/tree/v3.2-14) | PINNED `v3.2-14` | Zero-copy transport. |
| [`isaac_ros_image_pipeline`](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline/tree/v3.2-14) | PINNED `v3.2-14` | GPU image processing. |
| [`px4-ros2-interface-lib`](https://github.com/Auterion/px4-ros2-interface-lib/tree/1.4.0) | PINNED `1.4.0` | Auterion PX4 SDK. |
| [`rslidar_sdk`](https://github.com/RoboSense-LiDAR/rslidar_sdk/tree/v1.5.19) | PINNED `v1.5.19` | RoboSense SDK; builds `rslidar_sdk_node`. |
| [`rslidar_msg`](https://github.com/RoboSense-LiDAR/rslidar_msg/tree/v1.5.10) | PINNED `v1.5.10` | RoboSense message definitions. |

---

## Packages

| Package | Contents |
|---|---|
| [`arid_description`](local_ws/src/arid_description/) | Xacro, meshes, RViz config. |
| [`ros_gst_cameras`](local_ws/src/ros_gst_cameras/) | `gst_cam_node` and `gst_camera_manager`. |
| [`rslidar_coordinator`](local_ws/src/rslidar_coordinator/) | RSAIRY supervisor: SDK config, services, watchdog. |
| [`reset_ark_usb`](local_ws/src/reset_ark_usb/) | The `/reset_usb` service. |
| [`camera_calibration`](local_ws/auxiliary/camera_calibration/) | `cam_down` calibrator and pattern. |
| [`px4_vslam`](isaac_ros-dev/src/px4_vslam/) | VSLAM launch graph and the PX4 bridge. |
| [`px4_vslam_reactor`](isaac_ros-dev/src/px4_vslam_reactor/) | VSLAM jump gating and re-seat. |
| [`arid_supervisor`](isaac_ros-dev/src/arid_supervisor/) | VSLAM lifecycle service. |
