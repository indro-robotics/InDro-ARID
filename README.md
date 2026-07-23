# InDro ARID Workspace

**Autonomous Research Indoor Drone.** NVIDIA Jetson Orin · ARK PAB carrier · ROS 2 Humble · NVIDIA Isaac ROS · PX4

End-to-end repo for provisioning, building, and operating a deployed ARID:

- **Bootstrap** (`setup.sh`): provisioning from fresh Ubuntu 22.04 to flight-ready; safe to re-run at any point.
- **Robot description** (`arid_description`): xacro + meshes; `/robot_description` + `/tf_static` on boot.
- **CSI camera** (`ros_gst_cameras`): downward IMX219 pipeline (`cam_down`) with SetBool start/stop and a frame-flow watchdog.
- **LiDAR** (`rslidar_coordinator`): RoboSense RSAIRY supervisor; SetBool start/stop, latched `/alive`, auto-configured Ethernet link.
- **Visual odometry** (`px4_vslam`, `px4_vslam_reactor`, `arid_supervisor`): front RealSense → Isaac cuVSLAM → PX4 VIO, supervised. `initialize` / `deinitialize` from any terminal.
- **PX4 firmware**: InDro fork, `4026_arid_quad_v1_2` airframe, prebuilt binary shipped in-repo.
- **USB recovery** (`reset_ark_usb`): `/reset_usb` Trigger service.
- **Diagnostics**: `local_test` smoke test, `lidar_diag` LiDAR network walk-through, `config_lidar` LiDAR IP auto-detect, `config_realsense` serial assignment, `ver_cv_cams` live feed.
- **Helpers**: `wifi`, `update_submods`, `zt_join`, Foxglove bridge (apt, port 8765).

## Sensor configuration

| Sensor | Fit |
|---|---|
| 1x RealSense D43X | front stereo (VSLAM) |
| 1x RSAIRY LiDAR | RoboSense, Ethernet |
| 1x IMX219 CSI | `cam_down` (downward) |
| Optical flow + rangefinder | bottom pod |

> ROS 2 traffic stays on the drone (`ROS_DOMAIN_ID=23`, `ROS_LOCALHOST_ONLY=1`); remote visualization goes through the Foxglove bridge instead.

---

## At login

At login you should find the container running, the TF tree up, the camera and LiDAR idle, `/arid_supervisor/vslam_enable` on the graph, and `local_ws` already sourced.

---

## Useful aliases

**Host** (`setup.sh` bashrc step; `help` prints all):

| Alias | Action |
|---|---|
| `setup` | Run setup.sh. |
| `run_isaac` / `start_isaac` / `stop_isaac` / `isaac_bash` | Container: run / start / stop / shell. |
| `build_isaac` | Rebuild the container image. |
| `colcon_isaac` | Deinitialize → build container workspace → restart supervisor. |
| `clean_isaac` / `rosdep_isaac` | Clean / rosdep the container workspace. |
| `colcon_local` | Build `local_ws` (stops + restarts its host services). |
| `clean_local` / `rosdep_local` | Clean / rosdep `local_ws`. |
| `reset_usb` | USB hub reset. |
| `cam_down_*` | start / stop / status / alive for the pipeline. |
| `cam_refresh` | Re-read `pipelines.yaml` at runtime (stops pipelines first). |
| `cam_calibrate` | Calibrate `cam_down`. |
| `rslidar_*` | start / stop / status / alive / restart for the LiDAR. |
| `lidar_diag` / `config_lidar` | LiDAR diagnostic / IP auto-detect (`config_lidar` prepends sudo). |
| `config_realsense` | Assign the RealSense serial. |
| `local_test` / `ver_cv_cams` | Smoke test / live feed. |
| `wifi` / `zt_join` / `update_submods` | Wi-Fi / ZeroTier / submodule sync. |
| `foxglove_bridge` | Bridge on 8765. |
| `initialize` / `deinitialize` / `status` | VSLAM via the supervisor. |

**Container** (`arid_env.sh`): `vslam` (direct launch), `initialize` / `deinitialize` / `status`, `reset_usb`, `colcon_isaac` / `clean_isaac` / `rosdep_isaac`, `foxglove_bridge`, `help`.

---

## Camera (CSI)

The downward IMX219 runs one pipeline defined in [`pipelines.yaml`](local_ws/src/ros_gst_cameras/gst_camera_manager/config/pipelines.yaml): 1920×1080 at 15 fps, GRAY8 (IR-sensitive mono).

| Pipeline | Sensor | Frame | Topic root |
|---|---|---|---|
| `cam_down` | `sensor-id=0` | `bottom_visual_link` | `/cam_down` |

Start and stop it with the per-pipeline aliases, or the underlying service call:

```bash
cam_down_start / cam_down_stop / cam_down_status / cam_down_alive
ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool '{data: true}'
```

It publishes `image_raw`, `image_raw/compressed`, and `camera_info`; liveness is on `/gst_camera_manager/cam_down/alive` (latched). The manager's log is tee'd to `isaac_ros-dev/run_logs/gst_camera_manager/`. Details: [`ros_gst_cameras/README.md`](local_ws/src/ros_gst_cameras/README.md).

---

## LiDAR (RoboSense RSAIRY)

[`rslidar_coordinator`](local_ws/src/rslidar_coordinator/) supervises the RSAIRY: it spawns `rslidar_sdk_node` as a managed subprocess. The coordinator comes up at boot with the LiDAR idle until enabled, the same pattern as the camera manager. It publishes the point cloud only; there is no IMU topic.

```bash
rslidar_start
rslidar_stop
rslidar_status
rslidar_alive
rslidar_restart
```

`rslidar_start` is equivalent to:

```bash
ros2 service call /rslidar_coordinator/enable std_srvs/srv/SetBool '{data: true}'
```

| Topic / Service | Type | Notes |
|---|---|---|
| `/rslidar_points` | `sensor_msgs/PointCloud2` | Frame `rslidar_link`, BEST_EFFORT. |
| `/rslidar_coordinator/alive` | `std_msgs/Bool` (latched) | Frame-flow watchdog. |
| `/rslidar_coordinator/enable` | `std_srvs/SetBool` | Start / stop the subprocess. |
| `/rslidar_coordinator/status` | `std_srvs/Trigger` | `RUNNING (pid=N)` or `STOPPED`. |
| `/rslidar_coordinator/restart` | `std_srvs/Trigger` | Kill + respawn. |

`rslidar_link` is fixed to `base_link` in [`arid.xacro`](local_ws/src/arid_description/xacro/arid.xacro). The watchdog flips `/alive` false after `alive_threshold` (5 s) without frames; there is no auto-restart, so recover with `rslidar_start` / `rslidar_restart`. Details: [`rslidar_coordinator/README.md`](local_ws/src/rslidar_coordinator/README.md).

---

## Visual odometry (VSLAM packages)

Front RealSense (D43X IR stereo) → Isaac cuVSLAM → PX4 VIO. Config: [`vslam_config.yaml`](isaac_ros-dev/src/px4_vslam/config/vslam_config.yaml).

| Alias | Action |
|---|---|
| `initialize` | SetBool true on `/arid_supervisor/vslam_enable`. |
| `deinitialize` | SetBool false (refused unless landed). |
| `status` | vslam running + land state. |

The supervisor runs a **camera-proven bring-up** (details: [`arid_supervisor/README.md`](isaac_ros-dev/src/arid_supervisor/README.md)). Each launch logs to `/workspaces/isaac_ros-dev/run_logs/vslam/vslam.log`.

A direct launch bypasses the supervisor (in-container): `vslam` or `ros2 launch px4_vslam vslam.launch.py`. It blocks until `/robot_description` is up, then starts the RealSense driver, the VSLAM node, `vio_transform` (FLU→FRD bridge), and `vslam_reactor` (jump gating + `SetSlamPose` retry; tunables in [`px4_vslam_reactor.yaml`](isaac_ros-dev/src/px4_vslam_reactor/config/px4_vslam_reactor.yaml)).

**Before flying:** run `config_realsense` so the camera serial in `vslam_config.yaml` matches the installed RealSense.

Reference: [`px4_vslam/README.md`](isaac_ros-dev/src/px4_vslam/README.md), [`px4_vslam_reactor/README.md`](isaac_ros-dev/src/px4_vslam_reactor/README.md).

---

## USB reset

Use the `reset_usb` alias, or the underlying Trigger call:

```bash
reset_usb
ros2 service call /reset_usb std_srvs/srv/Trigger '{}'
```

`uhubctl` power-cycles the ARK PAB USB hub and `gpioset` pulses the FMU reset line (GPIO 85): never call it in flight. In-container, the `reset_usb` alias falls back to `systemctl start reset_usb.service` when the ROS service is absent. Details: [`reset_ark_usb/README.md`](local_ws/src/reset_ark_usb/README.md).

---

## Foxglove

Run the `foxglove_bridge` alias (host or container) to start the bridge on port 8765, then connect Foxglove Studio to `ws://<device-ip>:8765`.

---

## Smoke test

`local_test` (menu **2**) checks that the services are active, the aliases resolve, the Foxglove port is open, the full `cam_down` + `rslidar` lifecycles work (frame_id, rate, `/alive`, restart PID; the cloud-rate check SKIPs if the LiDAR is off), the supervisor is on the graph (if the container is up), and no subprocesses leak. It is safe to run any time and leaves the pipelines stopped. Each run logs to `log/smoke_test_log_*.log`.

---

## Camera feed check

`ver_cv_cams` opens the live `cam_down` stream in a cv2 window over NoMachine; `q` quits.

---

## setup.sh menu

Running `./setup.sh` with no arguments opens the interactive menu:

Every step detects what is already in place and only does what is missing, so re-running is always safe. Each run logs to `log/setup_log_*.log`.

```
./setup.sh
```

| Option | Action |
|---|---|
| **1** | Full setup |
| **2** | Smoke test |
| **3** | RealSense serial assignment |
| **4** | Camera feed check (`cam_down`) |
| **5** | LiDAR network diagnostic |
| **6** | LiDAR IP auto-detect |
| **7** | Wi-Fi connect |
| **8** | Camera calibration (`cam_down`) |
| **9** | Camera focus (`cam_down`, via Foxglove) |
| **10** | Build the Isaac container |
| **11** | Colcon-build the container workspace + start `arid_supervisor.service` (requires the container running) |
| **12** | ZeroTier join/switch |
| **13** | Uninstall (repo, OS, Docker engine kept) |

### LiDAR network diagnostic

`lidar_diag` walks the LiDAR path end to end: link state, NM profile, IP and route, an ARP probe at the detected IP, a passive `tcpdump` sniff plus `arp-scan` (these two need `sudo lidar_diag`), then coordinator/SDK runtime and cloud rate. It reads `.lidar/rslidar_detected.conf`.

The network it is checking:

| Setting | Value |
|---|---|
| Jetson NIC | `enP8p1s0` |
| LiDAR IP / Jetson IP | auto-detected by `config_lidar` |
| Factory-default LiDAR IP | `192.168.1.200` (fallback) |
| MSOP (point cloud) port | UDP `6699` |
| DIFOP (device info) port | UDP `7788` |
| IMU port (socket bound) | UDP `6688` |

On link-up, an NM dispatcher ARP-probes the LiDAR for up to 8 s: a response keeps the static `rslidar` profile, no response falls back to the DHCP `dev` profile. It auto-swaps on cable change.

RoboSense LiDARs store their own IP and their unicast target in firmware, and a prior RSView session may have moved them off factory defaults. `config_lidar` (also run by setup) handles this: it sniffs `enP8p1s0` with `tcpdump`, extracts the LiDAR's MAC + IPs, rewrites the `rslidar` NM profile and dispatcher to match, verifies ARP, and persists the result to `.lidar/rslidar_detected.conf` (read by `lidar_diag`). Run it after a LiDAR swap or reconfiguration:

```bash
config_lidar
```

If the LiDAR is unreachable, the fallback static (`192.168.1.102/24` → `192.168.1.200`) stays in place; re-run once it is connected and powered.

### Camera calibration

```bash
cam_calibrate
```

[`camera_calibration_auto/camera_calibrate.sh`](local_ws/auxiliary/camera_calibration/camera_calibration_auto/camera_calibrate.sh) (menu **8**) runs a venv-isolated interactive calibrator against the live `cam_down` pipeline. The default board is the included 10×7-square / 50 mm PDF. It writes the live calibration to `config/calibrations/cam_down.yaml` plus a timestamped store; afterwards set `calibration: "cam_down"` in `pipelines.yaml`. The script waits for a NoMachine session before starting (`NM_WAIT_S`, default 180 s). Details: [`camera_calibration/README.md`](local_ws/auxiliary/camera_calibration/README.md).

### Camera focus

Menu **9** starts `cam_down` and the Foxglove bridge; watch `/cam_down/image_raw/compressed` in Studio while adjusting the lens. `q` tears everything down.

---

## Full setup

`./setup.sh --full` walks the questionnaire once, then provisions everything unattended. Setup may reboot the drone one or more times along the way. To resume after a reboot, open a bash terminal: you will be prompted to continue, and the run picks up where it left off (camera verification, any queued container build, the smoke test). `--resume` is the same continuation invoked manually.

```
./setup.sh --full
./setup.sh --resume
```

### Steps (in order)

| Step | Does |
|---|---|
| **preflight** | Not root; git present; submodules initialized. |
| **collect_answers** | Questionnaire → `PRE_*` answers, persisted for the resume. |
| **first_boot** | One-time hostname/password. |
| **power** | Sets nvpmodel to maximum; apt-holds critical L4T packages. |
| **disable_updates** | Disables unattended-upgrades and the apt timers. |
| **enable_clock_sync** | Enables `systemd-timesyncd`. |
| **enable_user_linger** | Enables user lingering so `/run/user/<uid>` is created at boot (headless NoMachine fix). |
| **clean_nvidia_desktop** | Removes NVIDIA first-boot icons + L4T-README automount. |
| **ensure_wifi** | Joins the questionnaire's network. |
| **nomachine** | Detects install; prints manual hint if missing. |
| **repos** | ROS / NVIDIA / Docker apt repos, CDI config. |
| **apt** | Apt packages (Foxglove bridge, net tools). |
| **zerotier** | Installs daemon, joins via `zt_join.sh --setup`. `ACCESS_DENIED` = authorize later. |
| **px4_deps** | PX4 toolchain (skipped if `arm-none-eabi-gcc` present). |
| **git** | Credential cache, script perms, `update_submods.sh` pin-verify. |
| **docker_patches** | Injects `Dockerfile.arid` / `arid_env.sh` / `run_dev.sh` into `isaac_ros_common`. |
| **skip_worktree** | Hides per-deployment configs (serials, calibrations, tuning) from git status. |
| **bashrc** | Rewrites the ARID block: env exports + the full alias set + `help` + resume hook. |
| **permissions** | Sudoers (uhubctl, gpioset, systemctl, `usb_reset.sh`, `zerotier-cli`), udev, polkit, groups. |
| **uhubctl** | Builds from source if missing. |
| **lidar_sysctl** | Kernel UDP receive buffers → 25 MiB (`/etc/sysctl.d/99-rslidar.conf`). |
| **lidar_network** | NM profiles on `enP8p1s0` (`rslidar` static / `dev` DHCP fallback) + link-up dispatcher; then `config_lidar`. |
| **ros_workspace** | Python deps, rosdep, colcon-builds `local_ws`. |
| **docker** | Engine, NVIDIA runtime, docker group, buildx. |
| **systemd** | Installs + enables all units, installs the ROS-env `DefaultEnvironment` drop-in. |
| **realsense** | `config_realsense.sh` → RealSense serial into `vslam_config.yaml`. |
| **verify_cameras** | Live `cam_down` feed (if opted in). |
| **build_isaac** | Container image build; queued across the reboot. |
| **colcon_isaac** | Builds container workspace, restarts `arid_supervisor.service`. |
| **print_summary / prompt_reboot** | Summary; arms `~/.arid_resume_setup` and reboots. |

After setup and the reboot, commission the drone in this order: `lidar_diag`, `local_test`, `ver_cv_cams`, then calibrate and focus the down camera as needed.

---

## PX4 firmware

The PX4 fork (`PX4-InDro`) lives at [`local_ws/auxiliary/PX4-Autopilot/`](local_ws/auxiliary/PX4-Autopilot/) and carries the **`4026_arid_quad_v1_2`** airframe. A prebuilt binary ships at [`PX4_prebuilt/ARID_v1.2.px4`](local_ws/auxiliary/PX4_prebuilt/) if you would rather flash than build.

Building produces `build/ark_fmu-v6x_default/ark_fmu-v6x_default.px4`:

```bash
cd local_ws/auxiliary/PX4-Autopilot
make ark_fmu-v6x_default
```

After flashing, select the airframe from a MAVLink shell:

```
param set SYS_AUTOSTART 4026
param save
reboot
```

---

## Boot sequence

| Service | Does |
|---|---|
| **usbfs-memory** | usbfs buffer → 1000 MB (RealSense multi-stream). |
| **jetson-clocks** | Max clocks. |
| **start_isaac_docker** | Starts the Isaac container. |
| **arid_description** | `robot_state_publisher` (TF tree). |
| **gst_camera_manager** | Camera manager up, `cam_down` idle. |
| **rslidar_coordinator** | LiDAR supervisor up, SDK subprocess idle. |
| **usb_ros_reset** | Hosts `/reset_usb`. |
| **arid_supervisor** | Supervisor in-container via `docker exec`; VSLAM idle until `initialize`. |

---

> **First boot:** the supervisor needs the container workspace colcon-built. `--full` handles it; otherwise menu **11**, or:
>
> ```bash
> start_isaac && isaac_bash && colcon_isaac && exit
> sudo systemctl reset-failed arid_supervisor.service && sudo systemctl start arid_supervisor.service
> ```

## Development workflow

VSCode over Remote-SSH is the recommended editor: all code, builds, and the live runtime stay on the drone, and the repo ships VSCode configuration for a streamlined development experience.

```bash
ssh jetson@<device-ip>
```

On first open VSCode prompts to install the workspace's recommended extensions on the drone. `.vscode/` carries the extension list, Python/C++ lint + IntelliSense settings, and search excludes.

For direct virtual desktop access, connect a NoMachine session to `<device-ip>`. The GUI tools (camera feed window, the calibration GUI, camera focus) render on that desktop, so keep a session attached when using them.

### VSCode Remote-SSH offline fix

Remote-SSH fails offline (`Failed to download VS Code Server`) when the laptop's VSCode commit has no matching server staged on the drone. Freeze the laptop's commit once, then restart VSCode (client-scope settings, cannot live in `.vscode/`):

Windows (PowerShell):
```powershell
$f = "$env:APPDATA\Code\User\settings.json"
if (-not (Test-Path $f)) { New-Item -Force -Path $f -Value '{}' | Out-Null }
Copy-Item $f "$f.bak" -Force
$j = Get-Content $f -Raw | ConvertFrom-Json
$j | Add-Member -Force -NotePropertyName 'update.mode' -NotePropertyValue 'none'
$j | Add-Member -Force -NotePropertyName 'extensions.autoUpdate' -NotePropertyValue $false
$j | Add-Member -Force -NotePropertyName 'remote.SSH.localServerDownload' -NotePropertyValue 'off'
$j | ConvertTo-Json -Depth 50 | Set-Content $f -Encoding utf8
```

Linux (bash):
```bash
f=~/.config/Code/User/settings.json
mkdir -p "$(dirname "$f")"; [ -f "$f" ] || echo '{}' >"$f"
cp "$f" "$f.bak"
python3 - "$f" <<'EOF'
import json,sys
p=sys.argv[1]; d=json.load(open(p))
d.update({"update.mode":"none","extensions.autoUpdate":False,"remote.SSH.localServerDownload":"off"})
json.dump(d,open(p,"w"),indent=2)
EOF
```

Both back up to `settings.json.bak`. If the parse fails (`//` comments), restore the backup and add the three keys by hand.

### Isaac container

The host workspace bind-mounts into the container at `/workspaces/isaac_ros-dev`. Build with `colcon_isaac` (host or container); enter with `start_isaac` + `isaac_bash`. `local_ws/` builds on the host (`colcon_local`). For in-container IntelliSense, attach VSCode to `isaac_ros_dev-aarch64-container` with Dev Containers.

---

## Robot description

`arid_description.service` runs `robot_state_publisher` on [`xacro/arid.xacro`](local_ws/src/arid_description/xacro/arid.xacro). Frames: `base_link`, `base_footprint`, `autopilot`, 4 propellers, `front_realsense_link`, `rslidar_link`, `bottom_visual_link` (down cam), `flow_link`, `rangefinder_link`. Visualization: [`arid_description/README.md`](local_ws/src/arid_description/README.md).

---

## Package reference

| Package | Purpose |
|---|---|
| [`arid_description`](local_ws/src/arid_description/) | Xacro, meshes, RViz config. |
| [`ros_gst_cameras`](local_ws/src/ros_gst_cameras/) | CSI camera stack (`gst_cam_node` + `gst_camera_manager`). |
| [`rslidar_coordinator`](local_ws/src/rslidar_coordinator/) | RSAIRY supervisor: SDK config, services, watchdog. |
| [`reset_ark_usb`](local_ws/src/reset_ark_usb/) | `/reset_usb` service. |
| [`camera_calibration`](local_ws/auxiliary/camera_calibration/) | `cam_down` calibrator + pattern. |
| [`px4_vslam`](isaac_ros-dev/src/px4_vslam/) | VSLAM launch + PX4 bridge. |
| [`px4_vslam_reactor`](isaac_ros-dev/src/px4_vslam_reactor/) | VSLAM jump gating + re-seat. |
| [`arid_supervisor`](isaac_ros-dev/src/arid_supervisor/) | VSLAM lifecycle service (camera-proven bring-up, landed gate). |

| Submodule | Role |
|---|---|
| [`PX4-Autopilot`](local_ws/auxiliary/PX4-Autopilot/) | PX4 fork (`PX4-InDro`), ARID airframe. |
| [`px4_msgs`](isaac_ros-dev/src/px4_msgs/) | PX4 messages (`release/1.15`). Single submodule; `local_ws/src/px4_msgs` is a symlink to it. |
| [`rslidar_sdk`](local_ws/src/rslidar_sdk/) | RoboSense SDK; builds `rslidar_sdk_node` (nested `rs_driver` auto-initialized). |
| [`rslidar_msg`](local_ws/src/rslidar_msg/) | RoboSense message definitions. |
| [`isaac_ros_common`](isaac_ros-dev/src/isaac_ros_common/) | Isaac base (Dockerfile chain; ARID-patched). |
| [`isaac_ros_nitros`](isaac_ros-dev/src/isaac_ros_nitros/) | Zero-copy transport. |
| [`isaac_ros_image_pipeline`](isaac_ros-dev/src/isaac_ros_image_pipeline/) | GPU image processing. |
| [`isaac_ros_visual_slam`](isaac_ros-dev/src/isaac_ros_visual_slam/) | cuVSLAM backend. |
| [`realsense-ros`](isaac_ros-dev/src/realsense-ros/) | RealSense driver. |
| [`px4-ros2-interface-lib`](isaac_ros-dev/src/px4-ros2-interface-lib/) | Auterion PX4 SDK. |
