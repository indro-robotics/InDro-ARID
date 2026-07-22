# InDro ARID Workspace

**Autonomous Research Indoor Drone.** NVIDIA Jetson Orin · ARK PAB carrier · ROS 2 Humble · NVIDIA Isaac ROS · PX4

End-to-end repo for provisioning, building, and operating a deployed ARID:

- **Bootstrap** (`setup.sh`): idempotent provisioning from fresh Ubuntu 22.04 to flight-ready.
- **Robot description** (`arid_description`): xacro + meshes; `/robot_description` + `/tf_static` on boot.
- **CSI cameras** (`ros_gst_cameras`): front + down IMX219 pipelines with SetBool start/stop and a frame-flow watchdog.
- **Visual SLAM** (`px4_vslam`, `px4_vslam_reactor`, `arid_supervisor`): 3× RealSense → Isaac cuVSLAM → PX4 VIO, supervised. `initialize` / `deinitialize` from any terminal.
- **PX4 firmware**: InDro fork, `4026_arid_quad_v1_2` airframe; prebuilt `ARID_v1.2.px4` in `local_ws/auxiliary/PX4_prebuilt/`.
- **USB recovery** (`reset_ark_usb`): `/reset_usb` Trigger. `uhubctl` power-cycles the ARK PAB USB hub and `gpioset` pulses the FMU reset line (GPIO 85).
- **Diagnostics**: `local_test` smoke test, `config_realsense` serial assignment, `ver_cv_cams` live feeds.
- **Helpers**: `wifi`, `update_submods`, `zt_join`, Foxglove bridge (apt, port 8765).

> **DDS containment: redundant.** `ROS_DOMAIN_ID=23` AND `ROS_LOCALHOST_ONLY=1`, set everywhere (bashrc, container env, every ROS unit, systemd `DefaultEnvironment` drop-in). Off-box viewing = Foxglove bridge (TCP 8765), never raw DDS. FMU pairing: `UXRCE_DDS_PTCFG=1`, firmware default in `4026_arid_quad_v1_2`; without it all `/fmu` topics silently vanish.
>
> **librealsense = RSUSB 2.55.1 at `/usr/local` only.** apt `ros-humble-librealsense2` is purged + pinned out (host and container); `rosdep --skip-keys librealsense2`; builds pinned with `-Drealsense2_DIR`; colcon step self-heals a stray apt copy. Host serial detection uses the pip `pyrealsense2` wheel. (The apt V4L2 build drops HW metadata + multi-cam.)

---

## setup.sh

Bare invocation opens the interactive menu; `--full` runs the questionnaire then unattended full setup; `--resume` continues after the reboot (auto-triggered by the bashrc hook):

```
./setup.sh
./setup.sh --full
./setup.sh --resume
```

Every step is idempotent. Logs to `log/setup_log_*.log` (session-pinned across the resume reboot; smoke test: `smoke_test_log_*.log`; newest 10 kept).

| Option | Action |
|---|---|
| **1** | Full setup |
| **2** | Smoke test |
| **3** | RealSense serial assignment (front/left/right) |
| **4** | Camera feed check (front + down) |
| **5** | Wi-Fi connect |
| **6** | Camera calibration (front/down) |
| **7** | Camera focus (front/down, via Foxglove) |
| **8** | Build the Isaac container |
| **9** | Colcon-build the container workspace + start `arid_supervisor.service` (container must be running) |
| **10** | ZeroTier join/switch |
| **11** | Uninstall (everything setup.sh installed; strict confirmation; repo/OS/Docker engine kept) |

`--full` asks everything up front (hostname, password, Wi-Fi, NoMachine, ZeroTier, PX4 toolchain, RealSense, camera checks, Isaac build, smoke test, reboot).

### Steps (in order)

| Step | Does |
|---|---|
| **preflight** | Not root; git present; submodules initialized. |
| **collect_answers** | Questionnaire → `PRE_*` answers, persisted for the resume. |
| **first_boot** | One-time hostname/password. |
| **power** | nvpmodel max (non-interactive), apt-holds critical L4T packages. |
| **disable_updates** | Kills unattended-upgrades + apt timers. |
| **enable_clock_sync** | `systemd-timesyncd` on. |
| **enable_user_linger** | systemd owns `/run/user/<uid>` at boot (headless NoMachine black-screen fix). |
| **clean_nvidia_desktop** | Removes NVIDIA first-boot icons + L4T-README automount. |
| **ensure_wifi** | Joins the questionnaire's network. |
| **nomachine** | Detects install; prints manual hint if missing. |
| **repos** | ROS / NVIDIA / Docker apt repos, CDI config. |
| **apt** | Packages incl. foxglove bridge + net tools. NO apt librealsense (see blockquote). |
| **zerotier** | Installs daemon, joins via `zt_join.sh --setup`. `ACCESS_DENIED` = authorize later. |
| **px4_deps** | PX4 toolchain (skipped if `arm-none-eabi-gcc` present). |
| **git** | Credential cache, script perms, `update_submods.sh` pin-verify. |
| **docker_patches** | Injects `Dockerfile.arid` / `arid_env.sh` / `run_dev.sh` into `isaac_ros_common` (skip-worktree'd). |
| **skip_worktree** | Protects per-deployment configs (serials, calibrations, tuning) from git status. |
| **bashrc** | Rewrites the ARID block: env exports + the full alias set + `help` + resume hook. |
| **permissions** | Sudoers (uhubctl, gpioset, systemctl, `usb_reset.sh`, `zerotier-cli`), udev, polkit, groups. |
| **uhubctl** | Builds from source if missing. |
| **ros_workspace** | Python deps, rosdep, colcon-builds `local_ws`. |
| **docker** | Engine, NVIDIA runtime, docker group, buildx (each state-checked). |
| **systemd** | Installs + enables all units, installs the ROS-env `DefaultEnvironment` drop-in. |
| **realsense** | `config_realsense.sh` → three serials into `vslam_config.yaml`. |
| **verify_cameras** | Live front + down feeds (if opted in). |
| **build_isaac** | Queued container build (survives the reboot). |
| **colcon_isaac** | Builds container workspace, restarts `arid_supervisor.service`. |
| **print_summary / prompt_reboot** | Summary; arms `~/.arid_resume_setup` and reboots. |

---

## Boot sequence

| Service | Does |
|---|---|
| **usbfs-memory** | usbfs buffer → 1000 MB (RealSense multi-stream). |
| **jetson-clocks** | Max clocks. |
| **start_isaac_docker** | Starts the Isaac container. |
| **arid_description** | `robot_state_publisher` (TF tree). |
| **gst_camera_manager** | Camera manager up, both pipelines idle. Log tee'd to `isaac_ros-dev/run_logs/gst_camera_manager/`. |
| **usb_ros_reset** | Hosts `/reset_usb`. |
| **arid_supervisor** | Supervisor in-container via `docker exec`; VSLAM idle until `initialize`. |

At login: container running, TF up, cameras idle, `/arid_supervisor/vslam_enable` on the graph, `local_ws` sourced.

> **First boot:** the supervisor needs the container workspace colcon-built. `--full` handles it; otherwise menu **9**, or:
>
> ```bash
> start_isaac && isaac_bash && colcon_isaac && exit
> sudo systemctl reset-failed arid_supervisor.service && sudo systemctl start arid_supervisor.service
> ```

---

## Development workflow

SSH into the drone, edit everything there.

```bash
ssh jetson@<device-ip>
```

Open the repo in your editor. On first open VSCode prompts to install the workspace's recommended extensions on the drone.

### What the workspace config does

- **`.vscode/extensions.json`** drives the install-recommendations prompt.
- **`.vscode/settings.json`**: Pylance extra paths (ROS + container interface installs), autopep8 (`--max-line-length=100`), uncrustify with the ROS ament style, `linux-gcc-arm64` C++17 IntelliSense, search excludes for `build/` `install/` `log/`, `*.xacro`→xml file associations.
- **`.vscode/c_cpp_properties.json`**: `Linux` C/C++ IntelliSense config.

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

## Cameras (CSI)

Two IMX219 pipelines ([`pipelines.yaml`](local_ws/src/ros_gst_cameras/gst_camera_manager/config/pipelines.yaml)), 1920×1080@15fps, GRAY8 (IR-sensitive mono).

| Pipeline | Sensor | Frame | Topic root |
|---|---|---|---|
| `cam_front` | `sensor-id=0` | `top_visual_link` | `/cam_front` |
| `cam_down` | `sensor-id=1` | `bottom_visual_link` | `/cam_down` |

Per-pipeline aliases, or the underlying service call:

```bash
cam_front_start / cam_front_stop / cam_front_status / cam_front_alive
cam_down_start  / cam_down_stop  / cam_down_status  / cam_down_alive
ros2 service call /gst_camera_manager/cam_front std_srvs/srv/SetBool '{data: true}'
```

Each publishes `image_raw`, `image_raw/compressed`, `camera_info`; liveness on `/gst_camera_manager/<name>/alive` (latched). Details: [`ros_gst_cameras/README.md`](local_ws/src/ros_gst_cameras/README.md).

---

## Visual-Inertial SLAM

Three RealSense cameras (front/left/right) → Isaac cuVSLAM (6-camera stereo-multicam) → PX4 VIO. Config: [`vslam_config.yaml`](isaac_ros-dev/src/px4_vslam/config/vslam_config.yaml).

| Alias | Action |
|---|---|
| `initialize` | SetBool true on `/arid_supervisor/vslam_enable`. |
| `deinitialize` | SetBool false (refused unless landed). |
| `status` | vslam running + land state. |

The supervisor runs a **camera-proven bring-up** (details: [`arid_supervisor/README.md`](isaac_ros-dev/src/arid_supervisor/README.md)). Per-launch log: `/workspaces/isaac_ros-dev/run_logs/vslam/vslam.log`.

Direct launch (bypasses the supervisor, in-container): `vslam` or `ros2 launch px4_vslam vslam.launch.py`. Blocks until `/robot_description` is up, then starts the 3 drivers, the VSLAM node, `vio_transform` (FLU→FRD bridge), and `vslam_reactor` (jump gating + `SetSlamPose` retry; tunables in [`px4_vslam_reactor.yaml`](isaac_ros-dev/src/px4_vslam_reactor/config/px4_vslam_reactor.yaml)).

**Before flying:** run `config_realsense` so all three serials are assigned.

Reference: [`px4_vslam/README.md`](isaac_ros-dev/src/px4_vslam/README.md), [`px4_vslam_reactor/README.md`](isaac_ros-dev/src/px4_vslam_reactor/README.md), [`arid_supervisor/README.md`](isaac_ros-dev/src/arid_supervisor/README.md).

---

## PX4 firmware

Fork at [`local_ws/auxiliary/PX4-Autopilot/`](local_ws/auxiliary/PX4-Autopilot/) (`PX4-InDro`). Airframe **`4026_arid_quad_v1_2`** = v1.1 + `UXRCE_DDS_PTCFG=1` default. Prebuilt: [`PX4_prebuilt/ARID_v1.2.px4`](local_ws/auxiliary/PX4_prebuilt/).

Build to `build/ark_fmu-v6x_default/ark_fmu-v6x_default.px4`, or append `upload` to flash over USB with the FMU in bootloader mode:

```bash
cd local_ws/auxiliary/PX4-Autopilot
make ark_fmu-v6x_default
make ark_fmu-v6x_default upload
```

```
param set SYS_AUTOSTART 4026
param save
reboot
```

---

## Robot description

`arid_description.service` runs `robot_state_publisher` on [`urdf/arid.xacro`](local_ws/src/arid_description/urdf/arid.xacro). Frames: `base_link`, `autopilot`, 4 propellers, `front/left/right_realsense_link`, `top_visual_link` (front cam), `bottom_visual_link` (down cam), `flow_link`, `rangefinder_link`. Visualization: [`arid_description/README.md`](local_ws/src/arid_description/README.md).

---

## Foxglove

apt `foxglove_bridge` on host + container. `foxglove_bridge` alias → port 8765 → connect Studio to `ws://<device-ip>:8765`.

---

## Smoke test and diagnostics

`local_test` is the host-stack smoke test; `ver_cv_cams` opens both feeds, or `front` / `down` for one:

```bash
local_test
ver_cv_cams
```

`local_test` covers: services active, aliases resolve, Foxglove port, full `cam_front` + `cam_down` lifecycles (frame_id, rate, `/alive`), supervisor on the graph (if container up), no leaked subprocesses. Idempotent; leaves pipelines stopped.

`ver_cv_cams` opens the live streams in a cv2 window over NoMachine (`q` quits).

**Camera focus** (menu **7**): selected pipeline + Foxglove bridge; watch `/<cam>/image_raw/compressed` while adjusting the lens; `q` tears down.

---

## USB reset

The `reset_usb` alias, or the underlying Trigger call:

```bash
reset_usb
ros2 service call /reset_usb std_srvs/srv/Trigger '{}'
```

`uhubctl` power-cycles the ARK PAB USB hub and `gpioset` pulses the FMU reset line (GPIO 85), so `/reset_usb` also resets the flight controller: never call it in flight. Details: [`reset_ark_usb/README.md`](local_ws/src/reset_ark_usb/README.md).

---

## Camera calibration

Pass `front` or `down` (menu 6 prompts for it):

```bash
cam_calibrate front
```

[`camera_calibration_auto/camera_calibrate.sh`](local_ws/auxiliary/camera_calibration/camera_calibration_auto/camera_calibrate.sh): venv-isolated interactive calibrator against the live pipeline. Default board = included 10×7-square / 50 mm PDF. Writes the live calibration (`config/calibrations/<cam>.yaml`) + timestamped store. Afterwards set `calibration: "cam_front"` / `"cam_down"` in `pipelines.yaml`. Gates on a NoMachine session (`NM_WAIT_S`, default 180 s). Details: [`camera_calibration/README.md`](local_ws/auxiliary/camera_calibration/README.md).

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
| `cam_front_*` / `cam_down_*` | start / stop / status / alive per pipeline. |
| `cam_refresh` | Re-read `pipelines.yaml` at runtime (stops pipelines first). |
| `cam_calibrate front\|down` | Calibrate a CSI camera. |
| `config_realsense` | Assign the three RealSense serials. |
| `local_test` / `ver_cv_cams` | Smoke test / live feeds. |
| `wifi` / `zt_join` / `update_submods` | Wi-Fi / ZeroTier / submodule sync. |
| `foxglove_bridge` | Bridge on 8765. |
| `initialize` / `deinitialize` / `status` | VSLAM via the supervisor. |

**Container** (`arid_env.sh`): `vslam` (direct launch), `initialize` / `deinitialize` / `status`, `reset_usb`, `colcon_isaac` / `clean_isaac` / `rosdep_isaac`, `foxglove_bridge`, `help`.

---

## Package reference

| Package | Purpose |
|---|---|
| [`arid_description`](local_ws/src/arid_description/) | Xacro, meshes, RViz config. |
| [`ros_gst_cameras`](local_ws/src/ros_gst_cameras/) | CSI camera stack (`gst_cam_node` + `gst_camera_manager`). |
| [`reset_ark_usb`](local_ws/src/reset_ark_usb/) | `/reset_usb` service. |
| [`camera_calibration`](local_ws/auxiliary/camera_calibration/) | front/down calibrator + pattern. |
| [`px4_vslam`](isaac_ros-dev/src/px4_vslam/) | 3-cam VSLAM launch + PX4 bridge. |
| [`px4_vslam_reactor`](isaac_ros-dev/src/px4_vslam_reactor/) | VSLAM jump gating + re-seat. |
| [`arid_supervisor`](isaac_ros-dev/src/arid_supervisor/) | VSLAM lifecycle service (camera-proven bring-up, landed gate). |

| Submodule | Role |
|---|---|
| [`PX4-Autopilot`](local_ws/auxiliary/PX4-Autopilot/) | PX4 fork (`PX4-InDro`), ARID v1.2 airframe. |
| [`px4_msgs`](isaac_ros-dev/src/px4_msgs/) | PX4 messages (`release/1.15`). Single submodule; `local_ws/src/px4_msgs` is a symlink to it. |
| [`isaac_ros_common`](isaac_ros-dev/src/isaac_ros_common/) | Isaac base (Dockerfile chain; ARID-patched). |
| [`isaac_ros_nitros`](isaac_ros-dev/src/isaac_ros_nitros/) | Zero-copy transport. |
| [`isaac_ros_image_pipeline`](isaac_ros-dev/src/isaac_ros_image_pipeline/) | GPU image processing. |
| [`isaac_ros_visual_slam`](isaac_ros-dev/src/isaac_ros_visual_slam/) | cuVSLAM backend. |
| [`realsense-ros`](isaac_ros-dev/src/realsense-ros/) | RealSense driver (`4.51.1`). |
| [`px4-ros2-interface-lib`](isaac_ros-dev/src/px4-ros2-interface-lib/) | Auterion PX4 SDK (`1.4.0`). |
