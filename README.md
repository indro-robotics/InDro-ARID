# InDro ARID Workspace

**Autonomous Research Indoor Drone.** NVIDIA Jetson Orin · ARK PAB carrier · ROS 2 Humble · NVIDIA Isaac ROS · PX4

End-to-end repository for provisioning, building, and operating a deployed ARID. Contents:

- **Bootstrap** (`setup.sh`). Idempotent provisioning from a fresh Ubuntu 22.04 install: apt repos and ROS packages, kernel UDP buffer tuning, USB and GPIO udev rules, sudoers + group memberships, systemd unit installation, host `.bashrc` rewrite (alias set), local ROS 2 workspace build, and the Isaac ROS Docker container with the ARID-patched Dockerfile chain.
- **Robot description** (`arid_description`). URDF / xacro and meshes for the airframe and sensor frames. `robot_state_publisher` (systemd-managed) publishes `/robot_description` (latched) and `/tf_static` on boot.
- **CSI camera stack** (`ros_gst_cameras`). `gst_cam_node` (C++) wraps a GStreamer pipeline; `gst_camera_manager` (Python) supervises it, exposes `SetBool` start/stop services plus `status` / `status_all` / `stop_all` Triggers, and runs a frame-flow watchdog publishing a latched `Bool` per pipeline. Configured for both the front IMX219 (`cam_front`) and the downward IMX219 (`cam_down`) CSI pipelines.
- **Visual SLAM** (`px4_vslam`, `px4_vslam_reactor`, `arid_supervisor`). NVIDIA Isaac cuVSLAM against **three** Intel RealSense cameras (front / left / right, D43X-series IR stereo, 6-camera stereo-multicam), bridged into PX4 visual odometry via `vio_transform` (FLU to FRD). The reactor gates pose jumps and retries `SetSlamPose` on misalignment. The supervisor hosts a `SetBool` lifecycle service (`/arid_supervisor/vslam_enable`) that spawns and stops the VSLAM launch as a managed subprocess with a landed-state safety gate. Toggle on or off from any terminal with the `initialize` / `deinitialize` aliases (mirrored on host and container).
- **PX4 firmware**. InDro fork (`local_ws/auxiliary/PX4-Autopilot`, branch `PX4-InDro`) with the `4026_arid_quad_v1_2` airframe (`UXRCE_DDS_PTCFG=1` firmware default — the loopback pairing for `ROS_LOCALHOST_ONLY=1`). Flashable prebuilt artifact `ARID_v1.2.px4` in `local_ws/auxiliary/PX4_prebuilt/` for FMU bring-up without a host build environment.
- **USB recovery** (`reset_ark_usb`). ROS service hosting `/reset_usb` (Trigger). On call, the systemd one-shot power-cycles the ARK PAB USB hub via `uhubctl` + GPIO 85.
- **Host diagnostics**. `local_test` (host-stack lifecycle smoke test), `config_realsense` (auto-detect all three RealSense serials into `vslam_config.yaml`), `ver_cv_cams` (live `cam_front` + `cam_down` feeds in a cv2 window over NoMachine).
- **Host helpers**. `wifi` (interactive NetworkManager Wi-Fi join, scan-and-select or hidden-SSID), `update_submods` (submodule sync + pin verification, used by `setup.sh` and as a developer tool with `--update`), `zt_join` (ZeroTier network join/switch, single-network model), `initialize` / `deinitialize` (toggle VSLAM via the supervisor; survive terminal disconnect because the supervisor runs as a host-managed systemd service that `docker exec`s into the container).
- **Visualisation**. apt-installed Foxglove bridge on both host and container, configured to listen on TCP 8765.

> **ROS_LOCALHOST_ONLY = 1** everywhere (host bashrc, `arid_env.sh`, Dockerfile `ENV`, the `run_dev.sh` argument, every ROS systemd unit, and a systemd-manager `DefaultEnvironment` catch-all drop-in `10-arid-ros-env.conf`): DDS is confined to loopback + shared memory, so ROS discovery never leaks onto Wi-Fi/ZeroTier. Off-box viewing goes through the Foxglove **bridge** (TCP 8765), not raw DDS. Pairing requirement: the FMU's uXRCE-DDS client must also confine its participant — `UXRCE_DDS_PTCFG=1`, a **firmware default of the `4026_arid_quad_v1_2` airframe** (flash `ARID_v1.2.px4` + select `SYS_AUTOSTART 4026`); on older airframes set it manually (reboot required) or all `/fmu` topics silently vanish.
>
> **ROS_DOMAIN_ID = 23** (non-standard). The default ROS 2 domain is 0; ARID systems run on domain 23 to isolate from other ROS networks on the same LAN. This is exported by the host `.bashrc` and inside the Isaac container via `arid_env.sh`. Any tool that needs to see the graph (Foxglove, `ros2 topic list` from another machine) must be set to **`ROS_DOMAIN_ID=23`** as well, or it will see nothing.
>
> **librealsense = RSUSB 2.55.1 at `/usr/local` only.** The container links the RSUSB (`libusb` backend) librealsense built at `/usr/local` by the Dockerfile chain; the apt `ros-humble-librealsense2` package is deliberately purged and pinned out on **both** host and container (drop-in `99-arid-no-apt-librealsense`), `rosdep` runs with `--skip-keys librealsense2`, and the realsense-ros build is pinned with `-Drealsense2_DIR=/usr/local/lib/cmake/realsense2`. The colcon-build step self-heals the linkage if a stray apt copy reappears. The host uses the self-contained pip `pyrealsense2` wheel (for `config_realsense` serial detection). This avoids the V4L2-vs-RSUSB library split that silently drops HW metadata and multi-cam streams.

---

## setup.sh

```
./setup.sh              # interactive menu (full setup or individual tools)
./setup.sh --full       # questionnaire, then full setup unattended
./setup.sh --resume     # post-reboot continuation (smoke test + camera verification)
./setup.sh --help
```

Every step is **idempotent** and detects its current state from the system: installed packages, existing config files, running services, group memberships. Safe to re-run any time, after a `git pull`, after a reboot, on a freshly-flashed Jetson. The script installs and configures only what is missing.

Everything streams to a per-session log under `log/` (`setup_log_*.log`); a full-setup session pins its log path in `~/.arid_setup_log` so the post-reboot `--resume` half appends to the **same** file. The smoke test additionally gets its own `smoke_test_log_*.log`. Both prefixes are pruned to the newest 10.

With no arguments, an interactive single-keypress menu opens:

| Option | Action |
|---|---|
| **1** | Full setup (questionnaire then the chain below) |
| **2** | Smoke test (`local_test.sh`) |
| **3** | RealSense serial assignment (front/left/right, `config_realsense.sh`) |
| **4** | Camera feed check (front + down, `verify_cv_cams.sh`) |
| **5** | Wi-Fi connect (`wifi.sh`) |
| **6** | Camera calibration (auto flow front/down, `camera_calibration_auto/camera_calibrate.sh`) |
| **7** | Camera focus (front/down selector; Foxglove bridge + live `cam_front` / `cam_down` feed) |
| **8** | Build the Isaac container |
| **9** | Colcon-build the in-container workspace and bring `arid_supervisor.service` up. Refuses to run if the Isaac container is not currently running (`start_isaac` first, or `sudo systemctl start start_isaac_docker.service`). |
| **10** | ZeroTier join/switch (`setup_zerotier`: installs the daemon if missing, then joins via `scripts/zt_join.sh`) |
| **11** | Uninstall — stop and remove every systemd unit installed by setup.sh, the sudoers / polkit / udev rules, the ARID block in `~/.bashrc`, every setup.sh sentinel, the docker patches inside `isaac_ros_common`, and the Isaac container + image. Strict explicit confirmation at the prompt. Never runs as part of the full chain. The repo clone, hostname, password, group memberships, apt-mark holds, ROS 2 Humble, JetPack, and the Docker engine itself are left in place. |
| **q** | Quit |

`--full` front-loads a questionnaire (hostname, password, Wi-Fi, NoMachine, PX4 toolchain, RealSense assignment, camera verification, Isaac build, smoke test, reboot) and then runs every step without further prompts. `--resume` is invoked automatically by a hook in the host `.bashrc` on the next interactive shell after `setup.sh` armed `~/.arid_resume_setup` (i.e. after a reboot): it waits for boot-enabled units, runs camera verification, picks up a queued Isaac container build, and finishes with the smoke test.

All output is logged to `log/setup_log_<timestamp>.log`.

### Steps (in order)

Run in `--full` mode the steps execute in the order below. From the interactive menu, option **1) Full setup** triggers the same chain.

| Step | What it does |
|---|---|
| **preflight** | Refuses to run as root. Checks `git`. Aborts if any submodule is uninitialized. |
| **collect_answers** | Questionnaire (hostname, password, Wi-Fi, NoMachine, ZeroTier network id, PX4 toolchain, RealSense assignment, camera feed verification, Isaac build, smoke test, end-of-setup reboot). Records `PRE_*` answers so the remaining steps run without prompting; persisted to `~/.arid_questionnaire` so the post-reboot resume honours them. |
| **first_boot** | One-time hostname / password set. Writes `~/.arid_provisioned` so re-runs skip it. |
| **power** | Sets nvpmodel to max power (mode 0). Holds critical L4T and kernel packages from apt upgrades. |
| **disable_updates** | Disables `unattended-upgrades.service` and the apt-daily timers so a deployed drone does not pull surprise package changes. |
| **enable_clock_sync** | Enables `systemd-timesyncd.service` so PX4 timestamps remain monotonically correct after a power cycle. |
| **enable_user_linger** | `loginctl enable-linger` so systemd owns `/run/user/<uid>` at boot (headless NoMachine black-screen fix); repairs a root-owned runtime dir if one exists. |
| **clean_nvidia_desktop** | Removes NVIDIA first-boot desktop icons and the L4T-README auto-mount. |
| **ensure_wifi** | Calls `scripts/wifi.sh` to join the network captured in the questionnaire (or skips if declined). |
| **nomachine** | Detects an existing NoMachine install; prints a manual-install hint when the arm64 `.deb` needs to be downloaded. |
| **repos** | Adds ROS, NVIDIA Jetson, and Docker APT repos. Regenerates NVIDIA CDI config. |
| **apt** | Installs ROS packages, libusb, camera-info-manager, compressed-image-transport, `iputils-arping`, `tcpdump`, `arp-scan`, `ros-humble-foxglove-bridge`, `ros-humble-foxglove-msgs`. apt `ros-humble-librealsense2` is deliberately NOT installed (host or container): the container links the RSUSB 2.55.1 librealsense built at `/usr/local` by the image chain, and the host uses the self-contained pip `pyrealsense2` wheel. |
| **zerotier** | Installs the ZeroTier daemon (official installer) if missing and joins the questionnaire's network id via `scripts/zt_join.sh --setup` (single-network model; a node already on a network is left alone — switching is `zt_join`'s job). An `ACCESS_DENIED` join is not a failure: the node starts working the moment it is authorized in ZeroTier Central. |
| **px4_deps** | Runs PX4 `Tools/setup/ubuntu.sh` to install firmware build deps. Skipped automatically if `arm-none-eabi-gcc` is already on the system. |
| **git** | Sets git credential cache. Fixes script permissions. Runs `scripts/update_submods.sh` to sync every submodule to its pinned commit and verify the pins against the upstream tags / branches. |
| **docker_patches** | Copies patched `Dockerfile.arid`, `arid_env.sh`, and `run_dev.sh` into the `isaac_ros_common` submodule. Marks them skip-worktree so git ignores local changes. |
| **skip_worktree** | Discovers every `config/`, `cfg/`, and `camera_calibrations/` directory under both workspaces and marks the tracked files inside them as `skip-worktree`. Protects per-deployment values (camera serials, calibrations, pipeline tuning, VSLAM reactor thresholds) from accidentally landing in `git status`. To commit a real change to one of these files, use the standard `git update-index --no-skip-worktree <file>` dance. |
| **bashrc** | Rewrites the host `.bashrc` block: `ROS_DOMAIN_ID=23`, `ROS_LOCALHOST_ONLY=1`, workspace path exports, sources `local_ws/install/setup.bash`. Installs the post-reboot resume hook that fires `setup.sh --resume` on the next interactive shell. Adds aliases: `setup`, `run_isaac`, `colcon_isaac`, `clean_isaac`, `rosdep_isaac`, `colcon_local`, `reset_usb`, `foxglove_bridge`, `cam_front_*`, `cam_down_*`, `cam_refresh`, `cam_calibrate front\|down`, `local_test`, `config_realsense`, `wifi`, `ver_cv_cams`, `update_submods`, `zt_join`, `initialize`, `deinitialize`, `status`, plus a `help` command listing them all. |
| **permissions** | Sudoers rule (uhubctl, gpioset, systemctl, `usb_reset.sh`, `zerotier-cli`, all without password). USB and GPIO udev rules. Polkit rule for `reset_usb.service`. Adds user to `dialout` and `gpio` groups. |
| **uhubctl** | Builds and installs `uhubctl` from source. Skips if already installed. |
| **ros_workspace** | Installs Python deps (pyudev, pyserial, empy). Runs `rosdep install`. Builds `local_ws` with colcon. |
| **docker** | Each sub-step is independently checked: installs Docker engine if missing, enables the service if not running, configures the NVIDIA container runtime if not registered, adds the user to the `docker` group if not in it, installs `docker-buildx-plugin` if missing. |
| **systemd** | Copies all unit files from `local_ws/services/` and `isaac_ros-dev/services/` to `/etc/systemd/system/` and enables them, and installs the `10-arid-ros-env.conf` `DefaultEnvironment` drop-in (`ROS_LOCALHOST_ONLY=1`, `ROS_DOMAIN_ID=23`). See [Boot sequence](#boot-sequence) below. |
| **realsense** | Calls `scripts/config_realsense.sh` to detect the three RealSense serials (front / left / right, pyrealsense2 wheel with rs-enumerate-devices fallback) and write them into `vslam_config.yaml`. |
| **verify_cameras** | If the questionnaire opted in, runs `scripts/verify_cv_cams.sh` to open the live `cam_front` + `cam_down` feeds in a cv2 window over the connected NoMachine session. |
| **build_isaac** (queued) | If the questionnaire opted in, builds the Isaac container via `build_isaac_docker.sh`. The build is queued through a sentinel file so it survives a mid-run reboot. |
| **colcon_isaac** | Builds the in-container workspace via `docker exec ... colcon build --symlink-install --base-paths src --cmake-args -DBUILD_TESTING=OFF`, then `systemctl reset-failed` + `restart` on `arid_supervisor.service` so the supervisor advertises before the smoke test runs. Skipped if the Isaac container image does not exist. Prompts unless the operator already opted in via the questionnaire. Also exposed as menu option **9**. |
| **print_summary** | Lists every step that ran and every step that was skipped. |
| **prompt_reboot** | Arms `~/.arid_resume_setup` then reboots. The bashrc hook then fires `setup.sh --resume` on the next interactive shell, which waits for boot-enabled units, runs camera verification, picks up the queued Isaac build, recovers the supervisor units, and finishes with the smoke test. |

---

## Boot sequence

Auto-started services after boot:

| Service | What it does |
|---|---|
| **usbfs-memory.service** | One-shot. Raises `usbcore.usbfs_memory_mb` to 1000 (default 16) so high-bandwidth USB cameras (RealSense multi-stream) don't hit "Out of frame resources!" or watchdog timeouts. Runs before the Docker and camera services. |
| **jetson-clocks.service** | Locks CPU and GPU clocks to maximum frequency. Supplements nvpmodel. |
| **start_isaac_docker.service** | Pulls and starts the Isaac ROS Docker container (`isaac_ros_dev-aarch64-container`) so it's ready before any ROS nodes launch. |
| **arid_description.service** | Launches `robot_state_publisher` for the ARID xacro. Publishes `/robot_description` (latched) and `/tf_static`. The VSLAM stack waits for this before starting. |
| **gst_camera_manager.service** | Starts the GStreamer camera manager. Both the `cam_front` and `cam_down` pipelines are loaded but **idle**. Activate each via a SetBool service call. Launch output is tee'd to `isaac_ros-dev/run_logs/gst_camera_manager/` (previous log rotated to `gst_camera_manager.prev.log`). See [Cameras](#cameras-csi). |
| **usb_ros_reset.service** | Hosts the `/reset_usb` ROS 2 service (Trigger). Performs a hardware USB reset on the ARK PAB carrier on demand. |
| **arid_supervisor.service** | `After=` + `Requires=` + `PartOf=start_isaac_docker.service`. `docker exec`s the lifecycle supervisor inside the Isaac container so the `/arid_supervisor/vslam_enable` service stays alive across terminal disconnects. VSLAM itself is **idle** until `initialize` is called. `KillMode=mixed`, 30 s stop timeout, restart-burst cap. See [Visual-Inertial SLAM](#visual-inertial-slam-realsense--isaac-ros-vslam). |

### What to expect at login

1. Docker container running. `docker ps` shows `isaac_ros_dev-aarch64-container`.
2. ARID TF tree available. `ros2 topic echo /tf_static --once --qos-durability transient_local` returns immediately.
3. Camera manager idle. The `cam_front` and `cam_down` services exist but no frames flowing yet.
4. VSLAM supervisor up. `ros2 service list | grep arid_supervisor` returns `/arid_supervisor/vslam_enable`; VSLAM is idle until `initialize` is called.
5. Local workspace already sourced in `.bashrc`. ROS 2 packages in `local_ws` are immediately available.

> **First-boot caveat for `arid_supervisor.service`**: the supervisor service launches `ros2 launch arid_supervisor arid_supervisor.launch.py` inside the container, which requires the container-side workspace to have been colcon-built. `build_isaac` produces the container image but does not build the workspace on its own.
>
> The `--full` chain handles this automatically: the `colcon_isaac` step runs straight after `build_isaac` and resets / restarts `arid_supervisor.service`. If you skipped the prompt or only ran `build_isaac` by itself, finish the bring-up with menu option **9** or the manual recipe:
>
> ```bash
> start_isaac && isaac_bash      # enter the container
> colcon_isaac                   # build the workspace (arid_supervisor + px4_vslam etc.)
> exit
> sudo systemctl reset-failed arid_supervisor.service
> sudo systemctl start arid_supervisor.service
> ```
>
> Subsequent reboots are fine because the workspace install/ persists in the bind-mounted `isaac_ros-dev/`.

---

## Cameras (CSI)

Two CSI camera pipelines defined in [`local_ws/src/ros_gst_cameras/gst_camera_manager/config/pipelines.yaml`](local_ws/src/ros_gst_cameras/gst_camera_manager/config/pipelines.yaml). Both use an IMX219 sensor at 1920×1080 at 15 fps, converted to GRAY8 via `nvarguscamerasrc` → `nvvidconv` → `appsink`. The sensor is IR-sensitive, so the stream is published as mono for direct use in IR-aware computer-vision tasks (feature tracking, motion detection, fiducial decoding) without per-channel filtering.

| Pipeline | Sensor ID | Frame ID | Topic root |
|---|---|---|---|
| `cam_front` | `sensor-id=0` | `top_visual_link` | `/cam_front` |
| `cam_down` | `sensor-id=1` | `bottom_visual_link` | `/cam_down` |

**Start, stop, status:**

```bash
# Start (front / down)
ros2 service call /gst_camera_manager/cam_front std_srvs/srv/SetBool '{data: true}'
ros2 service call /gst_camera_manager/cam_down  std_srvs/srv/SetBool '{data: true}'

# Stop
ros2 service call /gst_camera_manager/cam_front std_srvs/srv/SetBool '{data: false}'
ros2 service call /gst_camera_manager/cam_down  std_srvs/srv/SetBool '{data: false}'

# Status (per-pipeline and global)
ros2 service call /gst_camera_manager/cam_front/status std_srvs/srv/Trigger '{}'
ros2 service call /gst_camera_manager/cam_down/status  std_srvs/srv/Trigger '{}'
ros2 service call /gst_camera_manager/status_all       std_srvs/srv/Trigger '{}'
ros2 service call /gst_camera_manager/stop_all         std_srvs/srv/Trigger '{}'
```

Each pipeline publishes `/<topic>/image_raw`, `/<topic>/image_raw/compressed`, and `/<topic>/camera_info`. Liveness is exposed on `/gst_camera_manager/<name>/alive` (latched `Bool`). For deeper details (encoding auto-detect, default camera_info, QoS, alive_threshold, troubleshooting), see [`ros_gst_cameras/README.md`](local_ws/src/ros_gst_cameras/README.md).

---

## Visual-Inertial SLAM (RealSense + Isaac ROS VSLAM)

Three RealSense cameras (front / left / right, D43X / D45X-series) feeding the Isaac ROS VSLAM node in a 6-camera stereo-multicam configuration, plus a PX4 bridge. Camera serials, resolution, and frame IDs are configured in [`isaac_ros-dev/src/px4_vslam/config/vslam_config.yaml`](isaac_ros-dev/src/px4_vslam/config/vslam_config.yaml) (`num_cameras: 6`).

The standard lifecycle path is through the supervisor, which survives terminal disconnect and is callable from either side:

```bash
initialize     # SetBool true  on /arid_supervisor/vslam_enable
deinitialize   # SetBool false on /arid_supervisor/vslam_enable
```

`arid_supervisor` runs inside the Isaac container, managed on the host by `arid_supervisor.service` (`docker exec`s into the container). On `true` it runs a camera-proven bring-up: USB pre-check for the RealSense cameras, spawn, then a log-watch gate on the driver's `RealSense Node Is Up!` marker with ONE automatic `/reset_usb` recovery attempt before failing verbatim (blocks ~15 s healthy, up to ~3 min through a recovery). On `false` it requires fresh `VehicleLandDetected.landed == True` from PX4 telemetry; refused otherwise. Teardown is SIGINT-drain, then SIGTERM/SIGKILL escalation, plus per-participant DDS shared-memory reclaim. Per-launch log at `/workspaces/isaac_ros-dev/run_logs/vslam/vslam.log` (truncated each launch).

Direct (non-supervised) launch is still available; useful for development and one-off bring-up:

```bash
# Inside the Isaac ROS container (use `start_isaac` then `isaac_bash`)
ros2 launch px4_vslam vslam.launch.py
# or via alias:
vslam
```

The launch blocks until `/robot_description` is on the graph (`arid_description.service` is up). It then starts:

- 3x RealSense drivers (`left_realsense`, `front_realsense`, `right_realsense`)
- Isaac ROS Visual SLAM node (6-camera stereo-multicam across the three IR stereo pairs)
- `vio_transform`: bridges VSLAM odometry into PX4 via uXRCE-DDS
- `vslam_reactor_node`: supervises the VSLAM solution, gates jumps, retries `SetSlamPose` on misalignment

Tunables for the reactor live in [`px4_vslam_reactor/config/px4_vslam_reactor.yaml`](isaac_ros-dev/src/px4_vslam_reactor/config/px4_vslam_reactor.yaml). See [`px4_vslam/README.md`](isaac_ros-dev/src/px4_vslam/README.md) and [`px4_vslam_reactor/README.md`](isaac_ros-dev/src/px4_vslam_reactor/README.md) for full topic, service, and config reference.

**Before flying:** edit `vslam_config.yaml` to match all three RealSense serial numbers and the actual mount frames in [`arid_description/urdf/arid.xacro`](local_ws/src/arid_description/urdf/arid.xacro).

---

## PX4 firmware

Custom PX4 fork at [`local_ws/auxiliary/PX4-Autopilot/`](local_ws/auxiliary/PX4-Autopilot/) (submodule, branch `PX4-InDro`). Includes the **ARID Quad V1.2** airframe (`4026_arid_quad_v1_2` — `4025_arid_quad_v1_1` plus `UXRCE_DDS_PTCFG=1` as a firmware default) and Jetson-friendly install patches. Prebuilt artifact: [`local_ws/auxiliary/PX4_prebuilt/ARID_v1.2.px4`](local_ws/auxiliary/PX4_prebuilt/).

### Build

Build the firmware for the target board, or append `upload` to flash over USB with the FMU in bootloader mode:

```bash
cd local_ws/auxiliary/PX4-Autopilot
make ark_fmu-v6x_default
make ark_fmu-v6x_default upload
```

Output: `build/ark_fmu-v6x_default/ark_fmu-v6x_default.px4`.

### Select the airframe at runtime

```
param set SYS_AUTOSTART 4026
param save
reboot
```

---

## Robot description (`arid_description`)

Xacro description and meshes for ARID. Auto-launched on boot by `arid_description.service`, which runs `display.launch.py` with `robot_state_publisher`. Frames published include `base_link`, `autopilot`, four propellers, `front_realsense_link`, `left_realsense_link`, `right_realsense_link`, `top_visual_link` (front CV-camera frame), `bottom_visual_link` (downward CV-camera frame), `flow_link`, and `rangefinder_link`. The xacro source lives at [`local_ws/src/arid_description/urdf/arid.xacro`](local_ws/src/arid_description/urdf/arid.xacro).

For RViz and Foxglove visualization with this xacro, see [`arid_description/README.md`](local_ws/src/arid_description/README.md).

---

## Foxglove visualization

`foxglove_bridge` is installed from apt on both the host (`ros-humble-foxglove-bridge`, set up by `setup.sh`'s `apt` step) and inside the Isaac container (same Debian, installed by the Dockerfile chain). No source build, no submodule.

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
# or (inside the container) via alias:
foxglove_bridge
```

Then connect Foxglove Studio to `ws://<device-ip>:8765`.

---

## Smoke test and diagnostics

Two helper scripts live in [`scripts/`](scripts/) and are wired into the host bashrc as aliases. After `setup.sh` and a reboot, run them in order: `local_test` for the full host-stack lifecycle smoke test, then `ver_cv_cams` to confirm both `cam_front` and `cam_down` actually produce frames over the live X display.

```bash
local_test
ver_cv_cams
```

Both are verbose, idempotent, and safe to run any time.

### `local_test`

```bash
local_test
```

Smoke test for the host stack. Each step prints what is being checked, the raw value it got, why it matters, and what to do if it fails. Leaves both camera pipelines stopped at the end. Destructive aliases (`reset_usb`, `clean_local`) are existence-checked only, not invoked.

Covers:

- The two host systemd services are active (`arid_description`, `gst_camera_manager`).
- All managed bashrc aliases are defined and resolve to runnable targets.
- `foxglove_bridge` is listening on TCP 8765 (launches it if not already running).
- Full `cam_down` lifecycle: stop → start → topics → `header.frame_id == bottom_visual_link` → publish rate over 5 s → `/alive` latched true → stop.
- Full `cam_front` lifecycle: stop → start → topics → `header.frame_id == top_visual_link` → publish rate over 5 s → `/alive` latched true → stop.
- Supervisor graph (optional, requires the Isaac container running): `/arid_supervisor/vslam_enable` advertised.
- No subprocess leaks at the end.

Source: [`scripts/local_test.sh`](scripts/local_test.sh).

### `ver_cv_cams`

```bash
ver_cv_cams            # both feeds
ver_cv_cams front      # front only
ver_cv_cams down       # downward only
```

Brings the selected pipeline(s) up via the `gst_camera_manager` service if not already streaming, then opens the live compressed image stream in a cv2 window on the connected NoMachine display. Press any key in the terminal to advance (or `q` to quit). The pipelines are left stopped if this script started them. Gracefully waits for a NoMachine session to attach before opening the window so a camera is never brought up into a dead display.

Source: [`scripts/verify_cv_cams.sh`](scripts/verify_cv_cams.sh).

### Camera focus (`setup.sh` menu option 7)

Brings up the selected CSI pipeline (`cam_front` or `cam_down`, default down) AND a Foxglove WebSocket bridge on port 8765, prints `ws://<device-ip>:8765` for every routable interface, and tells the operator to add an Image panel for `/<cam>/image_raw/compressed`. Used for adjusting the IMX219 lens focus by watching the live feed in Foxglove Studio. Press `q` in the terminal to stop and tear down. No standalone alias; accessed only through the interactive menu:

```bash
./setup.sh   # then press 7
```

---

## USB reset

The ARK PAB carrier's USB hub can be hardware-reset on demand via the `/reset_usb` service:

```bash
ros2 service call /reset_usb std_srvs/srv/Trigger '{}'
# or via alias:
reset_usb
```

The service node (`usb_ros_reset.service`) calls `systemctl start reset_usb.service`, which invokes `scripts/usb_reset.sh` (uhubctl). For details, see [`reset_ark_usb/README.md`](local_ws/src/reset_ark_usb/README.md).

---

## Camera calibration

One calibrator lives under [`local_ws/auxiliary/camera_calibration/`](local_ws/auxiliary/camera_calibration/):

- [`camera_calibration_auto/camera_calibrate.sh`](local_ws/auxiliary/camera_calibration/camera_calibration_auto/camera_calibrate.sh): the `cam_front` / `cam_down` auto flow, taking a `front|down` argument (the menu prompts for it). Bootstraps a `numpy<2` venv (so `cv_bridge` works), prompts for the board geometry (defaults to the included 10x7-square / 50 mm pattern at `calibration_pattern/calib_pattern.pdf`), brings the selected pipeline up via the `gst_camera_manager` service, runs the interactive calibrator, then writes the result to both the live pipeline calibration (`gst_camera_manager/config/calibrations/<cam>.yaml`) and the timestamped store (`camera_calibrations/<cam>/`). After the first successful run, set `calibration: "cam_front"` / `"cam_down"` in `pipelines.yaml` so the pipeline loads the new intrinsics.

> **Note:** the flow gates on a connected NoMachine session (up to `NM_WAIT_S`, default 180 s) before opening the calibrator GUI. Setup option 6 in the interactive menu runs it.

---

## Useful aliases

**Host** (set by `setup.sh`'s `bashrc` step):

| Alias | Action |
|---|---|
| `run_isaac` | Launch Isaac ROS Docker interactively. |
| `build_isaac` | Rebuild the Isaac ROS Docker image. |
| `start_isaac` | Start the container in the background. |
| `stop_isaac` | Stop the container. |
| `isaac_bash` | Open a shell inside the running container. |
| `setup` | Run the setup / provisioning script from anywhere. |
| `colcon_isaac` | Build the in-container workspace from the host (`scripts/colcon_isaac.sh`): deinitialize VSLAM, build, restart `arid_supervisor.service`. |
| `clean_isaac` | Clean the in-container workspace (via `scripts/in_isaac.sh`). |
| `rosdep_isaac` | Install rosdep deps inside the container (via `scripts/in_isaac.sh`). |
| `reset_usb` | Manually trigger the USB hub reset. |
| `colcon_local` | Build `local_ws` and source it (`scripts/colcon_local.sh`: stops the local_ws host services first, restarts them after). |
| `clean_local` | Clean `local_ws` build/install/log. |
| `rosdep_local` | Install rosdep deps for `local_ws`. |
| `cam_calibrate front\|down` | Calibrate the front or downward camera (`camera_calibration_auto/camera_calibrate.sh`). |
| `zt_join` | Join/switch ZeroTier network (`scripts/zt_join.sh`, single-network model). |
| `help` | Print the full ARID host-command list. |
| `foxglove_bridge` | Launch the Foxglove WebSocket bridge on port 8765. |

CSI cameras (`cam_front` + `cam_down`, supervised by `gst_camera_manager`):

| Alias | Action |
|---|---|
| `cam_front_start` | `SetBool(true)` on `/gst_camera_manager/cam_front`. |
| `cam_front_stop` | `SetBool(false)` on `/gst_camera_manager/cam_front`. |
| `cam_front_status` | Trigger `/gst_camera_manager/cam_front/status`. |
| `cam_front_alive` | Read the latched `/gst_camera_manager/cam_front/alive` Bool (TRANSIENT_LOCAL). |
| `cam_down_start` | `SetBool(true)` on `/gst_camera_manager/cam_down`. |
| `cam_down_stop` | `SetBool(false)` on `/gst_camera_manager/cam_down`. |
| `cam_down_status` | Trigger `/gst_camera_manager/cam_down/status`. |
| `cam_down_alive` | Read the latched `/gst_camera_manager/cam_down/alive` Bool (TRANSIENT_LOCAL). |
| `cam_refresh` | Re-read `gst_camera_manager/config/pipelines.yaml` at runtime. Stops any running pipelines first (publisher QoS is fixed at subprocess launch and can't change in place), clears per-pipeline state, then reloads from YAML. Manager-level services and the watchdog are untouched. Use after editing `pipelines.yaml` to avoid restarting `gst_camera_manager.service`. |

Smoke test and diagnostics:

| Alias | Action |
|---|---|
| `local_test` | Run the host-stack smoke test (`scripts/local_test.sh`). |
| `config_realsense` | Auto-detect all three RealSense serials (pyrealsense2) and write them into `vslam_config.yaml`. |
| `ver_cv_cams` | Live `cam_front` + `cam_down` feed in a cv2 window over NoMachine (`scripts/verify_cv_cams.sh`). |
| `wifi` | Interactive NetworkManager Wi-Fi connect (`scripts/wifi.sh`). |
| `update_submods` | Sync + pin-verify every submodule. `--update` advances live branches. |

VSLAM lifecycle (single SetBool service hosted by `arid_supervisor` inside the container; all aliases mirrored host-side):

| Alias | Action |
|---|---|
| `initialize` | `SetBool(true)` on `/arid_supervisor/vslam_enable`. Spawns `px4_vslam vslam.launch.py` as a managed subprocess. |
| `deinitialize` | `SetBool(false)` on `/arid_supervisor/vslam_enable`. Refused unless `VehicleLandDetected.landed == True` (fresh PX4 telemetry). |
| `status` | Trigger `/arid_supervisor/status`: vslam running (true/false) + land state. Side-effect-free. |

**Inside the container** (set by `arid_env.sh`):

| Alias | Action |
|---|---|
| `reset_usb` | Trigger the USB hub reset via the ROS service. |
| `vslam` | Launch VSLAM directly (`ros2 launch px4_vslam vslam.launch.py`), bypassing the supervisor. |
| `initialize` | Same as the host alias. |
| `deinitialize` | Same as the host alias. |
| `status` | Same as the host alias (guarded on the supervisor being up). |
| `foxglove_bridge` | Start the Foxglove bridge on port 8765. |
| `rosdep_isaac` | Install rosdep deps for `isaac_ros_dev`. |
| `colcon_isaac` | Build the `isaac_ros_dev` workspace. |
| `clean_isaac` | Clean the `isaac_ros_dev` build/install/log. |

---

## Package reference

### Custom packages (in this repo)

| Package | Purpose |
|---|---|
| [`local_ws/src/arid_description`](local_ws/src/arid_description/) | Xacro, meshes, RViz config. Auto-launched on boot. |
| [`local_ws/src/ros_gst_cameras`](local_ws/src/ros_gst_cameras/) | ROS 2 GStreamer-based camera stack (`gst_cam_node` and `gst_camera_manager`). |
| [`local_ws/src/reset_ark_usb`](local_ws/src/reset_ark_usb/) | ROS 2 service wrapping the systemd USB-reset unit. |
| [`local_ws/auxiliary/camera_calibration`](local_ws/auxiliary/camera_calibration/) | Camera calibration (NoMachine-friendly): the setup.sh-integrated `camera_calibration_auto/` front/down flow + checkerboard pattern. |
| [`isaac_ros-dev/src/px4_vslam`](isaac_ros-dev/src/px4_vslam/) | RealSense + Isaac VSLAM launch package + PX4 bridge (three-camera stereo-multicam). |
| [`isaac_ros-dev/src/px4_vslam_reactor`](isaac_ros-dev/src/px4_vslam_reactor/) | Gates VSLAM solution jumps and retries `SetSlamPose` on misalignment. YAML-tunable thresholds. |
| [`isaac_ros-dev/src/arid_supervisor`](isaac_ros-dev/src/arid_supervisor/) | Lifecycle supervisor: hosts `/arid_supervisor/vslam_enable` (SetBool) to spawn and stop `px4_vslam` as a managed subprocess. Landed-state safety gate on disable. |

### Submodules (external upstreams)

| Submodule | Role |
|---|---|
| [`local_ws/auxiliary/PX4-Autopilot`](local_ws/auxiliary/PX4-Autopilot/) | PX4 flight-stack fork (`indro-robotics/PX4-Autopilot @ PX4-InDro`). Adds the ARID Quad V1.1 airframe and Jetson install patches. |
| [`local_ws/src/px4_msgs`](local_ws/src/px4_msgs/) | Upstream PX4 message definitions (`PX4/px4_msgs @ release/1.15`). Required by the host-side `usb_ros_reset`-style nodes that touch PX4 telemetry. |
| [`isaac_ros-dev/src/isaac_ros_common`](isaac_ros-dev/src/isaac_ros_common/) | NVIDIA Isaac ROS shared base (Dockerfile chain, common message types, build helpers). Patched by `setup.sh` to inject `Dockerfile.arid` and `arid_env.sh`. |
| [`isaac_ros-dev/src/isaac_ros_nitros`](isaac_ros-dev/src/isaac_ros_nitros/) | NVIDIA's NITROS framework. Zero-copy intra-process tensor and image transport that the VSLAM and image pipeline build on. |
| [`isaac_ros-dev/src/isaac_ros_image_pipeline`](isaac_ros-dev/src/isaac_ros_image_pipeline/) | Isaac ROS GPU image-processing nodes (`RectifyNode`, `ImageFormatConverterNode`, etc.). Used by VSLAM and consumable by other CV pipelines. |
| [`isaac_ros-dev/src/isaac_ros_visual_slam`](isaac_ros-dev/src/isaac_ros_visual_slam/) | NVIDIA's GPU-accelerated visual SLAM node and its message/service interfaces. The VSLAM backend launched by `px4_vslam`. |
| [`isaac_ros-dev/src/realsense-ros`](isaac_ros-dev/src/realsense-ros/) | Intel RealSense ROS 2 driver pinned at `4.51.1`. Drives the three RealSense cameras feeding VSLAM. |
| [`isaac_ros-dev/src/px4-ros2-interface-lib`](isaac_ros-dev/src/px4-ros2-interface-lib/) | Auterion's `px4_ros2_cpp` library at `1.4.0`. High-level mode and control SDK for writing custom flight modes against PX4. |
| [`isaac_ros-dev/src/px4_msgs`](isaac_ros-dev/src/px4_msgs/) | Same upstream PX4 messages, mirrored inside the Isaac ROS workspace so container-side packages can build against them. |
