# InDro ARID Workspace

**Autonomous Research Indoor Drone.** NVIDIA Jetson Orin · ARK PAB carrier · ROS 2 Humble · NVIDIA Isaac ROS · PX4

End-to-end repository for provisioning, building, and operating a deployed ARID. Contents:

- **Bootstrap** (`setup.sh`). Idempotent provisioning from a fresh Ubuntu 22.04 install: apt repos and ROS packages, NetworkManager profiles + dispatcher for the LiDAR Ethernet path, kernel UDP buffer tuning, USB and GPIO udev rules, sudoers + group memberships, systemd unit installation, host `.bashrc` rewrite (alias set), local ROS 2 workspace build, and the Isaac ROS Docker container with the ARID-patched Dockerfile chain.
- **Robot description** (`arid_description`). URDF / xacro and meshes for the airframe and sensor frames. `robot_state_publisher` (systemd-managed) publishes `/robot_description` (latched) and `/tf_static` on boot.
- **CSI camera stack** (`ros_gst_cameras`). `gst_cam_node` (C++) wraps a GStreamer pipeline; `gst_camera_manager` (Python) supervises it, exposes `SetBool` start/stop services plus `status` / `status_all` / `stop_all` Triggers, and runs a frame-flow watchdog publishing a latched `Bool` per pipeline. Currently configured for the downward IMX219 (`cam_down`).
- **RSAIRY LiDAR** (`rslidar_coordinator`). Supervisor for the RoboSense RSAIRY SDK. `SetBool` / `Trigger` services, latched `alive` Bool, process-group teardown that survives the SDK's `ERRCODE_MSOPTIMEOUT` retry loop, and a NetworkManager dispatcher that auto-configures the LiDAR Ethernet link with DHCP fallback.
- **Visual SLAM** (`px4_vslam`, `px4_vslam_reactor`). NVIDIA Isaac cuVSLAM against the front Intel RealSense (D43X-series IR stereo), bridged into PX4 visual odometry via `vio_transform` (FLU → FRD). The reactor gates pose jumps and retries `SetSlamPose` on misalignment.
- **PX4 firmware**. InDro fork (`local_ws/auxiliary/PX4-Autopilot`, branch `PX4-InDro`) with the `4025_arid_quad_v1_1` airframe. Flashable prebuilt artifacts in `local_ws/auxiliary/PX4_prebuilt/` for FMU bring-up without a host build environment.
- **USB recovery** (`reset_ark_usb`). ROS service hosting `/reset_usb` (Trigger). On call, the systemd one-shot power-cycles the ARK PAB USB hub via `uhubctl` + GPIO 85.
- **Host diagnostics**. `lidar_diag` (LiDAR network walk-through), `local_test` (host-stack lifecycle smoke test), `config_lidar` (LiDAR IP auto-detect + NM profile rewrite), `config_realsense` (front RealSense serial auto-detect into `vslam_config.yaml`).
- **Visualisation**. apt-installed Foxglove bridge on both host and container, configured to listen on TCP 8765.

> **ROS_DOMAIN_ID = 23** (non-standard). The default ROS 2 domain is 0; ARID systems run on domain 23 to isolate from other ROS networks on the same LAN. This is exported by the host `.bashrc` and inside the Isaac container via `arid_env.sh`. Any tool that needs to see the graph (Foxglove, `ros2 topic list` from another machine) must be set to **`ROS_DOMAIN_ID=23`** as well, or it will see nothing.

---

## setup.sh

```
./setup.sh
```

Every step is **idempotent** and detects its current state from the system: installed packages, existing config files, running services, group memberships. Safe to re-run any time, after a `git pull`, after a reboot, on a freshly-flashed Jetson. The script installs and configures only what is missing.

All output is logged to `log/setup_log_<timestamp>.log`.

### Steps (in order)

| Step | What it does |
|---|---|
| **power** | Sets nvpmodel to max power (mode 0). Holds critical L4T and kernel packages from apt upgrades. |
| **repos** | Adds ROS, NVIDIA Jetson, and Docker APT repos. Regenerates NVIDIA CDI config. |
| **apt** | Installs ROS packages, libusb, camera-info-manager, compressed-image-transport, `iputils-arping`, `ros-humble-foxglove-bridge`, `ros-humble-foxglove-msgs`. |
| **px4_deps** | Runs PX4 `Tools/setup/ubuntu.sh` (interactive prompt) to install firmware build deps. **Skipped automatically if `arm-none-eabi-gcc` is already on the system** (PX4's setup script has run here before). |
| **git** | Sets git credential cache. Fixes script permissions. Initializes and updates submodules. |
| **docker_patches** | Copies patched `Dockerfile.arid`, `arid_env.sh`, and `run_dev.sh` into the `isaac_ros_common` submodule. Marks them skip-worktree so git ignores local changes. |
| **skip_worktree** | Marks tracked files inside `ros_gst_cameras/gst_camera_manager/config/` and `px4_vslam/config/` as skip-worktree, so local edits (camera serials, calibrations, pipeline tuning) don't appear in `git status` or get pushed by accident. |
| **bashrc** | Rewrites the host `.bashrc` block: `ROS_DOMAIN_ID=23`, workspace path exports, sources `local_ws/install/setup.bash`. Adds aliases: `run_isaac`, `colcon_local`, `reset_usb`, `foxglove_bridge`, `cam_down_*`, `rslidar_*`, `lidar_diag`, `local_test`. |
| **permissions** | Sudoers rule (uhubctl, gpioset, systemctl, `usb_reset.sh`, all without password). USB and GPIO udev rules. Polkit rule for `reset_usb.service`. Adds user to `dialout` and `gpio` groups. |
| **uhubctl** | Builds and installs `uhubctl` from source. Skips if already installed. |
| **lidar_sysctl** | Writes `/etc/sysctl.d/99-rslidar.conf` raising `net.core.rmem_max` and `rmem_default` to 25 MiB so the RSAIRY's bursty UDP traffic does not overflow the kernel receive queue. |
| **lidar_network** | Creates two NetworkManager connections on `enP8p1s0`: `rslidar` (static, priority 10, with explicit `ipv4.routes` for the connected subnet) and `dev` (DHCP, priority 0). Installs `/etc/NetworkManager/dispatcher.d/90-rslidar` which ARP-probes the LiDAR for up to 8 s on link-up and falls back to DHCP if no response. If a LiDAR is reachable, `config_lidar` then sniffs the wire and rewrites both the NM profile and the dispatcher to match the LiDAR's firmware-side IPs. See [LiDAR auto-setup](#auto-setup-config_lidar). |
| **systemd** | Copies and enables all systemd services. See [Boot sequence](#boot-sequence) below. |
| **ros_workspace** | Installs Python deps (pyudev, pyserial, empy). Runs `rosdep install`. Builds `local_ws` with colcon. |
| **docker** | Each sub-step is independently checked: installs Docker engine if missing, enables the service if not running, configures the NVIDIA container runtime if not registered, adds the user to the `docker` group if not in it, installs `docker-buildx-plugin` if missing. |

---

## Boot sequence

Auto-started services after boot:

| Service | What it does |
|---|---|
| **usbfs-memory.service** | One-shot. Raises `usbcore.usbfs_memory_mb` to 1000 (default 16) so high-bandwidth USB cameras (RealSense multi-stream) don't hit "Out of frame resources!" or watchdog timeouts. Runs before the Docker and camera services. |
| **jetson-clocks.service** | Locks CPU and GPU clocks to maximum frequency. Supplements nvpmodel. |
| **start_isaac_docker.service** | Pulls and starts the Isaac ROS Docker container (`isaac_ros_dev-aarch64-container`) so it's ready before any ROS nodes launch. |
| **arid_description.service** | Launches `robot_state_publisher` for the ARID xacro. Publishes `/robot_description` (latched) and `/tf_static`. The VSLAM stack waits for this before starting. |
| **gst_camera_manager.service** | Starts the GStreamer camera manager. The `cam_down` pipeline is loaded but **idle**. Activate via a SetBool service call. See [Cameras](#cameras-csi). |
| **rslidar_coordinator.service** | Starts the RoboSense RSAIRY supervisor. Manages the `rslidar_sdk_node` subprocess. Coordinator is up at boot; LiDAR is **idle** until SetBool. See [LiDAR](#lidar-robosense-rsairy). |
| **usb_ros_reset.service** | Hosts the `/reset_usb` ROS 2 service (Trigger). Performs a hardware USB reset on the ARK PAB carrier on demand. |

### What to expect at login

1. Docker container running. `docker ps` shows `isaac_ros_dev-aarch64-container`.
2. ARID TF tree available. `ros2 topic echo /tf_static --once --qos-durability transient_local` returns immediately.
3. Camera manager idle. The `cam_down` service exists but no frames flowing yet.
4. LiDAR coordinator idle. `/rslidar_coordinator/enable` service exists; no cloud flowing yet.
5. Local workspace already sourced in `.bashrc`. ROS 2 packages in `local_ws` are immediately available.

---

## Cameras (CSI)

One CSI camera pipeline defined in [`local_ws/src/ros_gst_cameras/gst_camera_manager/config/pipelines.yaml`](local_ws/src/ros_gst_cameras/gst_camera_manager/config/pipelines.yaml). Uses an IMX219 sensor at 1920×1080 at 20 fps (delivered ~16 Hz), converted to GRAY8 via `nvarguscamerasrc` → `nvvidconv` → `appsink`. The sensor is IR-sensitive, so the stream is published as mono for direct use in IR-aware computer-vision tasks (feature tracking, motion detection, fiducial decoding) without per-channel filtering.

| Pipeline | Sensor ID | Frame ID | Topic root |
|---|---|---|---|
| `cam_down` | `sensor-id=0` | `bottom_visual_link` | `/cam_down` |

**Start, stop, status:**

```bash
# Start
ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool '{data: true}'

# Stop
ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool '{data: false}'

# Status (per-pipeline and global)
ros2 service call /gst_camera_manager/cam_down/status  std_srvs/srv/Trigger '{}'
ros2 service call /gst_camera_manager/status_all       std_srvs/srv/Trigger '{}'
ros2 service call /gst_camera_manager/stop_all         std_srvs/srv/Trigger '{}'
```

Each pipeline publishes `/<topic>/image_raw`, `/<topic>/image_raw/compressed`, and `/<topic>/camera_info`. Liveness is exposed on `/gst_camera_manager/<name>/alive` (latched `Bool`). For deeper details (encoding auto-detect, default camera_info, QoS, alive_threshold, troubleshooting), see [`ros_gst_cameras/README.md`](local_ws/src/ros_gst_cameras/README.md).

---

## LiDAR (RoboSense RSAIRY)

A RoboSense RSAIRY 3-D LiDAR is supervised by [`local_ws/src/rslidar_coordinator`](local_ws/src/rslidar_coordinator/). The coordinator spawns `rslidar_sdk_node` as a managed subprocess. The coordinator is up at boot via `rslidar_coordinator.service`. The LiDAR pipeline itself is **idle** until SetBool, matching the `gst_camera_manager` pattern.

The SDK reads its config from [`rslidar_coordinator/config/rslidar.yaml`](local_ws/src/rslidar_coordinator/config/rslidar.yaml). The `rslidar_sdk` submodule is never patched. Cloud-only at the moment. IMU parsing is disabled because the SDK's IMU parser is gated by a compile-time flag that this workspace deliberately does not flip.

### Hardware and network

| Setting | Value |
|---|---|
| Jetson NIC | `enP8p1s0` |
| LiDAR IP / Jetson IP | auto-detected by `config_lidar` (see below) |
| Factory-default LiDAR IP | `192.168.1.200` (fallback only) |
| MSOP (point-cloud) port | UDP `6699` |
| DIFOP (device info) port | UDP `7788` |
| IMU port (socket bound) | UDP `6688` |

Setup.sh's `lidar_network` step configures NetworkManager with two profiles on `enP8p1s0`: `rslidar` (static, priority 10) and `dev` (DHCP, priority 0). On link-up, an NM dispatcher script ARP-probes the LiDAR for up to 8 s. If it responds, the static profile stays active. If not, the system falls back to DHCP. Plug into the LiDAR for a sub-second static. Plug into a router for an 8 s wait followed by DHCP. Auto-swaps on cable change.

The `rslidar` profile includes an explicit `ipv4.routes` entry. Without it, NetworkManager sets `noprefixroute` on the manual address and the kernel never installs a connected route for the LiDAR subnet, so traffic is silently routed via the wifi default gateway. The dispatcher's `arping` also pins its source IP with `-s` for the same reason.

### Auto-setup (`config_lidar`)

RoboSense LiDARs store their own IP and the host IP they unicast to in non-volatile firmware. Any prior RSView session may have moved them off the factory defaults. To handle that, `setup.sh` runs `scripts/config_lidar.sh` after creating the NM profiles, and the command is also exposed as the `config_lidar` alias for manual use.

What it does:

1. Sniffs `enP8p1s0` for 10 s with `tcpdump` (ARP + UDP on the LiDAR ports).
2. Extracts the LiDAR's MAC, source IP, and the destination (host) IP it expects.
3. Rewrites `ipv4.addresses` and `ipv4.routes` on the `rslidar` NM connection to match.
4. Rewrites the dispatcher with the discovered IPs.
5. Brings up `rslidar` and verifies the LiDAR answers ARP at the discovered IP.
6. Persists the detected values to `.lidar/rslidar_detected.conf` (workspace-local, gitignored). `lidar_diag` reads this file.

Run it manually any time a LiDAR is swapped, reconfigured via RSView, or moved between hosts. The `config_lidar` alias prepends `sudo` because the script needs `tcpdump` in promiscuous mode and writes to `/etc/NetworkManager/`:

```bash
config_lidar
```

If the LiDAR is unreachable at setup time, the fallback static (`192.168.1.102/24` targeting `192.168.1.200`) stays in place. Re-run `config_lidar` once the LiDAR is plugged in and powered.

### Start, stop, status

The host aliases are the recommended driver:

```bash
rslidar_start
rslidar_stop
rslidar_status
rslidar_alive
rslidar_restart
```

`rslidar_start` is equivalent to a direct SetBool call on the coordinator's enable service:

```bash
ros2 service call /rslidar_coordinator/enable std_srvs/srv/SetBool '{data: true}'
```

### Topics and frame

| Topic | Type | Notes |
|---|---|---|
| `/rslidar_points` | `sensor_msgs/PointCloud2` | Point cloud. Frame `rslidar_link`. BEST_EFFORT QoS. |
| `/rslidar_coordinator/alive` | `std_msgs/Bool` (latched, TRANSIENT_LOCAL) | Liveness from frame-flow watchdog. |
| `/rslidar_coordinator/enable` | `std_srvs/SetBool` | Start (`true`) or stop (`false`) the subprocess. |
| `/rslidar_coordinator/status` | `std_srvs/Trigger` | Returns `RUNNING (pid=N)` or `STOPPED`. |
| `/rslidar_coordinator/restart` | `std_srvs/Trigger` | Kill + respawn. |

The cloud is stamped in `rslidar_link`, which is fixed to `base_link` by the joint defined in [`arid.xacro`](local_ws/src/arid_description/xacro/arid.xacro) at `(0.062732, 0, 0.16457)` with `pitch=0.485201` rad (about 27.8° forward tilt).

### Watchdog

The coordinator subscribes to `/rslidar_points` and ticks at 2 Hz. If frames stop arriving for more than `alive_threshold` seconds (default `5.0`), it flips `/rslidar_coordinator/alive` to `false`. Subprocess death is also detected and logged via `journalctl -u rslidar_coordinator -f`. **No auto-restart**: recovery requires an explicit `rslidar_start` or `rslidar_restart`.

---

## Visual-Inertial SLAM (RealSense + Isaac ROS VSLAM)

One front-mounted RealSense camera (D43X-series) feeding the Isaac ROS VSLAM node, plus a PX4 bridge. Camera serial, resolution, and frame ID are configured in [`isaac_ros-dev/src/px4_vslam/config/vslam_config.yaml`](isaac_ros-dev/src/px4_vslam/config/vslam_config.yaml).

```bash
# Inside the Isaac ROS container (use `start_isaac` then `isaac_bash`)
ros2 launch px4_vslam vslam.launch.py
# or via alias:
vslam
```

The launch blocks until `/robot_description` is on the graph (`arid_description.service` is up). It then starts:

- 1× RealSense driver (`front_realsense`)
- Isaac ROS Visual SLAM node (2-camera stereo on the front IR pair)
- `vio_transform`: bridges VSLAM odometry into PX4 via uXRCE-DDS
- `vslam_reactor_node`: supervises VSLAM, gates jumps, retries SetSlamPose on misalignment

Tunables for the reactor live in [`px4_vslam_reactor/config/reactor_conf.yaml`](isaac_ros-dev/src/px4_vslam_reactor/config/reactor_conf.yaml). See [`px4_vslam/README.md`](isaac_ros-dev/src/px4_vslam/README.md) and [`px4_vslam_reactor/README.md`](isaac_ros-dev/src/px4_vslam_reactor/README.md) for full topic, service, and config reference.

**Before flying:** edit `vslam_config.yaml` to match the specific RealSense serial number and the actual mount frame in [`arid_description/xacro/arid.xacro`](local_ws/src/arid_description/xacro/arid.xacro).

---

## PX4 firmware

Custom PX4 fork at [`local_ws/auxiliary/PX4-Autopilot/`](local_ws/auxiliary/PX4-Autopilot/) (submodule, branch `PX4-InDro`). Includes the **ARID Quad V1.1** airframe (`4025_arid_quad_v1_1`) and Jetson-friendly install patches.

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
param set SYS_AUTOSTART 4025
param save
reboot
```

---

## Robot description (`arid_description`)

Xacro description and meshes for ARID. Auto-launched on boot by `arid_description.service`, which runs `display.launch.py` with `robot_state_publisher`. Frames published include `base_link`, `autopilot`, four propellers, `front_realsense_link`, `rslidar_link` (RoboSense RSAIRY mount), `bottom_visual_link` (CV-camera frame), `flow_link`, and `rangefinder_link`. The xacro source lives at [`local_ws/src/arid_description/xacro/arid.xacro`](local_ws/src/arid_description/xacro/arid.xacro).

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

Two helper scripts live in [`scripts/`](scripts/) and are wired into the host bashrc as aliases. After `setup.sh` and a reboot, run both in order: `lidar_diag` for network + LiDAR reachability + coordinator/SDK runtime status, then `local_test` for the full host-stack lifecycle smoke test.

```bash
lidar_diag
local_test
```

Both are verbose, idempotent, and safe to run any time.

### `lidar_diag`

Run without `sudo` for read-only checks; the passive `tcpdump` and `arp-scan` steps require root:

```bash
lidar_diag
sudo lidar_diag
```

Reads the auto-detected IPs from `.lidar/rslidar_detected.conf` (populated by `config_lidar`). Walks through:

1. **Interface and link**: carrier up, link speed/duplex, MTU, host MAC, lifetime RX counters.
2. **Active NM profile** on `enP8p1s0`: `rslidar` (static) vs `dev` (DHCP fallback).
3. **IPv4 address and route**: confirms the detected host IP is assigned and a route to the LiDAR subnet exists via this NIC.
4. **ARP probe at the detected LiDAR IP**: response time, replying MAC, match against the detected config.
5. **Passive sniff** (needs sudo): `tcpdump` on UDP 6699/7788/6688 with packets-per-second estimate. Reveals where the LiDAR actually is if it has been reconfigured.
6. **Active subnet scan** with `arp-scan` (needs sudo): last resort.
7. **`rslidar_coordinator` and SDK runtime**: service active, SDK subprocess PID, bound UDP ports, `/rslidar_points` rate over 3 s, `/rslidar_coordinator/alive` value.

Source: [`scripts/lidar_diag.sh`](scripts/lidar_diag.sh).

### `local_test`

```bash
local_test
```

Smoke test for the host stack. Each step prints what is being checked, the raw value it got, why it matters, and what to do if it fails. Leaves both pipelines stopped at the end. Destructive aliases (`reset_usb`, `clean_local`) are existence-checked only, not invoked.

Covers:

- The three host systemd services are active (`arid_description`, `gst_camera_manager`, `rslidar_coordinator`).
- All managed bashrc aliases are defined and resolve to runnable targets.
- `foxglove_bridge` is listening on TCP 8765 (launches it if not already running).
- Full `cam_down` lifecycle: stop → start → topics → `header.frame_id == bottom_visual_link` → publish rate over 5 s → `/alive` latched true → stop.
- Full `rslidar` lifecycle: stop → start → topics → `/alive` readable → cloud rate (SKIP if LiDAR is off) → restart produces a new PID → stop.
- No subprocess leaks at the end.

Source: [`scripts/local_test.sh`](scripts/local_test.sh).

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

A wrapper around `ros2 camera_calibration` is at [`local_ws/auxiliary/camera_calibration/`](local_ws/auxiliary/camera_calibration/). Auto-detects display, bootstraps a numpy<2 venv (so `cv_bridge` works), interactively picks the topic, saves output as `<topic_slug>_calibration.yaml` next to the script. Built to work over a NoMachine remote display.

```bash
./local_ws/auxiliary/camera_calibration/camera_calibrate.sh
```

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
| `reset_usb` | Manually trigger the USB hub reset. |
| `colcon_local` | Build `local_ws` and source it. |
| `clean_local` | Clean `local_ws` build/install/log. |
| `rosdep_local` | Install rosdep deps for `local_ws`. |
| `foxglove_bridge` | Launch the Foxglove WebSocket bridge on port 8765. |

CSI camera (`cam_down`, supervised by `gst_camera_manager`):

| Alias | Action |
|---|---|
| `cam_down_start` | `SetBool(true)` on `/gst_camera_manager/cam_down`. |
| `cam_down_stop` | `SetBool(false)` on `/gst_camera_manager/cam_down`. |
| `cam_down_status` | Trigger `/gst_camera_manager/cam_down/status`. |
| `cam_down_alive` | Read the latched `/gst_camera_manager/cam_down/alive` Bool (TRANSIENT_LOCAL). |

RSAIRY LiDAR (supervised by `rslidar_coordinator`):

| Alias | Action |
|---|---|
| `rslidar_start` | `SetBool(true)` on `/rslidar_coordinator/enable`. |
| `rslidar_stop` | `SetBool(false)` on `/rslidar_coordinator/enable`. |
| `rslidar_status` | Trigger `/rslidar_coordinator/status`. |
| `rslidar_alive` | Read the latched `/rslidar_coordinator/alive` Bool (TRANSIENT_LOCAL). |
| `rslidar_restart` | Trigger `/rslidar_coordinator/restart` (stop + respawn). |

Smoke test and diagnostics:

| Alias | Action |
|---|---|
| `local_test` | Run the host-stack smoke test (`scripts/local_test.sh`). |
| `lidar_diag` | Run the LiDAR network diagnostic (`scripts/lidar_diag.sh`). |
| `config_lidar` | Auto-detect LiDAR IPs and rewrite the NM profile. Prepends `sudo`. |

**Inside the container** (set by `arid_env.sh`):

| Alias | Action |
|---|---|
| `reset_usb` | Trigger the USB hub reset via the ROS service. |
| `vslam` | Launch VSLAM (`px4_vslam`). |
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
| [`local_ws/src/rslidar_coordinator`](local_ws/src/rslidar_coordinator/) | Supervisor for the RoboSense RSAIRY LiDAR. Owns the SDK config, exposes SetBool and Trigger services, runs a frame-flow watchdog. |
| [`local_ws/src/reset_ark_usb`](local_ws/src/reset_ark_usb/) | ROS 2 service wrapping the systemd USB-reset unit. |
| [`local_ws/auxiliary/camera_calibration`](local_ws/auxiliary/camera_calibration/) | Camera-calibration launcher (NoMachine-friendly). |
| [`isaac_ros-dev/src/px4_vslam`](isaac_ros-dev/src/px4_vslam/) | RealSense + Isaac VSLAM launch package + PX4 bridge. |
| [`isaac_ros-dev/src/px4_vslam_reactor`](isaac_ros-dev/src/px4_vslam_reactor/) | VSLAM-to-PX4 supervisor with YAML-tunable thresholds. |

### Submodules (external upstreams)

| Submodule | Role |
|---|---|
| [`local_ws/auxiliary/PX4-Autopilot`](local_ws/auxiliary/PX4-Autopilot/) | PX4 flight-stack fork (`indro-robotics/PX4-Autopilot @ PX4-InDro`). Adds the ARID Quad V1.1 airframe and Jetson install patches. |
| [`local_ws/src/px4_msgs`](local_ws/src/px4_msgs/) | Upstream PX4 message definitions (`PX4/px4_msgs @ release/1.15`). Required by the host-side `usb_ros_reset`-style nodes that touch PX4 telemetry. |
| [`local_ws/src/rslidar_sdk`](local_ws/src/rslidar_sdk/) | RoboSense LiDAR SDK pinned at tag `v1.5.19` (`RoboSense-LiDAR/rslidar_sdk`). Builds `rslidar_sdk_node`, the driver spawned by `rslidar_coordinator`. Nested submodule `rs_driver` is auto-initialized. |
| [`local_ws/src/rslidar_msg`](local_ws/src/rslidar_msg/) | RoboSense LiDAR ROS-message definitions pinned at tag `v1.5.10` (`RoboSense-LiDAR/rslidar_msg`). Pure-message package consumed by `rslidar_sdk` and any LiDAR consumer that touches raw packet types. |
| [`isaac_ros-dev/src/isaac_ros_common`](isaac_ros-dev/src/isaac_ros_common/) | NVIDIA Isaac ROS shared base (Dockerfile chain, common message types, build helpers). Patched by `setup.sh` to inject `Dockerfile.arid` and `arid_env.sh`. |
| [`isaac_ros-dev/src/isaac_ros_nitros`](isaac_ros-dev/src/isaac_ros_nitros/) | NVIDIA's NITROS framework. Zero-copy intra-process tensor and image transport that the VSLAM and image pipeline build on. |
| [`isaac_ros-dev/src/isaac_ros_image_pipeline`](isaac_ros-dev/src/isaac_ros_image_pipeline/) | Isaac ROS GPU image-processing nodes (`RectifyNode`, `ImageFormatConverterNode`, etc.). Used by VSLAM and consumable by other CV pipelines. |
| [`isaac_ros-dev/src/isaac_ros_visual_slam`](isaac_ros-dev/src/isaac_ros_visual_slam/) | NVIDIA's GPU-accelerated visual SLAM node and its message/service interfaces. The VSLAM backend launched by `px4_vslam`. |
| [`isaac_ros-dev/src/realsense-ros`](isaac_ros-dev/src/realsense-ros/) | Intel RealSense ROS 2 driver pinned at `4.51.1`. Drives the front RealSense camera feeding VSLAM. |
| [`isaac_ros-dev/src/px4-ros2-interface-lib`](isaac_ros-dev/src/px4-ros2-interface-lib/) | Auterion's `px4_ros2_cpp` library at `1.4.0`. High-level mode and control SDK for writing custom flight modes against PX4. |
| [`isaac_ros-dev/src/px4_msgs`](isaac_ros-dev/src/px4_msgs/) | Same upstream PX4 messages, mirrored inside the Isaac ROS workspace so container-side packages can build against them. |
