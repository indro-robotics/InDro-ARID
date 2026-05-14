# ARID Drone Workspace

NVIDIA Jetson Orin · ROS 2 Humble · Isaac ROS · PX4

End-to-end workspace for the **ARID** quadrotor: bootstrap script, robot description, GStreamer-based CSI camera stack, RealSense visual-inertial SLAM, RoboSense RSAIRY LiDAR coordinator, PX4 fork, USB-reset service, and Foxglove visualization.

> **ROS_DOMAIN_ID = 23** (non-standard). The default ROS 2 domain is 0; ARID systems run on domain 23 to isolate from other ROS networks on the same LAN. This is exported automatically by the host `.bashrc` and inside the Isaac container via `arid_env.sh`. Any tool that needs to see the graph (Foxglove, `ros2 topic list` from another machine, etc.) must be set to **`ROS_DOMAIN_ID=23`** as well, or it will see nothing.

---

## setup.sh

```
./setup.sh
```

Every step is **idempotent** and detects its current state from the system (installed packages, existing config files, running services, group memberships). Safe to re-run any time — after a `git pull`, after a reboot, on a freshly-flashed Jetson, etc. The script installs and configures only what's missing.

All output is logged to `log/setup_log_<timestamp>.log`.

### Steps (in order)

| Step | What it does |
|---|---|
| **power** | Sets nvpmodel to max power (mode 0); holds critical L4T / kernel packages from apt upgrades |
| **repos** | Adds ROS, Nvidia Jetson, and Docker APT repos; regenerates NVIDIA CDI config |
| **apt** | Installs ROS packages, libusb, camera-info-manager, compressed-image-transport, `iputils-arping`, etc. |
| **px4_deps** | Runs PX4 `Tools/setup/ubuntu.sh` (interactive prompt) to install firmware build deps. **Skipped automatically if `arm-none-eabi-gcc` is already on the system** (i.e. PX4's setup script has run here before) |
| **git** | Sets git credential cache; fixes script permissions; initializes and updates submodules |
| **docker_patches** | Copies patched `Dockerfile.arid`, `arid_env.sh`, and `run_dev.sh` into the `isaac_ros_common` submodule; marks them skip-worktree so git ignores local changes |
| **skip_worktree** | Marks tracked files inside `ros_gst_cameras/gst_camera_manager/config/` and `px4_vslam/config/` as skip-worktree, so local edits (camera serials, calibrations, pipeline tuning) don't appear in `git status` or get pushed by accident |
| **bashrc** | Rewrites the host `.bashrc` block: `ROS_DOMAIN_ID=23`, workspace path exports, sources `local_ws/install/setup.bash`, adds aliases (`run_isaac`, `colcon_local`, `reset_usb`, `foxglove_bridge`, `cam_down_start`/`stop`/`status`/`alive`, `rslidar_start`/`stop`/`status`/`alive`/`restart`, etc.) |
| **permissions** | Sudoers rule (uhubctl, gpioset, systemctl, `usb_reset.sh` — all without password); USB + GPIO udev rules; polkit rule for `reset_usb.service`; adds user to `dialout` + `gpio` groups |
| **uhubctl** | Builds and installs `uhubctl` from source (skips if already installed) |
| **lidar_sysctl** | Writes `/etc/sysctl.d/99-rslidar.conf` raising `net.core.rmem_max` / `rmem_default` to 25 MiB so the RSAIRY firehose doesn't overflow the kernel UDP queue |
| **lidar_network** | Creates two NetworkManager connections on `enP8p1s0`: `rslidar` (static `192.168.1.102/24`, priority 10) and `dev` (DHCP, priority 0). Installs `/etc/NetworkManager/dispatcher.d/90-rslidar` — ARPs the LiDAR at `192.168.1.200` for up to 8 s on link-up; falls back to DHCP if no response |
| **systemd** | Copies and enables all systemd services (see [Boot sequence](#boot-sequence) below) |
| **ros_workspace** | Installs Python deps (pyudev, pyserial, empy); runs `rosdep install`; builds `local_ws` with colcon |
| **docker** | Each sub-step is independently checked: installs Docker engine if missing, enables the service if not running, configures the NVIDIA container runtime if not registered, adds the user to the `docker` group if not present, installs `docker-buildx-plugin` if missing |

---

## Boot sequence

Auto-started services after boot:

| Service | What it does |
|---|---|
| **usbfs-memory.service** | One-shot: raises `usbcore.usbfs_memory_mb` to 1000 (default 16) so high-bandwidth USB cameras (RealSense multi-stream) don't hit "Out of frame resources!" / watchdog timeouts. Runs before the Docker/camera services |
| **jetson-clocks.service** | Locks CPU/GPU clocks to maximum frequency (supplements nvpmodel) |
| **start_isaac_docker.service** | Pulls and starts the Isaac ROS Docker container (`isaac_ros_dev-aarch64-container`) so it's ready before any ROS nodes launch |
| **arid_description.service** | Launches `robot_state_publisher` for the ARID xacro — publishes `/robot_description` (latched) and `/tf_static`. The VSLAM stack waits for this before starting |
| **gst_camera_manager.service** | Starts the GStreamer camera manager — the `cam_down` pipeline is loaded but **idle**; activate via a SetBool service call (see [Cameras](#cameras-csi)) |
| **rslidar_coordinator.service** | Starts the RoboSense RSAIRY supervisor — manages the `rslidar_sdk_node` subprocess. Coordinator is up at boot; LiDAR is **idle** until SetBool. See [LiDAR](#lidar-robosense-rsairy) |
| **usb_ros_reset.service** | Hosts the `/reset_usb` ROS 2 service (Trigger) — performs a hardware USB reset on the ARK PAB carrier on demand |

### What to expect at login

1. Docker container running — `docker ps` shows `isaac_ros_dev-aarch64-container`.
2. ARID TF tree available — `ros2 topic echo /tf_static --once --qos-durability transient_local` returns immediately.
3. Camera manager idle — the `cam_down` service exists but no frames flowing yet.
4. LiDAR coordinator idle — `/rslidar_coordinator/enable` service exists; no cloud flowing yet.
5. Local workspace already sourced in `.bashrc`; ROS 2 packages in `local_ws` are immediately available.

---

## Cameras (CSI)

One CSI camera pipeline defined in [`local_ws/src/ros_gst_cameras/gst_camera_manager/config/pipelines.yaml`](local_ws/src/ros_gst_cameras/gst_camera_manager/config/pipelines.yaml). Uses an IMX219 sensor @ 1920×1080 mono via `nvarguscamerasrc` → `nvvidconv` → `appsink`.

| Pipeline | Sensor ID | Frame ID | Topic root |
|---|---|---|---|
| `cam_down`  | `sensor-id=0` | `bottom_visual_link` | `/cam_down` |

**Start / stop / status:**

```bash
# Start
ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool '{data: true}'

# Stop
ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool '{data: false}'

# Status (per-pipeline + global)
ros2 service call /gst_camera_manager/cam_down/status  std_srvs/srv/Trigger '{}'
ros2 service call /gst_camera_manager/status_all       std_srvs/srv/Trigger '{}'
ros2 service call /gst_camera_manager/stop_all         std_srvs/srv/Trigger '{}'
```

Each pipeline publishes `/<topic>/image_raw`, `/<topic>/image_raw/compressed`, and `/<topic>/camera_info`. Liveness is exposed on `/gst_camera_manager/<name>/alive` (latched `Bool`). For deeper details (encoding auto-detect, default camera_info, QoS, alive_threshold, troubleshooting): see [`ros_gst_cameras/README.md`](local_ws/src/ros_gst_cameras/README.md).

---

## LiDAR (RoboSense RSAIRY)

A RoboSense RSAIRY 3-D LiDAR is supervised by [`local_ws/src/rslidar_coordinator`](local_ws/src/rslidar_coordinator/), which spawns `rslidar_sdk_node` as a managed subprocess. The coordinator is up at boot via `rslidar_coordinator.service`; the LiDAR pipeline itself is **idle** until you call SetBool, matching the `gst_camera_manager` ergonomic.

The SDK reads our config at [`rslidar_coordinator/config/rslidar.yaml`](local_ws/src/rslidar_coordinator/config/rslidar.yaml) — the `rslidar_sdk` submodule is never patched. Cloud-only at the moment (IMU parsing is disabled because the SDK's IMU parser is gated by a compile-time flag we deliberately don't flip).

### Hardware + network

| Setting | Value |
|---|---|
| LiDAR IP | `192.168.1.200` (RoboSense factory default) |
| Jetson IP on LiDAR subnet | `192.168.1.102/24` |
| Jetson NIC | `enP8p1s0` |
| MSOP (point-cloud) port | UDP `6699` |
| DIFOP (device info) port | UDP `7788` |

Setup.sh's `lidar_network` step configures NetworkManager with two profiles on `enP8p1s0` — `rslidar` (static, priority 10) and `dev` (DHCP, priority 0). On link-up an NM dispatcher script probes the LiDAR via ARP for up to 8 s; if it responds, we stay on the static profile; if not, we fall back to DHCP. Result: plug into the LiDAR → ~1 s static. Plug into a router → ~8 s wait, then DHCP. Auto-swap on cable change.

### Start / stop / status

```bash
# Start the LiDAR pipeline (spawns rslidar_sdk_node)
rslidar_start
# or:
ros2 service call /rslidar_coordinator/enable std_srvs/srv/SetBool '{data: true}'

# Stop
rslidar_stop

# Status (RUNNING (pid=N) / STOPPED)
rslidar_status

# Read latched liveness Bool
rslidar_alive

# One-shot kick (stop + start)
rslidar_restart
```

### Topics + frame

| Topic | Type | Notes |
|---|---|---|
| `/rslidar_points` | `sensor_msgs/PointCloud2` | Point cloud, frame `rslidar_link`, BEST_EFFORT QoS |
| `/rslidar_coordinator/alive` | `std_msgs/Bool` (latched, TRANSIENT_LOCAL) | Liveness from frame-flow watchdog |
| `/rslidar_coordinator/enable` | `std_srvs/SetBool` | Start (`true`) / stop (`false`) the subprocess |
| `/rslidar_coordinator/status` | `std_srvs/Trigger` | Returns `RUNNING (pid=N)` or `STOPPED` |
| `/rslidar_coordinator/restart` | `std_srvs/Trigger` | Kill + respawn |

The cloud is stamped in `rslidar_link`, which is fixed to `base_link` by the joint defined in [`arid.xacro`](local_ws/src/arid_description/xacro/arid.xacro) at `(0.062732, 0, 0.16457)` with `pitch=0.485201` rad (≈27.8° forward tilt).

### Watchdog

The coordinator subscribes to `/rslidar_points` and ticks at 2 Hz. If frames stop arriving for more than `alive_threshold` seconds (default `5.0`), it flips `/rslidar_coordinator/alive` to `false`. Subprocess death is also detected and logged via `journalctl -u rslidar_coordinator -f`. **No auto-restart** — recovery requires an explicit `rslidar_start` / `rslidar_restart`.

---

## Visual-Inertial SLAM (RealSense + Isaac ROS VSLAM)

One front-mounted RealSense camera (D43X-series) feeding the Isaac ROS VSLAM node, plus a PX4 bridge. Camera serial, resolution, and frame ID are configured in [`isaac_ros-dev/src/px4_vslam/config/vslam_config.yaml`](isaac_ros-dev/src/px4_vslam/config/vslam_config.yaml).

```bash
# Inside the Isaac ROS container (use `start_isaac` then `isaac_bash`)
ros2 launch px4_vslam vslam.launch.py
# or just:
vslam
```

The launch blocks until `/robot_description` is on the graph (i.e. `arid_description.service` is up). It then starts:
- 1 × RealSense driver (`front_realsense`)
- Isaac ROS Visual SLAM node (2-camera stereo on the front IR pair)
- `vio_transform` — bridges VSLAM odometry into PX4 via uXRCE-DDS
- `vslam_reactor_node` — supervises VSLAM, gates jumps, retries SetSlamPose on misalignment

Tunables for the reactor live in [`px4_vslam_reactor/config/reactor_conf.yaml`](isaac_ros-dev/src/px4_vslam_reactor/config/reactor_conf.yaml). See [`px4_vslam/README.md`](isaac_ros-dev/src/px4_vslam/README.md) and [`px4_vslam_reactor/README.md`](isaac_ros-dev/src/px4_vslam_reactor/README.md) for full topic / service / config reference.

**Before flying:** edit `vslam_config.yaml` to match your specific RealSense serial number and the actual mount frame in [`arid_description/xacro/arid.xacro`](local_ws/src/arid_description/xacro/arid.xacro).

---

## PX4 firmware

Custom PX4 fork at [`local_ws/auxiliary/PX4-Autopilot/`](local_ws/auxiliary/PX4-Autopilot/) (submodule, branch `PX4-InDro`). Includes the **ARID Quad V1.1** airframe (`4025_arid_quad_v1_1`) and Jetson-friendly install patches.

### Build

```bash
cd local_ws/auxiliary/PX4-Autopilot
make ark_fmu-v6x_default                # build for ARK FMU v6X
make ark_fmu-v6x_default upload         # flash over USB (FMU in bootloader mode)
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

Xacro description + meshes for ARID. Auto-launched on boot by `arid_description.service`, which runs `display.launch.py` with `robot_state_publisher`. Frames published include `base_link`, `autopilot`, four propellers, the `front_realsense_link`, the `rslidar_link` (RoboSense RSAIRY mount), the `bottom_visual_link` CV-camera frame, `flow_link`, and `rangefinder_link`. The xacro source lives at [`local_ws/src/arid_description/xacro/arid.xacro`](local_ws/src/arid_description/xacro/arid.xacro).

For RViz / Foxglove visualization with this xacro, see [`arid_description/README.md`](local_ws/src/arid_description/README.md).

---

## Foxglove visualization

`foxglove_bridge` is installed from apt on both the host (`ros-humble-foxglove-bridge`, set up by `setup.sh`'s `apt` step) and inside the Isaac container (same Debian, installed by the Dockerfile chain). No source build, no submodule.

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
# or (inside the container) just:
foxglove_bridge
```

Then connect Foxglove Studio to `ws://<jetson-ip>:8765`.

---

## USB reset

The ARK PAB carrier's USB hub can be hardware-reset on demand via the `/reset_usb` service:

```bash
ros2 service call /reset_usb std_srvs/srv/Trigger '{}'
# or just:
reset_usb
```

The service node (`usb_ros_reset.service`) calls `systemctl start reset_usb.service`, which invokes `scripts/usb_reset.sh` (uhubctl). For details: [`reset_ark_usb/README.md`](local_ws/src/reset_ark_usb/README.md).

---

## Camera calibration

A wrapper around `ros2 camera_calibration` is at [`local_ws/auxiliary/camera_calibration/`](local_ws/auxiliary/camera_calibration/) — auto-detects display, bootstraps a numpy<2 venv (so `cv_bridge` works), interactively picks the topic, saves output as `<topic_slug>_calibration.yaml` next to the script. Built to work over a NoMachine remote display.

```bash
./local_ws/auxiliary/camera_calibration/camera_calibrate.sh
```

---

## Useful aliases

**Host (set by setup.sh):**

```bash
run_isaac        # Launch Isaac ROS Docker interactively
build_isaac      # Rebuild the Isaac ROS Docker image
start_isaac      # Start the container in background
stop_isaac       # Stop the container
isaac_bash       # Open a shell inside the running container
reset_usb        # Manually trigger USB hub reset
colcon_local     # Build local_ws and source it
clean_local      # Clean local_ws build/install/log
rosdep_local     # Install rosdep deps for local_ws
foxglove_bridge  # Launch the Foxglove WebSocket bridge on port 8765

# CSI camera (cam_down — gst_camera_manager)
cam_down_start   # SetBool(true)  on /gst_camera_manager/cam_down
cam_down_stop    # SetBool(false) on /gst_camera_manager/cam_down
cam_down_status  # Trigger /gst_camera_manager/cam_down/status
cam_down_alive   # Read the latched /gst_camera_manager/cam_down/alive Bool (TRANSIENT_LOCAL)

# RSAIRY LiDAR (rslidar_coordinator)
rslidar_start    # SetBool(true)  on /rslidar_coordinator/enable
rslidar_stop     # SetBool(false) on /rslidar_coordinator/enable
rslidar_status   # Trigger /rslidar_coordinator/status
rslidar_alive    # Read the latched /rslidar_coordinator/alive Bool (TRANSIENT_LOCAL)
rslidar_restart  # Trigger /rslidar_coordinator/restart (one-shot kick)
```

**Inside the container (set by `arid_env.sh`):**

```bash
reset_usb        # Trigger USB reset via ROS service
vslam            # Launch VSLAM (px4_vslam)
foxglove_bridge  # Start Foxglove bridge on port 8765
rosdep_isaac     # Install rosdep deps for isaac_ros_dev
colcon_isaac     # Build isaac_ros_dev workspace
clean_isaac      # Clean isaac_ros_dev build/install/log
```

---

## Package reference

### Custom packages (in this repo)

| Package | Purpose |
|---|---|
| [`local_ws/src/arid_description`](local_ws/src/arid_description/) | Xacro, meshes, RViz config; auto-launched on boot |
| [`local_ws/src/ros_gst_cameras`](local_ws/src/ros_gst_cameras/) | ROS 2 GStreamer-based camera stack (`gst_cam_node` + `gst_camera_manager`) |
| [`local_ws/src/rslidar_coordinator`](local_ws/src/rslidar_coordinator/) | Supervisor for the RoboSense RSAIRY LiDAR — owns the SDK config, exposes SetBool/Trigger services, runs a frame-flow watchdog |
| [`local_ws/src/reset_ark_usb`](local_ws/src/reset_ark_usb/) | ROS 2 service wrapping the systemd USB-reset unit |
| [`local_ws/auxiliary/camera_calibration`](local_ws/auxiliary/camera_calibration/) | Camera-calibration launcher (NoMachine-friendly) |
| [`isaac_ros-dev/src/px4_vslam`](isaac_ros-dev/src/px4_vslam/) | RealSense + Isaac VSLAM launch package + PX4 bridge |
| [`isaac_ros-dev/src/px4_vslam_reactor`](isaac_ros-dev/src/px4_vslam_reactor/) | VSLAM ↔ PX4 supervisor with YAML-tunable thresholds |

### Submodules (external upstreams)

| Submodule | Role |
|---|---|
| [`local_ws/auxiliary/PX4-Autopilot`](local_ws/auxiliary/PX4-Autopilot/) | PX4 flight-stack fork (`indro-robotics/PX4-Autopilot @ PX4-InDro`) — adds the ARID Quad V1.1 airframe + Jetson install patches |
| [`local_ws/src/px4_msgs`](local_ws/src/px4_msgs/) | Upstream PX4 message definitions (`PX4/px4_msgs @ release/1.15`) — required by the host-side `usb_ros_reset`-style nodes that touch PX4 telemetry |
| [`local_ws/src/rslidar_sdk`](local_ws/src/rslidar_sdk/) | RoboSense LiDAR SDK pinned at tag `v1.5.19` (`RoboSense-LiDAR/rslidar_sdk`). Builds `rslidar_sdk_node`, the driver spawned by `rslidar_coordinator`. Nested submodule `rs_driver` is auto-initialized. |
| [`local_ws/src/rslidar_msg`](local_ws/src/rslidar_msg/) | RoboSense LiDAR ROS-message definitions pinned at tag `v1.5.10` (`RoboSense-LiDAR/rslidar_msg`). Pure-message package consumed by `rslidar_sdk` and any LiDAR consumer that touches raw packet types. |
| [`isaac_ros-dev/src/isaac_ros_common`](isaac_ros-dev/src/isaac_ros_common/) | NVIDIA Isaac ROS shared base (Dockerfile chain, common message types, build helpers); patched by `setup.sh` to inject `Dockerfile.arid` and `arid_env.sh` |
| [`isaac_ros-dev/src/isaac_ros_nitros`](isaac_ros-dev/src/isaac_ros_nitros/) | NVIDIA's NITROS framework — zero-copy intra-process tensor / image transport that the VSLAM and image pipeline build on |
| [`isaac_ros-dev/src/isaac_ros_image_pipeline`](isaac_ros-dev/src/isaac_ros_image_pipeline/) | Isaac ROS GPU image-processing nodes (`RectifyNode`, `ImageFormatConverterNode`, etc.) — used by VSLAM and consumable by other CV pipelines |
| [`isaac_ros-dev/src/isaac_ros_visual_slam`](isaac_ros-dev/src/isaac_ros_visual_slam/) | NVIDIA's GPU-accelerated visual SLAM node + its message/service interfaces — the VSLAM backend launched by `px4_vslam` |
| [`isaac_ros-dev/src/realsense-ros`](isaac_ros-dev/src/realsense-ros/) | Intel RealSense ROS 2 driver pinned at `4.51.1` — drives the front RealSense camera feeding VSLAM |
| [`isaac_ros-dev/src/px4-ros2-interface-lib`](isaac_ros-dev/src/px4-ros2-interface-lib/) | Auterion's `px4_ros2_cpp` library at `1.4.0` — high-level mode/control SDK for writing custom flight modes against PX4 |
| [`isaac_ros-dev/src/px4_msgs`](isaac_ros-dev/src/px4_msgs/) | Same upstream PX4 messages, mirrored inside the Isaac ROS workspace so container-side packages can build against them |
