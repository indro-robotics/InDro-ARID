# Cypher Drone Workspace

NVIDIA Jetson Orin · ROS2 Humble · Isaac ROS · PX4

---

## setup.sh

```
./setup.sh [--fresh | --patch]
```

- **`--fresh`** — first-time installation on a new system
- **`--patch`** — re-run after a `git pull` to pick up config/service changes
- No flag: auto-selects `patch` if the sentinel `/etc/cypher_first_setup_done` exists, otherwise prompts

All output is logged to `log/setup_log_<timestamp>.log`.

### Steps (in order)

| Step | What it does | Mode |
|---|---|---|
| **power** | Sets nvpmodel to max power (mode 0); holds critical L4T/kernel packages from apt upgrades | both |
| **repos** | Adds ROS, Nvidia Jetson, and Docker APT repos; regenerates NVIDIA CDI config | both |
| **apt** | Installs ROS packages, libusb, camera-info-manager, compressed-image-transport, etc. | both |
| **px4_deps** | Runs PX4 `ubuntu.sh` dependency installer (interactive prompt) | fresh only |
| **git** | Sets git credential cache; fixes script permissions; runs `update_isaac_submods.sh` | both |
| **docker_patches** | Copies patched Dockerfiles and scripts into the `isaac_ros_common` submodule; marks them skip-worktree so git ignores local changes | both |
| **skip_worktree** | Finds all `config/`, `cfg/`, and `camera_calibrations/` dirs under `src/` and marks their tracked files skip-worktree (protects calibrations from being overwritten by git) | both |
| **bashrc** | Rewrites the `# BEGIN CYPHER SETUP … # END CYPHER SETUP` block: sets `ROS_DOMAIN_ID=23`, exports workspace paths, sources `local_ws/install/setup.bash`, adds aliases (`run_isaac`, `build_isaac`, `colcon_local`, `reset_usb`, etc.) | both |
| **permissions** | Writes sudoers rule (uhubctl, gpioset, systemctl, usb_reset.sh without password); udev rules for USB hub and GPIO; polkit rule for `reset_usb.service`; adds user to `dialout` and `gpio` groups | both |
| **uhubctl** | Builds and installs `uhubctl` from source (skips if already installed) | both |
| **systemd** | Copies and enables all systemd services (see below) | both |
| **aravis** | Removes old apt aravis 0.8; builds aravis 0.10 from source for LUCID Phoenix GigE support; installs `aravissrc` GStreamer plugin | both |
| **gige_ethernet** | Interactive: selects ethernet interface for GigE camera; discovers camera IP via `arv-tool-0.10`; configures static NetworkManager connection with MTU 9000 (jumbo frames); tunes kernel receive buffers | both |
| **ros_workspace** | Installs Python deps (websockets, pyudev, pyserial, empy); runs `rosdep install`; builds `local_ws` with colcon | both |
| **docker** | Installs Docker engine + NVIDIA runtime (fresh); or ensures Docker is running (patch) | both |

After a fresh run, a sentinel is written to `/etc/cypher_first_setup_done` so future runs default to patch mode.

---

## Boot sequence (systemd services)

On every boot, the following services start automatically:

| Service | What it does |
|---|---|
| **jetson-clocks.service** | Locks CPU/GPU clocks to maximum frequency (supplements nvpmodel) |
| **start_isaac_docker.service** | Pulls and starts the Isaac ROS Docker container (`isaac_ros_dev-aarch64-container`) so it is ready before any ROS nodes launch |
| **rosbridge_websocket.service** | Starts `rosbridge_server` on port 9090 — provides WebSocket access to the ROS graph for Foxglove, web UIs, and ground station tools |
| **uwb_ros_node.service** | Starts the UWB ranging node (MAVLink bridge for UWB localization) |
| **gst_camera_manager.service** | Starts the GStreamer camera manager node — manages LUCID Phoenix 4K GigE camera pipeline via SetBool services; publishes latched `alive` topics |
| **usb_ros_reset.service** | Listens for USB hub state and resets ARK PAB USB if needed on startup |
| **reset_usb.service** | One-shot USB hub reset via uhubctl (triggered by usb_ros_reset or manually) |

### What to expect at login

1. Docker container is already running (`docker ps` should show `isaac_ros_dev-aarch64-container`)
2. `rosbridge_websocket` is live on port 9090
3. `gst_camera_manager` is running — LUCID camera pipeline can be started immediately via:
   ```
   ros2 service call /gst_camera_manager/phoenix_4k std_srvs/srv/SetBool '{data: true}'
   ```
4. Local workspace is already sourced in `.bashrc` — ROS2 packages in `local_ws` are available immediately

### Useful aliases

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
```

**Inside the container (set by cypher_env.sh):**

```bash
reset_usb        # Trigger USB reset via ROS service
vslam            # Launch VSLAM (px4_vslam)
state_machine    # Launch the PX4 state machine
foxglove_bridge  # Start Foxglove bridge on port 8765
rosdep_isaac     # Install rosdep deps for isaac_ros_dev
colcon_isaac     # Build isaac_ros_dev workspace
```

> The following require the state machine to be running (`state_machine` alias):

```bash
takeoff          # Command takeoff at 1.5m loiter altitude
land             # Command landing
cycle            # Start shelf scan cycle (3.5m shelf, 0.20 m/s, 1.0m distance, 45° AMR orientation)
halt             # Halt in midair
fland            # Force land immediately on-the-spot
fmu_reboot       # Reboot the FMU (PX4)
fkill            # Panic stop (kill motors)
```