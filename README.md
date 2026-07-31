# ARID

The ARID (Autonomous Research Indoor Drone) is an indoor quadrotor built on an NVIDIA Jetson Orin with an ARK PAB carrier, running ROS 2 Humble, NVIDIA Isaac ROS and PX4. This repository provisions the drone, builds both workspaces, and operates the flight stack. ROS 2 traffic is restricted to the drone (`ROS_DOMAIN_ID=23`, `ROS_LOCALHOST_ONLY=1`), and remote visualization goes through Foxglove.

- `setup.sh` with `setup/`: provisioning, from a fresh Ubuntu 22.04 install to flight-ready.
- `arid_description`: robot description, publishing `/robot_description` and `/tf_static`.
- `ros_gst_cameras`: the two IMX219 CSI video pipelines.
- `px4_vslam`, `px4_vslam_reactor`, `arid_supervisor`: RealSense visual odometry into PX4.
- `reset_ark_usb`: the `/reset_usb` service.
- `PX4-Autopilot`: the InDro PX4 fork carrying the ARID airframe.

---

## Sensors

The airframe carries three sensor groups.

| Sensor | Fit |
|---|---|
| 3x RealSense D435 | front, left, right; IR stereo feeding VSLAM |
| 2x IMX219 CSI | `cam_front` forward, `cam_down` downward; operator video only |
| ARK optical flow + rangefinder | bottom pod |

---

## After boot

A login shell has `local_ws` sourced and the alias set installed. The units below are already running, both CSI pipelines are idle, and VSLAM stays idle until `initialize`.

| Unit | Function |
|---|---|
| `usbfs-memory` | usbfs buffer at 1000 MB for the RealSense streams. |
| `jetson-clocks` | Jetson clocks and fan at maximum. |
| `start_isaac_docker` | Isaac container. |
| `arid_description` | `robot_state_publisher`, the TF tree. |
| `gst_camera_manager` | CSI camera manager. |
| `usb_ros_reset` | Hosts `/reset_usb`. |
| `arid_supervisor` | VSLAM lifecycle service, inside the container. |

Stopping `arid_supervisor.service` runs `ExecStopPost` on every stop path, crash included, and is airborne-gated: `airborne_check.sh` decides whether flight is proven, and `reap_stack.sh` reaps the orphaned launch tree when it is not. `deinitialize` requires a fresh landed sample and is refused otherwise. Details: [`arid_supervisor/README.md`](isaac_ros-dev/src/arid_supervisor/README.md).

On a drone whose container workspace has never been built, the supervisor cannot start. `setup.sh --full` covers this; otherwise run menu **9**, or build and start it by hand:

```bash
start_isaac
isaac_bash
```

Inside the container, build and leave:

```bash
colcon_isaac
exit
```

Back on the host, clear the failure count and start the unit:

```bash
sudo systemctl reset-failed arid_supervisor.service && sudo systemctl start arid_supervisor.service
```

---

## Aliases

Every alias below is available in a host shell, and `help` prints the same set with descriptions.

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

Inside the container the set is smaller: `vslam` (direct launch), `initialize`, `deinitialize`, `status`, `reset_usb`, `colcon_isaac`, `clean_isaac`, `rosdep_isaac`, `foxglove_bridge` and `help`.

---

## CSI video

Two IMX219 pipelines defined in [`pipelines.yaml`](local_ws/src/ros_gst_cameras/gst_camera_manager/config/pipelines.yaml) run at 1920x1080 at 15 fps in GRAY8, unrotated and uncalibrated. They are operator video feeds and nothing in the container subscribes to them.

| Pipeline | Sensor | Frame | Topic root |
|---|---|---|---|
| `cam_front` | `sensor-id=0` | `top_visual_link` | `/cam_front` |
| `cam_down` | `sensor-id=1` | `bottom_visual_link` | `/cam_down` |

Start and stop each pipeline with its aliases, stop both with `cam_stop`, or call the manager's SetBool service directly.

```bash
cam_front_start
cam_front_stop
cam_front_status
cam_front_alive
cam_stop
ros2 service call /gst_camera_manager/cam_front std_srvs/srv/SetBool '{data: true}'
```

A running pipeline publishes `image_raw`, `image_raw/compressed` and `camera_info` under its topic root, and reports frame flow on the latched `/gst_camera_manager/<name>/alive`. The manager tees its log to `isaac_ros-dev/run_logs/gst_camera_manager/`. Pipeline fields and troubleshooting are in [`ros_gst_cameras/README.md`](local_ws/src/ros_gst_cameras/README.md).

---

## Visual odometry

Isaac cuVSLAM produces the visual odometry solution. The three RealSense cameras feed it as a six-stream stereo multicam, one IR pair per camera. The reactor filters and re-seats that solution, and `vio_transform` publishes it to PX4 on `/fmu/in/vehicle_visual_odometry`. Each camera streams `infra1` and `infra2` at 640x360x60 with colour, depth and IMU off, and VO continues on the remaining two stereo pairs through the loss of one whole camera.

> Never select a 90 fps RealSense profile. That USB service interval stalls the bus.

| Alias | Action |
|---|---|
| `initialize` | SetBool true on `/arid_supervisor/vslam_enable`. |
| `deinitialize` | SetBool false, refused unless landed. |
| `status` | VSLAM running state plus land state. |

The supervisor manages the VSLAM lifecycle behind a camera-proven bringup gate and a landed-state interlock. Bringup runs one `reset_usb` recovery cycle before reporting failure, so `initialize` returns in about 15 s on a healthy stack and takes up to about 3 minutes when a camera needs that cycle. Teardown requires a fresh `landed == True` sample. Each launch writes `run_logs/vslam/vslam.log`. See [`arid_supervisor/README.md`](isaac_ros-dev/src/arid_supervisor/README.md).

A direct launch bypasses the supervisor and is the development path. From inside the container, `vslam` blocks until `/robot_description` is up, then starts the RealSense drivers, the VSLAM node, `vio_transform` and `vslam_reactor`.

The reactor gates position jumps and velocity outliers, re-seats VSLAM against the PX4 solution. Only a committed origin seat bumps `/reactor/vio_reset_epoch`, which `vio_transform` forwards as `VehicleOdometry.reset_counter`; a jump re-seat writes the flight controller's own pose into cuVSLAM and sends no reset flag. An overall verdict is latched on `/reactor/vo_healthy`, which goes false on an exhausted re-seat budget or on EV publish silence. It is operator-facing only; no node subscribes to it. Tunables are in [`px4_vslam_reactor/README.md`](isaac_ros-dev/src/px4_vslam_reactor/README.md).

### RealSense serials

The RealSense serials live in the VSLAM config, which ships in two forms. `vslam_config.template.yaml` is tracked and holds the structure and tunables with empty `serial_no` fields; `vslam_config.yaml` is untracked and holds this drone's serials. `config_realsense` regenerates the live file from the template on every run and re-splices the existing serials, so a pulled template reaches a provisioned drone. A live config holding fewer than three serials is reseeded blank with a warning, and those serials are not recoverable from it.

Run `config_realsense` before flying so all three serials match the installed cameras. The full run shows a live feed per camera and pins each serial to its left, front or right mount; `config_realsense --reseed-only` refreshes from the template without touching hardware.

---

## USB reset

`reset_usb` power-cycles the ARK PAB USB hub with `uhubctl` and toggles the GPIO line that controls the standalone USB3 port. The `/reset_usb` service runs the same script through `reset_usb.service`, so either form works.

```bash
reset_usb
ros2 service call /reset_usb std_srvs/srv/Trigger '{}'
```

> Never reset USB in flight. The RealSense streams drop with the hub and visual odometry stops.

Details: [`reset_ark_usb/README.md`](local_ws/src/reset_ark_usb/README.md).

---

## Foxglove

The `foxglove_bridge` alias starts the bridge on port 8765, on the host or in the container. Connect Foxglove Studio to `ws://<device-ip>:8765`; the URDF is on `/robot_description` with panel frame `base_link`.

---

## Health checks

Two checks cover the host stack: `local_test` and `ver_cv_cams`.

`local_test` runs the smoke test.

```bash
local_test
```

It checks that the host services are active, the aliases resolve, port 8765 is open, both `cam_front` and `cam_down` complete a full lifecycle (frame ID, rate, `/alive`), the supervisor is on the graph when the container is up, and no subprocesses leak. It is safe to run at any time and leaves the pipelines stopped. The bare alias prints to the terminal; menu option **2** also tees to `log/smoke_test_log_*.log`.

`ver_cv_cams` opens the live CSI feeds in a window on the NoMachine desktop. Any key advances, `q` quits. Pass `front` or `down` for a single feed.

```bash
ver_cv_cams
ver_cv_cams front
```

---

## setup.sh

Running `./setup.sh` with no arguments opens the interactive menu. Every step detects what is already in place and does only what is missing, so re-running is safe. Each run logs to `log/setup_log_*.log`, and Ctrl+C offers quit-or-continue.

```bash
./setup.sh
```

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

Option **9** requires the container to be running.

### Camera calibration

`cam_calibrate` takes `front` or `down`; menu **6** prompts for the camera.

```bash
cam_calibrate front
```

The calibrator runs against the live pipeline and waits for a NoMachine session before starting. The default board is the included 10x7-square, 50 mm PDF. It writes `config/calibrations/<cam>.yaml` plus a timestamped copy; set `calibration: "cam_front"` or `"cam_down"` in `pipelines.yaml` and restart that pipeline to load the new intrinsics. Details: [`camera_calibration/README.md`](local_ws/auxiliary/camera_calibration/README.md).

### Camera focus

Menu **7** starts the selected CSI pipelines and the Foxglove bridge, then confirms each stream is sustained. Watch `/cam_front/image_raw/compressed` and `/cam_down/image_raw/compressed` in Foxglove Studio while adjusting the lens; any key stops them.

---

## Full setup

`./setup.sh --full` walks the questionnaire once, then provisions everything unattended. Reboots are automatic and happen at most twice: once after the install steps if ARK-OS, ROS 2 or JetPack was installed, and once before the smoke test if anything was built and that reboot was not declined. After each one, open a bash terminal and answer the prompt to resume. `--resume` is the same continuation invoked manually, and `--continue` re-enters the tail with finished steps skipped.

```bash
./setup.sh --full
./setup.sh --resume
./setup.sh --continue
```

### Steps

The first steps run once per invocation. Phase A (install and host configuration) is checkpointed to `~/.arid_progress`, so a resume or a re-run after a failure skips what already finished. Phase B is the builds and the live camera steps.

| Step | Does |
|---|---|
| **preflight** | Not root; git present; submodules initialized and non-empty. |
| **collect_answers** | Questionnaire, persisted for the resume. |
| **power** | nvpmodel maximum; apt-holds critical L4T packages. |
| **first_boot** | One-time hostname and password. |
| **ensure_wifi** | Joins the network from the questionnaire. |
| **nomachine** | Installs or upgrades the arm64 package. |
| **enable_user_linger** | Creates `/run/user/<uid>` at boot for headless NoMachine. |
| **clean_nvidia_desktop** | Removes NVIDIA first-boot icons and the L4T-README automount. |
| **disable_updates** | Turns off unattended-upgrades and the apt timers. |
| **ark_os** | Clones ARK-OS, then runs its `install.sh` and `install_ros2.sh` unattended from a generated `user.env`. Installs JetPack when absent, reinstalls it on request. A failed install prompts retry, skip or exit. |
| *Phase A, checkpointed* | |
| **repos** | ROS, NVIDIA and Docker apt repos; CDI configuration. |
| **apt** | Apt packages: chrony, Foxglove bridge, OpenCV, camera calibration, net tools. |
| **clock_sync** | chrony on, `systemd-timesyncd` off. |
| **zerotier** | Installs the daemon and joins. `ACCESS_DENIED` means authorize later. |
| **px4_deps** | PX4 Python dependencies; ARM toolchain on demand. |
| **git** | Credential cache, script permissions, submodule pin verify, `run_logs` symlink. |
| **docker_patches** | Injects `Dockerfile.arid`, `arid_env.sh` and `run_dev.sh` into `isaac_ros_common`. |
| **skip_worktree** | Hides per-drone camera calibrations from git status. |
| **bashrc** | Rewrites the ARID block: exports, aliases, `help`, resume hook. |
| **permissions** | Sudoers, udev, polkit, groups. |
| **uhubctl** | Builds from source if missing. |
| **ros_workspace** | Python dependencies, rosdep, colcon build of `local_ws`. |
| **docker** | Engine, NVIDIA runtime, docker group, buildx. |
| **systemd** | Installs every unit the repo ships and enables all but `reset_usb.service`. |
| **realsense** | Reseeds `vslam_config.yaml`, then assigns the three serials. |
| *Phase B* | |
| **build_isaac** | Container image build, queued across the reboot. |
| **colcon_isaac** | Builds the container workspace and restarts `arid_supervisor.service`. |
| **verify_cameras** | Live front and down feeds, if opted in. |
| **camera_focus** | Foxglove focus pass over both CSI cameras, if opted in. |
| **summary, reboot, smoke test** | Summary, one reboot if anything built, then `local_test`. |

Calibration is not part of the run: calibrate both CSI cameras once setup finishes.

---

## PX4 firmware

The PX4 fork is at [`local_ws/auxiliary/PX4-Autopilot/`](local_ws/auxiliary/PX4-Autopilot/) on branch `PX4-InDro` and carries the `4026_arid_quad_v1_2` airframe. A built image is kept at [`px4_compiled/arid.px4`](local_ws/auxiliary/px4_compiled/), with the raw `arid.bin` alongside it.

Building produces `build/ark_fmu-v6x_default/ark_fmu-v6x_default.px4`:

```bash
cd local_ws/auxiliary/PX4-Autopilot
make ark_fmu-v6x_default
```

Flashing is done from the ARK-OS web interface, which takes the `.px4` file directly.

After flashing, select the airframe from a MAVLink shell:

```
param set SYS_AUTOSTART 4026
param save
reboot
```

---

## Robot description

`arid_description.service` runs `robot_state_publisher` on [`urdf/arid.xacro`](local_ws/src/arid_description/urdf/arid.xacro). The tree carries `base_link`, `base_footprint`, `autopilot`, four propellers, `front_realsense_link`, `left_realsense_link`, `right_realsense_link`, `top_visual_link` for the front CSI camera, `bottom_visual_link` for the down CSI camera, `flow_link` and `rangefinder_link`. Visualization is covered in [`arid_description/README.md`](local_ws/src/arid_description/README.md).

---

## Development

VSCode over Remote-SSH is the editor this repository is set up for: all code, builds and the live runtime stay on the drone.

```bash
ssh jetson@<device-ip>
```

`.vscode/` carries the recommended extension list, the Python and C++ lint and IntelliSense settings, and the search excludes. On first open VSCode offers to install those extensions on the drone.

For a virtual desktop, connect a NoMachine session to `<device-ip>`. The camera feed windows and the calibration GUI render on that desktop, so keep a session attached while using them.

The host workspace bind-mounts into the container at `/workspaces/isaac_ros-dev`. Build it with `colcon_isaac` from either side and enter it with `start_isaac` followed by `isaac_bash`. `local_ws` builds on the host with `colcon_local`. For in-container IntelliSense, attach VSCode to `isaac_ros_dev-aarch64-container` with Dev Containers.

### VSCode Remote-SSH offline fix

Remote-SSH fails offline with `Failed to download VS Code Server` when the laptop's VSCode commit has no matching server staged on the drone. Freeze the laptop's commit once, then restart VSCode. These are client-scope settings and cannot live in `.vscode/`.

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

Both back up to `settings.json.bak`. If the parse fails on `//` comments, restore the backup and add the three keys by hand.

### Submodules

[`update_submods.sh`](scripts/update_submods.sh) enforces two classes. LIVE entries are branch-tracked, so HEAD must sit on that remote branch, at or behind the tip. PINNED entries must sit on an exact tag and are re-checked out if they drift. `update_submods.sh --update` advances both.

| Submodule | Class | Role |
|---|---|---|
| [`PX4-Autopilot`](https://github.com/indro-robotics/PX4-Autopilot/tree/PX4-InDro) | LIVE `PX4-InDro` | PX4 fork with the ARID airframe. |
| [`realsense-ros`](https://github.com/indro-robotics/realsense-ros/tree/v4.51.1) | LIVE `v4.51.1` | RealSense driver: `hw_reset` service, hot-removal guards, `color_format`, noise log filter, claim retry. |
| [`isaac_ros_visual_slam`](https://github.com/indro-robotics/isaac_ros_visual_slam/tree/v3.2-14) | LIVE `v3.2-14` | cuVSLAM backend with the stale-tolerant image synchronizer. |
| [`px4_msgs`](https://github.com/PX4/px4_msgs/tree/release/1.15) | LIVE `release/1.15` | PX4 messages; `local_ws/src/px4_msgs` symlinks to it. |
| [`isaac_ros_common`](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/tree/v3.2-14) | PINNED `v3.2-14` | Isaac base image and Dockerfile chain. |
| [`isaac_ros_nitros`](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros/tree/v3.2-14) | PINNED `v3.2-14` | Zero-copy transport. |
| [`isaac_ros_image_pipeline`](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline/tree/v3.2-14) | PINNED `v3.2-14` | GPU image processing. |
| [`px4-ros2-interface-lib`](https://github.com/Auterion/px4-ros2-interface-lib/tree/1.4.0) | PINNED `1.4.0` | Auterion PX4 SDK. |

---

## Package reference

The two workspaces carry these first-party components.

| Package | Purpose |
|---|---|
| [`arid_description`](local_ws/src/arid_description/) | Xacro, meshes, RViz config. |
| [`ros_gst_cameras`](local_ws/src/ros_gst_cameras/) | CSI camera stack: `gst_cam_node` and `gst_camera_manager`. |
| [`reset_ark_usb`](local_ws/src/reset_ark_usb/) | The `/reset_usb` service. |
| [`camera_calibration`](local_ws/auxiliary/camera_calibration/) | `cam_front` and `cam_down` calibrator and pattern. |
| [`px4_vslam`](isaac_ros-dev/src/px4_vslam/) | Three-camera VSLAM launch and PX4 bridge. |
| [`px4_vslam_reactor`](isaac_ros-dev/src/px4_vslam_reactor/) | VSLAM jump gating and re-seat. |
| [`arid_supervisor`](isaac_ros-dev/src/arid_supervisor/) | VSLAM lifecycle service. |
