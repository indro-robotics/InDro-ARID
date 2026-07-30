# InDro ARID Workspace

ARID is the **Autonomous Research Indoor Drone**: an NVIDIA Jetson Orin on an ARK PAB carrier,
running ROS 2 Humble, NVIDIA Isaac ROS and PX4. This repository provisions, builds and operates a
deployed airframe, and carries the flight-code workspaces, the container definition, the
provisioning script and the PX4 fork.

| Sensor | Fit |
|---|---|
| RealSense D435 | Front, IR stereo for VSLAM, `640x360x60` infra1 + infra2; colour, depth and IMU off. |
| RoboSense RSAIRY LiDAR | Ethernet on `enP8p1s0`, point cloud only. |
| IMX219 CSI camera | Downward (`cam_down`), 1920x1080 at 15 fps mono, unrotated and uncalibrated. |
| ARK optical flow | Bottom pod, with rangefinder. |

The system breaks down into these functional blocks.

- **Bootstrap** (`setup.sh`): fresh Ubuntu 22.04 to flight-ready; safe to re-run at any point.
- **Robot description** (`arid_description`): xacro and meshes, publishing `/robot_description` and `/tf_static`.
- **CSI camera** (`ros_gst_cameras`): the `cam_down` GStreamer pipeline with a SetBool service and a frame-flow watchdog.
- **LiDAR** (`rslidar_coordinator`): RSAIRY supervisor with SetBool start/stop, latched `/alive`, and an auto-configured Ethernet link.
- **Visual odometry** (`px4_vslam`, `px4_vslam_reactor`, `vslam_sentry`, `arid_supervisor`): the RealSense feeds Isaac cuVSLAM, which is filtered and forwarded to PX4 as VIO.
- **USB recovery** (`reset_ark_usb`): the `/reset_usb` Trigger service.
- **PX4 firmware**: the InDro fork with the `4026_arid_quad_v1_2` airframe, plus a prebuilt binary.

ROS 2 traffic stays on the drone (`ROS_DOMAIN_ID=23`, `ROS_LOCALHOST_ONLY=1`), and remote
visualization goes through the Foxglove bridge.

---

## At login

On a booted drone the Isaac container is running, the TF tree is up, the camera and LiDAR are
idle, `/arid_supervisor/vslam_enable` is on the graph, and `local_ws` is sourced.

## Aliases

Every alias below is available in a host shell; `help` prints the same set with descriptions.

| Alias | Action |
|---|---|
| `setup` | Run setup.sh. |
| `run_isaac` / `start_isaac` / `stop_isaac` / `isaac_bash` | Container: run / start / stop / shell. |
| `build_isaac` | Rebuild the container image. |
| `colcon_isaac` | Deinitialize, build the container workspace, restart the supervisor. |
| `clean_isaac` / `rosdep_isaac` | Clean / rosdep the container workspace. |
| `colcon_local` | Build `local_ws`, stopping and restarting its host services. |
| `clean_local` / `rosdep_local` | Clean / rosdep `local_ws`. |
| `reset_usb` | USB hub reset. |
| `cam_down_start` / `cam_down_stop` / `cam_down_status` / `cam_down_alive` | Down-camera pipeline. |
| `cam_refresh` | Re-read `pipelines.yaml`, stopping pipelines first. |
| `cam_calibrate` | Calibrate `cam_down`. |
| `rslidar_start` / `rslidar_stop` / `rslidar_status` / `rslidar_alive` / `rslidar_restart` | LiDAR driver. |
| `lidar_diag` / `config_lidar` | LiDAR diagnostic / IP auto-detect. |
| `config_realsense` | Reseed `vslam_config.yaml` and assign the RealSense serial. |
| `local_test` / `ver_cv_cams` | Smoke test / live camera feed. |
| `wifi` / `zt_join` / `update_submods` | Wi-Fi / ZeroTier / submodule sync. |
| `foxglove_bridge` | Bridge on port 8765. |
| `initialize` / `deinitialize` / `status` | VSLAM through the supervisor. |
| `sentry` | `vslam_sentry` status JSON. |

Inside the container the set is smaller: `vslam` (direct launch), `initialize`, `deinitialize`,
`status`, `reset_usb`, `colcon_isaac`, `clean_isaac`, `rosdep_isaac`, `foxglove_bridge`, `help`.

## Down camera

The downward IMX219 runs the single `cam_down` pipeline defined in
[`pipelines.yaml`](local_ws/src/ros_gst_cameras/gst_camera_manager/config/pipelines.yaml): CSI
sensor 0, 1920x1080 at 15 fps, GRAY8, frame `bottom_visual_link`. Start and stop it with the
aliases or the underlying service call.

```bash
cam_down_start
cam_down_stop
cam_down_status
cam_down_alive
ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool '{data: true}'
```

A started pipeline publishes `/cam_down/image_raw`, `/cam_down/image_raw/compressed` and
`/cam_down/camera_info`, with liveness on the latched `/gst_camera_manager/cam_down/alive`. The
manager log is tee'd to `isaac_ros-dev/run_logs/gst_camera_manager/`. Pipeline fields and
troubleshooting are in [`ros_gst_cameras/README.md`](local_ws/src/ros_gst_cameras/README.md).

## LiDAR

[`rslidar_coordinator`](local_ws/src/rslidar_coordinator/) supervises the RSAIRY and runs the
RoboSense SDK as a managed subprocess. The coordinator comes up at boot with the LiDAR idle until
enabled, and publishes the point cloud only.

```bash
rslidar_start
rslidar_stop
rslidar_status
rslidar_alive
rslidar_restart
```

`rslidar_start` is equivalent to the service call:

```bash
ros2 service call /rslidar_coordinator/enable std_srvs/srv/SetBool '{data: true}'
```

| Topic / Service | Type | Notes |
|---|---|---|
| `/rslidar_points` | `sensor_msgs/PointCloud2` | Frame `rslidar_link`, BEST_EFFORT. |
| `/rslidar_coordinator/alive` | `std_msgs/Bool` (latched) | Frame-flow watchdog. |
| `/rslidar_coordinator/enable` | `std_srvs/SetBool` | Start / stop the subprocess. |
| `/rslidar_coordinator/status` | `std_srvs/Trigger` | `RUNNING (pid=N)` or `STOPPED`. |
| `/rslidar_coordinator/restart` | `std_srvs/Trigger` | Kill and respawn. |

The watchdog flips `/alive` false after 5 s without frames. There is no auto-restart, so recover
with `rslidar_start` or `rslidar_restart`. The coordinator's parameters and SDK config keys are in
[`rslidar_coordinator/README.md`](local_ws/src/rslidar_coordinator/README.md), and the network
layout is under the LiDAR diagnostic below.

## Visual odometry

The front RealSense IR pair feeds Isaac cuVSLAM, the reactor filters and re-seats that solution,
and `vio_transform` publishes it to PX4 on `/fmu/in/vehicle_visual_odometry`. Both infra streams
are required, and the loss of either stops VO.

| Alias | Action |
|---|---|
| `initialize` | SetBool true on `/arid_supervisor/vslam_enable`. |
| `deinitialize` | SetBool false, refused unless landed. |
| `status` | vslam running plus land state. |
| `sentry` | Per-camera and VO health JSON. |

The **supervisor** manages the VSLAM lifecycle behind a camera-proven bringup gate and a
landed-state interlock. Bringup runs one `reset_usb` recovery cycle before
reporting failure, and teardown requires a fresh `landed == True` sample. Each launch writes
`run_logs/vslam/vslam.log`. See
[`arid_supervisor/README.md`](isaac_ros-dev/src/arid_supervisor/README.md).

A direct launch bypasses the supervisor and is the development path. From inside the container,
`vslam` (or `ros2 launch px4_vslam vslam.launch.py`) blocks until `/robot_description` is up, then
starts the RealSense driver, the VSLAM node, `vio_transform`, `vslam_reactor` and `vslam_sentry`.

The **reactor** gates jumps and velocity outliers, re-seats VSLAM against the PX4 solution, and
withholds VO from EKF2 while cuVSLAM frame cadence is degraded. It latches gate state on
`/reactor/cadence_gated`, the engagement count on `/reactor/cadence_gate_count`, and an overall
verdict on `/reactor/vo_healthy`, which goes false on a re-seat burst or on EV publish silence.
All three are forensics topics with no consumer on this drone. Tunables are documented in
[`px4_vslam_reactor/README.md`](isaac_ros-dev/src/px4_vslam_reactor/README.md).

The **sentry** watches per-stream `camera_info` rate and VO cadence, classifies which layer broke,
and hardware-resets a wedged camera. It never restarts VSLAM and never touches fusion. Its log is
`run_logs/sentry/sentry.log`.

| Name | Type | Direction |
|---|---|---|
| `/vslam_sentry/status` | `std_msgs/String` (latched) | pub, status JSON every 15 s |
| `/vslam_sentry/healthy` | `std_msgs/Bool` (latched) | pub, true when settled and all states good |
| `/vslam_sentry/status_now` | `std_srvs/Trigger` | srv, same JSON on demand |
| `/vslam_sentry/reset_front` | `std_srvs/Trigger` | srv, manual reset |
| `/front_realsense/hw_reset` | `std_srvs/Trigger` | cli, driver reset |

### RealSense serial

The VSLAM config exists in two forms. `vslam_config.template.yaml` is tracked and holds the fleet
structure and tunables with an empty `serial_no`; `vslam_config.yaml` is untracked and holds this
drone's serial. `config_realsense` regenerates the live file from the template on every run and
re-splices the existing serial, so a pulled template reaches a provisioned drone. If the live
config has no serial to preserve, it is reseeded blank with a warning and the serial must be
reassigned.

Run `config_realsense` before flying so the serial matches the installed camera. The full run
probes USB and writes the detected serial; `config_realsense --reseed-only` refreshes from the
template without touching hardware.

## USB reset

The `reset_usb` alias power-cycles the ARK PAB USB hub with `uhubctl` and pulses the FMU reset
line with `gpioset`.

```bash
reset_usb
ros2 service call /reset_usb std_srvs/srv/Trigger '{}'
```

> `/reset_usb` resets the flight controller. Never call it in flight.

## Foxglove

Run the `foxglove_bridge` alias on the host or in the container to start the bridge on port 8765,
then connect Foxglove Studio to `ws://<device-ip>:8765`.

## Health checks

`local_test` runs the smoke test. It checks that the host services are active, the aliases
resolve, port 8765 is open, the full `cam_down` and `rslidar` lifecycles work (frame_id, rate,
`/alive`, restart PID), the supervisor is on the graph when the container is up, and no
subprocesses leak. It is safe to run at any time and leaves the pipelines stopped. The bare alias
prints to the terminal, and menu option **2** also tees to `log/smoke_test_log_*.log`.

`ver_cv_cams` opens the live `cam_down` stream in a window on the NoMachine desktop; `q` quits.

`sentry` prints the per-camera and VO health JSON.

---

## setup.sh menu

Running `./setup.sh` with no arguments opens the menu. Every step detects what is already in place
and does only what is missing, so re-running is safe. Each run logs to `log/setup_log_*.log`, and
Ctrl+C ends setup at any point.

```bash
./setup.sh
```

| Option | Action |
|---|---|
| **1** | Full setup |
| **2** | Smoke test |
| **3** | RealSense serial assignment |
| **4** | Camera feed check |
| **5** | LiDAR network diagnostic |
| **6** | LiDAR IP auto-detect |
| **7** | Wi-Fi connect |
| **8** | Camera calibration |
| **9** | Camera focus |
| **10** | Build the Isaac container image |
| **11** | Build the Isaac workspace and restart `arid_supervisor.service` |
| **12** | Build `local_ws` |
| **13** | ZeroTier join/switch |
| **14** | Uninstall, keeping the repo, OS and Docker engine |

Option **11** needs the container running.

### LiDAR network diagnostic

`lidar_diag` walks the LiDAR path end to end: link state, NetworkManager profile, IP and route, an
ARP probe at the detected address, then coordinator and SDK runtime plus cloud rate. A passive
`tcpdump` sniff and an `arp-scan` sweep are added when it is run as `sudo lidar_diag`.

| Setting | Value |
|---|---|
| Jetson NIC | `enP8p1s0` |
| LiDAR IP / Jetson IP | Auto-detected by `config_lidar` |
| Factory-default LiDAR IP | `192.168.1.200` |
| MSOP (point cloud) port | UDP `6699` |
| DIFOP (device info) port | UDP `7788` |
| IMU port (socket bound) | UDP `6688` |

On link-up a NetworkManager dispatcher ARP-probes the LiDAR for up to 8 s. A response keeps the
static `rslidar` profile; no response falls back to the DHCP `dev` profile, so the same NIC can be
swapped between LiDAR and router.

RoboSense LiDARs store their own address and their unicast target in firmware, and a prior RSView
session may have moved them off the factory defaults. `config_lidar` sniffs `enP8p1s0`, extracts
the LiDAR MAC and addresses, rewrites the `rslidar` profile and the dispatcher to match, and
verifies the result by ARP. Run it after a LiDAR swap or reconfiguration.

```bash
config_lidar
```

If the LiDAR is unreachable the static fallback (`192.168.1.102/24` to `192.168.1.200`) stays in
place; re-run once the LiDAR is connected and powered.

### Camera calibration

`cam_calibrate` runs a venv-isolated interactive calibrator against the live `cam_down` pipeline.

```bash
cam_calibrate
```

The default board is the included 10x7-square, 50 mm PDF. The calibrator writes
`config/calibrations/cam_down.yaml` plus a timestamped record; afterwards set
`calibration: "cam_down"` in `pipelines.yaml` so the pipeline loads the new intrinsics. The script
waits for a NoMachine session before starting. See
[`camera_calibration/README.md`](local_ws/auxiliary/camera_calibration/README.md).

### Camera focus

Menu option **9** starts `cam_down` and the Foxglove bridge so you can watch
`/cam_down/image_raw/compressed` in Studio while adjusting the lens. Pressing `q` tears both down.

---

## Full setup

`./setup.sh --full` walks the questionnaire once, then provisions everything unattended. When
anything built, setup reboots before the smoke test so the test validates a clean boot. To resume
after that reboot, open a bash terminal and answer the prompt. `--resume` is the same continuation
invoked manually, and `--continue` re-enters the tail with finished steps skipped.

```bash
./setup.sh --full
./setup.sh --resume
```

Phase A steps are checkpointed to `~/.arid_progress`, so a resume or a re-run after a failure skips
what already finished.

| Step | Does |
|---|---|
| **preflight** | Not root; git present; submodules initialized and non-empty. |
| **collect_answers** | Questionnaire, persisted for the resume. |
| **first_boot** | One-time hostname and password. |
| **power** | Sets nvpmodel to maximum; apt-holds critical L4T packages. |
| **disable_updates** | Disables unattended-upgrades and the apt timers. |
| **enable_user_linger** | Enables lingering so `/run/user/<uid>` exists at boot. |
| **clean_nvidia_desktop** | Removes NVIDIA first-boot icons and the L4T-README automount. |
| **ensure_wifi** | Joins the questionnaire's network. |
| **nomachine** | Detects the install; prints a manual hint if missing. |
| *Phase A, checkpointed* | |
| **repos** | ROS, NVIDIA and Docker apt repos, CDI config. |
| **apt** | Apt packages: chrony, Foxglove bridge, OpenCV, camera calibration, net tools. |
| **clock_sync** | Enables chrony and disables `systemd-timesyncd`, so no NTP step lands mid-mission. |
| **zerotier** | Installs the daemon and joins. `ACCESS_DENIED` means authorize later. |
| **px4_deps** | PX4 toolchain; pins `numpy<2`. |
| **git** | Credential cache, script permissions, submodule pin verification. |
| **docker_patches** | Injects the ARID Dockerfile, `arid_env.sh` and `run_dev.sh` into `isaac_ros_common`. |
| **skip_worktree** | Hides per-drone camera calibrations from git status. |
| **bashrc** | Rewrites the ARID block: env exports, the alias set, `help`, resume hook. |
| **permissions** | Sudoers, udev, polkit, groups. |
| **uhubctl** | Builds from source if missing. |
| **lidar_sysctl** | Kernel UDP receive buffers to 25 MiB. |
| **lidar_network** | NetworkManager profiles on `enP8p1s0` and the link-up dispatcher, then `config_lidar`. |
| **ros_workspace** | Python deps, rosdep, colcon build of `local_ws`. |
| **docker** | Engine, NVIDIA runtime, docker group, buildx. |
| **systemd** | Installs every unit the repo ships and enables all but `reset_usb.service`. |
| **realsense** | Reseeds `vslam_config.yaml`, then assigns the detected serial. |
| *Phase B* | |
| **build_isaac** | Container image build, queued across the reboot. |
| **colcon_isaac** | Builds the container workspace and restarts `arid_supervisor.service`. |
| **verify_cameras / camera_focus** | Live `cam_down` feed, then the Foxglove focus loop if opted in. |
| **reboot + smoke test** | Summary, reboot if anything built, then `local_test.sh` on the clean boot. |

Work through the validation sequence below once setup finishes, then calibrate and focus the down
camera as needed.

## Post-update validation

This is the pre-flight checklist after any pull, rebuild, or reprovision. Run it with props off,
disarmed, and the drone on the ground. A FAIL stops the sequence.

**1. Serials**

```bash
config_realsense
grep serial_no ~/workspaces/isaac_ros-dev/src/px4_vslam/config/vslam_config.yaml
```

Expect one non-empty serial. Blank means bringup cannot map the camera and the sentry idles with
no watchdog.

**2. Submodules and forks**

```bash
update_submods
```

Expect `All submodules verified.`. A `commit is NOT on origin/<branch>` message means a fork
drifted; run `scripts/update_submods.sh --update`. Upstream `realsense-ros` in place of the fork
removes `/front_realsense/hw_reset`, which is the sentry's only recovery path.

**3. Build**

```bash
colcon_local
colcon_isaac
```

`colcon_isaac` deinitializes first (refused unless landed), builds in-container, and restarts
`arid_supervisor.service`. A build failure leaves the supervisor on old code with the stack down;
fix it before continuing. VSLAM does not restart on its own.

**4. Clean boot**

```bash
sudo reboot
```

Then, back on the drone:

```bash
systemctl is-active start_isaac_docker arid_supervisor arid_description gst_camera_manager rslidar_coordinator usb_ros_reset
local_test
```

Expect `active` six times and an all-PASS `local_test`. A dead supervisor at this point is usually
an unbuilt container workspace.

**5. LiDAR**

```bash
lidar_diag
rslidar_start
rslidar_status
rslidar_alive
ros2 topic hz --qos-reliability best_effort /rslidar_points
rslidar_stop
```

Add `sudo` to `lidar_diag` for the tcpdump and arp-scan legs. Expect `RUNNING (pid=N)`, a latched
`data: true`, and a steady cloud rate. An `/alive` of false after 5 s of silence means the SDK is
up but no frames are arriving: re-run `config_lidar`, then `lidar_diag`.

**6. Bringup**

```bash
initialize
status
```

This blocks about 15 s when healthy and up to about 3 min on a double failure. Expect
`Response(success=True` then `vslam: running | land: landed`. A refusal carries verbatim driver
evidence: repeated `Error starting device` is camera or USB, and `no factory exists` is a failed
`image_transport` plugin load.

**7. Sentry**

```bash
sentry
ros2 topic echo --once --qos-durability transient_local /vslam_sentry/healthy
```

After the 40 s settle expect `"vslam":{"state":"OK"` with the camera `HEALTHY`, and `data: true`.
Stuck at `SETTLING` past a minute means the streams never reached 30 Hz. `NO CAMERAS parsed` in
`run_logs/sentry/sentry.log` means a blank serial, so go back to step 1. `ESCALATED` means the
camera exhausted its resets; treat it as hardware.

**8. Reactor telemetry**

```bash
ros2 topic echo --once --qos-durability transient_local /reactor/cadence_gated
ros2 topic echo --once --qos-durability transient_local /reactor/cadence_gate_count
ros2 topic echo --once --qos-durability transient_local /reactor/vo_healthy
```

Expect `data: false`, `data: 0` and `data: true` on a healthy idle stack. All three are latched, so
a late subscriber still reads the state. A non-zero gate count before flight means cadence
starvation already happened, and a false `vo_healthy` means a re-seat burst or an EV publish
silence; find either before flying.

**9. Airborne gate, exercised on the ground**

```bash
ros2 topic echo --once --qos-reliability best_effort --qos-durability volatile \
  /fmu/out/vehicle_land_detected px4_msgs/msg/VehicleLandDetected | grep '^landed'
docker exec -u admin isaac_ros_dev-aarch64-container \
  bash -lc /workspaces/isaac_ros-dev/container_scripts/airborne_check.sh; echo "exit=$?"
```

Expect `landed: true`, then `airborne_check: landed` with `exit=1`. Exit 1 means flight is not
proven, which is what permits a reap. A `no publisher on /fmu/out/vehicle_land_detected` also exits
1 and means the PX4 uXRCE-DDS bridge is down, leaving the supervisor's land interlock nothing to
gate on. `AIRBORNE - stack preserved` on the ground is a FAIL: the land detector is wrong, and a
supervisor stop would leave an orphan holding the camera.

**10. Supervisor restart and orphan reap**

```bash
sudo systemctl restart arid_supervisor.service
sleep 25
pgrep -af 'ros2 launch px4_vslam vslam[.]launch[.]py'
rslidar_status
status
```

Expect no `pgrep` match and `vslam: stopped`, with `rslidar_status` unchanged. Any remaining
process in the vslam tree means the reap failed and it still holds the camera. The
supervisor respawns within 5 s; more than 5 restarts in 60 s leaves the unit failed, so read
`journalctl -u arid_supervisor -n 100`.

**11. Bringup over an orphan**

```bash
initialize
sentry
```

Expect success even if step 10 left a tree behind: landed is proven, so the supervisor reaps the
unowned tree and continues into a fresh bringup. `unowned vslam trees present ... land state not
proven` means step 9's publisher is missing.

---

## PX4 firmware

The PX4 fork lives at [`local_ws/auxiliary/PX4-Autopilot/`](local_ws/auxiliary/PX4-Autopilot/) and
carries the `4026_arid_quad_v1_2` airframe. A prebuilt binary ships at
[`px4_compiled/arid.px4`](local_ws/auxiliary/px4_compiled/) for flashing without a build, with the
raw `arid.bin` alongside it.

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

## Boot sequence

These units come up at boot.

| Service | Does |
|---|---|
| **usbfs-memory** | Raises the usbfs buffer to 1000 MB for RealSense multi-stream. |
| **jetson-clocks** | Sets Jetson clocks and fan to maximum. |
| **start_isaac_docker** | Starts the Isaac container. |
| **arid_description** | `robot_state_publisher` and the TF tree. |
| **gst_camera_manager** | Camera manager up, `cam_down` idle. |
| **rslidar_coordinator** | LiDAR supervisor up, SDK subprocess idle. |
| **usb_ros_reset** | Hosts `/reset_usb`. |
| **arid_supervisor** | Supervisor in-container, VSLAM idle until `initialize`. |

Stopping `arid_supervisor.service` is airborne-gated. On every stop path, including a crash,
`ExecStopPost` runs
[`airborne_check.sh`](isaac_ros-dev/container_scripts/airborne_check.sh): a fresh `landed: false`
sample means proven flight and nothing is reaped, so the VSLAM tree keeps running. Anything else
falls through to [`reap_stack.sh`](isaac_ros-dev/container_scripts/reap_stack.sh), which
group-SIGINTs the `px4_vslam` launch tree, drains for 25 s, then SIGKILLs. `deinitialize` is gated
the same way and is refused while airborne.

## Robot description

`arid_description.service` runs `robot_state_publisher` on
[`xacro/arid.xacro`](local_ws/src/arid_description/xacro/arid.xacro). The tree carries
`base_link`, `base_footprint`, `autopilot`, four propellers, `front_realsense_link`,
`rslidar_link`, `bottom_visual_link`, `flow_link` and `rangefinder_link`. Visualization is covered
in [`arid_description/README.md`](local_ws/src/arid_description/README.md).

## Development workflow

VSCode over Remote-SSH is the recommended editor: all code, builds, and the live runtime stay on
the drone, and the repo ships a `.vscode/` configuration with the recommended extension list,
Python and C++ lint settings, and search excludes. On first open VSCode offers to install those
extensions on the drone.

```bash
ssh jetson@<device-ip>
```

For a virtual desktop, connect a NoMachine session to `<device-ip>`. The camera feed window, the
calibration GUI and the camera-focus loop render on that desktop, so keep a session attached while
using them.

The host workspace bind-mounts into the container at `/workspaces/isaac_ros-dev`. Build it with
`colcon_isaac` from either side and enter it with `start_isaac` followed by `isaac_bash`.
`local_ws` builds on the host with `colcon_local`. For in-container IntelliSense, attach VSCode to
`isaac_ros_dev-aarch64-container` with Dev Containers.

### VSCode Remote-SSH offline fix

Remote-SSH fails offline with `Failed to download VS Code Server` when the laptop's VSCode commit
has no matching server staged on the drone. Freeze the laptop's commit once, then restart VSCode.
These are client-scope settings and cannot live in `.vscode/`.

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

Both back up to `settings.json.bak`. If the parse fails on `//` comments, restore the backup and
add the three keys by hand.

## Package reference

| Package | Purpose |
|---|---|
| [`arid_description`](local_ws/src/arid_description/) | Xacro, meshes, RViz config. |
| [`ros_gst_cameras`](local_ws/src/ros_gst_cameras/) | CSI camera stack. |
| [`rslidar_coordinator`](local_ws/src/rslidar_coordinator/) | RSAIRY supervisor: SDK config, services, watchdog. |
| [`reset_ark_usb`](local_ws/src/reset_ark_usb/) | The `/reset_usb` service. |
| [`camera_calibration`](local_ws/auxiliary/camera_calibration/) | `cam_down` calibrator and pattern. |
| [`px4_vslam`](isaac_ros-dev/src/px4_vslam/) | VSLAM launch and PX4 bridge. |
| [`px4_vslam_reactor`](isaac_ros-dev/src/px4_vslam_reactor/) | VSLAM gating and re-seat. |
| [`vslam_sentry`](isaac_ros-dev/src/vslam_sentry/) | Camera and VO health watchdog. |
| [`arid_supervisor`](isaac_ros-dev/src/arid_supervisor/) | VSLAM lifecycle service. |

[`update_submods.sh`](scripts/update_submods.sh) enforces two classes. LIVE entries are
branch-tracked, so HEAD must sit on that remote branch at or behind the tip. PINNED entries must
sit on an exact tag and are re-checked out if they drift. `update_submods.sh --update` advances
both.

| Submodule | Class | Role |
|---|---|---|
| [`PX4-Autopilot`](local_ws/auxiliary/PX4-Autopilot/) | LIVE `PX4-InDro` | PX4 fork with the ARID airframe. |
| [`realsense-ros`](isaac_ros-dev/src/realsense-ros/) | LIVE `v4.51.1` | RealSense driver fork: `hw_reset` service, hot-removal guards, `color_format`, log aggregation, claim retry. |
| [`isaac_ros_visual_slam`](isaac_ros-dev/src/isaac_ros_visual_slam/) | LIVE `v3.2-14` | cuVSLAM backend fork: stale-tolerant image synchronizer. |
| [`px4_msgs`](isaac_ros-dev/src/px4_msgs/) | LIVE `release/1.15` | PX4 messages; `local_ws/src/px4_msgs` symlinks to it. |
| [`isaac_ros_common`](isaac_ros-dev/src/isaac_ros_common/) | PINNED `v3.2-14` | Isaac base image chain. |
| [`isaac_ros_nitros`](isaac_ros-dev/src/isaac_ros_nitros/) | PINNED `v3.2-14` | Zero-copy transport. |
| [`isaac_ros_image_pipeline`](isaac_ros-dev/src/isaac_ros_image_pipeline/) | PINNED `v3.2-14` | GPU image processing. |
| [`px4-ros2-interface-lib`](isaac_ros-dev/src/px4-ros2-interface-lib/) | PINNED `1.4.0` | Auterion PX4 SDK. |
| [`rslidar_sdk`](local_ws/src/rslidar_sdk/) | PINNED `v1.5.19` | RoboSense SDK; builds `rslidar_sdk_node`. |
| [`rslidar_msg`](local_ws/src/rslidar_msg/) | PINNED `v1.5.10` | RoboSense message definitions. |
