# InDro ARID Workspace

**Autonomous Research Indoor Drone.** NVIDIA Jetson Orin · ARK PAB carrier · ROS 2 Humble · NVIDIA Isaac ROS · PX4

End-to-end repo for provisioning, building, and operating a deployed ARID:

- **Bootstrap** (`setup.sh` + `setup/`) provisions a fresh Ubuntu 22.04 install to flight-ready.
- **Robot description** (`arid_description`) publishes `/robot_description` and `/tf_static` on boot.
- **CSI cameras** (`ros_gst_cameras`) run the front and down IMX219 pipelines.
- **Visual odometry** (`px4_vslam`, `px4_vslam_reactor`, `vslam_sentry`, `arid_supervisor`) feeds 3× RealSense through Isaac cuVSLAM into PX4 VIO, supervised and watchdogged.
- **PX4 firmware** is the InDro fork with the ARID airframe.
- **USB recovery** (`reset_ark_usb`) hosts the `/reset_usb` service.
- **Diagnostics**: `local_test` smoke test, `config_realsense` serial assignment, `ver_cv_cams` live feeds, `sentry` health JSON.
- **Helpers**: `wifi`, `update_submods`, `zt_join`, Foxglove bridge.

## Sensor configuration

| Sensor | Fit |
|---|---|
| 3x RealSense D435 | front / left / right IR stereo (VSLAM); `640x360x60` infra1 + infra2, colour / depth / IMU off |
| 2x IMX219 CSI | `cam_front` (front) + `cam_down` (downward); unrotated, uncalibrated, video only |
| ARK optical flow + rangefinder | bottom pod |

> ROS 2 traffic is restricted to the drone: `ROS_DOMAIN_ID=23`, `ROS_LOCALHOST_ONLY=1`. Remote visualization goes through Foxglove.

---

## After boot

After a normal boot you land in a shell with the container running, the TF tree up, both camera pipelines idle, `/arid_supervisor/vslam_enable` on the graph, and `local_ws` sourced.

---

## Useful aliases

**Host** (installed by the `setup.sh` bashrc step; `help` prints them all):

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
| `cam_stop` | Stop both pipelines. |
| `cam_refresh` | Re-read `pipelines.yaml` at runtime (stops pipelines first). |
| `cam_calibrate front\|down` | Calibrate a CSI camera. |
| `config_realsense` | Reseed `vslam_config.yaml` + assign the three RealSense serials. |
| `local_test` / `ver_cv_cams` | Smoke test / live feeds. |
| `wifi` / `zt_join` / `update_submods` | Wi-Fi / ZeroTier / submodule sync. |
| `foxglove_bridge` | Bridge on 8765. |
| `initialize` / `deinitialize` / `status` | VSLAM via the supervisor. |
| `sentry` | `vslam_sentry` status JSON. |

**Container** (`arid_env.sh`): `vslam` (direct launch), `initialize` / `deinitialize` / `status`, `reset_usb`, `colcon_isaac` / `clean_isaac` / `rosdep_isaac`, `foxglove_bridge`, `help`.

---

## Cameras (CSI)

Two IMX219 pipelines ([`pipelines.yaml`](local_ws/src/ros_gst_cameras/gst_camera_manager/config/pipelines.yaml)) run at 1920×1080@15fps in GRAY8 (IR-sensitive mono), unrotated and uncalibrated - operator video feeds, nothing in-container subscribes them.

| Pipeline | Sensor | Frame | Topic root |
|---|---|---|---|
| `cam_front` | `sensor-id=0` | `top_visual_link` | `/cam_front` |
| `cam_down` | `sensor-id=1` | `bottom_visual_link` | `/cam_down` |

Start and stop with the per-pipeline aliases, or call the manager's SetBool service directly:

```bash
cam_front_start / cam_front_stop / cam_front_status / cam_front_alive
cam_down_start  / cam_down_stop  / cam_down_status  / cam_down_alive
ros2 service call /gst_camera_manager/cam_front std_srvs/srv/SetBool '{data: true}'
```

Each pipeline publishes `image_raw`, `image_raw/compressed`, and `camera_info`, reports liveness on `/gst_camera_manager/<name>/alive` (latched), and sits under a frame-flow watchdog. The manager's log is tee'd to `isaac_ros-dev/run_logs/gst_camera_manager/`. Details: [`ros_gst_cameras/README.md`](local_ws/src/ros_gst_cameras/README.md).

---

## Visual odometry (VSLAM packages)

Three RealSense cameras (front/left/right) feed Isaac cuVSLAM (`num_cameras: 6`, stereo-multicam), which feeds PX4 VIO via `/fmu/in/vehicle_visual_odometry`. Each camera streams `infra1` + `infra2` at **640x360x60** - not 90 fps: that service interval stalls the USB bus. Depth, colour and IMU are off.

`min_num_images: 4` survives a full single-camera loss (two stereo pairs keep VO alive, degraded). It applies **after** cuVSLAM init only - init still needs one `camera_info` from all 6 streams.

| Alias | Action |
|---|---|
| `initialize` | SetBool true on `/arid_supervisor/vslam_enable`. |
| `deinitialize` | SetBool false (refused unless landed). |
| `status` | vslam running + land state. |
| `sentry` | Per-camera + VO health JSON. |

**Config split.** [`vslam_config.template.yaml`](isaac_ros-dev/src/px4_vslam/config/vslam_config.template.yaml) is tracked and carries the fleet structure + tunables with blank `serial_no`; `vslam_config.yaml` is the live per-drone file - untracked and gitignored, template plus this drone's three serials. `config_realsense` reseeds the live file from the template and re-splices the existing serials on every run, so a pulled template reaches a provisioned drone. Setup calls `config_realsense --reseed-only` unconditionally before any camera gate; that path never prompts and never touches hardware. A config with fewer than 3 serials is reseeded **blank** with a loud warning - the serials are not restorable there. **Before flying:** run `config_realsense` so all three serials are assigned.

**Supervisor** ([`arid_supervisor/README.md`](isaac_ros-dev/src/arid_supervisor/README.md)) owns the VSLAM lifecycle. Each launch logs to `run_logs/vslam/vslam.log`.

- Camera-proven bring-up: USB pre-check, log-watch gate on 3 distinct `RealSense Node Is Up!` tags, fail-fast on `Error starting device`, one `reset_usb` recovery cycle. ~15 s healthy, ~3 min worst.
- Kill-before-spinup: unowned `px4_vslam` trees are reaped first when landed is proven, refused otherwise.
- Landed-gated teardown: `enable=false` needs a fresh `landed == True`; stale or airborne refuses.
- `Restart=always` (a node crash makes `ros2 launch` exit 0, so `on-failure` never fires); `StartLimitBurst=5` per 60 s caps the loop.

**Reactor** ([`px4_vslam_reactor.yaml`](isaac_ros-dev/src/px4_vslam_reactor/config/px4_vslam_reactor.yaml)) gates jumps and velocity outliers, withholds VO during cadence starvation, and re-seats the solution via `SetSlamPose`. Cadence gate engages on one stamp gap ≥ `cadence_gate_hard_s` (0.40 s) or 5 consecutive gaps in [0.15 s, 0.40 s); releases on 5 nominal samples, no timed escape. State on `/reactor/cadence_gated` (Bool, latched), engagements on `/reactor/cadence_gate_count` (UInt32, latched). Telemetry only - nothing on ARID consumes them.

**Sentry** ([`vslam_sentry/README.md`](isaac_ros-dev/src/vslam_sentry/README.md)) is the device-plane watchdog: it classifies which layer broke from per-stream `camera_info` rate + VO cadence and hardware-resets a wedged camera, one at a time. It never restarts VSLAM, never gates launch, never touches fusion. Blank serials do not kill it - it logs `NO CAMERAS parsed` and idles. Log: `run_logs/sentry/sentry.log`.

| Name | Type | Dir |
|---|---|---|
| `/vslam_sentry/status` | `std_msgs/String` (latched) | pub - status JSON, refreshed every 15 s |
| `/vslam_sentry/healthy` | `std_msgs/Bool` (latched) | pub - true only when settled, VO `OK`, all cameras `HEALTHY` |
| `/vslam_sentry/status_now` | `std_srvs/Trigger` | srv - same JSON on demand (`sentry`) |
| `/vslam_sentry/reset_{front,left,right}` | `std_srvs/Trigger` | srv - manual per-camera reset |
| `/{front,left,right}_realsense/hw_reset` | `std_srvs/Trigger` | cli - driver reset, the primary reset path |

For a direct launch that bypasses the supervisor, run `vslam` in the container. The launch blocks until `/robot_description` is up, then starts the three drivers, the VSLAM node, `vio_transform` (FLU→FRD), the reactor and the sentry.

Reference: [`px4_vslam/README.md`](isaac_ros-dev/src/px4_vslam/README.md), [`px4_vslam_reactor/README.md`](isaac_ros-dev/src/px4_vslam_reactor/README.md).

---

## USB reset

The `reset_usb` alias, or the underlying Trigger call:

```bash
reset_usb
ros2 service call /reset_usb std_srvs/srv/Trigger '{}'
```

The service uses `uhubctl` to power-cycle the ARK PAB USB hub and `gpioset` to pulse the FMU reset line (GPIO 85), so never call it in flight. Details: [`reset_ark_usb/README.md`](local_ws/src/reset_ark_usb/README.md).

---

## Foxglove

Run the `foxglove_bridge` alias (host or container) and connect Studio to `ws://<device-ip>:8765`.

---

## Smoke test

`local_test` is the host-stack smoke test:

```bash
local_test
```

It checks that the services are active, the aliases resolve, the Foxglove port answers, both `cam_front` and `cam_down` complete a full lifecycle (frame_id, rate, `/alive`), the supervisor is on the graph (when the container is up), and no subprocesses leak. It is safe to run any time and leaves the pipelines stopped. The bare alias only prints; run through menu **2** it also tees to `log/smoke_test_log_*.log`.

---

## Camera feed check

`ver_cv_cams` opens both live streams in a cv2 window over NoMachine (`q` quits); pass `front` or `down` for a single feed:

```bash
ver_cv_cams
```

---

## setup.sh

Running `setup.sh` with no arguments opens the interactive menu:

```
./setup.sh
```

Every step detects what is already in place and only does what is missing, so re-running is always safe. Each run logs to `log/setup_log_*.log`.

| Option | Action |
|---|---|
| **1** | Full setup |
| **2** | Smoke test |
| **3** | RealSense serial assignment (front/left/right) |
| **4** | Camera feed check (front / down / both) |
| **5** | Wi-Fi connect |
| **6** | Camera calibration (front/down) |
| **7** | Camera focus (front / down / both, via Foxglove) |
| **8** | Build the Isaac container |
| **9** | Build the Isaac workspace + restart `arid_supervisor.service` (requires the container running) |
| **10** | Build `local_ws` |
| **11** | ZeroTier join/switch |
| **12** | Uninstall (repo, OS, Docker engine kept) |

Ctrl+C at any point offers quit-or-continue and clears the resume hooks on quit.

### Layout

`setup.sh` keeps the orchestration plus the menu actions (RealSense, camera verify/focus/calibrate, uninstall); the provisioning steps live in `setup/`:

| File | Holds |
|---|---|
| `setup/io.sh` | Colours, output helpers, ERR + INT traps, `ask_yn`, `prompt_*`, user-exit cleanup. |
| `setup/system.sh` | Power, first boot, Wi-Fi, NoMachine, repos, apt, PX4 deps, git, bashrc, permissions, uhubctl, systemd, `local_ws`. |
| `setup/container.sh` | Docker engine, Isaac Dockerfile patches, skip-worktree, image + workspace builds. |
| `setup/network.sh` | ZeroTier. |
| `setup/arid_resume_prompt.sh` | The `~/.bashrc` resume prompt (sourced, never inlined). |

---

## Camera calibration

Pass `front` or `down` (menu 6 prompts for it):

```bash
cam_calibrate front
```

[`camera_calibration_auto/camera_calibrate.sh`](local_ws/auxiliary/camera_calibration/camera_calibration_auto/camera_calibrate.sh) runs a venv-isolated interactive calibrator against the live pipeline. The default board is the included 10×7-square / 50 mm PDF. It writes the live calibration to `config/calibrations/<cam>.yaml` plus a timestamped store; afterwards set `calibration: "cam_front"` / `"cam_down"` in `pipelines.yaml`. The script waits for a NoMachine session before starting (`NM_WAIT_S`, default 180 s). Details: [`camera_calibration/README.md`](local_ws/auxiliary/camera_calibration/README.md).

---

## Camera focus

Menu **7** starts the selected pipeline(s) plus the Foxglove bridge and confirms each stream is sustained. Watch `/<cam>/image_raw/compressed` in Studio while adjusting the lens; any key tears it down (bridge process group + pipelines).

---

## Full setup

`--full` walks the questionnaire once, then runs unattended. If anything was built, setup reboots once before the smoke test - a clean boot is what the smoke test validates. To resume, open a bash terminal: you are prompted to continue. `--resume` is the same continuation invoked manually; `--continue` re-enters the tail after an abort, skipping every checkpointed step.

```
./setup.sh --full
./setup.sh --resume
./setup.sh --continue
```

### Steps (in order)

Pre-tail (once per run), then the checkpointed tail: **Phase A** (install + host config) and **Phase B** (builds + live camera steps).

| Step | Does |
|---|---|
| **preflight** | Not root; git present; submodules initialized (emptied trees flagged). |
| **collect_answers** | Questionnaire → `PRE_*` answers, persisted for the resume. |
| **power** | nvpmodel maximum; apt-holds critical L4T packages. |
| **first_boot** | One-time hostname/password. |
| **ensure_wifi** | Joins the questionnaire's network. |
| **nomachine** | Installs/upgrades the arm64 .deb (skipped inside a NoMachine session). |
| **enable_user_linger** | `/run/user/<uid>` created at boot (headless NoMachine fix). |
| **clean_nvidia_desktop** | Removes NVIDIA first-boot icons + L4T-README automount. |
| **disable_updates** | apt drop-in, unattended-upgrades and the apt timers off. |
| *Phase A* | |
| **repos** | ROS / NVIDIA / Docker apt repos, CDI config (`--mode=csv`). |
| **apt** | Apt packages (chrony, Foxglove bridge, OpenCV, calibrator, net tools). |
| **clock_sync** | chrony on, systemd-timesyncd off: chrony steps at boot and slews after, so no mid-mission clock step. |
| **zerotier** | Installs daemon, joins via `zt_join.sh --setup`. `ACCESS_DENIED` = authorize later. |
| **px4_deps** | PX4 Python deps (numpy capped) + toolchain on demand. |
| **git** | Credential cache, script perms, `update_submods.sh` pin-verify, `run_logs` symlink. |
| **docker_patches** | Injects `Dockerfile.arid` / `arid_env.sh` / `run_dev.sh` into `isaac_ros_common`. |
| **skip_worktree** | Protects per-deployment camera calibrations; clears stale marks so tuning config stays committable. |
| **bashrc** | Rewrites the ARID block: env exports + the alias set + `help` + resume hook. |
| **permissions** | Sudoers (visudo-validated), udev, polkit, groups. |
| **uhubctl** | Builds from source if missing. |
| **ros_workspace** | Python deps, rosdep, colcon-builds `local_ws`. |
| **docker** | Engine, NVIDIA runtime, docker group, buildx. |
| **systemd** | Installs every shipped unit and enables all but `reset_usb.service` (on-demand oneshot); ROS-env `DefaultEnvironment` drop-in. |
| **realsense** | Reseeds `vslam_config.yaml` from the template, then assigns the three serials. |
| *Phase B* | |
| **build_isaac** | Container image build; queued across a reboot. |
| **colcon_isaac** | Builds the container workspace, restarts `arid_supervisor.service`. |
| **verify_cameras** | Live front + down feeds (if opted in). |
| **camera_focus** | Foxglove focus pass over both CSI cameras (if opted in). |
| **summary → reboot → smoke test** | Summary; one reboot if anything built; then `local_test`. |

---

## Post-update validation

Ordered pre-flight checklist after any pull, rebuild, or reprovision. Props off, disarmed, drone on the ground. A FAIL stops the sequence.

**1. Serials**

```bash
config_realsense
grep serial_no ~/workspaces/isaac_ros-dev/src/px4_vslam/config/vslam_config.yaml
```

Expect three distinct non-empty serials (front/left/right). Blank = the live config was reseeded without them: bringup cannot map cameras and the sentry idles with no watchdog.

**2. Submodules / forks**

```bash
update_submods
```

Expect `All submodules verified.`: `realsense-ros` on `origin/v4.51.1`, `isaac_ros_visual_slam` on `origin/v3.2-14`, the four PINNED entries on their exact tags. `commit is NOT on origin/<branch>` means the fork drifted - run `scripts/update_submods.sh --update`. Upstream instead of the fork = no `hw_reset` service (sentry cannot reset a camera) and no `stale_stream_timeout_ms` (one dead camera stalls VO).

**3. Build**

```bash
colcon_local
colcon_isaac
```

`colcon_isaac` deinitializes first (refused unless landed), builds in-container, restarts `arid_supervisor.service`. A build failure leaves the supervisor on old code with the stack down - fix before continuing. VSLAM is deliberately not auto-restarted.

**4. Clean boot**

```bash
sudo reboot
```

then, back on the drone:

```bash
systemctl is-active start_isaac_docker arid_supervisor arid_description gst_camera_manager usb_ros_reset
local_test
```

Expect `active` five times and an all-PASS `local_test` (services, aliases, port 8765, both CSI lifecycles, supervisor on the graph, no leaked subprocesses). A dead supervisor here is normally an unbuilt container workspace.

**5. Bringup**

```bash
initialize
status
```

Blocks ~15 s healthy, up to ~3 min. Expect `Response(success=True` then `vslam: running | land: landed`. A refusal carries verbatim driver evidence: repeated `Error starting device` is camera/USB, `no factory exists` is a failed `image_transport` plugin load.

**6. Sentry**

```bash
sentry
ros2 topic echo --once --qos-durability transient_local /vslam_sentry/healthy
```

After the 40 s settle expect `"vslam":{"state":"OK"` with all three cameras `HEALTHY`, and `data: true`. Stuck `SETTLING` past a minute = streams never reached `min_hz` (30). `NO CAMERAS parsed` in `~/workspaces/isaac_ros-dev/run_logs/sentry/sentry.log` = blank serials, go back to step 1. `ESCALATED` = the camera exhausted its resets; treat as hardware.

**7. Cadence-gate telemetry**

```bash
ros2 topic echo --once --qos-durability transient_local /reactor/cadence_gated
ros2 topic echo --once --qos-durability transient_local /reactor/cadence_gate_count
```

Expect `data: false` and `data: 0` on a healthy idle stack. Both latched, so a late subscriber still reads the state. A non-zero count before flight means cadence starvation already happened - find it first. Nothing consumes these topics; they are forensics.

**8. Airborne gate, exercised on the ground**

```bash
ros2 topic echo --once --qos-reliability best_effort --qos-durability volatile \
  /fmu/out/vehicle_land_detected px4_msgs/msg/VehicleLandDetected | grep '^landed'
docker exec -u admin isaac_ros_dev-aarch64-container \
  bash -lc /workspaces/isaac_ros-dev/container_scripts/airborne_check.sh; echo "exit=$?"
```

Expect `landed: true`, then `airborne_check: landed` with `exit=1`. Exit 1 means "flight not proven", which is what permits a reap - the correct ground answer. The script only reads a topic; it reaps nothing itself. `no publisher on /fmu/out/vehicle_land_detected` also exits 1 (fail toward cleanup) and means the PX4 uXRCE-DDS bridge is down: fix it, or the supervisor's land interlock has nothing to gate on. `AIRBORNE - stack preserved` on the ground is a FAIL: the land detector is lying and a supervisor stop would leave an orphan holding the cameras.

**9. Supervisor restart + orphan reap**

```bash
sudo systemctl restart arid_supervisor.service
sleep 25
pgrep -af 'ros2 launch px4_vslam vslam[.]launch[.]py'
status
```

Expect no `pgrep` match and `vslam: stopped`: landed, so `ExecStopPost` fell through to `reap_stack.sh` and drained the group. Survivors mean the reap failed and they still hold the cameras. `Restart=always` respawns the supervisor within 5 s; more than 5 restarts in 60 s trips `StartLimitBurst` and the unit stays failed - read `journalctl -u arid_supervisor -n 100`.

**10. Kill-before-spinup**

```bash
initialize
sentry
```

Expect success even if step 9 left a tree behind: landed is proven, so the supervisor reaps the unowned tree and continues into a fresh bringup. `unowned vslam trees present ... land state not proven` means step 8's publisher is missing - fix that first, then retry.

---

## PX4 firmware

The fork lives at [`local_ws/auxiliary/PX4-Autopilot/`](local_ws/auxiliary/PX4-Autopilot/) (`PX4-InDro`) and carries the **`4026_arid_quad_v1_2`** airframe. A prebuilt image ships at [`PX4_prebuilt/ARID_v1.2.px4`](local_ws/auxiliary/PX4_prebuilt/).

Building produces `build/ark_fmu-v6x_default/ark_fmu-v6x_default.px4`:

```bash
cd local_ws/auxiliary/PX4-Autopilot
make ark_fmu-v6x_default
```

Select the airframe from a MAVLink shell:

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
| **gst_camera_manager** | Camera manager up, both pipelines idle. |
| **usb_ros_reset** | Hosts `/reset_usb`. |
| **arid_supervisor** | Supervisor in-container via `docker exec`; VSLAM idle until `initialize`. |

Stopping `arid_supervisor.service` is **airborne-gated**. On every stop path, crash included, `ExecStopPost` runs [`airborne_check.sh`](isaac_ros-dev/container_scripts/airborne_check.sh): only a fresh `landed: false` sample proves flight, and then nothing is reaped so the VSLAM tree keeps flying. Landed, no publisher, or no sample all fall through to [`reap_stack.sh`](isaac_ros-dev/container_scripts/reap_stack.sh), which group-SIGINTs the `px4_vslam` launch tree by real pgid, drains 25 s, then SIGKILLs - otherwise an orphan holds the cameras. `deinitialize` is gated the same way.

> **First boot:** the supervisor cannot start until the container workspace has been colcon-built. `--full` handles this; otherwise run menu **9**, or:
>
> ```bash
> start_isaac && isaac_bash && colcon_isaac && exit
> sudo systemctl reset-failed arid_supervisor.service && sudo systemctl start arid_supervisor.service
> ```

---

## Development workflow

VSCode over Remote-SSH is the recommended editor: all code, builds, and the live runtime stay on the drone, and the repo ships VSCode configuration for a streamlined development experience.

```bash
ssh jetson@<device-ip>
```

On first open VSCode prompts to install the workspace's recommended extensions on the drone. `.vscode/` carries the extension list, Python/C++ lint + IntelliSense settings, and search excludes.

For direct virtual desktop access, connect a NoMachine session to `<device-ip>`. The GUI tools (camera feed windows, the calibration GUI) render on that desktop, so keep a session attached when using them.

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

The host workspace bind-mounts into the container at `/workspaces/isaac_ros-dev`. Build with `colcon_isaac` (host or container); enter with `start_isaac` + `isaac_bash`. `local_ws/` builds on the host with `colcon_local`. For in-container IntelliSense, attach VSCode to `isaac_ros_dev-aarch64-container` with Dev Containers.

---

## Robot description

`arid_description.service` runs `robot_state_publisher` on [`urdf/arid.xacro`](local_ws/src/arid_description/urdf/arid.xacro). Frames: `base_link`, `base_footprint`, `autopilot`, 4 propellers, `front/left/right_realsense_link`, `top_visual_link` (front cam), `bottom_visual_link` (down cam), `flow_link`, `rangefinder_link`. Visualization: [`arid_description/README.md`](local_ws/src/arid_description/README.md).

---

## Package reference

| Package | Purpose |
|---|---|
| [`arid_description`](local_ws/src/arid_description/) | Xacro, meshes, RViz config. |
| [`ros_gst_cameras`](local_ws/src/ros_gst_cameras/) | CSI camera stack (`gst_cam_node` + `gst_camera_manager`). |
| [`reset_ark_usb`](local_ws/src/reset_ark_usb/) | `/reset_usb` service. |
| [`camera_calibration`](local_ws/auxiliary/camera_calibration/) | front/down calibrator + pattern. |
| [`px4_vslam`](isaac_ros-dev/src/px4_vslam/) | 3-cam VSLAM launch + PX4 bridge. |
| [`px4_vslam_reactor`](isaac_ros-dev/src/px4_vslam_reactor/) | VSLAM jump + cadence gating, re-seat. |
| [`vslam_sentry`](isaac_ros-dev/src/vslam_sentry/) | Device-plane camera/VO watchdog with per-camera hardware reset. |
| [`arid_supervisor`](isaac_ros-dev/src/arid_supervisor/) | VSLAM lifecycle service (camera-proven bring-up, landed gate). |

[`update_submods.sh`](scripts/update_submods.sh) enforces two classes: **LIVE** entries are branch-tracked (HEAD must be on that remote branch, tip or behind), **PINNED** entries must sit on an exact tag and are re-checked out if they drift. `update_submods.sh --update` advances both.

| Submodule | Class | Role |
|---|---|---|
| [`PX4-Autopilot`](local_ws/auxiliary/PX4-Autopilot/) | LIVE `PX4-InDro` | PX4 fork, ARID airframe. |
| [`realsense-ros`](isaac_ros-dev/src/realsense-ros/) | LIVE `v4.51.1` | RealSense driver - **indro-robotics fork replacing the upstream pin**: `hw_reset` service (the sentry's reset path), hot-removal crash guards, `color_format` parameter, log-spam aggregation. |
| [`isaac_ros_visual_slam`](isaac_ros-dev/src/isaac_ros_visual_slam/) | LIVE `v3.2-14` | cuVSLAM backend - **indro-robotics fork replacing the upstream pin**: stale-tolerant image synchronizer (`stale_stream_timeout_ms`), so one dead camera no longer stalls VO. |
| [`px4_msgs`](isaac_ros-dev/src/px4_msgs/) | LIVE `release/1.15` | PX4 messages. Single submodule; `local_ws/src/px4_msgs` is a symlink to it. |
| [`isaac_ros_common`](isaac_ros-dev/src/isaac_ros_common/) | PINNED `v3.2-14` | Isaac base (Dockerfile chain; ARID-patched). |
| [`isaac_ros_nitros`](isaac_ros-dev/src/isaac_ros_nitros/) | PINNED `v3.2-14` | Zero-copy transport. |
| [`isaac_ros_image_pipeline`](isaac_ros-dev/src/isaac_ros_image_pipeline/) | PINNED `v3.2-14` | GPU image processing. |
| [`px4-ros2-interface-lib`](isaac_ros-dev/src/px4-ros2-interface-lib/) | PINNED `1.4.0` | Auterion PX4 SDK. |

> The two forks track branches whose names look like tags. A bare checkout would prefer the same-named tag, so `--update` uses `git checkout -B <branch> origin/<branch>`.
