# InDro ARID Workspace

**Autonomous Research Indoor Drone.** NVIDIA Jetson Orin · ARK PAB carrier · ROS 2 Humble · NVIDIA Isaac ROS · PX4

End-to-end repo for provisioning, building, and operating a deployed ARID:

- **Bootstrap** (`setup.sh`): provisioning from fresh Ubuntu 22.04 to flight-ready; safe to re-run at any point.
- **Robot description** (`arid_description`): xacro + meshes; `/robot_description` + `/tf_static` on boot.
- **CSI camera** (`ros_gst_cameras`): downward IMX219 pipeline (`cam_down`) with SetBool start/stop and a frame-flow watchdog.
- **LiDAR** (`rslidar_coordinator`): RoboSense RSAIRY supervisor; SetBool start/stop, latched `/alive`, auto-configured Ethernet link.
- **Visual odometry** (`px4_vslam`, `px4_vslam_reactor`, `vslam_sentry`, `arid_supervisor`): front RealSense → Isaac cuVSLAM → PX4 VIO, supervised and watchdogged. `initialize` / `deinitialize` from any terminal.
- **PX4 firmware**: InDro fork, `4026_arid_quad_v1_2` airframe, prebuilt binary shipped in-repo.
- **USB recovery** (`reset_ark_usb`): `/reset_usb` Trigger service.
- **Diagnostics**: `local_test` smoke test, `lidar_diag` LiDAR network walk-through, `config_lidar` LiDAR IP auto-detect, `config_realsense` serial assignment, `ver_cv_cams` live feed, `sentry` health JSON.
- **Helpers**: `wifi`, `update_submods`, `zt_join`, Foxglove bridge (apt, port 8765).

## Sensor configuration

| Sensor | Fit |
|---|---|
| 1x RealSense | front IR stereo (VSLAM), `640x360x60` infra1 + infra2, colour / depth / IMU off |
| 1x RSAIRY LiDAR | RoboSense, Ethernet |
| 1x IMX219 CSI | `cam_down` (downward); unrotated, uncalibrated, video only |
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
| `config_realsense` | Reseed `vslam_config.yaml` from the template + assign the RealSense serial. |
| `local_test` / `ver_cv_cams` | Smoke test / live feed. |
| `wifi` / `zt_join` / `update_submods` | Wi-Fi / ZeroTier / submodule sync. |
| `foxglove_bridge` | Bridge on 8765. |
| `initialize` / `deinitialize` / `status` | VSLAM via the supervisor. |
| `sentry` | `vslam_sentry` status JSON. |

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

Front RealSense (IR stereo, `640x360x60`) → Isaac cuVSLAM (`num_cameras: 2`) → PX4 VIO via `/fmu/in/vehicle_visual_odometry`.

`min_num_images: 2` means both infra streams are required and **no** camera loss is survivable - the single camera is the whole VO input. It applies after cuVSLAM init only; init still needs one `camera_info` from both streams. `stale_stream_timeout_ms` is inert here: dropping a stale stream leaves 1 < 2, so no set emits either way.

| Alias | Action |
|---|---|
| `initialize` | SetBool true on `/arid_supervisor/vslam_enable`. |
| `deinitialize` | SetBool false (refused unless landed). |
| `status` | vslam running + land state. |
| `sentry` | Per-camera + VO health JSON. |

**Supervisor** ([`arid_supervisor/README.md`](isaac_ros-dev/src/arid_supervisor/README.md)) owns the VSLAM lifecycle. Each launch logs to `run_logs/vslam/vslam.log`.

- Camera-proven bring-up: USB pre-check, log-watch gate on `RealSense Node Is Up!`, fail-fast on `Error starting device`, one `reset_usb` recovery cycle.
- Kill-before-spinup: unowned `px4_vslam` trees are reaped first when landed is proven, refused otherwise.
- Landed-gated teardown: `enable=false` needs a fresh `landed == True`; stale or airborne refuses.
- `Restart=always` (a node crash makes `ros2 launch` exit 0, so `on-failure` never fires); `StartLimitBurst=5` per 60 s caps the loop.

A direct launch bypasses the supervisor (in-container): `vslam` or `ros2 launch px4_vslam vslam.launch.py`. It blocks until `/robot_description` is up, then starts the RealSense driver, the VSLAM node, `vio_transform` (FLU→FRD bridge), `vslam_reactor` and `vslam_sentry`.

**Reactor** ([`px4_vslam_reactor.yaml`](isaac_ros-dev/src/px4_vslam_reactor/config/px4_vslam_reactor.yaml)) gates jumps and velocity outliers, re-seats via `SetSlamPose`, and withholds VO during cadence starvation: one stamp gap ≥ `cadence_gate_hard_s` (0.40 s) or 5 consecutive gaps in [0.15 s, 0.40 s) engages; 5 nominal samples release, no timed escape. State on `/reactor/cadence_gated` (Bool, latched), engagements on `/reactor/cadence_gate_count` (UInt32, latched). Telemetry only - nothing on ARID consumes them.

**Sentry** ([`vslam_sentry/README.md`](isaac_ros-dev/src/vslam_sentry/README.md)) is the device-plane watchdog: per-stream `camera_info` rate plus VO cadence, classifies which layer broke, hardware-resets a wedged camera. It never restarts VSLAM, never gates launch, never touches fusion. With one camera a dead stream *is* the VO outage, so calm-window deferral is off (`reset_defer_max_s: 0.0`) - EKF2 coasts on ARK flow + rangefinder while the camera re-enumerates. A blank serial does not kill it: it logs `NO CAMERAS parsed` and idles. Log: `run_logs/sentry/sentry.log`.

| Name | Type | Dir |
|---|---|---|
| `/vslam_sentry/status` | `std_msgs/String` (latched) | pub - status JSON, refreshed every 15 s |
| `/vslam_sentry/healthy` | `std_msgs/Bool` (latched) | pub - true only when settled, VO `OK`, camera `HEALTHY` |
| `/vslam_sentry/status_now` | `std_srvs/Trigger` | srv - same JSON on demand (`sentry`) |
| `/vslam_sentry/reset_front` | `std_srvs/Trigger` | srv - manual reset |
| `/front_realsense/hw_reset` | `std_srvs/Trigger` | cli - driver reset, the primary reset path |

### vslam_config: template + live

| File | Tracked | Role |
|---|---|---|
| [`vslam_config.template.yaml`](isaac_ros-dev/src/px4_vslam/config/vslam_config.template.yaml) | yes | fleet structure + tunables, empty `serial_no` |
| `vslam_config.yaml` | no (untracked + gitignored) | the live file: template + this drone's serial |

`config_realsense` regenerates the live file from the template on every run, re-splicing the existing serial, so a pulled template reaches a provisioned drone. Setup calls `config_realsense --reseed-only` unconditionally before any camera gate; that path never prompts and never touches hardware. A config with no serial is reseeded **blank** with a loud warning - the serial is not restorable there. The full run also probes USB and writes the detected serial.

**Before flying:** run `config_realsense` so the serial matches the installed RealSense.

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

`local_test` (menu **2**) checks that the services are active, the aliases resolve, the Foxglove port is open, the full `cam_down` + `rslidar` lifecycles work (frame_id, rate, `/alive`, restart PID; the cloud-rate check SKIPs if the LiDAR is off), the supervisor is on the graph (if the container is up), and no subprocesses leak. It is safe to run any time and leaves the pipelines stopped. The bare alias only prints; run through menu **2** it also tees to `log/smoke_test_log_*.log`.

---

## Camera feed check

`ver_cv_cams` opens the live `cam_down` stream in a cv2 window over NoMachine; `q` quits.

---

## setup.sh menu

Running `./setup.sh` with no arguments opens the interactive menu:

Every step detects what is already in place and only does what is missing, so re-running is always safe. Each run logs to `log/setup_log_*.log`. Ctrl+C ends setup at any point and clears the resume hooks.

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
| **11** | Build the Isaac workspace + restart `arid_supervisor.service` (requires the container running) |
| **12** | Build `local_ws` |
| **13** | ZeroTier join/switch |
| **14** | Uninstall (repo, OS, Docker engine kept) |

`setup.sh` keeps the orchestration plus the menu actions (RealSense, camera verify/focus/calibrate, uninstall); the provisioning steps live in `setup/`:

| Module | Holds |
|---|---|
| `setup/io.sh` | colours, `step`/`ok`/`warn`, `ask_yn`, ERR + Ctrl+C traps, `prompt_*` |
| `setup/system.sh` | power, first boot, Wi-Fi, NoMachine, apt, git, bashrc, permissions, systemd, `local_ws` |
| `setup/network.sh` | ZeroTier |
| `setup/lidar.sh` | RSAIRY sysctl + NetworkManager profiles + dispatcher |
| `setup/container.sh` | Docker, Isaac patches, skip-worktree, image + workspace builds |
| `setup/arid_resume_prompt.sh` | the `~/.bashrc` resume-after-reboot prompt |

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

`./setup.sh --full` walks the questionnaire once, then provisions everything unattended. When anything built, setup reboots before the smoke test so the test validates a clean boot. To resume after that reboot, open a bash terminal: you are prompted to continue. `--resume` is the same continuation invoked manually; `--continue` re-enters the tail with finished steps skipped.

```
./setup.sh --full
./setup.sh --resume
```

Phase A steps are checkpointed to `~/.arid_progress`, so a resume (or a re-run after a failure) skips what already finished instead of redoing and re-prompting it.

### Steps (in order)

| Step | Does |
|---|---|
| **preflight** | Not root; git present; submodules initialized and non-empty. |
| **collect_answers** | Questionnaire → `PRE_*` answers, persisted for the resume. |
| **first_boot** | One-time hostname/password. |
| **power** | Sets nvpmodel to maximum; apt-holds critical L4T packages. |
| **disable_updates** | Disables unattended-upgrades and the apt timers. |
| **enable_user_linger** | Enables user lingering so `/run/user/<uid>` is created at boot (headless NoMachine fix). |
| **clean_nvidia_desktop** | Removes NVIDIA first-boot icons + L4T-README automount. |
| **ensure_wifi** | Joins the questionnaire's network. |
| **nomachine** | Detects install; prints manual hint if missing. |
| *Phase A - checkpointed* | |
| **repos** | ROS / NVIDIA / Docker apt repos, CDI config. |
| **apt** | Apt packages (chrony, Foxglove bridge, OpenCV, camera calibration, net tools). |
| **clock_sync** | Enables chrony and disables `systemd-timesyncd`: chrony steps only at boot and slews after, so no NTP step lands mid-mission and tears a hole in the VO timestamps. |
| **zerotier** | Installs daemon, joins via `zt_join.sh --setup`. `ACCESS_DENIED` = authorize later. |
| **px4_deps** | PX4 toolchain (skipped if `arm-none-eabi-gcc` present); pins `numpy<2`. |
| **git** | Credential cache, script perms, `update_submods.sh` pin-verify. |
| **docker_patches** | Injects `Dockerfile.arid` / `arid_env.sh` / `run_dev.sh` into `isaac_ros_common`. |
| **skip_worktree** | Hides per-drone camera calibrations from git status, and clears marks on anything no longer per-drone. Tuning config stays tracked and committable. |
| **bashrc** | Rewrites the ARID block: env exports + the full alias set + `help` + resume hook. |
| **permissions** | Sudoers (uhubctl, gpioset, systemctl, `usb_reset.sh`, `zerotier-cli`), udev, polkit, groups. |
| **uhubctl** | Builds from source if missing. |
| **lidar_sysctl** | Kernel UDP receive buffers → 25 MiB (`/etc/sysctl.d/99-rslidar.conf`). |
| **lidar_network** | NM profiles on `enP8p1s0` (`rslidar` static / `dev` DHCP fallback) + link-up dispatcher; then `config_lidar`. |
| **ros_workspace** | Python deps, rosdep, colcon-builds `local_ws`. |
| **docker** | Engine, NVIDIA runtime, docker group, buildx. |
| **systemd** | Installs every unit the repo ships and enables all but `reset_usb.service` (on-demand oneshot); installs the ROS-env `DefaultEnvironment` drop-in. |
| **realsense** | Reseeds `vslam_config.yaml` from the template, then assigns the detected serial. |
| *Phase B* | |
| **build_isaac** | Container image build; queued across the reboot. |
| **colcon_isaac** | Builds the container workspace, restarts `arid_supervisor.service`. Enter = rebuild. |
| **verify_cameras / camera_focus** | Live `cam_down` feed, then the Foxglove focus loop (if opted in). |
| **reboot + smoke test** | Summary, reboot if anything built, then unit recovery + `local_test.sh` on the clean boot. |

After setup, work through **Post-update validation** below, then calibrate and focus the down camera as needed.

---

## Post-update validation

Ordered pre-flight checklist after any pull, rebuild, or reprovision. Props off, disarmed, drone on the ground. A FAIL stops the sequence.

**1. Serials**

```bash
config_realsense
grep serial_no ~/workspaces/isaac_ros-dev/src/px4_vslam/config/vslam_config.yaml
```

Expect one non-empty serial. Blank = the live config was reseeded without it: bringup cannot map the camera and the sentry idles with no watchdog.

**2. Submodules / forks**

```bash
update_submods
```

Expect `All submodules verified.`: `realsense-ros` on `origin/v4.51.1`, `isaac_ros_visual_slam` on `origin/v3.2-14`, the six PINNED entries on their exact tags. `commit is NOT on origin/<branch>` means the fork drifted - run `scripts/update_submods.sh --update`. Upstream instead of the fork = no `/front_realsense/hw_reset`, so the sentry cannot reset the camera and its only recovery path is gone.

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
systemctl is-active start_isaac_docker arid_supervisor arid_description gst_camera_manager rslidar_coordinator usb_ros_reset
local_test
```

Expect `active` six times and an all-PASS `local_test` (services, aliases, port 8765, `cam_down` + `rslidar` lifecycles, supervisor on the graph, no leaked subprocesses). A dead supervisor here is normally an unbuilt container workspace.

**5. LiDAR**

```bash
lidar_diag                     # sudo lidar_diag for the tcpdump + arp-scan legs
rslidar_start
rslidar_status
rslidar_alive
ros2 topic hz --qos-reliability best_effort /rslidar_points
rslidar_stop
```

Expect `RUNNING (pid=N)`, latched `data: true`, and a steady cloud rate. `/alive` false after 5 s of silence = the SDK is up but no frames: re-run `config_lidar` (LiDAR IP moved, e.g. after an RSView session or a swap), then `lidar_diag`. There is no auto-restart; recover with `rslidar_restart`.

**6. Bringup**

```bash
initialize
status
```

Blocks ~15 s healthy, up to ~3 min. Expect `Response(success=True` then `vslam: running | land: landed`. A refusal carries verbatim driver evidence: repeated `Error starting device` is camera/USB, `no factory exists` is a failed `image_transport` plugin load.

**7. Sentry**

```bash
sentry
ros2 topic echo --once --qos-durability transient_local /vslam_sentry/healthy
```

After the 40 s settle expect `"vslam":{"state":"OK"` with the camera `HEALTHY`, and `data: true`. Stuck `SETTLING` past a minute = streams never reached `min_hz` (30). `NO CAMERAS parsed` in `~/workspaces/isaac_ros-dev/run_logs/sentry/sentry.log` = blank serial, go back to step 1. `ESCALATED` = the camera exhausted its resets; treat as hardware.

**8. Cadence-gate telemetry**

```bash
ros2 topic echo --once --qos-durability transient_local /reactor/cadence_gated
ros2 topic echo --once --qos-durability transient_local /reactor/cadence_gate_count
```

Expect `data: false` and `data: 0` on a healthy idle stack. Both latched, so a late subscriber still reads the state. A non-zero count before flight means cadence starvation already happened - find it first. Nothing consumes these topics; they are forensics.

**9. Airborne gate, exercised on the ground**

```bash
ros2 topic echo --once --qos-reliability best_effort --qos-durability volatile \
  /fmu/out/vehicle_land_detected px4_msgs/msg/VehicleLandDetected | grep '^landed'
docker exec -u admin isaac_ros_dev-aarch64-container \
  bash -lc /workspaces/isaac_ros-dev/container_scripts/airborne_check.sh; echo "exit=$?"
```

Expect `landed: true`, then `airborne_check: landed` with `exit=1`. Exit 1 means "flight not proven", which is what permits a reap - the correct ground answer. The script only reads a topic; it reaps nothing itself. `no publisher on /fmu/out/vehicle_land_detected` also exits 1 (fail toward cleanup) and means the PX4 uXRCE-DDS bridge is down: fix it, or the supervisor's land interlock has nothing to gate on. `AIRBORNE - stack preserved` on the ground is a FAIL: the land detector is lying and a supervisor stop would leave an orphan holding the camera.

**10. Supervisor restart + orphan reap**

```bash
sudo systemctl restart arid_supervisor.service
sleep 25
pgrep -af 'ros2 launch px4_vslam vslam[.]launch[.]py'
rslidar_status
status
```

Expect no `pgrep` match and `vslam: stopped`: landed, so `ExecStopPost` fell through to `reap_stack.sh` and drained the group. `rslidar_status` must be unchanged - the reap pattern carries the `px4_vslam` token precisely so the host LiDAR, camera-manager and description units survive it. Survivors of the vslam tree mean the reap failed and they still hold the camera. `Restart=always` respawns the supervisor within 5 s; more than 5 restarts in 60 s trips `StartLimitBurst` and the unit stays failed - read `journalctl -u arid_supervisor -n 100`.

**11. Kill-before-spinup**

```bash
initialize
sentry
```

Expect success even if step 10 left a tree behind: landed is proven, so the supervisor reaps the unowned tree and continues into a fresh bringup. `unowned vslam trees present ... land state not proven` means step 9's publisher is missing - fix that first, then retry.

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

Stopping `arid_supervisor.service` is **airborne-gated**. On every stop path, including a crash, `ExecStopPost` runs [`airborne_check.sh`](isaac_ros-dev/container_scripts/airborne_check.sh): a fresh `landed: false` sample means proven flight and nothing is reaped, so the VSLAM tree keeps flying. Anything else (landed, no publisher, no sample) falls through to [`reap_stack.sh`](isaac_ros-dev/container_scripts/reap_stack.sh), which group-SIGINTs the `px4_vslam` launch tree, drains 25 s, then SIGKILLs - otherwise an orphan holds the camera. `deinitialize` is gated the same way and is refused while airborne.

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
| [`vslam_sentry`](isaac_ros-dev/src/vslam_sentry/) | Camera/VO health watchdog + targeted hardware reset. |
| [`arid_supervisor`](isaac_ros-dev/src/arid_supervisor/) | VSLAM lifecycle service (camera-proven bring-up, landed gate). |

[`update_submods.sh`](scripts/update_submods.sh) enforces two classes: **LIVE** entries are branch-tracked (HEAD must be on that remote branch, tip or behind), **PINNED** entries must sit on an exact tag and are re-checked out if they drift. `update_submods.sh --update` advances both.

| Submodule | Class | Role |
|---|---|---|
| [`PX4-Autopilot`](local_ws/auxiliary/PX4-Autopilot/) | LIVE `PX4-InDro` | PX4 fork, ARID airframe. |
| [`realsense-ros`](isaac_ros-dev/src/realsense-ros/) | LIVE `v4.51.1` | RealSense driver - InDro fork replacing the upstream pin: `hw_reset` service (the sentry's reset path), hot-removal crash guards, `color_format`, log-spam aggregation. |
| [`isaac_ros_visual_slam`](isaac_ros-dev/src/isaac_ros_visual_slam/) | LIVE `v3.2-14` | cuVSLAM backend - InDro fork replacing the upstream pin: stale-tolerant image synchronizer (`stale_stream_timeout_ms`). |
| [`px4_msgs`](isaac_ros-dev/src/px4_msgs/) | LIVE `release/1.15` | PX4 messages. Single submodule; `local_ws/src/px4_msgs` is a symlink to it. |
| [`isaac_ros_common`](isaac_ros-dev/src/isaac_ros_common/) | PINNED `v3.2-14` | Isaac base (Dockerfile chain; ARID-patched). |
| [`isaac_ros_nitros`](isaac_ros-dev/src/isaac_ros_nitros/) | PINNED `v3.2-14` | Zero-copy transport. |
| [`isaac_ros_image_pipeline`](isaac_ros-dev/src/isaac_ros_image_pipeline/) | PINNED `v3.2-14` | GPU image processing. |
| [`px4-ros2-interface-lib`](isaac_ros-dev/src/px4-ros2-interface-lib/) | PINNED `1.4.0` | Auterion PX4 SDK. |
| [`rslidar_sdk`](local_ws/src/rslidar_sdk/) | PINNED `v1.5.19` | RoboSense SDK; builds `rslidar_sdk_node` (nested `rs_driver` auto-initialized). |
| [`rslidar_msg`](local_ws/src/rslidar_msg/) | PINNED `v1.5.10` | RoboSense message definitions. |

> The two forks track branches whose names look like tags. A bare checkout would prefer the same-named tag, so `--update` uses `git checkout -B <branch> origin/<branch>`.
