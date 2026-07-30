# ARID

The ARID (Autonomous Research Indoor Drone) is an indoor quadrotor built on an NVIDIA Jetson Orin with an ARK PAB carrier, running ROS 2 Humble, NVIDIA Isaac ROS and PX4. This repository provisions the drone, builds both workspaces, and operates the flight stack. ROS 2 traffic is restricted to the drone (`ROS_DOMAIN_ID=23`, `ROS_LOCALHOST_ONLY=1`), and remote visualization goes through Foxglove.

- `setup.sh` with `setup/`: provisioning, from a fresh Ubuntu 22.04 install to flight-ready.
- `arid_description`: robot description, publishing `/robot_description` and `/tf_static`.
- `ros_gst_cameras`: the two IMX219 CSI video pipelines.
- `px4_vslam`, `px4_vslam_reactor`, `vslam_sentry`, `arid_supervisor`: RealSense visual odometry into PX4.
- `reset_ark_usb`: the `/reset_usb` service.
- `PX4-Autopilot`: the InDro PX4 fork carrying the ARID airframe.

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
| `usbfs-memory` | usbfs buffer at 1000 MB for RealSense multi-stream. |
| `jetson-clocks` | Maximum clocks. |
| `start_isaac_docker` | Isaac container. |
| `arid_description` | `robot_state_publisher`, the TF tree. |
| `gst_camera_manager` | CSI camera manager, pipelines idle. |
| `usb_ros_reset` | Hosts `/reset_usb`. |
| `arid_supervisor` | VSLAM lifecycle service, inside the container. |

Stopping `arid_supervisor.service` is airborne-gated. While flight is proven the VSLAM tree is left running; on every other stop path, crash included, the orphaned launch tree is reaped so it does not keep holding the cameras. `deinitialize` is gated the same way. Details: [`arid_supervisor/README.md`](isaac_ros-dev/src/arid_supervisor/README.md).

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

`help` prints the host set, and `help` inside the container prints the container set.

| Host alias | Action |
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
| `cam_refresh` | Re-read `pipelines.yaml` at runtime. |
| `cam_calibrate front\|down` | Calibrate a CSI camera. |
| `config_realsense` | Assign the three RealSense serials. |
| `local_test` | Host-stack smoke test. |
| `ver_cv_cams` | Live CSI feeds. |
| `wifi` / `zt_join` / `update_submods` | Wi-Fi picker, ZeroTier, submodule sync. |
| `foxglove_bridge` | Bridge on port 8765. |
| `initialize` / `deinitialize` / `status` | VSLAM through the supervisor. |
| `sentry` | `vslam_sentry` status JSON. |

Container aliases: `vslam`, `initialize`, `deinitialize`, `status`, `reset_usb`, `colcon_isaac`, `clean_isaac`, `rosdep_isaac`, `foxglove_bridge`, `help`.

---

## CSI cameras

Two IMX219 pipelines ([`pipelines.yaml`](local_ws/src/ros_gst_cameras/gst_camera_manager/config/pipelines.yaml)) run at 1920x1080 at 15 fps in GRAY8, unrotated and uncalibrated. They are operator video feeds; nothing in the container subscribes to them.

| Pipeline | Sensor | Frame | Topic root |
|---|---|---|---|
| `cam_front` | `sensor-id=0` | `top_visual_link` | `/cam_front` |
| `cam_down` | `sensor-id=1` | `bottom_visual_link` | `/cam_down` |

Start and stop with the per-pipeline aliases, or call the manager's SetBool service directly:

```bash
cam_front_start
cam_front_stop
ros2 service call /gst_camera_manager/cam_front std_srvs/srv/SetBool '{data: true}'
```

Each pipeline publishes `image_raw`, `image_raw/compressed` and `camera_info`, reports liveness on `/gst_camera_manager/<name>/alive` (latched), and runs under a frame-flow watchdog. Details: [`ros_gst_cameras/README.md`](local_ws/src/ros_gst_cameras/README.md).

---

## Visual odometry

The three RealSense cameras feed Isaac cuVSLAM as six IR streams (stereo multicam), and the filtered solution reaches PX4 on `/fmu/in/vehicle_visual_odometry`. Each camera streams `infra1` and `infra2` at 640x360x60 with colour, depth and IMU off.

> Never raise the RealSense profile to 90 fps. That service interval stalls the USB bus.

| Alias | Action |
|---|---|
| `initialize` | SetBool true on `/arid_supervisor/vslam_enable`. |
| `deinitialize` | SetBool false, refused unless landed. |
| `status` | VSLAM running state plus land state. |
| `sentry` | Per-camera and VO health JSON. |

The supervisor manages the VSLAM lifecycle behind a camera-proven bringup gate and a landed interlock, and each launch logs to `run_logs/vslam/vslam.log`. A healthy `initialize` returns in about 15 s and can take up to about 3 minutes when a camera needs a recovery cycle. The reactor filters the SLAM stream into PX4 and publishes gate telemetry on `/reactor/`; the sentry watches per-camera stream rates and hardware-resets one wedged camera at a time. Interfaces and tunables are in the package READMEs: [`arid_supervisor`](isaac_ros-dev/src/arid_supervisor/README.md), [`px4_vslam`](isaac_ros-dev/src/px4_vslam/README.md), [`px4_vslam_reactor`](isaac_ros-dev/src/px4_vslam_reactor/README.md), [`vslam_sentry`](isaac_ros-dev/src/vslam_sentry/README.md).

Run `config_realsense` before flying, so all three RealSense serials are assigned. A live config holding fewer than three serials is reseeded blank with a warning, and those serials are not recoverable from it. The template and live-file split behind that is described in [`px4_vslam`](isaac_ros-dev/src/px4_vslam/README.md).

For a direct launch that bypasses the supervisor, run `vslam` in the container.

---

## USB reset

The `reset_usb` alias wraps the Trigger call, and either form works.

```bash
reset_usb
ros2 service call /reset_usb std_srvs/srv/Trigger '{}'
```

> `/reset_usb` power-cycles the ARK PAB USB hub and pulses the FMU reset line. Never call it in flight.

Details: [`reset_ark_usb/README.md`](local_ws/src/reset_ark_usb/README.md).

---

## Foxglove

Run the `foxglove_bridge` alias on the host or in the container and connect Studio to `ws://<device-ip>:8765`. The URDF is on `/robot_description` with panel frame `base_link`.

---

## Smoke test

`local_test` exercises the host stack:

```bash
local_test
```

It checks that the services are active, the aliases resolve, the Foxglove port answers, both `cam_front` and `cam_down` complete a full lifecycle (frame ID, rate, `/alive`), the supervisor is on the graph when the container is up, and no subprocesses leak. It is safe to run at any time and leaves the pipelines stopped. The bare alias prints only; menu **2** also tees to `log/smoke_test_log_*.log`.

---

## Camera feed check

`ver_cv_cams` opens the live CSI streams in a cv2 window over NoMachine. Any key advances, `q` quits. Pass `front` or `down` for a single feed:

```bash
ver_cv_cams
ver_cv_cams front
```

---

## Post-update validation

This is the pre-flight checklist to run after any pull, rebuild, or reprovision. Run it with the props off, the drone disarmed and on the ground, and stop the sequence at the first FAIL.

**1. Serials**

```bash
config_realsense
grep serial_no ~/workspaces/isaac_ros-dev/src/px4_vslam/config/vslam_config.yaml
```

Expect three distinct non-empty serials. Blank serials mean bringup cannot map cameras and the sentry idles with no watchdog.

**2. Submodules**

```bash
update_submods
```

Expect `All submodules verified.` A `commit is NOT on origin/<branch>` line means a fork has drifted: run `scripts/update_submods.sh --update`.

**3. Build**

```bash
colcon_local
colcon_isaac
```

`colcon_isaac` deinitializes first (refused unless landed), builds in-container, then restarts `arid_supervisor.service`. A build failure leaves the supervisor on old code with the stack down, so fix it before continuing. VSLAM does not come back automatically; `initialize` brings it up.

**4. Clean boot**

```bash
sudo reboot
```

Then, back on the drone:

```bash
systemctl is-active usbfs-memory start_isaac_docker arid_supervisor arid_description gst_camera_manager usb_ros_reset
local_test
```

Expect `active` five times and an all-PASS `local_test`. A failed supervisor at this point usually means the container workspace was never built.

**5. Bringup**

```bash
initialize
status
```

Expect `Response(success=True` then `vslam: running | land: landed`. A refusal carries verbatim driver evidence: repeated `Error starting device` points at camera or USB, `no factory exists` at a failed `image_transport` plugin load.

**6. Sentry**

```bash
sentry
ros2 topic echo --once --qos-durability transient_local /vslam_sentry/healthy
```

After the 40 s settle expect `"vslam":{"state":"OK"` with all three cameras `HEALTHY`, and `data: true`. `SETTLING` past a minute means the streams never reached `min_hz`. `NO CAMERAS parsed` in `~/workspaces/isaac_ros-dev/run_logs/sentry/sentry.log` means blank serials: return to step 1. `ESCALATED` means the camera exhausted its resets; treat it as hardware.

**7. Reactor telemetry**

```bash
ros2 topic echo --once --qos-durability transient_local /reactor/cadence_gated
ros2 topic echo --once --qos-durability transient_local /reactor/cadence_gate_count
ros2 topic echo --once --qos-durability transient_local /reactor/vo_healthy
```

Expect `data: false`, `data: 0` and `data: true` on a healthy idle stack. All three are latched, so a late subscriber still reads the state. A non-zero gate count before flight means cadence starvation has already occurred: find the cause first.

**8. Airborne gate, exercised on the ground**

```bash
ros2 topic echo --once --qos-reliability best_effort --qos-durability volatile \
  /fmu/out/vehicle_land_detected px4_msgs/msg/VehicleLandDetected | grep '^landed'
docker exec -u admin isaac_ros_dev-aarch64-container \
  bash -lc /workspaces/isaac_ros-dev/container_scripts/airborne_check.sh; echo "exit=$?"
```

Expect `landed: true`, then `airborne_check: landed` with `exit=1`. Exit 1 states that flight is not proven, which is what permits a reap. `no publisher on /fmu/out/vehicle_land_detected` also exits 1 and means the PX4 uXRCE-DDS bridge is down: fix it, or the land interlock has nothing to gate on. `AIRBORNE - stack preserved` on the ground is a FAIL, because a supervisor stop would then leave an orphan holding the cameras.

**9. Supervisor restart and orphan reap**

```bash
sudo systemctl restart arid_supervisor.service
sleep 25
pgrep -af 'ros2 launch px4_vslam vslam[.]launch[.]py'
status
```

Expect no `pgrep` match and `vslam: stopped`. A match means the reap failed and those processes still hold the cameras. More than 5 supervisor restarts in 60 s leaves the unit failed: read `journalctl -u arid_supervisor -n 100`.

**10. Reap before spinup**

```bash
initialize
sentry
```

Expect success even if step 9 left a tree behind, since landed is proven and the supervisor reaps the unowned tree before a fresh bringup. `unowned vslam trees present ... land state not proven` means the publisher from step 8 is missing: fix that first, then retry.

---

## setup.sh

Running `setup.sh` with no arguments opens the interactive menu:

```bash
./setup.sh
```

Every step detects what is already in place and does only what is missing, so re-running is safe. Each run logs to `log/setup_log_*.log`, and Ctrl+C offers quit-or-continue.

| Option | Action |
|---|---|
| **1** | Full setup |
| **2** | Smoke test |
| **3** | RealSense serial assignment |
| **4** | Camera feed check |
| **5** | Wi-Fi connect |
| **6** | Camera calibration |
| **7** | Camera focus |
| **8** | Build the Isaac container |
| **9** | Build the Isaac workspace and restart `arid_supervisor.service` |
| **10** | Build `local_ws` |
| **11** | ZeroTier join or switch |
| **12** | Uninstall, keeping the repo, the OS and the Docker engine |

Option 9 requires the container to be running.

### Camera calibration

`cam_calibrate` takes `front` or `down`; menu **6** prompts for the camera:

```bash
cam_calibrate front
```

[`camera_calibrate.sh`](local_ws/auxiliary/camera_calibration/camera_calibration_auto/camera_calibrate.sh) runs an interactive calibrator against the live pipeline and waits for a NoMachine session before starting. The default board is the included 10x7-square, 50 mm PDF. It writes the calibration to `config/calibrations/<cam>.yaml`; afterwards set `calibration: "cam_front"` or `"cam_down"` in `pipelines.yaml`. Details: [`camera_calibration/README.md`](local_ws/auxiliary/camera_calibration/README.md).

### Camera focus

Menu **7** starts the selected pipelines plus the Foxglove bridge and confirms each stream is sustained. Watch `/<cam>/image_raw/compressed` in Studio while adjusting the lens; any key tears it down.

---

## Full setup

`--full` walks the questionnaire once, then runs unattended. If anything was built, setup reboots once and finishes with the smoke test. To resume afterwards, open a bash terminal and answer the continue prompt. `--resume` invokes that continuation manually, and `--continue` re-enters the tail after an abort, skipping every checkpointed step.

```bash
./setup.sh --full
./setup.sh --resume
./setup.sh --continue
```

### Steps

The pre-tail steps run once per invocation. The checkpointed tail is Phase A (install and host configuration) then Phase B (builds and live camera steps).

| Step | Does |
|---|---|
| **preflight** | Checks non-root, git present, submodules initialized. |
| **collect_answers** | Questionnaire, persisted for the resume. |
| **power** | nvpmodel maximum; apt-holds critical L4T packages. |
| **first_boot** | One-time hostname and password. |
| **ensure_wifi** | Joins the network from the questionnaire. |
| **nomachine** | Installs or upgrades the arm64 package. |
| **enable_user_linger** | Creates `/run/user/<uid>` at boot for headless NoMachine. |
| **clean_nvidia_desktop** | Removes NVIDIA first-boot icons and the L4T-README automount. |
| **disable_updates** | Turns off unattended-upgrades and the apt timers. |
| *Phase A* | |
| **repos** | ROS, NVIDIA and Docker apt repos; CDI configuration. |
| **apt** | Apt packages: chrony, Foxglove bridge, OpenCV, calibrator, net tools. |
| **clock_sync** | chrony on, systemd-timesyncd off. |
| **zerotier** | Installs the daemon and joins. `ACCESS_DENIED` means authorize later. |
| **px4_deps** | PX4 Python dependencies and toolchain on demand. |
| **git** | Credential cache, script permissions, submodule pin verify, `run_logs` symlink. |
| **docker_patches** | Injects `Dockerfile.arid`, `arid_env.sh` and `run_dev.sh` into `isaac_ros_common`. |
| **skip_worktree** | Protects per-deployment camera calibrations. |
| **bashrc** | Rewrites the ARID block: exports, aliases, `help`, resume hook. |
| **permissions** | Sudoers, udev, polkit, groups. |
| **uhubctl** | Builds from source if missing. |
| **ros_workspace** | Python dependencies, rosdep, colcon build of `local_ws`. |
| **docker** | Engine, NVIDIA runtime, docker group, buildx. |
| **systemd** | Installs every shipped unit and enables all but `reset_usb.service`. |
| **realsense** | Reseeds `vslam_config.yaml`, then assigns the three serials. |
| *Phase B* | |
| **build_isaac** | Container image build, queued across a reboot. |
| **colcon_isaac** | Builds the container workspace, restarts `arid_supervisor.service`. |
| **verify_cameras** | Live front and down feeds, if opted in. |
| **camera_focus** | Foxglove focus pass over both CSI cameras, if opted in. |
| **summary, reboot, smoke test** | Summary, one reboot if anything built, then `local_test`. |

---

## PX4 firmware

The fork is at [`local_ws/auxiliary/PX4-Autopilot/`](local_ws/auxiliary/PX4-Autopilot/) on branch `PX4-InDro` and carries the `4026_arid_quad_v1_2` airframe. A built image for it is kept at [`px4_compiled/arid.px4`](local_ws/auxiliary/px4_compiled/), with `arid.bin` alongside it.

Building produces `build/ark_fmu-v6x_default/ark_fmu-v6x_default.px4`:

```bash
cd local_ws/auxiliary/PX4-Autopilot
make ark_fmu-v6x_default
```

Flashing is done from the ARK-OS web interface, which takes the `.px4` file directly.

Select the airframe from a MAVLink shell:

```
param set SYS_AUTOSTART 4026
param save
reboot
```

---

## Development

VSCode over Remote-SSH is the editor this repository is set up for: all code, builds and the live runtime stay on the drone.

```bash
ssh jetson@<device-ip>
```

On first open VSCode offers to install the workspace's recommended extensions on the drone. `.vscode/` carries the extension list, the Python and C++ lint and IntelliSense settings, and the search excludes.

For a virtual desktop, connect a NoMachine session to `<device-ip>`. The camera feed windows and the calibration GUI render on that desktop, so keep a session attached while using them.

The host workspace bind-mounts into the container at `/workspaces/isaac_ros-dev`. Build it with `colcon_isaac` from either side and enter it with `start_isaac` then `isaac_bash`. `local_ws` builds on the host with `colcon_local`. For in-container IntelliSense, attach VSCode to `isaac_ros_dev-aarch64-container` with Dev Containers.

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

---

## Robot description

`arid_description.service` runs `robot_state_publisher` on [`urdf/arid.xacro`](local_ws/src/arid_description/urdf/arid.xacro). The frames are `base_link`, `base_footprint`, `autopilot`, four propellers, `front/left/right_realsense_link`, `top_visual_link` for the front CSI camera, `bottom_visual_link` for the down CSI camera, `flow_link` and `rangefinder_link`. Visualization: [`arid_description/README.md`](local_ws/src/arid_description/README.md).

---

## Package reference

The two workspaces carry these first-party components.

| Package | Purpose |
|---|---|
| [`arid_description`](local_ws/src/arid_description/) | Xacro, meshes, RViz config. |
| [`ros_gst_cameras`](local_ws/src/ros_gst_cameras/) | CSI camera stack: `gst_cam_node` and `gst_camera_manager`. |
| [`reset_ark_usb`](local_ws/src/reset_ark_usb/) | `/reset_usb` service. |
| [`camera_calibration`](local_ws/auxiliary/camera_calibration/) | Front and down calibrator plus pattern. |
| [`px4_vslam`](isaac_ros-dev/src/px4_vslam/) | Three-camera VSLAM launch and PX4 bridge. |
| [`px4_vslam_reactor`](isaac_ros-dev/src/px4_vslam_reactor/) | VSLAM jump and cadence gating, re-seat. |
| [`vslam_sentry`](isaac_ros-dev/src/vslam_sentry/) | Camera and VO watchdog with per-camera hardware reset. |
| [`arid_supervisor`](isaac_ros-dev/src/arid_supervisor/) | VSLAM lifecycle service. |

[`update_submods.sh`](scripts/update_submods.sh) enforces two classes. LIVE entries are branch-tracked, so HEAD must sit on that remote branch. PINNED entries must sit on an exact tag and are re-checked out if they drift. `update_submods.sh --update` advances both.

| Submodule | Class | Role |
|---|---|---|
| [`PX4-Autopilot`](local_ws/auxiliary/PX4-Autopilot/) | LIVE `PX4-InDro` | PX4 fork with the ARID airframe. |
| [`realsense-ros`](isaac_ros-dev/src/realsense-ros/) | LIVE `v4.51.1` | RealSense driver: `hw_reset` service, hot-removal guards, `color_format`, claim retry. |
| [`isaac_ros_visual_slam`](isaac_ros-dev/src/isaac_ros_visual_slam/) | LIVE `v3.2-14` | cuVSLAM backend with the stale-tolerant image synchronizer. |
| [`px4_msgs`](isaac_ros-dev/src/px4_msgs/) | LIVE `release/1.15` | PX4 messages; `local_ws/src/px4_msgs` is a symlink to it. |
| [`isaac_ros_common`](isaac_ros-dev/src/isaac_ros_common/) | PINNED `v3.2-14` | Isaac base and Dockerfile chain. |
| [`isaac_ros_nitros`](isaac_ros-dev/src/isaac_ros_nitros/) | PINNED `v3.2-14` | Zero-copy transport. |
| [`isaac_ros_image_pipeline`](isaac_ros-dev/src/isaac_ros_image_pipeline/) | PINNED `v3.2-14` | GPU image processing. |
| [`px4-ros2-interface-lib`](isaac_ros-dev/src/px4-ros2-interface-lib/) | PINNED `1.4.0` | Auterion PX4 SDK. |
