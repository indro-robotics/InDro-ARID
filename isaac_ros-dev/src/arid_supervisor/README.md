# arid_supervisor

Always-on lifecycle manager for the ARID VSLAM stack. One long-lived ROS 2 node, started at
boot by `arid_supervisor.service` inside the Isaac container. Starts and stops `px4_vslam`
as a managed subprocess behind a camera-proven bringup gate and a landed-state interlock.
Every entry point (`initialize`, `deinitialize`, manual `ros2 service call`) goes through it.

Never controls the drone, never fuses state. Process lifecycle only.

## Services

Hosted under the `arid_supervisor` node namespace.

| Service | Type | Purpose |
| --- | --- | --- |
| `~/vslam_enable` | `std_srvs/SetBool` | `true`: camera-proven bringup. `false`: teardown (landed-gated). |
| `~/status` | `std_srvs/Trigger` | `success` = vslam running; message adds land state. |

Subscribes `/fmu/out/vehicle_land_detected` for the interlock. Subprocess output:
`/workspaces/isaac_ros-dev/run_logs/<name>/<name>.log`, truncated per launch.

## Bringup gate

`vslam_enable=true` returns `success=true` only once all 3 RealSense (front/left/right) are
up. Blocks the caller about 15 s healthy, up to about 3 min on double failure.

Camera count = the `*_realsense` sections of `vslam_config.yaml` carrying a serial; blank
(unprovisioned) or unreadable falls back to 3.

0. Kill-before-spinup: unowned vslam trees are reaped first when landed is proven, refused
   otherwise.
1. USB pre-check: 3 RealSense on the bus, else one `/reset_usb` and recheck. Still short:
   stack never launched, response carries per-device USB evidence.
2. Log watch: success once 3 distinct cameras log `RealSense Node Is Up!`; fail-fast on
   `Error starting device` (terminal per camera) and on `no factory exists` (image_transport
   plugin load failed: markers still print, publishers dead); 40 s backstop for silent hangs.
3. One recovery cycle on failure: teardown, `/reset_usb`, respawn, re-watch. Second failure
   stops the stack, `success=false`. No retry ladder.

Failure messages carry verbatim driver evidence, clipped to about 500 chars (full detail in
the node journal). Relay them unchanged.

> `/reset_usb` power-cycles the camera USB hub and pulses the FMU reset line. Bringup only,
> drone disarmed on the ground; never issue it later in the mission.

## Idempotency

- `enable=true` with the stack running: no-op, `success=true`, message
  `vslam already running (up <N>s, 3/3 cameras at bringup)`.
- `enable=false` with nothing running: no-op, `success=true`.
- Double-spawn impossible: a second enable queues behind an in-flight bringup and resolves to
  the no-op.
- Foreign stacks (direct `ros2 launch px4_vslam vslam.launch.py`): reaped when landed is
  proven, else refused with the colliding node names. A recently crashed one persists until
  its DDS lease expires: wait about 10 s and retry.

## Interlock

`vslam_enable=false` requires a fresh `landed == True` sample; stale or airborne refuses
the disable. The supervisor never force-disarms and never lands the drone.

Teardown SIGINTs the whole process group and waits for it to drain, escalating to SIGTERM
then SIGKILL only on stall. A dead `ros2 launch` leader whose children survive is reaped by
its remembered process group, so `false` never answers "already stopped" over a live tree.

A supervisor stop while provably airborne leaves the stack running (log line, no teardown);
land, then `deinitialize` / `initialize`.

## systemd unit

`isaac_ros-dev/services/arid_supervisor.service` runs the launch via `docker exec` into
`isaac_ros_dev-aarch64-container` as user `admin`, ordered after and bound to
`start_isaac_docker.service`.

`Restart=always`: a node crash makes `ros2 launch` exit 0, so `on-failure` never fires.
`ExecStopPost` runs on every stop path incl. crash and is airborne-gated:
`container_scripts/airborne_check.sh` proves flight (stack preserved), otherwise
`container_scripts/reap_stack.sh` reaps the orphaned launch tree.

Restart after any change to the node, its launch graphs, or the workspace build:

```bash
sudo systemctl restart arid_supervisor.service
sudo systemctl status arid_supervisor.service
```

## Deployment

After pulling onto another system:

Rebuild the workspace. Symlink-install: Python edits need only a supervisor restart; new
files, `setup.py`, or launch-graph changes need the build:

```bash
colcon_isaac
```

Install the unit (`setup.sh` does this in a full run):

```bash
sudo cp -f /home/jetson/workspaces/isaac_ros-dev/services/arid_supervisor.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl restart arid_supervisor.service
```

Propagate `arid_env.sh`: `container_scripts/arid_env.sh` is the tracked source, built into
the image; rebuild the image or copy the file into the live container.

Set the per-drone RealSense serials: run `config_realsense` (writes
`px4_vslam/config/vslam_config.yaml`).

Confirm the RealSense USB PID: the `rs_usb_pids` parameter defaults to the D43X family;
pin the exact PID after checking `cat /sys/bus/usb/devices/*/idProduct`.

Confirm the supervisor is live:

```bash
ros2 service call /arid_supervisor/status std_srvs/srv/Trigger "{}"
```
