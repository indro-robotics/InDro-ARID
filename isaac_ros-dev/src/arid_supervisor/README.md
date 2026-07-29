# arid_supervisor

The supervisor is the always-on lifecycle manager for the ARID VSLAM stack. It is one long-lived
ROS 2 node, started at boot by `arid_supervisor.service` inside the Isaac container, and it runs
`px4_vslam` as a managed subprocess behind a camera-proven bringup gate and a landed-state
interlock. Every entry point (`initialize`, `deinitialize`, a manual `ros2 service call`) goes
through it.

The node manages process lifecycle only. It never commands the drone and never fuses state.

## Services

Both services are hosted under the `arid_supervisor` node namespace.

| Service | Type | Purpose |
| --- | --- | --- |
| `~/vslam_enable` | `std_srvs/SetBool` | `true` runs the camera-proven bringup; `false` tears down, landed-gated. |
| `~/status` | `std_srvs/Trigger` | `success` is vslam running; the message adds land state. |

The node subscribes `/fmu/out/vehicle_land_detected` for the interlock. Subprocess output goes to
`/workspaces/isaac_ros-dev/run_logs/<name>/<name>.log`, truncated per launch.

## Bringup gate

`vslam_enable=true` returns `success=true` only once the front RealSense is up. It blocks the
caller about 15 s on a healthy bringup and up to about 3 min on a double failure.

1. USB pre-check. The RealSense must be on the bus, otherwise one `/reset_usb` and a recheck. If
   it is still absent the stack is never launched and the response carries per-device USB evidence.
2. Log watch. Success on `RealSense Node Is Up!`; fail-fast on `Error starting device` (terminal)
   and on `no factory exists` (an `image_transport` plugin race where markers print but no images
   flow); a 40 s backstop catches silent hangs.
3. One recovery cycle on failure: teardown, `/reset_usb`, respawn, re-watch. A second failure stops
   the stack and returns `success=false`. There is no retry ladder.

Failure messages carry verbatim driver evidence clipped to about 500 characters, with the full
detail in the node journal. Relay them unchanged.

> `/reset_usb` power-cycles the camera USB hub and pulses the FMU reset line. Use it at bringup
> only, with the drone disarmed on the ground.

## Idempotency

- `enable=true` with the stack running is a no-op returning `success=true` and the message
  `vslam already running (up <N>s, 1/1 camera at bringup)`.
- `enable=false` with nothing running is a no-op returning `success=true`.
- A second enable queues behind an in-flight bringup and resolves to the no-op, so a double spawn
  cannot occur.
- Unowned trees, left by a direct `ros2 launch px4_vslam vslam.launch.py` or by a crashed
  supervisor, are reaped on both paths when the drone is provably landed. Airborne or unknown land
  state refuses the call and names the colliding processes.

## Interlock

`vslam_enable=false` requires a fresh `landed == True` sample; a stale or airborne sample refuses
the disable. The supervisor never force-disarms and never lands the drone.

Teardown SIGINTs the whole process group and waits for it to drain, escalating to SIGTERM and then
SIGKILL only on a stall. A terminated `ros2 launch` leader is not an escape: the group id is
remembered at spawn, so surviving container children are still reaped.

A supervisor stop while the drone is provably airborne leaves vslam running. Land, then
`deinitialize` or `initialize`.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `rs_usb_pids` | D43X family | USB product ids the pre-check accepts as a RealSense. |

Pin the exact product id once confirmed with `cat /sys/bus/usb/devices/*/idProduct`.

## systemd unit

`isaac_ros-dev/services/arid_supervisor.service` runs the launch through `docker exec` into
`isaac_ros_dev-aarch64-container` as user `admin`, ordered after and bound to
`start_isaac_docker.service`.

`ExecStopPost` runs on every stop path including a crash. `airborne_check.sh` exiting 0 means
proven flight and preserves the stack; anything else runs `reap_stack.sh`, whose pattern is
qualified to `ros2 launch px4_vslam vslam.launch.py` so the host-side `rslidar_coordinator`,
`gst_camera_manager` and `arid_description` units are untouched. The unit respawns within 5 s of
any exit, and more than 5 restarts in 60 s leaves it failed.

Restart the unit after any change to the node, its launch graphs, or the workspace build:

```bash
sudo systemctl restart arid_supervisor.service
sudo systemctl status arid_supervisor.service
```

The node is installed with `--symlink-install`, so a Python edit needs only the restart while new
files, `setup.py` changes, or launch-graph changes need `colcon_isaac` first.

Confirm the supervisor is live:

```bash
ros2 service call /arid_supervisor/status std_srvs/srv/Trigger "{}"
```
