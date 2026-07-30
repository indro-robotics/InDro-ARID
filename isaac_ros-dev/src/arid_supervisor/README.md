# arid_supervisor

The supervisor is the lifecycle manager for the ARID VSLAM stack. It is one long-lived ROS 2 node, started at boot by `arid_supervisor.service` inside the Isaac container, and it starts and stops `px4_vslam` as a managed subprocess behind a camera-proven bringup gate and a landed-state interlock. Every entry point (`initialize`, `deinitialize`, a manual `ros2 service call`) goes through it.

It handles process lifecycle only: it does not command the drone and does not fuse state.

## Services

The services are hosted under the `arid_supervisor` node namespace.

| Service | Type | Purpose |
| --- | --- | --- |
| `~/vslam_enable` | `std_srvs/SetBool` | `true` runs the camera-proven bringup; `false` tears the stack down, landed-gated. |
| `~/status` | `std_srvs/Trigger` | `success` is true while vslam is running; the message adds land state. |

The node subscribes to `/fmu/out/vehicle_land_detected` for the interlock. Subprocess output goes to `/workspaces/isaac_ros-dev/run_logs/<name>/<name>.log`, truncated on each launch.

## Bringup gate

`vslam_enable=true` returns `success=true` only once every configured RealSense is up. It blocks the caller about 15 s on a healthy stack and up to about 3 minutes when a camera needs the recovery cycle. The camera count comes from the `*_realsense` sections of `vslam_config.yaml` that carry a serial; a blank or unreadable config falls back to 3.

1. Unowned vslam trees are reaped before spinup when landed is proven, and the call is refused otherwise.
2. USB pre-check: three RealSense must be on the bus, else one `/reset_usb` and a recheck. Still short, and the stack is never launched; the response carries per-device USB evidence.
3. Log watch: success once that many distinct cameras log `RealSense Node Is Up!`. It fails fast on `no factory exists` (the `image_transport` plugin load failed, so the markers still print but the publishers are dead) and on a dead launch process; a 40 s backstop bounds everything else. `Error starting device` does not fail the gate: the driver retries a lost claim every 6 s, so a camera can log it and still come up.
4. One recovery cycle on failure: teardown, `/reset_usb`, respawn, re-watch. A second failure stops the stack and returns `success=false`.

Failure messages carry verbatim driver evidence clipped to about 500 characters, with the full detail in the node journal. Relay them unchanged.

> `/reset_usb` power-cycles the camera USB hub and pulses the FMU reset line, so bringup is the only time it may run. Never issue it later in the mission.

## Idempotency

Repeat calls resolve without disturbing a running stack.

- `enable=true` with the stack running is a no-op returning `success=true` and `vslam already running (up <N>s, 3/3 cameras at bringup)`.
- `enable=false` with nothing running is a no-op returning `success=true`.
- A second enable queues behind an in-flight bringup and resolves to the no-op, so a double spawn cannot happen.
- Foreign stacks started by a direct `ros2 launch px4_vslam vslam.launch.py` are reaped when landed is proven and refused otherwise, with the colliding node names in the message. A recently crashed one persists until its DDS lease expires: wait about 10 s and retry.

## Interlock

`vslam_enable=false` requires a fresh `landed == True` sample; a stale or airborne sample refuses the disable. The supervisor never force-disarms and never lands the drone.

Teardown SIGINTs the whole process group and waits for it to drain, escalating to SIGTERM then SIGKILL only on a stall. A `ros2 launch` leader that has exited while its children persist is reaped by its remembered process group, so `false` never answers "already stopped" over a live tree.

A supervisor stop while flight is proven leaves the stack running and logs that it did; land, then use `deinitialize` or `initialize`.

## systemd unit

`arid_supervisor.service` runs the launch through `docker exec` into `isaac_ros_dev-aarch64-container` as user `admin`, ordered after and bound to `start_isaac_docker.service`. It restarts on every exit, and more than 5 restarts in 60 s leaves the unit failed.

`ExecStopPost` runs on every stop path, crash included, and is airborne-gated: `container_scripts/airborne_check.sh` decides whether flight is proven, and `container_scripts/reap_stack.sh` reaps the orphaned launch tree when it is not.

Restart the unit after any change to the node, its launch graphs, or the workspace build:

```bash
sudo systemctl restart arid_supervisor.service
sudo systemctl status arid_supervisor.service
```

The workspace is symlink-installed, so a Python edit needs only the restart. New files, `setup.py` changes and launch-graph changes need `colcon_isaac` first.

Confirm the supervisor is live:

```bash
ros2 service call /arid_supervisor/status std_srvs/srv/Trigger "{}"
```
