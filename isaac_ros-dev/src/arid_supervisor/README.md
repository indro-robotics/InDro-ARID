# arid_supervisor

The supervisor is the lifecycle manager for the ARID VSLAM stack. It is one long-lived ROS 2 node, started at boot by `arid_supervisor.service` inside the Isaac container, and it starts and stops `px4_vslam` as a managed subprocess behind a camera-proven bringup gate and a landed-state interlock. `initialize`, `deinitialize` and a manual `ros2 service call` all drive it through `vslam_enable`; a direct `ros2 launch px4_vslam vslam.launch.py` bypasses it and is handled as an unowned tree.

The node manages process lifecycle only. It has no publishers and issues no flight commands.

## Services

The supervisor hosts two services under its own node name.

| Service | Type | Purpose |
| --- | --- | --- |
| `/arid_supervisor/vslam_enable` | `std_srvs/SetBool` | `true` runs the camera-proven bringup; `false` tears the stack down, landed-gated. |
| `/arid_supervisor/status` | `std_srvs/Trigger` | `success` is true while vslam is running; the message adds land state. |

The node subscribes to `/fmu/out/vehicle_land_detected` for the interlock. Launch output goes to `/workspaces/isaac_ros-dev/run_logs/vslam/vslam.log`, truncated on each launch.

## Bringup gate

`vslam_enable=true` returns `success=true` only once all three RealSense (left, front, right) are up. It blocks the caller about 15 s on a healthy bringup and up to about 3 minutes when the recovery cycle runs. The camera count comes from the `*_realsense` sections of `px4_vslam/config/vslam_config.yaml` that carry a serial; a blank or unreadable config falls back to three.

1. Unowned vslam trees are reaped before spinup when landed is proven, and the call is refused otherwise.
2. USB pre-check. All three RealSense must be on the bus, otherwise one `/reset_usb` and a recheck. If a camera is still absent the stack is never launched and the response carries per-device USB evidence.
3. Log watch. The gate clears once three distinct camera nodes log `RealSense Node Is Up!`. It fails fast on `no factory exists` (the `image_transport` plugin load failed, so the markers still print while no image flows) and on a dead launch process; a 40 s backstop bounds everything else. `Error starting device` does not fail the gate: the driver retries a lost claim every 6 s, so a camera can log it and still come up.
4. One recovery cycle on failure: teardown, relaunch, re-watch. `/reset_usb` runs first only when a camera has left the bus; when every camera is still enumerated the failure is claim-side and the relaunch runs without a bus cycle. A second failure stops the stack and returns `success=false`.

Failure messages carry verbatim driver evidence clipped to about 500 characters, with the full detail in the `arid_supervisor.service` journal. Relay them unchanged.

> `/reset_usb` power-cycles the camera USB hub and the standalone USB3 port, which reboots the flight controller. Use it at bringup only, with the drone disarmed on the ground.

## Idempotency

Repeat calls resolve without disturbing a running stack.

- `enable=true` with the stack running is a no-op returning `success=true` and `vslam already running (up <N>s, 3/3 cameras at bringup)`.
- `enable=false` with nothing running is a no-op returning `success=true`.
- A second enable queues behind an in-flight bringup and resolves to the no-op, so a double spawn cannot occur.
- Unowned trees, left by a direct launch or by a crashed supervisor, are reaped on both paths when the drone is provably landed. Airborne or unknown land state refuses the call and names the offending nodes or process ids.

> A vslam node that terminated moments ago stays on the ROS graph until its DDS lease expires, and the supervisor refuses a bringup over a tree it has no process left to reap. Retry after about 10 s.

## Interlock

`vslam_enable=false` requires a `landed == True` sample no older than 3.5 s. An older sample reads as unknown and an airborne sample reads as flying; both refuse the disable.

Teardown SIGINTs the whole process group and waits up to 25 s for it to drain, escalating to SIGTERM for 5 s and then SIGKILL only on a stall. A `ros2 launch` leader that has exited while its children persist is reaped through its remembered process group, so `false` never answers "already stopped" over a live tree.

A supervisor stop while flight is proven leaves the stack running and logs that it did. Land, then use `deinitialize` or `initialize`.

## Parameters

The node declares one parameter.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `rs_usb_pids` | `0b07`, `0b3a`, `0b3d`, `0b64`, `0b5c` | USB product ids the pre-check accepts as a RealSense. |

Pin the exact product id once it is confirmed with `cat /sys/bus/usb/devices/*/idProduct`.

## systemd unit

`isaac_ros-dev/services/arid_supervisor.service` runs the launch through `docker exec` into `isaac_ros_dev-aarch64-container` as user `admin`, ordered after and bound to `start_isaac_docker.service`. It restarts 5 s after any exit other than a manual `systemctl stop`, and more than 5 starts in 60 s leaves the unit failed.

`ExecStopPost` runs on every stop path, crash included, and is airborne-gated: `container_scripts/airborne_check.sh` decides whether flight is proven, and `container_scripts/reap_stack.sh` reaps the orphaned launch tree when it is not. That reap is qualified to the vslam launch tree, so the other units on the machine are untouched.

The workspace is symlink-installed, so a Python edit takes effect on a unit restart. New files, `setup.py` changes and launch-graph changes need `colcon_isaac`, which tears the stack down, rebuilds and restarts the unit itself.

```bash
sudo systemctl restart arid_supervisor.service
sudo systemctl status arid_supervisor.service
```

Confirm the supervisor is live with the `status` alias or the call it wraps:

```bash
ros2 service call /arid_supervisor/status std_srvs/srv/Trigger "{}"
```
