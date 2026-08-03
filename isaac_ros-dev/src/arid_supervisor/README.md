# arid_supervisor

The supervisor starts and stops the `px4_vslam` launch tree as a managed subprocess, behind a camera-proven bringup gate and a landed-state interlock. It is one long-lived node named `arid_supervisor`, started at boot by `arid_supervisor.service`.

## Operator commands

| Command | Invokes | Result |
| --- | --- | --- |
| `initialize` | `/arid_supervisor/vslam_enable` with `data: true`, 300 s per call, 3 attempts | VSLAM launch tree on the graph |
| `deinitialize` | `/arid_supervisor/status`, then `vslam_enable` with `data: false` | launch tree stopped, its `/dev/shm` segments reclaimed |
| `status` | `/arid_supervisor/status` | response message carries vslam state and land state |
| `vslam` | `ros2 launch px4_vslam vslam.launch.py` directly | unowned tree; the supervisor reaps it when landed, refuses otherwise |

## VSLAM launch tree

| Node | Package |
| --- | --- |
| `/vslam_container` | `rclcpp_components` |
| `/front_realsense/front_realsense_link` | `realsense2_camera` |
| `/visual_slam_node` | `isaac_ros_visual_slam` |
| `/vio_transform/vio_transform` | `px4_vslam` |
| `/vslam_reactor` | `px4_vslam_reactor` |

The topics these nodes carry are documented in [`px4_vslam`](../px4_vslam/README.md).

## Services hosted

| Service | Type | Effect |
| --- | --- | --- |
| `/arid_supervisor/vslam_enable` | `std_srvs/srv/SetBool` | `data: true` runs the camera-proven bringup, `data: false` the landed-gated teardown |
| `/arid_supervisor/status` | `std_srvs/srv/Trigger` | `success` is true while the launch tree runs; message adds `land: landed`, `airborne` or `unknown` |

## Bringup

`vslam_enable` with `data: true` returns `success=true` only once one `RealSense Node Is Up!` marker per camera reaches the launch log.

| Stage | Action | Bound |
| --- | --- | --- |
| Unowned-tree check | `pgrep` on the launch cmdline plus `ros2 node list --no-daemon`; landed-proven trees reaped | 20 s scan, 10 s for DDS to drop them |
| USB pre-check | orphaned `/dev/bus/usb` nodes pruned, RealSense nodes set to `root:plugdev 0666`, devices counted | one `/reset_usb`, 20 s re-enumeration |
| Camera gate | tails the launch log for one marker per camera | 40 s backstop |
| Recovery | one teardown and relaunch; `/reset_usb` first only when a camera left the bus | single attempt |

The camera count is the number of `*_realsense` sections carrying `serial_no` in `px4_vslam/config/vslam_config.yaml`, read once at process start; unreadable falls back to one.

| Log line during the gate | Effect |
| --- | --- |
| `no factory exists` | fails at once: the `image_transport` publishers failed to construct and no image flows |
| `Error starting device` | carried into the failure report as evidence; the driver retries the claim |
| launch process exit | fails at once, with the count reached |

Failure responses carry verbatim driver evidence clipped to 500 characters. The full report is in the supervisor journal and in `/workspaces/isaac_ros-dev/run_logs/vslam/vslam.log`, truncated at each launch.

> `/reset_usb` power-cycles the camera hub and reboots the flight controller. Call `initialize` only on the ground, disarmed.

## Teardown

`vslam_enable` with `data: false` requires a `landed: true` sample under 3.5 s old. Teardown SIGINTs the process group and waits 25 s, escalates to SIGTERM for 5 s, then SIGKILLs.

- A `ros2 launch` leader that exited while its children persist is reaped through its remembered process group.
- Shared-memory segments mapped only by the torn-down tree are removed from `/dev/shm`.
- A supervisor stop with flight proven leaves the tree running.

## Topics

| Subscribed topic | Type | Use | QoS |
| --- | --- | --- | --- |
| `/fmu/out/vehicle_land_detected` | `px4_msgs/msg/VehicleLandDetected` | the landed interlock; a sample older than 3.5 s reads as unknown | best-effort, volatile, keep-last 5 |

The node publishes no topics.

## Services called

| Service | Type | When |
| --- | --- | --- |
| `/reset_usb` ([`reset_ark_usb`](../../../local_ws/src/reset_ark_usb/README.md)) | `std_srvs/srv/Trigger` | the pre-check or the recovery finds fewer RealSense on the bus than cameras |
| `reset_usb.service` | systemd unit over the mounted D-Bus socket | the `/reset_usb` call fails or returns `success=False` |

Both are subprocess calls, each capped at 30 s.

## Parameters

| Parameter | Default | Controls |
| --- | --- | --- |
| `rs_usb_pids` | `0b07`, `0b3a`, `0b3d`, `0b64`, `0b5c` | USB product ids counted as a RealSense, against vendor id `8086`; sampled once at construction |

Pin the exact id with `cat /sys/bus/usb/devices/*/idProduct`.

## Response messages

| Message | Cause | Action |
| --- | --- | --- |
| `vslam already running` | the stack is already up | none |
| `vslam already stopped` | no tree to stop | none |
| `unowned vslam trees reaped` | an orphan tree cleared under a fresh landed proof | none |
| `drone not landed; land before disabling vslam` | a fresh `landed: false` sample | land |
| `land state unknown (no recent PX4 telemetry)` | no sample within 3.5 s | restore the PX4 link |
| `unowned vslam trees present ... land state not proven` | orphan tree and no landed proof | land, then repeat the call |
| `refusing vslam bringup: vslam nodes already on the ROS graph` | land state unproven, or nodes outlive the reap | land, then repeat once discovery drops them |
| `camera(s) absent from the bus (dead VBUS/cable/port)` | still unenumerated after `/reset_usb` | check the cable, the port and VBUS |
| `vslam camera bringup failed twice (single-recovery policy)` | the gate failed on both attempts | read the two failure reports in the response |
| `vslam bringup aborted by unexpected exception` | the bringup raised; the unproven stack is stopped | read the exception in the response |

## systemd unit

`arid_supervisor.service` runs `ros2 launch arid_supervisor arid_supervisor.launch.py` through `docker exec -u admin` into `isaac_ros_dev-aarch64-container`.

- Restarts 5 s after any exit other than an explicit stop; 5 starts in 60 s leaves the unit failed.
- Stopping the unit tears the tree down unless flight is proven, and takes up to 150 s.
- Python edits to this package take effect on a unit restart; new files need `colcon_isaac`.

```bash
sudo systemctl restart arid_supervisor.service
```
