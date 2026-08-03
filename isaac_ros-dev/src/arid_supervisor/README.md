# arid_supervisor

`arid_supervisor` is the lifecycle manager for the ARID VSLAM stack. It hosts two services that start and stop `px4_vslam` as a subprocess behind a camera-proven bringup gate and a landed interlock.

## Services hosted

The executor is single-threaded. A second enable queues behind an in-flight bringup instead of spawning a second stack.

| Service | Type | Effect |
| --- | --- | --- |
| `/arid_supervisor/vslam_enable` | `std_srvs/SetBool` | `data: true` runs the gated bringup of `ros2 launch px4_vslam vslam.launch.py`. |
| `/arid_supervisor/vslam_enable` | `std_srvs/SetBool` | `data: false` tears that process group down, landed-gated. |
| `/arid_supervisor/status` | `std_srvs/Trigger` | `success` is true while the launch tree is alive. |

The enable blocks for the length of the bringup and the disable for the length of the teardown.

## Operator commands

The aliases are defined in `container_scripts/arid_env.sh` and run inside the Isaac container.

| Command | Invokes | Result |
| --- | --- | --- |
| `initialize` | `/arid_supervisor/vslam_enable` `{data: true}`, 300 s per call | up to three attempts, teardown between; prints the response message |
| `deinitialize` | `/arid_supervisor/status`, then `vslam_enable` `{data: false}` | refuses locally when vslam runs and land state is not `landed` |
| `status` | `/arid_supervisor/status` | `vslam: <running\|stopped> \| land: <landed\|airborne\|unknown>` |
| `vslam` | `ros2 launch px4_vslam vslam.launch.py` | unmanaged tree; the supervisor treats it as unowned |

## Bringup

`vslam_enable` with `data: true` returns true only once three distinct camera nodes have logged their up-marker. The count comes from the `*_realsense` sections of `px4_vslam/config/vslam_config.yaml` carrying a serial, and falls back to three.

| Stage | Action | On failure |
| --- | --- | --- |
| Unowned-tree guard | `pgrep` on the launch pattern; `ros2 node list --no-daemon` for `visual_slam`, `vslam_container` | landed: reap, 10 s DDS settle, rescan; otherwise refuse |
| USB pre-check | prune orphaned `/dev/bus/usb` nodes, restore RealSense nodes to `root:plugdev 0666`, count VID `8086` devices | fewer than 3: one `/reset_usb`, 20 s for re-enumeration; still short: refuse |
| Launch | `ros2 launch px4_vslam vslam.launch.py` in its own session, log truncated | leader exit during the gate ends the attempt |
| Camera gate | count distinct node tags logging `RealSense Node Is Up!` | `no factory exists`, launch exit, or the 40 s backstop ends the attempt |
| Recovery | one cycle: stop, `/reset_usb` only if a camera left the bus, repair nodes, relaunch | second gate failure stops the stack and returns `success=false` |

`Error starting device` in the log is evidence in the failure report, never a gate verdict.

> A vslam node whose process has exited stays on the ROS graph until DDS discovery drops it, and a bringup over it is refused. Retry after 10 s.

> `/reset_usb` power-cycles the camera USB hub and the standalone USB3 port, which reboots the flight controller. Call it with the drone disarmed on the ground.

## Teardown

`vslam_enable` with `data: false` requires a `landed` sample newer than 3.5 s. An older sample reads as unknown and refuses the call.

| Step | Bound |
| --- | --- |
| SIGINT the process group | 25 s to drain |
| SIGTERM on stall | 5 s |
| SIGKILL | none |

- Straggler `setsid` groups take the same ladder.
- The tree's own `/dev/shm/fastrtps_*` GUID segments are reclaimed once no live process maps them.
- `fastrtps_port*` segments are shared by every DDS participant and are never removed.
- A `ros2 launch` leader that exited while its children persist is reaped through its remembered process group.

Stopping the node while flight is proven leaves the vslam tree running. Land, then run `deinitialize`.

## Service responses

Evidence is clipped to 500 characters in the response. The full text is in the `arid_supervisor.service` journal.

| Call | `success` | Message |
| --- | --- | --- |
| `true`, stack already running | true | `vslam already running (up <N>s, 3/3 cameras at bringup)` |
| `true`, gate cleared | true | `vslam up: 3/3 cameras in <N>s` |
| `true`, cleared after the recovery cycle | true | `vslam up: 3/3 cameras in <N>s (after one recovery: <how>)` |
| `true`, both attempts failed | false | verbatim gate evidence from both attempts |
| `true`, unowned tree, land state not proven | false | `refusing vslam bringup: vslam nodes already on the ROS graph but NOT managed by this supervisor` |
| `false`, stack running and landed | true | `vslam stopped` |
| `false`, nothing running | true | `vslam already stopped` |
| `false`, unowned trees, landed | true | `unowned vslam trees reaped (<what>)` |
| `false`, unowned trees, land state not proven | false | `unowned vslam trees present (<what>) and land state not proven - land first` |
| `false`, airborne | false | `drone not landed; land before disabling vslam` |
| `false`, telemetry older than 3.5 s | false | `land state unknown (no recent PX4 telemetry); cannot disable vslam` |

## Topics

The node publishes no topics.

| Subscribed topic | Type | Use | QoS |
| --- | --- | --- | --- |
| `/fmu/out/vehicle_land_detected` | `px4_msgs/VehicleLandDetected` | `landed` flag for the interlock | best effort, volatile, keep last 5 |

## Services called

The call runs as a `ros2 service call` subprocess capped at 30 s, falling back to `systemctl start reset_usb.service`.

| Service | Type | When |
| --- | --- | --- |
| `/reset_usb` | `std_srvs/Trigger` | pre-check or recovery finds fewer than three RealSense on the bus |

## Parameters

Pin the exact product id with `cat /sys/bus/usb/devices/*/idProduct`.

| Parameter | Default | Controls |
| --- | --- | --- |
| `rs_usb_pids` | `0b07`, `0b3a`, `0b3d`, `0b64`, `0b5c` | USB product ids accepted as a RealSense under VID `8086` |

## Runtime

`arid_supervisor.service` starts the node in the Isaac container at boot. Launch output goes to `/workspaces/isaac_ros-dev/run_logs/vslam/vslam.log`, truncated on each launch.
