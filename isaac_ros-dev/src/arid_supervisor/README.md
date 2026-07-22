# arid_supervisor

Always-on lifecycle owner for the ARID VSLAM stack. A single long-lived ROS 2 node,
started at boot by `arid_supervisor.service` inside the Isaac container, that starts and
stops `px4_vslam` as a managed subprocess behind a landed-state safety interlock and a
camera-proven bringup gate. Every operator entry point (the `initialize` and `deinitialize`
scripts, and manual `ros2 service call`) drives the stack through this node, so all callers
inherit the same protection.

The node never controls the drone and never fuses state. It owns process lifecycle only.

## Services

Hosted under the `arid_supervisor` node namespace.

| Service | Type | Purpose |
| --- | --- | --- |
| `~/vslam_enable` | `std_srvs/SetBool` | `true` runs the camera-proven vslam bringup gate; `false` tears vslam down (landed-gated). |
| `~/status` | `std_srvs/Trigger` | Side-effect-free health query: `success` = vslam running; message adds freshest land state. |

The node also subscribes to `/fmu/out/vehicle_land_detected` to drive the land interlock.

Per-run subprocess output is written to
`/workspaces/isaac_ros-dev/run_logs/<name>/<name>.log`, truncated on each launch.

## Camera-proven vslam bringup

`vslam_enable=true` returns `success=true` only once all 3 RealSense (front/left/right) are
actually up. The gate lives in the handler so no caller can skip it. Healthy bringup blocks
the caller about 15 s; a double failure blocks up to about 3 min.

Pre-check: count RealSense devices (VID `8086`, one of the D43X PIDs, matched via the
`rs_usb_pids` ROS parameter) on `/sys/bus/usb/devices`. If any camera is short, issue
one `/reset_usb` and wait for re-enumeration. Still short means the stack is never launched
and the response carries the per-device USB evidence.

Watch: tail the per-launch-truncated vslam log. Success once 3 distinct cameras (their unique
per-camera node tags `left_/front_/right_realsense.*_realsense_link`, not raw marker count)
emit `RealSense Node Is Up!`. Fail-fast the moment `Error starting device` appears (terminal
per camera; the upstream retry patch is reverted) or the launch process dies. A 40 s backstop
covers silent hangs.

Recovery: on gate failure, run exactly one recovery cycle: teardown, `/reset_usb`, respawn,
re-watch. A second failure stops the stack and returns `success=false`. There is no retry
ladder.

Response contract: the `message` field carries verbatim evidence, clipped to about 500 chars
with full detail in the node journal. Success reports camera count and elapsed time. Failure
reports the driver's terminal `Error starting device` lines, which cameras did come up, and
the last WARN/ERROR lines. Callers relay this message unchanged.

`/reset_usb` is invoked as a `ros2 service call` subprocess, not an rclpy client call: this
node spins under the default single-threaded executor, so a synchronous client call from
inside a service callback would deadlock. On ARID `/reset_usb` (`reset_ark_usb` -> `uhubctl`
+ GPIO85) power-cycles the ARK PAB USB hub the RealSense cameras sit on, so it is only ever
issued here, during pre-mission bringup with the drone disarmed on the ground.

## Idempotency and reentrancy

`vslam_enable=true` on a live supervisor-owned stack is a no-op: the stack is untouched,
`success=true`, message `vslam already running (up <N>s, 3/3 cameras at bringup)`. The
no-op is logged so it stays visible to post-run forensics.

`vslam_enable=false` with nothing running is a `success=true` no-op.

Double-spawn is impossible. Every callback runs in the node's default mutually-exclusive
callback group under the single-threaded executor, so a second `enable(true)` cannot
interleave with an in-flight bringup: it queues on the executor and lands in the
already-running no-op once the first returns. An internal lock preserves this guarantee even
if the executor model changes. If bringup aborts on an unexpected exception, the unproven
stack is stopped (and tracking dropped if even that fails) so the next call cannot falsely
no-op as already running.

Legacy stacks are refused. A vslam graph not owned by this supervisor (a direct
`ros2 launch px4_vslam vslam.launch.py`) is detected pre-spawn via the graph node list and
refused with the colliding node names, because spawning over it only fails later mid-gate on
node and camera collisions. A freshly-crashed foreign stack can linger until its DDS lease
expires; the refusal message says to wait about 10 s and retry.

## Interlock

- `vslam_enable=false` requires a fresh `VehicleLandDetected.landed == True` sample. If land
  state is stale or the drone is airborne, the disable is refused with `success=false`. The
  supervisor never force-disarms and never lands the drone.

Teardown SIGINTs the whole process group and waits for it to drain (nodes release their DDS
shm), escalating to SIGTERM then SIGKILL only on stall. It reaps orphaned setsid pipeline
groups (which survive a mid-teardown parent death otherwise) and reclaims only the
per-participant DDS shm segments the torn-down stack owned, never the domain-global port
segments other participants co-map.

## systemd unit

The unit is `arid_supervisor.service`, tracked at `isaac_ros-dev/services/`. It runs
`ros2 launch arid_supervisor arid_supervisor.launch.py` via `docker exec` into
`isaac_ros_dev-aarch64-container` as user `admin` (uid 1000, so control-service calls share
Fast-DDS shm with the host and dev shell). It is ordered after and bound to
`start_isaac_docker.service`, with an `ExecStartPre` that waits up to 60 s for the container
to report running.

`ExecStart` sets `-e PYTHONUNBUFFERED=1`. Piped stdout is block-buffered at 4 KiB, so short
INFO lines (idempotent no-ops, gate results) would otherwise sit unflushed for minutes and
blind journal forensics.

`ExecStop` uses `pkill -TERM -f 'arid_supervisor_node|arid_supervisor[.]launch[.]py'`. The
pattern targets both the launch parent and the node binary and nothing else. Two lessons are
baked into it. A broad bare `arid_supervisor` pattern also matches unrelated cmdlines in the
shared host PID namespace (a shell running `systemctl restart arid_supervisor.service`),
killing the invoking shell. Matching the launch file alone was proven insufficient on
2026-07-08: killing the launch parent orphaned the node, leaving duplicate DDS service
servers, so the node must receive the signal directly. `TimeoutStopSec` is 30 s so a
many-node rclpy graph drains before the next start collides on address-in-use.

`Restart=on-failure` with a 5-in-60 s start limit caps restart loops so a missing or misbuilt
package fails loudly instead of filling the journal.

Restart after any change to the node, its subprocesses' launch graphs, or the workspace
build:

```bash
sudo systemctl restart arid_supervisor.service
sudo systemctl status arid_supervisor.service
```

## Deployment

After pulling these changes onto another system, do all of the following before the
supervisor behaves as documented.

Rebuild the workspace so the new node and script entry points are installed. The build uses
symlink-install, so Python source edits take effect on a supervisor restart without a
rebuild, but any new file, changed `setup.py`, or changed launch graph needs the build:

```bash
colcon_isaac
```

Install the updated unit file and reload systemd. `setup.sh` copies
`isaac_ros-dev/services/*.service` into `/etc/systemd/system/`; to apply this unit alone
without a full setup run:

```bash
sudo cp -f /home/jetson/workspaces/isaac_ros-dev/services/arid_supervisor.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl restart arid_supervisor.service
```

Propagate `arid_env.sh`. `container_scripts/arid_env.sh` is the tracked source; `setup.sh`
regenerates `src/isaac_ros_common/docker/scripts/arid_env.sh` from it, and `Dockerfile.arid`
bakes that into the image at `/etc/profile.d/arid_env.sh`. A running container keeps the baked
copy until the image is rebuilt, so either rebuild the container image or copy the updated
file into the live container.

Set the per-drone RealSense serials. `px4_vslam/config/vslam_config.yaml` holds the D43X
serial numbers (`serial_no` in the `left_realsense`, `front_realsense`, and `right_realsense`
blocks) and is marked skip-worktree, so a pull does not overwrite the local values but a fresh
clone needs them filled for this drone. The camera-proven gate keys on `RealSense Node Is
Up!`; a wrong serial makes the driver claim the wrong device and the gate reports the mismatch
verbatim.

Confirm the RealSense USB PID. The `rs_usb_pids` ROS parameter defaults to the D43X
family (`0b07 0b3a 0b3d 0b64 0b5c`); pin the exact PID once confirmed on hardware with
`cat /sys/bus/usb/devices/*/idProduct` (VID is always `8086`).

Confirm the supervisor is live:

```bash
ros2 service call /arid_supervisor/status std_srvs/srv/Trigger "{}"
```
