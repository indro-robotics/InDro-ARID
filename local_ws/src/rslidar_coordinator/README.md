# rslidar_coordinator

Supervisor for the RoboSense RSAIRY 3-D LiDAR. The coordinator owns the SDK config and spawns `rslidar_sdk_node` (from the upstream [`rslidar_sdk`](https://github.com/RoboSense-LiDAR/rslidar_sdk) submodule) as a managed subprocess. The `rslidar_sdk` submodule is never patched; this package passes its own config to the SDK via the `config_path` ROS parameter at spawn time.

Mirrors the operational shape of [`gst_camera_manager`](../ros_gst_cameras/): the LiDAR pipeline is **idle at boot**, started on demand via a `SetBool` service. Clean stop, status, and restart from any ROS client, plus a latched `/alive` Bool other consumers can subscribe to with TRANSIENT_LOCAL QoS.

| Component | Detail |
|---|---|
| Sensor | RoboSense RSAIRY (Ethernet, solid-state hemispheric scanner) |
| Driver | `rslidar_sdk_node` (binary from `rslidar_sdk` submodule at tag `v1.5.19`; nested `rs_driver` at `v1.5.19`) |
| Messages | `rslidar_msg` submodule at tag `v1.5.10` (pure message definitions, no node or launch) |
| Coordinator | `rslidar_coordinator_node.py` (Python, this package) |
| Cloud topic | `/rslidar_points` (`sensor_msgs/PointCloud2`, BEST_EFFORT, frame `rslidar_link`) |
| IMU topic | **Not published.** IMU parsing is gated by the SDK's `ENABLE_IMU_DATA_PARSE` cmake flag, deliberately not flipped (keeps the submodule untouched). `imu_port: 0` in the YAML. |
| Mount frame | `rslidar_link` (fixed-joint child of `base_link` in `arid_description/xacro/arid.xacro`) |

---

## Build and launch

```bash
cd ~/workspaces/local_ws
colcon build --packages-select rslidar_coordinator --symlink-install
source install/setup.bash
ros2 launch rslidar_coordinator rslidar_coordinator.launch.py
```

The systemd unit `rslidar_coordinator.service` (installed by the workspace-level `setup.sh`) launches the coordinator on boot. The SDK subprocess is **not** spawned at boot. Topics that exist from boot: `/rslidar_coordinator/enable`, `/status`, `/restart`, and `/alive` (initially `false`). Topics that don't exist until enable: `/rslidar_points`.

---

## Service interface

| Service | Type | What it does |
|---|---|---|
| `/rslidar_coordinator/enable` | `std_srvs/SetBool` | `data: true` spawns `rslidar_sdk_node`. `data: false` SIGTERMs the process group (3 s grace), then SIGKILL fallback. Returns only after the SDK binary is genuinely gone. |
| `/rslidar_coordinator/status` | `std_srvs/Trigger` | Returns `RUNNING (pid=N)` or `STOPPED`. |
| `/rslidar_coordinator/restart` | `std_srvs/Trigger` | Stop then start. |

```bash
ros2 service call /rslidar_coordinator/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /rslidar_coordinator/enable std_srvs/srv/SetBool '{data: false}'
ros2 service call /rslidar_coordinator/status std_srvs/srv/Trigger '{}'
```

Host aliases (set by `setup.sh`'s bashrc step): `rslidar_start`, `rslidar_stop`, `rslidar_status`, `rslidar_alive`, `rslidar_restart`.

---

## Watchdog: `/rslidar_coordinator/alive`

Latched `Bool` on `/rslidar_coordinator/alive` (TRANSIENT_LOCAL, depth 1). Tick at 2 Hz (0.5 s timer). Flips `false` on:

1. **Process death.** `proc.poll()` returns a non-`None` exit code. Logs `rslidar_sdk_node died (exit=N)`.
2. **Stalled cloud.** No `/rslidar_points` message arrived within `alive_threshold` seconds (default `5.0`). Logs `No frames on /rslidar_points for X.XXs (threshold 5.0s)`.

Re-promotes to `true` when frames resume. The timer is seeded to `now()` at subprocess spawn, so the first `alive_threshold` seconds after `enable(true)` act as a startup grace window.

**No auto-restart.** On subprocess death the coordinator logs and flips `/alive=false`. Recovery is explicit (`rslidar_start` or `rslidar_restart`).

---

## Subprocess termination (process-group semantics)

The SDK chain is `ros2 run rslidar_sdk rslidar_sdk_node ...`. The `ros2` Python wrapper forks the actual binary. SIGTERM to the wrapper alone leaves the binary orphaned, especially when the SDK is busy looping on `ERRCODE_MSOPTIMEOUT`. The coordinator handles this by:

1. **Spawn:** `subprocess.Popen(cmd, preexec_fn=os.setsid)`. The child becomes the leader of a new session AND process group, with `pgid == pid`. `self.proc_pgid` is cached at spawn time so the group ID stays valid after the wrapper exits.
2. **Terminate:** `killpg(pgid, SIGTERM)` signals every process in the group. Then `_wait_pgroup_empty(pgid, terminate_grace)` polls.
3. **Wait loop:** each iteration does `self.proc.poll()` (reaps the wrapper's zombie; zombies count as group members under `killpg(0)`), then `killpg(pgid, 0)` (existence check). Raises `ProcessLookupError` when the group has no live members. Without the in-loop `poll()`, the wait never sees the group as empty because the zombie wrapper lingers.
4. **SIGKILL fallback:** if the group hasn't drained after `terminate_grace` (default 3 s), `killpg(pgid, SIGKILL)`, then another 2 s wait.

Net effect: `rslidar_stop` only returns "stopped" once the SDK binary is genuinely gone.

---

## Config: `config/rslidar.yaml`

This package owns the SDK config. The `rslidar_sdk` submodule is never patched. The coordinator passes the config's absolute install-share path via the SDK node's `config_path` ROS parameter at spawn:

```bash
ros2 run rslidar_sdk rslidar_sdk_node \
    --ros-args -p config_path:=<install/rslidar_coordinator/share/rslidar_coordinator/config/rslidar.yaml>
```

YAML summary:

| Field | Value | Notes |
|---|---|---|
| `lidar_type` | `RSAIRY` | |
| `msop_port` | `6699` | point-cloud packets |
| `difop_port` | `7788` | device info |
| `imu_port` | `0` | IMU disabled (SDK parser gated by compile-time flag) |
| `min_distance` | `0.5` m | filter out close-range prop reflections |
| `max_distance` | `20.0` m | adjust for op envelope |
| `use_lidar_clock` | `true` | timestamps from LiDAR clock |
| `dense_points` | `true` | strip NaN points at source |
| `ts_first_point` | `true` | timestamp uses first point in scan |
| `ros_frame_id` | `rslidar_link` | matches xacro |
| `ros_send_point_cloud_topic` | `/rslidar_points` | |
| `ros_queue_length` | `10` | publisher queue depth |

`pcap_*` fields are deliberately omitted (only used in `msg_source: 3` file-replay mode). `ros_recv_packet_topic` is omitted (this package publishes, does not subscribe). `ros_send_imu_data_topic` is omitted (no IMU).

This file is **not** marked skip-worktree. The values are deliberate defaults the team agreed on. Add skip-worktree later if site-specific overrides become a concern.

---

## Coordinator runtime parameters

Declared in `rslidar_coordinator_node.py` via `declare_parameter`:

| Parameter | Default | Meaning |
|---|---|---|
| `alive_threshold` | `5.0` s | Seconds of cloud-topic silence before `/alive` flips false. |
| `terminate_grace` | `3.0` s | SIGTERM-to-SIGKILL grace window. |

Override at launch:
```bash
ros2 run rslidar_coordinator rslidar_coordinator_node --ros-args -p alive_threshold:=2.0
```

---

## Network setup

RSAIRY is an Ethernet device. The workspace-level `setup.sh` (`lidar_network` step) handles all networking. Summary:

| Setting | Value |
|---|---|
| LiDAR IP | `192.168.1.200` (RoboSense factory default) |
| LiDAR destination IP | `192.168.1.102` (Jetson side) |
| Subnet | `/24` (255.255.255.0) |
| MSOP port | UDP `6699` (point cloud) |
| DIFOP port | UDP `7788` (device info) |
| Jetson NIC | `enP8p1s0` (ARK PAB built-in Ethernet) |
| NM connection | `rslidar` (manual, priority 10, with explicit `ipv4.routes`) |
| Fallback NM connection | `dev` (DHCP, priority 0, `dhcp-timeout 8`) |
| Kernel UDP buffer | `net.core.rmem_max` = `net.core.rmem_default` = 25 MiB (`/etc/sysctl.d/99-rslidar.conf`) |

The NM dispatcher at `/etc/NetworkManager/dispatcher.d/90-rslidar` ARP-probes `192.168.1.200` from source `192.168.1.102` for up to 8 s on link-up. If the LiDAR responds the static profile stays active. Otherwise the system falls back to DHCP. Useful for hot-swapping between LiDAR and router on the same NIC.

For LiDAR network diagnostics, run the host-side `lidar_diag` script (alias points at `scripts/lidar_diag.sh`).

---

## Scoped out

This package intentionally does NOT:

- Publish TF. Body orientation comes from PX4 and VSLAM. The LiDAR IMU does not enter the TF tree.
- Run FAST-LIO or any LIO fusion. Clouds are published; downstream consumers (if any) subscribe directly.
- Republish or re-frame the cloud. Stays in `rslidar_link`. Consumers transform via TF if needed.
- Use CycloneDDS. The workspace is FastDDS-only, system-default profile.
- Support multiple LiDARs. Single-LiDAR scope by design.

---

## Operational reference

```bash
# Verify the coordinator service is up
systemctl status rslidar_coordinator

# Start
rslidar_start

# Verify cloud is flowing in the right frame
ros2 topic echo --once --qos-reliability best_effort /rslidar_points --field header

# Read the latched alive Bool
rslidar_alive

# Watch logs live
journalctl -u rslidar_coordinator -f

# Stop
rslidar_stop

# Run the diagnostic when something looks wrong
lidar_diag
sudo lidar_diag
```

---

## Dependencies

**ROS packages** (declared in [`package.xml`](package.xml)):

- `rclpy`: Python ROS 2 client library.
- `std_msgs`, `std_srvs`, `sensor_msgs`: service and message types.
- `rslidar_sdk`: the driver this package supervises (exec_depend; built first so the binary exists at runtime).

**Runtime:**

- `rslidar_coordinator.service` (installed by `setup.sh`) launches the coordinator at boot.
- LiDAR network configured (handled by `setup.sh`'s `lidar_network` step).
- LiDAR powered and wired to `enP8p1s0` for actual data flow.

## License

Apache-2.0.
