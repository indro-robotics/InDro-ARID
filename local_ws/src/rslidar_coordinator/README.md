# rslidar_coordinator

This package supervises the RoboSense RSAIRY 3-D LiDAR. It runs `rslidar_sdk_node` as a managed
subprocess and passes this package's SDK config to it through the `config_path` ROS parameter at
spawn time.

The pipeline is idle at boot and starts on demand through a `SetBool` service. Stop, status and
restart are available to any ROS client, and a latched `/alive` Bool carries frame-flow health to
subscribers using TRANSIENT_LOCAL QoS.

| Component | Detail |
|---|---|
| Sensor | RoboSense RSAIRY, Ethernet, solid-state hemispheric scanner. |
| Driver | `rslidar_sdk_node`, from the unpatched upstream `rslidar_sdk` submodule. |
| Coordinator | `rslidar_coordinator_node.py`. |
| Cloud topic | `/rslidar_points`, `sensor_msgs/PointCloud2`, BEST_EFFORT, frame `rslidar_link`. |
| IMU topic | Not published. The SDK build has IMU parsing compiled out. |
| Mount frame | `rslidar_link`, a fixed-joint child of `base_link`. |

`rslidar_coordinator.service` launches the coordinator at boot. From boot the services and
`/alive` exist; `/rslidar_points` appears only once the SDK subprocess is enabled.

---

## Service interface

| Service | Type | What it does |
|---|---|---|
| `/rslidar_coordinator/enable` | `std_srvs/SetBool` | `true` spawns `rslidar_sdk_node`; `false` terminates the process group and returns once it has exited. |
| `/rslidar_coordinator/status` | `std_srvs/Trigger` | Returns `RUNNING (pid=N)` or `STOPPED`. |
| `/rslidar_coordinator/restart` | `std_srvs/Trigger` | Stop, then start. |

```bash
ros2 service call /rslidar_coordinator/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /rslidar_coordinator/enable std_srvs/srv/SetBool '{data: false}'
ros2 service call /rslidar_coordinator/status std_srvs/srv/Trigger '{}'
```

The host aliases `rslidar_start`, `rslidar_stop`, `rslidar_status`, `rslidar_alive` and
`rslidar_restart` wrap the same calls.

---

## Watchdog

`/rslidar_coordinator/alive` is a latched `Bool` (TRANSIENT_LOCAL, depth 1) ticked at 2 Hz. It
flips false in two cases:

1. The subprocess exited, logged as `rslidar_sdk_node died (exit=N)`.
2. No `/rslidar_points` message arrived within `alive_threshold` seconds, logged as
   `No frames on /rslidar_points for X.XXs`.

It returns to true when frames resume. The timer is seeded at subprocess spawn, so the first
`alive_threshold` seconds after an enable act as a startup grace window.

There is no auto-restart. On subprocess death the coordinator logs and flips `/alive` false;
recovery is explicit through `rslidar_start` or `rslidar_restart`.

---

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `alive_threshold` | `5.0` s | Cloud-topic silence before `/alive` flips false. |
| `terminate_grace` | `3.0` s | SIGTERM-to-SIGKILL grace on stop. |

Override at launch:

```bash
ros2 run rslidar_coordinator rslidar_coordinator_node --ros-args -p alive_threshold:=2.0
```

---

## SDK config

`config/rslidar.yaml` is handed to the SDK node at spawn.

| Field | Value | Notes |
|---|---|---|
| `lidar_type` | `RSAIRY` | |
| `msop_port` | `6699` | Point-cloud packets. |
| `difop_port` | `7788` | Device info. |
| `imu_port` | `6688` | Socket bound only; no IMU messages are produced. |
| `min_distance` | `0.2` m | Near-range cutoff. |
| `max_distance` | `200` m | Far-range cutoff. |
| `use_lidar_clock` | `false` | Host ROS time; `true` needs PTP sync or TF lookups fail. |
| `dense_points` | `true` | NaN points stripped at the source. |
| `ts_first_point` | `false` | Timestamp is the end of scan. |
| `ros_frame_id` | `rslidar_link` | Matches the xacro. |
| `ros_send_point_cloud_topic` | `/rslidar_points` | |
| `ros_queue_length` | `5` | Publisher queue depth, matching sensor_data QoS. |

`ros_recv_packet_topic`, `ros_send_packet_topic` and `ros_send_imu_data_topic` are set but carry no
data: packet publishing is off and there is no IMU parser.

---

## Scope

The coordinator publishes clouds and nothing else. It broadcasts no TF, runs no LIO fusion, does
not re-frame the cloud out of `rslidar_link`, and supports a single LiDAR. Consumers transform
through the TF tree if they need another frame.

---

## Operational reference

| Command | Action |
|---|---|
| `systemctl status rslidar_coordinator` | Coordinator service state. |
| `rslidar_start` / `rslidar_stop` | Start / stop the SDK subprocess. |
| `ros2 topic echo --once --qos-reliability best_effort /rslidar_points --field header` | Verify cloud flow and frame. |
| `rslidar_alive` | Read the latched alive Bool. |
| `journalctl -u rslidar_coordinator -f` | Live logs. |
| `lidar_diag` | Full network and runtime diagnostic; `sudo` adds the sniff and scan legs. |
| `config_lidar` | Re-detect the LiDAR addresses after a swap or reconfiguration. |

The LiDAR is an Ethernet device on `enP8p1s0`; without a configured link and a powered LiDAR the
subprocess starts but no frames arrive.

---

## Dependencies

ROS packages are declared in [`package.xml`](package.xml): `rclpy`, `std_msgs`, `std_srvs`,
`sensor_msgs`, and `rslidar_sdk` as the supervised driver.

## License

Apache-2.0.
