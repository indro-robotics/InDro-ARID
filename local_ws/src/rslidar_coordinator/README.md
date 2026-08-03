# rslidar_coordinator

`rslidar_coordinator` supervises the RoboSense RSAIRY LiDAR. It runs `rslidar_sdk_node` as a subprocess under ROS services and reports frame flow on a latched `alive` topic.

## Startup

`rslidar_coordinator.service` runs the node from boot. The services and `/rslidar_coordinator/alive` exist from then; `/rslidar_points` appears only after an enable call.

Stopping the service terminates the SDK subprocess.

## Operator commands

| Command | Invokes | Result |
|---|---|---|
| `rslidar_start` | `/rslidar_coordinator/enable` `SetBool{true}` | forks `rslidar_sdk_node`; `/rslidar_points` appears; `alive` true |
| `rslidar_stop` | `/rslidar_coordinator/enable` `SetBool{false}` | subprocess terminated; `/rslidar_points` stops; `alive` false |
| `rslidar_restart` | `/rslidar_coordinator/restart` `Trigger` | stop then start; the response carries the new pid |
| `rslidar_status` | `/rslidar_coordinator/status` `Trigger` | `RUNNING (pid=N)` or `STOPPED` |
| `rslidar_alive` | echoes `/rslidar_coordinator/alive` | latched `data: true` or `data: false` |

## Services hosted

| Service | Type | Effect |
|---|---|---|
| `/rslidar_coordinator/enable` | `std_srvs/SetBool` | `true` forks the SDK subprocess; `false` terminates it and returns after it exits |
| `/rslidar_coordinator/status` | `std_srvs/Trigger` | `RUNNING (pid=N)` or `STOPPED` |
| `/rslidar_coordinator/restart` | `std_srvs/Trigger` | terminates the subprocess, then forks a new one |

The fork is `ros2 run rslidar_sdk rslidar_sdk_node --ros-args -p config_path:=/home/jetson/workspaces/local_ws/install/rslidar_coordinator/share/rslidar_coordinator/config/rslidar.yaml`.

The package calls no services.

## Published topics

| Topic | Type | Published by | Published | QoS |
|---|---|---|---|---|
| `/rslidar_coordinator/alive` | `std_msgs/Bool` | coordinator | node start as `false`, then every liveness transition | reliable, transient-local, depth 1 |
| `/rslidar_points` | `sensor_msgs/PointCloud2` | SDK subprocess | every scan while it runs, frame `rslidar_link` | reliable, volatile, depth 5 |

## Subscribed topics

| Topic | Type | QoS | Used for |
|---|---|---|---|
| `/rslidar_points` | `sensor_msgs/PointCloud2` | best-effort, depth 1 | the coordinator's evidence that clouds are flowing |

## Liveness

The watchdog evaluates the subprocess at 2 Hz.

| Event | `alive` | Log |
|---|---|---|
| node start | `false` | |
| subprocess forked | `true` within 0.5 s, from the seeded timer rather than a cloud | `Spawned rslidar_sdk_node`, `Frames flowing on /rslidar_points.` |
| cloud arrives | `true` | |
| no cloud for `alive_threshold` | `false` | `No frames on /rslidar_points for X.XXs` |
| subprocess exits | `false` | `rslidar_sdk_node exited unexpectedly (exit=N)` |

The timer is seeded at the fork, so `alive` is true for the first `alive_threshold` seconds whether or not a cloud arrives. The coordinator does not respawn the subprocess; recover with `rslidar_restart`.

> The topic is transient-local. `ros2 topic echo` without `--qos-durability transient_local` returns nothing until the next transition.

## Parameters

| Parameter | Default | Controls |
|---|---|---|
| `alive_threshold` | `5.0` | seconds without a cloud before `alive` goes false |
| `terminate_grace` | `3.0` | seconds between SIGTERM and SIGKILL when stopping the subprocess |

## SDK configuration

The coordinator passes the installed copy of `config/rslidar.yaml` as the SDK node's `config_path`, so an edit needs `colcon_local` before `rslidar_start`. The SDK's own `config/config.yaml` is not read.

| Key | Value | Effect |
|---|---|---|
| `msg_source` | `1` | packets come from the live LiDAR over UDP |
| `send_packet_ros` | `false` | raw packets are not published |
| `send_point_cloud_ros` | `true` | the point cloud is published |
| `lidar_type` | `RSAIRY` | sensor model the driver decodes |
| `msop_port` | `6699` | UDP port carrying point-cloud packets |
| `difop_port` | `7788` | UDP port carrying device info |
| `imu_port` | `6688` | socket bound; the SDK builds with IMU parsing off |
| `user_layer_bytes` | `0` | no user layer in the packet |
| `tail_layer_bytes` | `0` | no tail layer in the packet |
| `min_distance` | `0.2` | near cutoff in metres |
| `max_distance` | `200` | far cutoff in metres |
| `use_lidar_clock` | `false` | stamps the cloud with host ROS time |
| `dense_points` | `true` | NaN points discarded at the driver |
| `ts_first_point` | `false` | stamp is the end of the scan |
| `start_angle` / `end_angle` | `0` / `360` | full azimuth sweep |
| `ros_frame_id` | `rslidar_link` | frame stamped on the cloud, fixed under `base_link` in `arid_description` |
| `ros_send_point_cloud_topic` | `/rslidar_points` | cloud topic |
| `ros_recv_packet_topic` / `ros_send_packet_topic` | `/rslidar_packets` | unused; packet publishing is off |
| `ros_send_imu_data_topic` | `/rslidar_imu_data` | unused; no IMU parser in the build |
| `ros_queue_length` | `5` | depth of the cloud publisher queue |

## Troubleshooting

Both the coordinator and the SDK subprocess log to `journalctl -u rslidar_coordinator`.

| Symptom | Cause | Action |
|---|---|---|
| `status` reports `RUNNING`, `alive` false after `alive_threshold` | no cloud packets reaching `enP8p1s0` | `lidar_diag`; after a LiDAR swap, `config_lidar` |
| `rslidar_sdk_node exited unexpectedly (exit=N)` | the subprocess exited and is not respawned | `rslidar_restart` |
