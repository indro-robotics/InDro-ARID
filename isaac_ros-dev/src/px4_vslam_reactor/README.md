# px4_vslam_reactor

The reactor gates the cuVSLAM odometry stream into the PX4 bridge. It withholds frames carrying tracking jumps, re-seats cuVSLAM onto the FMU pose, and republishes the FMU pose in ROS frame conventions.

## Startup

The [`px4_vslam`](../px4_vslam/) stack launch starts the executable `vslam_reactor_node` as node `vslam_reactor` and applies [`config/px4_vslam_reactor.yaml`](config/px4_vslam_reactor.yaml).

> Construction blocks until `visual_slam/set_slam_pose` is advertised, logging `waiting for visual_slam/set_slam_pose service...` every 2 s.

## Frame admission

Each frame from `/visual_slam/vis/slam_odometry` passes the stages below in order.

| Stage | Condition | Effect |
| --- | --- | --- |
| Ingress | Non-finite field or zero-norm quaternion | Frame dropped |
| Tracking check | `vo_state` is not 1, or a re-seat is in flight | Frame dropped |
| Origin seat | First tracked frame while `cs_ev_pos` is false | `SetSlamPose` at zero position, zero yaw, FMU roll and pitch |
| Pre-fusion stream | `cs_ev_pos` false | Frame published, jump gate not applied |
| Jump gate | Linear velocity >= `lin_vel_gate`, angular >= `ang_vel_gate_dps`, or step > `VO_pos_delta_lim` across > `VO_rate_lim` | Frame withheld, `SetSlamPose` at the current FMU pose |
| Bypass window | Within `vslam_stabilization_time` and 2 rebased frames of a committed re-seat, capped at 3 s | Jump gate skipped, baseline rebased; frames stamped before the commit withheld |
| Settle gate | Yaw error >= `align_yaw_deg` or position error >= `align_pos_m` against `/reactor/drone_odom` | Frame published for `set_origin_settle_time` |
| Settle timeout, origin seat | Window elapses without alignment | Origin re-injected |
| Settle timeout, jump re-seat | Window elapses without alignment | Alignment abandoned, streaming continues |
| Re-seat freshness | Cached FMU sample older than `set_pose_max_odom_age` | Re-seat skipped, retried on the next frame |
| Burst limit | `reseat_burst_max` jump re-seats inside `reseat_burst_window_s` | Jump re-seats blocked, `/reactor/vo_healthy` false |
| Busy watchdog | No `SetSlamPose` response within `set_pose_busy_timeout_s` | Call cleared, publishing resumes, bypass window opened |

## Subscribed topics

The reactor takes its odometry and tracking state from cuVSLAM, and its pose and fusion state from PX4.

| Topic | Type | QoS | Carries |
| --- | --- | --- | --- |
| `/visual_slam/vis/slam_odometry` | `nav_msgs/Odometry` | Best effort, volatile, depth 1 | cuVSLAM pose stream, the frame the gates judge |
| `/visual_slam/status` | `isaac_ros_visual_slam_interfaces/VisualSlamStatus` | Best effort, volatile, depth 1 | `vo_state`, 1 while tracking |
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | Best effort, transient local, depth 1 | FMU pose in FRD, source of the ROS-frame outputs and the re-seat seed |
| `/fmu/out/estimator_status_flags` | `px4_msgs/EstimatorStatusFlags` | Best effort, transient local, depth 1 | `cs_ev_pos`, latched true on EV fusion start |
| `/reactor/drone_odom` | `nav_msgs/Odometry` | Best effort, volatile, depth 1 | Own output, cached `sync_cache_sz` deep for re-seat seeding and settle comparison |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | TransformListener defaults | Buffered only; the node runs no transform lookup |

## Published topics

Both `/reactor` pose outputs and the `map` to `px4` transform carry the FMU timestamp, replaced by the node clock whenever the skew exceeds `fmu_stamp_max_skew_s`.

| Topic | Type | QoS | Published | Carries |
| --- | --- | --- | --- | --- |
| `/visual_slam/filt_slam_odometry` | `nav_msgs/Odometry` | Best effort, volatile, depth 1 | Each admitted frame | Unmodified cuVSLAM pose, read by `vio_transform` |
| `/reactor/drone_odom` | `nav_msgs/Odometry` | Best effort, volatile, depth 1 | Each `/fmu/out/vehicle_odometry` message | FMU pose in FLU, frame `map`, child `px4` |
| `/reactor/drone_pose` | `geometry_msgs/PoseStamped` | Best effort, volatile, depth 1 | Each `/fmu/out/vehicle_odometry` message | Same pose without the twist |
| `/tf` | `tf2_msgs/TFMessage` | Broadcaster default | Each `/fmu/out/vehicle_odometry` message | `map` to `px4` transform |
| `/reactor/vio_reset_epoch` | `std_msgs/UInt8` | Reliable, transient local, depth 1 | Startup, then each committed origin seat | Counter wrapping at 255, written by `vio_transform` into `VehicleOdometry.reset_counter` |
| `/reactor/vo_healthy` | `std_msgs/Bool` | Reliable, transient local, depth 1 | Startup, then on change | False on a spent re-seat budget or EV silence past `ev_silence_max_s`; no subscriber in this repository |

## Services hosted

The reactor hosts one service, an operator-driven origin injection.

| Service | Type | Effect | Refused when |
| --- | --- | --- | --- |
| `/visual_slam/set_reactor_pose` | `std_srvs/Trigger` | Calls `SetSlamPose` at zero position and zero yaw, keeping FMU roll and pitch | `vo_state` is not 1, or a re-seat is in flight |

```bash
ros2 service call /visual_slam/set_reactor_pose std_srvs/srv/Trigger
```

Success reports acceptance, not completion: the injection is skipped when no FMU sample is fresher than `set_pose_max_odom_age`. A committed injection logs `>>> SET ORIGIN (settling) <<<` and increments `/reactor/vio_reset_epoch`.

## Services called

The reactor calls one service, on the cuVSLAM node.

| Service | Type | Called on |
| --- | --- | --- |
| `/visual_slam/set_slam_pose` | `isaac_ros_visual_slam_interfaces/SetSlamPose` | Origin seat, jump-gate rejection, origin settle timeout, `/visual_slam/set_reactor_pose` |

## Parameters

Every parameter below is declared with the listed default and overridden by [`config/px4_vslam_reactor.yaml`](config/px4_vslam_reactor.yaml).

| Parameter | Default | Controls |
| --- | --- | --- |
| `vslam_stabilization_time` | 1.0 s | Floor of the post-re-seat bypass window |
| `lin_vel_gate` | 5.0 m/s | Jump-gate linear-velocity threshold |
| `ang_vel_gate_dps` | 200.0 deg/s | Jump-gate angular-velocity threshold |
| `VO_rate_lim` | 0.5 s | Stamp interval above which a position step counts as a slow teleport |
| `VO_pos_delta_lim` | 0.4 m | Position step counted as a slow teleport |
| `sync_cache_sz` | 300 | Depth of the `/reactor/drone_odom` cache |
| `align_yaw_deg` | 2.0 deg | Settle-exit yaw tolerance |
| `align_pos_m` | 0.10 m | Settle-exit 3D position tolerance |
| `set_origin_settle_time` | 10.0 s | Injection window before the origin is re-injected |
| `set_pose_max_odom_age` | 0.030 s | Maximum age of the FMU sample seeding a re-seat |
| `fmu_stamp_max_skew_s` | 0.5 s | Stamp skew before outputs are re-stamped with the node clock |
| `set_pose_busy_timeout_s` | 3.0 s | Wait before a stalled `SetSlamPose` call is cleared |
| `reseat_burst_max` | 5 | Jump re-seats allowed inside the burst window |
| `reseat_burst_window_s` | 10.0 s | Rolling window for the re-seat budget |
| `ev_silence_max_s` | 2.0 s | EV output silence, with frames still arriving, before `/reactor/vo_healthy` latches false |

Edit the YAML and restart the stack. The workspace is symlink-installed, so a value change needs no rebuild.
