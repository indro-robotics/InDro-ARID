# px4_vslam_reactor

`px4_vslam_reactor` filters the Isaac VSLAM odometry stream before PX4 fuses it. The node withholds jumped frames, re-seats the SLAM pose onto the FMU pose, and publishes the reset epoch EKF2 receives as `VehicleOdometry.reset_counter`.

## Node

[`px4_vslam/launch/vslam.launch.py`](../px4_vslam/launch/vslam.launch.py) starts the `vslam_reactor_node` executable as node `vslam_reactor` in the root namespace, with [`config/px4_vslam_reactor.yaml`](config/px4_vslam_reactor.yaml).

> Construction blocks until `visual_slam/set_slam_pose` is advertised, logging `waiting for visual_slam/set_slam_pose service...` every 2 s.

## Subscribed topics

| Topic | Type | QoS | Carries |
| --- | --- | --- | --- |
| `/visual_slam/vis/slam_odometry` | `nav_msgs/Odometry` | best-effort, volatile, depth 1 | cuVSLAM pose; the frame the filter admits or withholds. |
| `/visual_slam/status` | `isaac_ros_visual_slam_interfaces/VisualSlamStatus` | best-effort, volatile, depth 1 | `vo_state`; 1 is tracking. |
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | best-effort, transient-local, depth 1 | FMU pose in FRD; source of the TF, both `/reactor` poses and every re-seat. |
| `/fmu/out/estimator_status_flags` | `px4_msgs/EstimatorStatusFlags` | best-effort, transient-local, depth 1 | `cs_ev_pos`; marks the start of EKF2 EV fusion. |
| `/reactor/drone_odom` | `nav_msgs/Odometry` | best-effort, volatile, depth 1 | Own output, cached `sync_cache_sz` deep for re-seats and alignment. |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | default | TF listener buffer; no lookup is performed. |

## Published topics

| Topic | Type | QoS | When |
| --- | --- | --- | --- |
| `/visual_slam/filt_slam_odometry` | `nav_msgs/Odometry` | best-effort, volatile, depth 1 | Every admitted SLAM frame, unmodified; read by `vio_transform`. |
| `/reactor/drone_odom` | `nav_msgs/Odometry` | best-effort, volatile, depth 1 | Every FMU odometry message; FLU pose, frame `map`, child `px4`, twist unset. |
| `/reactor/drone_pose` | `geometry_msgs/PoseStamped` | best-effort, volatile, depth 1 | Same header and pose as `/reactor/drone_odom`. |
| `/reactor/vio_reset_epoch` | `std_msgs/UInt8` | reliable, transient-local, depth 1 | 0 at startup, then +1 on each committed origin seat, wrapping at 255. |
| `/reactor/vo_healthy` | `std_msgs/Bool` | reliable, transient-local, depth 1 | True at startup, then on each latch change. |
| `/tf` | `tf2_msgs/TFMessage` | default | `map` to `px4`, every FMU odometry message. |

The TF and both `/reactor` poses carry the FMU timestamp, or node time when the two differ by more than `fmu_stamp_max_skew_s`.

## Services hosted

| Service | Type | Effect |
| --- | --- | --- |
| `/visual_slam/set_reactor_pose` | `std_srvs/Trigger` | Seats the SLAM origin; refused while not tracking or re-seating. |

## Services called

| Service | Type | When |
| --- | --- | --- |
| `/visual_slam/set_slam_pose` | `isaac_ros_visual_slam_interfaces/SetSlamPose` | Origin seat: first tracked frame before EV fusion, on `/visual_slam/set_reactor_pose`, on an origin settle timeout. |
| `/visual_slam/set_slam_pose` | `isaac_ros_visual_slam_interfaces/SetSlamPose` | Jump re-seat: a frame fails the jump gate and the burst budget holds. |

## Parameters

| Parameter | Default | Controls |
| --- | --- | --- |
| `vslam_stabilization_time` | 1.0 s | Floor of the post-re-seat bypass window. |
| `lin_vel_gate` | 5.0 m/s | Jump-gate linear-velocity limit. |
| `ang_vel_gate_dps` | 200.0 deg/s | Jump-gate angular-velocity limit. |
| `VO_pos_delta_lim` | 0.4 m | Position step counted as a slow teleport. |
| `VO_rate_lim` | 0.5 s | Stamp interval above which that position step counts. |
| `sync_cache_sz` | 300 | Depth of the `/reactor/drone_odom` cache. |
| `align_yaw_deg` | 2.0 deg | Settle-exit yaw tolerance. |
| `align_pos_m` | 0.10 m | Settle-exit 3D position tolerance. |
| `set_origin_settle_time` | 10.0 s | Settle window before re-injection or abandonment. |
| `set_pose_max_odom_age` | 0.030 s | Max age of the FMU odometry receipt seeding a re-seat. |
| `fmu_stamp_max_skew_s` | 0.5 s | Skew before outputs carry node time. |
| `set_pose_busy_timeout_s` | 3.0 s | Wait before a stalled `SetSlamPose` call is cleared. |
| `reseat_burst_max` | 5 | Committed jump re-seats allowed inside the window. |
| `reseat_burst_window_s` | 10.0 s | Rolling window for the jump re-seat budget. |
| `ev_silence_max_s` | 2.0 s | EV output silence before `/reactor/vo_healthy` latches false. |

The node declares every default, so it runs without the file. The workspace is symlink-installed, so an edited value applies at the next stack start.

## Origin injection

`/visual_slam/set_reactor_pose` is the one reactor interface an operator calls.

```bash
ros2 service call /visual_slam/set_reactor_pose std_srvs/srv/Trigger "{}"
```

| Stage | Result |
| --- | --- |
| Invokes | `SetSlamPose` on `visual_slam_node`, position (0,0,0), yaw zeroed, FMU roll and pitch kept. |
| On success | `/reactor/vio_reset_epoch` increments; `vio_transform` stamps it into `reset_counter` on `/fmu/in/vehicle_visual_odometry`. |
| Then | Frames stream until SLAM and FMU agree within `align_yaw_deg` and `align_pos_m`. |
| Refused | `success` false while `vo_state` is not 1 or a re-seat is in flight. |
| No dispatch | `success` true, no call made, when the FMU sample is older than `set_pose_max_odom_age`. |

## Frame filtering

A frame reaches `/visual_slam/filt_slam_odometry` after these stages, in order.

| Stage | Condition | Outcome |
| --- | --- | --- |
| Ingress | Pose finite, quaternion non-zero | Dropped otherwise, logged at a 1 s throttle. |
| Tracking | `vo_state` 1 and no re-seat in flight | Dropped otherwise. |
| Before EV fusion | `cs_ev_pos` false | First frame seats the origin, later frames publish ungated. |
| Bypass window | Open after a committed re-seat | Jump gate skipped; frames stamped before the commit are withheld. |
| Jump gate | Over `lin_vel_gate`, over `ang_vel_gate_dps`, or a step over `VO_pos_delta_lim` across more than `VO_rate_lim` | Withheld, and a jump re-seat is dispatched. |
| Settle | Re-seat committed, poses not yet aligned | Published for `set_origin_settle_time`. |

Both velocities come from the pose step against the last admitted frame, not the message twist. The bypass window closes at `vslam_stabilization_time` plus two post-commit frames, and at 3 s regardless.

## Re-seat and settle

The pose written into SLAM sets the reset epoch and the settle-timeout outcome.

| Kind | Pose written | Reset epoch | On settle timeout |
| --- | --- | --- | --- |
| Origin | (0,0,0), yaw zeroed, FMU roll and pitch | Bumped on success | Origin re-injected. |
| Jump | FMU position and full orientation | Unchanged | Alignment abandoned, streaming continues. |

Alignment compares the SLAM pose against the time-matched `/reactor/drone_odom` sample. Jump re-seats are budgeted at `reseat_burst_max` inside a rolling `reseat_burst_window_s`, and origin injections are not counted.

## VO health

`/reactor/vo_healthy` latches false on either condition and returns true when both clear.

- Jump re-seat budget exhausted.
- No EV output for `ev_silence_max_s` while SLAM frames arrive and EKF2 fuses EV, checked at 1 Hz.

Nothing in this repo subscribes to the topic, and the reactor commands no flight action.
