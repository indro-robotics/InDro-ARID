# px4_vslam_reactor

The reactor gates the Isaac VSLAM odometry stream into PX4. It withholds jumped frames, re-seats the SLAM pose onto the FMU pose, and publishes the reset epoch that reaches EKF2 as `VehicleOdometry.reset_counter`.

## Node

[`px4_vslam/launch/vslam.launch.py`](../px4_vslam/launch/vslam.launch.py) runs the `vslam_reactor_node` executable as node `vslam_reactor` in the root namespace, with [`config/px4_vslam_reactor.yaml`](config/px4_vslam_reactor.yaml).

> Construction blocks until `visual_slam/set_slam_pose` is advertised, logging `waiting for visual_slam/set_slam_pose service...` every 2 s.

## Subscribed topics

| Topic | Type | QoS | Carries |
|---|---|---|---|
| `/visual_slam/vis/slam_odometry` | `nav_msgs/Odometry` | best-effort, volatile, depth 1 | Raw SLAM pose; drives the filtering path. |
| `/visual_slam/status` | `isaac_ros_visual_slam_interfaces/VisualSlamStatus` | best-effort, volatile, depth 1 | `vo_state`; 1 is tracking. |
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | best-effort, transient-local, depth 1 | FMU pose in FRD; source of the TF and both `/reactor` poses. |
| `/fmu/out/estimator_status_flags` | `px4_msgs/EstimatorStatusFlags` | best-effort, transient-local, depth 1 | `cs_ev_pos`; marks EKF2 EV fusion start. |
| `/reactor/drone_odom` | `nav_msgs/Odometry` | best-effort, volatile, depth 1 | Own output, cached `sync_cache_sz` deep to seed re-seats and settle checks. |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | default | TF listener buffer; no lookup is performed. |

## Published topics

| Topic | Type | QoS | When |
|---|---|---|---|
| `/visual_slam/filt_slam_odometry` | `nav_msgs/Odometry` | best-effort, volatile, depth 1 | Every admitted SLAM frame; read by `vio_transform`. |
| `/reactor/drone_odom` | `nav_msgs/Odometry` | best-effort, volatile, depth 1 | Every FMU odometry message; pose in FLU, twist unset. |
| `/reactor/drone_pose` | `geometry_msgs/PoseStamped` | best-effort, volatile, depth 1 | The same pose and header as `/reactor/drone_odom`. |
| `/reactor/vio_reset_epoch` | `std_msgs/UInt8` | reliable, transient-local, depth 1 | At startup as 0, then on each committed origin seat. |
| `/reactor/vo_healthy` | `std_msgs/Bool` | reliable, transient-local, depth 1 | At startup as true, then on each latch change. |
| `/tf` | `tf2_msgs/TFMessage` | default | `map` to `px4`, every FMU odometry message. |

Both `/reactor` poses use frame `map` and child frame `px4`.

## Services hosted

| Service | Type | Effect |
|---|---|---|
| `/visual_slam/set_reactor_pose` | `std_srvs/Trigger` | Injects the origin into SLAM; refused while not tracking or re-seating. |

## Services called

| Service | Type | When |
|---|---|---|
| `/visual_slam/set_slam_pose` | `isaac_ros_visual_slam_interfaces/SetSlamPose` | Origin seat: once before EV fusion, on the trigger service, on an origin settle timeout. |
| `/visual_slam/set_slam_pose` | `isaac_ros_visual_slam_interfaces/SetSlamPose` | Jump re-seat: a frame fails the jump gate and the burst budget holds. |

## Origin injection

The trigger service is the one reactor interface an operator calls.

```bash
ros2 service call /visual_slam/set_reactor_pose std_srvs/srv/Trigger "{}"
```

| Stage | Result |
|---|---|
| Invokes | `SetSlamPose` with position (0,0,0), yaw zeroed, FMU roll and pitch kept. |
| On success | `/reactor/vio_reset_epoch` increments; `vio_transform` stamps it into `VehicleOdometry.reset_counter`. |
| Then | Frames stream until SLAM and FMU agree within `align_yaw_deg` and `align_pos_m`. |
| Refused | `success` false while `vo_state` is not 1 or a re-seat is in flight. |

## Frame filtering

A frame published on `/visual_slam/filt_slam_odometry` has passed these stages in order.

| Stage | Condition | Outcome |
|---|---|---|
| Ingress | Pose finite, quaternion non-zero | Dropped otherwise, logged at a 1 s throttle. |
| Tracking | `vo_state` 1 and no re-seat in flight | Dropped otherwise. |
| Before EV fusion | EKF2 not fusing EV | First frame injects the origin, later frames publish ungated. |
| Bypass window | Inside the window a committed re-seat opened | Jump gate skipped; frames stamped before the commit are withheld. |
| Jump gate | Over `lin_vel_gate`, over `ang_vel_gate_dps`, or a step over `VO_pos_delta_lim` with an interval over `VO_rate_lim` | Withheld, and a jump re-seat is dispatched. |
| Settle | Re-seat committed, poses not yet aligned | Published for `set_origin_settle_time`. |

Both velocities are measured from the pose step against the last admitted frame, not from the message twist. The bypass window closes once `vslam_stabilization_time` has elapsed and two frames stamped after the commit have rebased the comparison baseline, and at 3 s regardless.

## Re-seat and settle

The pose written into SLAM decides the reset epoch and the settle-timeout outcome.

| Kind | Pose written | Reset epoch | On settle timeout |
|---|---|---|---|
| Origin | (0,0,0), yaw zeroed, FMU roll and pitch | Bumped on success | Origin re-injected. |
| Jump | FMU position and full orientation | Not bumped | Alignment abandoned, streaming continues. |

Alignment compares the SLAM pose against the time-matched `/reactor/drone_odom` sample. Jump re-seats are budgeted at `reseat_burst_max` inside a rolling `reseat_burst_window_s`, and past the budget the jump path stops re-seating until the count decays; origin injections are not counted. A `SetSlamPose` call with no response within `set_pose_busy_timeout_s` is cleared and opens the bypass window.

## VO health

`/reactor/vo_healthy` latches false on either condition and returns true when both clear.

- Jump re-seat budget exhausted.
- No EV output for `ev_silence_max_s` while SLAM frames arrive and EKF2 fuses EV, checked at 1 Hz.

Nothing in this repo subscribes to the topic, and the reactor commands no flight action.

## FMU stamp clamp

When the FMU timestamp differs from node time by more than `fmu_stamp_max_skew_s`, the `map` to `px4` transform, `/reactor/drone_odom` and `/reactor/drone_pose` carry node time for that message. Excursions are counted and logged at a 5 s throttle.

## Parameters

| Parameter | Default | Controls |
|---|---|---|
| `vslam_stabilization_time` | 1.0 s | Floor of the post-re-seat bypass window. |
| `lin_vel_gate` | 5.0 m/s | Jump-gate linear-velocity limit. |
| `ang_vel_gate_dps` | 200.0 deg/s | Jump-gate angular-velocity limit. |
| `VO_pos_delta_lim` | 0.4 m | Position step counted as a slow teleport. |
| `VO_rate_lim` | 0.5 s | Stamp interval above which that position step counts. |
| `sync_cache_sz` | 300 | Depth of the `/reactor/drone_odom` cache. |
| `align_yaw_deg` | 2.0 deg | Settle-exit yaw tolerance. |
| `align_pos_m` | 0.10 m | Settle-exit 3D position tolerance. |
| `set_origin_settle_time` | 10.0 s | Settle window before re-injection or abandonment. |
| `set_pose_max_odom_age` | 0.030 s | Max age of the last FMU odometry receipt seeding a re-seat. |
| `fmu_stamp_max_skew_s` | 0.5 s | Skew before outputs carry node time. |
| `set_pose_busy_timeout_s` | 3.0 s | Wait before a stalled `SetSlamPose` call is cleared. |
| `reseat_burst_max` | 5 | Committed jump re-seats allowed inside the window. |
| `reseat_burst_window_s` | 10.0 s | Rolling window for the jump re-seat budget. |
| `ev_silence_max_s` | 2.0 s | EV output silence before `/reactor/vo_healthy` latches false. |

The node declares every default, so it runs without the file. The workspace is symlink-installed, so an edited value applies at the next stack start.
