# px4_vslam_reactor

The reactor is the gate between visual SLAM and PX4. It filters the SLAM odometry stream, re-seats the SLAM pose onto the FMU pose when the stream jumps, and publishes a reset epoch that reaches EKF2 as `VehicleOdometry.reset_counter`.

The reactor runs no filter of its own and takes no inertial input.

The package contributes one node, `vslam_reactor_node`, brought up by the [`px4_vslam`](../px4_vslam/) stack launch.

---

## Function

The reactor filters the SLAM stream, holds SLAM aligned with the PX4 solution, and exposes a manual origin reset.

- **Filter.** Velocity and position-delta gates reject teleports and tracking glitches; accepted frames go out on `/visual_slam/filt_slam_odometry`.
- **Re-seat.** A rejected frame calls `SetSlamPose` with the current FMU pose; the manual trigger calls it with a zero origin. The reactor keeps publishing until the PX4 estimator agrees with the new pose.
- **Signal.** A committed origin seat bumps `/reactor/vio_reset_epoch`, which `vio_transform` forwards as `VehicleOdometry.reset_counter`.

## Gates

Every frame that reaches PX4 passes the tracking check: SLAM reporting `vo_state` 1 with no re-seat in flight. The jump gate applies only after PX4 reports EV fusion; before that the reactor injects the origin once, then streams tracked frames ungated while rebasing the jump-gate baseline.

**Velocity and jump gate.** A frame is rejected when its linear velocity reaches `lin_vel_gate`, its angular velocity reaches `ang_vel_gate_dps`, or its position step exceeds `VO_pos_delta_lim` while the stamp interval exceeds `VO_rate_lim`. Both velocities come from the pose step against the last accepted frame, not from the message twist. Rejection withholds the frame from PX4 and triggers a re-seat onto the FMU pose.

**Post-re-seat bypass.** A committed re-seat opens a window in which the jump gate is bypassed and the comparison baseline is rebased onto every arriving frame. The window closes once `vslam_stabilization_time` has elapsed and at least two frames stamped after the commit have rebased the baseline, and it is capped at 3 s. Frames stamped before the commit rebase the baseline but are withheld from PX4. The `set_pose_busy_timeout_s` watchdog clears a `SetSlamPose` call whose response never arrives and opens the same window.

**Displacement and settle gate.** After a re-seat the reactor keeps publishing until the SLAM and FMU poses agree within `align_yaw_deg` and `align_pos_m`. An origin settle that reaches `set_origin_settle_time` without agreement re-injects the origin. A jump settle that reaches it reports an error, abandons the alignment check and keeps streaming.

**Re-seat burst limit.** Jump re-seats are budgeted at `reseat_burst_max` committed re-seats per rolling `reseat_burst_window_s`. Past the budget, further jump re-seats are blocked until the count decays back under it. Origin re-injections are not counted against the budget.

## VO health

`/reactor/vo_healthy` latches false on either of two conditions and returns to true when both clear.

- Exhausted re-seat budget.
- EV publish silence: no output for longer than `ev_silence_max_s` while SLAM frames are still arriving and PX4 reports EV fusion.

No node in this repo subscribes to the topic, and the reactor takes no action on the latch.

## Epochs and reset_counter

Only a committed origin seat bumps the epoch on `/reactor/vio_reset_epoch`. `vio_transform` forwards it into `VehicleOdometry.reset_counter`, which re-anchors EKF2 onto the new origin. A jump re-seat writes the FMU's own pose into SLAM and sends no reset flag.

## Origin injection

An origin injection zeroes position and yaw and keeps the FMU roll and pitch. It runs once before EV fusion starts, on the manual trigger, and on an origin settle timeout. A jump re-seat instead writes the FMU position and full orientation.

## FMU stamp clamp

When the FMU timestamp skews from now by more than `fmu_stamp_max_skew_s`, the `map` to `px4` TF, `/reactor/drone_odom` and `/reactor/drone_pose` are re-stamped with node time. Excursions are counted and logged at a 5 s throttle.

---

## Inputs

The reactor takes odometry and tracking state from SLAM, pose and fusion state from PX4, and its own converted PX4 odometry.

| Topic | Type | From | Used for |
|---|---|---|---|
| `/visual_slam/vis/slam_odometry` | `nav_msgs/Odometry` | Isaac VSLAM | Raw odometry stream. |
| `/visual_slam/status` | `isaac_ros_visual_slam_interfaces/VisualSlamStatus` | Isaac VSLAM | `vo_state` tracking check. |
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | PX4 | Source of the `map` to `px4` TF and both `/reactor` pose outputs. |
| `/fmu/out/estimator_status_flags` | `px4_msgs/EstimatorStatusFlags` | PX4 | EV-fusion detection off `cs_ev_pos`. |
| `/reactor/drone_odom` | `nav_msgs/Odometry` | this node | Cached at `sync_cache_sz` for re-seat seeding and settle comparison. |

## Outputs

The reactor publishes the filtered stream, the PX4 pose in ROS conventions, and its epoch and health telemetry.

| Topic | Type | Purpose |
|---|---|---|
| `/visual_slam/filt_slam_odometry` | `nav_msgs/Odometry` | Jump-filtered odometry, consumed by `vio_transform`. |
| `/reactor/drone_odom` | `nav_msgs/Odometry` | PX4 odom in ROS conventions (FRD to FLU); pose only, twist unset. |
| `/reactor/drone_pose` | `geometry_msgs/PoseStamped` | Same pose as `/reactor/drone_odom`. |
| `/reactor/vio_reset_epoch` | `std_msgs/UInt8` (latched) | Origin-seat epoch. |
| `/reactor/vo_healthy` | `std_msgs/Bool` (latched) | False on an exhausted re-seat budget or EV publish silence. |

## Services

The reactor hosts one service and calls one.

| Interface | Direction | Type | Purpose |
|---|---|---|---|
| `/visual_slam/set_reactor_pose` | hosts | `std_srvs/Trigger` | Request a zero-origin injection. Returns false while SLAM is not tracking or a re-seat is in flight. |
| `/visual_slam/set_slam_pose` | calls | `isaac_ros_visual_slam_interfaces/SetSlamPose` | Re-seat call to the SLAM backend. |

```bash
ros2 service call /visual_slam/set_reactor_pose std_srvs/srv/Trigger "{}"
```

## TF

The reactor broadcasts the FMU pose as `map` to `px4`.

---

## Parameters

Values come from [`config/px4_vslam_reactor.yaml`](config/px4_vslam_reactor.yaml), loaded by [`px4_vslam/launch/vslam.launch.py`](../px4_vslam/launch/vslam.launch.py). The node declares a default for every key, so it runs if the file is absent or omits a key. Angular gates are entered in degrees and converted in the node.

| Parameter | Units | Controls |
|---|---|---|
| `vslam_stabilization_time` | s | Floor of the post-re-seat jump-gate bypass window. |
| `lin_vel_gate` | m/s | Linear-velocity ceiling for jump detection. |
| `ang_vel_gate_dps` | deg/s | Angular-velocity ceiling for jump detection. |
| `VO_rate_lim` + `VO_pos_delta_lim` | s, m | Slow-jump detection; the stamp interval and the position step must both exceed these. |
| `sync_cache_sz` | count | Cache depth for PX4-to-SLAM time alignment. |
| `align_yaw_deg` | deg | Settle-exit yaw tolerance. |
| `align_pos_m` | m | Settle-exit 3D position tolerance. |
| `set_origin_settle_time` | s | Settle window before the origin is re-injected or a jump settle is abandoned. |
| `set_pose_max_odom_age` | s | Max age of the last PX4 odometry receipt; a re-seat is deferred past it. |
| `fmu_stamp_max_skew_s` | s | Max \|now - FMU stamp\| before re-stamping outputs. |
| `set_pose_busy_timeout_s` | s | Clears a stalled `SetSlamPose` call. |
| `reseat_burst_max` | count | Committed jump re-seats allowed in the window. |
| `reseat_burst_window_s` | s | Rolling window for the re-seat budget. |
| `ev_silence_max_s` | s | EV output silence, with frames arriving, before `/reactor/vo_healthy` latches false. |

Edit the YAML and restart the stack. The workspace is symlink-installed, so a YAML change needs no rebuild.

---

## Running

A standalone run against a live SLAM graph uses the in-node parameter defaults, not the YAML.

```bash
ros2 run px4_vslam_reactor vslam_reactor_node
```

> Started without the SLAM backend, the node blocks in construction and logs `waiting for visual_slam/set_slam_pose service...` every 2 s until the service is advertised.
