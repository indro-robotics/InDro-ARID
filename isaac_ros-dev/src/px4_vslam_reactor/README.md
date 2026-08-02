# px4_vslam_reactor

The reactor is the gate between visual SLAM and PX4. It watches the incoming SLAM solution, rejects samples carrying tracking jumps or teleports, and re-anchors SLAM onto the PX4 solution when the two diverge.

All sensor fusion stays in PX4. The reactor does not fuse IMU data, run a filter, or cross-check against inertial state. It provides a jump-free visual-odometry stream and signals EKF2 when that stream has moved discontinuously.

The package contributes one node, `vslam_reactor_node`, brought up by the [`px4_vslam`](../px4_vslam/) stack launch.

---

## Function

The reactor filters the SLAM stream, keeps SLAM anchored to the PX4 solution, and exposes a manual origin reset.

- **Filter.** Velocity and position-delta gates reject teleports and tracking glitches; accepted frames go out on `/visual_slam/filt_slam_odometry`.
- **Re-anchor.** A rejected frame or a manual trigger calls `SetSlamPose` on the SLAM backend, and the reactor keeps injecting the stream until the PX4 estimator agrees with the new pose.
- **Signal.** A committed origin seat bumps `/reactor/vio_reset_epoch`, which reaches EKF2 as `VehicleOdometry.reset_counter`.

## Gates

Every SLAM frame runs through the gate chain. A frame reaches PX4 only when SLAM reports tracking and no re-seat is in flight.

**Velocity and jump gate.** A frame is rejected when its linear velocity exceeds `lin_vel_gate`, its angular velocity exceeds `ang_vel_gate_dps`, or its position step exceeds `VO_pos_delta_lim` while the stamp interval exceeds `VO_rate_lim`. Rejection withholds the frame from PX4 and triggers an in-flight re-seat onto the FMU pose.

**Post-re-seat bypass.** A committed re-seat opens a window in which the jump gate is bypassed and the comparison baseline is rebased onto every arriving frame, so the pose step the re-seat creates is not judged as a SLAM jump. The window closes once `vslam_stabilization_time` has elapsed and at least two frames stamped after the commit have rebased the baseline, and it is capped at 3 s. Frames stamped before the commit rebase the baseline but are withheld from PX4, since they may still carry the pre-re-seat pose under the already-bumped reset counter.

**Displacement and settle gate.** After an origin injection the reactor keeps injecting the stream until the SLAM and FMU poses agree within `align_yaw_deg` and `align_pos_m`, and re-injects the origin if `set_origin_settle_time` elapses first. A settle that follows a jump re-seat is abandoned instead of re-injected: it reports an error and keeps streaming, because an origin injection zeroes yaw and would discard the heading datum in flight.

**Re-seat burst limit.** Jump re-seats are budgeted at `reseat_burst_max` committed re-seats per rolling `reseat_burst_window_s`. Once the budget is spent, further jump re-seats are blocked until the rate decays back under it, because re-seating at that rate cannot recover SLAM and only feeds EKF2 repeated resets. Blocking a re-seat never bumps the reset epoch. Origin re-injections are bounded by the settle timeout and are not counted against the budget.

A `SetSlamPose` call that never returns would suppress EV publishing indefinitely; the `set_pose_busy_timeout_s` watchdog clears the stalled call.

## VO health

`/reactor/vo_healthy` latches false on either of two conditions and returns to true when both clear.

- Exhausted re-seat budget.
- EV publish silence: no output for longer than `ev_silence_max_s` while SLAM frames are still arriving and EKF2 is fusing EV.

Nothing consumes the topic on this drone, and the reactor never commands a flight action off it.

## Epochs and reset_counter

Only a committed origin seat bumps the epoch on `/reactor/vio_reset_epoch`. `vio_transform` forwards it into `VehicleOdometry.reset_counter`, which tells EKF2 to reset its EV-aided states onto the new origin. A jump re-seat writes the FMU's own pose into SLAM, so post-seat EV already agrees with EKF2 and no reset flag is sent; any residual step is ordinary innovation.

## Origin injection

The pre-takeoff datum is zero position and zero yaw, keeping the FMU roll and pitch so the map frame stays gravity-aligned. In-flight re-seats keep the FMU's full orientation, which holds the re-anchor near zero.

## FMU stamp clamp

When the FMU timestamp skews from now by more than `fmu_stamp_max_skew_s`, the `map` to `px4` TF, `/reactor/drone_odom` and `/reactor/drone_pose` are re-stamped with node time. uXRCE timesync excursions can otherwise pass boot-relative or future stamps through and corrupt tf2 buffers. Excursions are counted and reported at a throttle.

---

## Inputs

The reactor takes odometry and tracking state from SLAM, and pose and fusion state from PX4.

| Topic | Type | From | Used for |
|---|---|---|---|
| `/visual_slam/vis/slam_odometry` | `nav_msgs/Odometry` | Isaac VSLAM | Raw odometry stream. |
| `/visual_slam/status` | `isaac_ros_visual_slam_interfaces/VisualSlamStatus` | Isaac VSLAM | Tracking state. |
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | PX4 | Pose for re-seat seeding and alignment comparison. |
| `/fmu/out/estimator_status_flags` | `px4_msgs/EstimatorStatusFlags` | PX4 | EV-fusion detection. |

## Outputs

The reactor publishes the filtered stream, the PX4 pose in ROS conventions, and its epoch and health telemetry.

| Topic | Type | Purpose |
|---|---|---|
| `/visual_slam/filt_slam_odometry` | `nav_msgs/Odometry` | Jump-filtered odometry; downstream consumers should prefer this. |
| `/reactor/drone_odom` | `nav_msgs/Odometry` | PX4 odom in ROS conventions (FRD to FLU). |
| `/reactor/drone_pose` | `geometry_msgs/PoseStamped` | Same, pose only, for visualization. |
| `/reactor/vio_reset_epoch` | `std_msgs/UInt8` (latched) | Origin-seat epoch. |
| `/reactor/vo_healthy` | `std_msgs/Bool` (latched) | False on an exhausted re-seat budget or EV publish silence. |

## Services

The reactor hosts one service and calls one.

| Interface | Direction | Type | Purpose |
|---|---|---|---|
| `visual_slam/set_reactor_pose` | hosts | `std_srvs/Trigger` | Force a zero-origin re-injection. Returns false while SLAM is not tracking or a re-seat is in flight. |
| `visual_slam/set_slam_pose` | calls | `isaac_ros_visual_slam_interfaces/SetSlamPose` | Re-seat call to the SLAM backend. |

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
| `set_origin_settle_time` | s | Injection window before the origin is re-injected. |
| `set_pose_max_odom_age` | s | Max age of the PX4 odom sample seeding a re-seat. |
| `fmu_stamp_max_skew_s` | s | Max \|now - FMU stamp\| before re-stamping outputs. |
| `set_pose_busy_timeout_s` | s | Clears a stalled `SetSlamPose` call. |
| `reseat_burst_max` | count | Committed jump re-seats allowed in the window. |
| `reseat_burst_window_s` | s | Rolling window for the re-seat budget. |
| `ev_silence_max_s` | s | EV output silence, with frames arriving, before `/reactor/vo_healthy` latches false. |

Edit the YAML and restart the stack. The workspace is symlink-installed, so a YAML change needs no rebuild.

---

## Running

The reactor comes up with the SLAM stack launch. Run it standalone against an already running SLAM graph.

```bash
ros2 run px4_vslam_reactor vslam_reactor_node
```

> Started without the SLAM backend, the node blocks in construction and logs `waiting for visual_slam/set_slam_pose service...` every 2 s until the service is advertised.
