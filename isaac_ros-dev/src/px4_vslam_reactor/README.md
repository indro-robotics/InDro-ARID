# px4_vslam_reactor

The reactor coordinates the external visual-SLAM source with PX4. It watches the incoming SLAM solution, gates bad samples such as tracking jumps, teleports and cadence starvation, and re-anchors SLAM onto the PX4 solution when the two diverge.

All sensor fusion stays in PX4. The reactor does not fuse IMU data, run a filter, or cross-check against inertial state; it provides a jump-free visual-odometry stream and signals EKF2 when that stream has moved discontinuously.

The [`px4_vslam`](../px4_vslam/) launch starts it as `vslam_reactor_node`.

---

## Function

1. It filters the SLAM odometry stream. Velocity and position-delta gates reject teleports and tracking glitches, and the clean stream is republished on `/visual_slam/filt_slam_odometry`.
2. It detects reset-misalignment. After a `SetSlamPose` call it keeps injecting the stream until the VSLAM pose agrees with the PX4 estimator within tolerance; if the settle window elapses without alignment, the origin is re-injected and the window restarts.
3. It hosts a force-reset service, `visual_slam/set_reactor_pose`, which commands a re-injection of the zero origin (zero position, zero yaw, gravity-aligned roll and pitch) at any time.

## Gates

Every VSLAM frame runs through `slam_odom_callback`. A frame reaches PX4 only when VSLAM reports tracking (`vslam_status == 1`) and the reactor is not mid-reseat.

**Velocity and jump gate.** A frame exceeding `lin_vel_gate`, `ang_vel_gate_dps`, or the `VO_pos_delta_lim` and `VO_rate_lim` slow-jump pair is rejected, and the rejection triggers an in-flight re-seat. The slow-jump term is suppressed while the cadence gate is engaged, where normal motion across a gated gap would read as a jump.

**Post-re-seat bypass.** A committed re-seat opens a short window in which the jump gate is bypassed and the baseline is rebased onto every arriving frame, so the pose step the re-seat creates is not re-judged as a cuVSLAM jump. The window closes once `vslam_stabilization_time` has elapsed and at least two frames stamped after the commit have rebased the baseline, and it is hard-capped at 3 s. Frames stamped before the commit rebase the baseline but are withheld from PX4, since they may still carry the pre-re-seat pose under the already-bumped reset counter.

**Displacement and settle gate.** After an origin injection the reactor keeps injecting until the VSLAM and FMU poses agree within `align_yaw_deg` and `align_pos_m`, and re-injects if `set_origin_settle_time` elapses first.

**Cadence gate.** VO is withheld from EKF2 while cuVSLAM stamp cadence is degraded. Engagement is two-tier on VO header-stamp gaps: one gap at or over `cadence_gate_hard_s`, or `cadence_gate_sustained_samples` consecutive gaps between `cadence_gate_s` and `cadence_gate_hard_s`. A lone lesser gap does not engage, and any nominal sample breaks the lesser-gap streak. `cadence_release_samples` nominal frames release the gate; there is no timed escape. Anomalous stamps carry no cadence information: they neither count toward release nor break a release streak in progress. Stamp gaps over 5 s are treated as stamp anomalies and reseed the tracker instead of engaging. While gated, all settles are withheld and the settle countdown pauses, but jump detection stays live so a genuine jump still re-seats. The withhold never bumps the epoch. State is latched on `/reactor/cadence_gated` and engagements count on `/reactor/cadence_gate_count`.

**Re-seat burst limit.** Jump re-seats are budgeted at `reseat_burst_max` committed re-seats per rolling `reseat_burst_window_s`. Once the budget is spent, further jump re-seats are blocked, because re-seating at that rate cannot recover cuVSLAM and only feeds EKF2 a reset storm. Blocking a re-seat never bumps the reset epoch. Origin re-injections are bounded by the settle timeout and are not counted against the budget.

A `SetSlamPose` call that never returns would suppress EV publishing indefinitely; the `set_pose_busy_timeout_s` watchdog clears the hung call.

## VO health

`/reactor/vo_healthy` latches false on either of two conditions and returns to true when both clear. The first is an exhausted re-seat budget. The second is EV publish silence: no output to `vio_transform` for longer than `ev_silence_max_s` while cuVSLAM frames are still arriving and EV fusion has started, which is the case every input-side monitor reads as healthy. The reactor never commands a flight action off this signal.

## Epochs and reset_counter

Only a committed ORIGIN seat bumps the epoch on `/reactor/vio_reset_epoch`. `vio_transform` forwards it into `VehicleOdometry.reset_counter`, which tells EKF2 to reset its EV-aided states onto the new origin. A jump re-seat writes the FMU's own pose into cuVSLAM, so post-seat EV already agrees with EKF2 and no reset flag is sent; any residual step is ordinary innovation.

## Origin injection

On init the pre-takeoff datum is zero position and zero yaw, keeping the FMU roll and pitch so the frame stays gravity-aligned. The magnetometer is disabled, so the heading datum is arbitrary and a zero-yaw origin removes a re-anchor jump that EKF2 would reject. In-flight re-seats keep the FMU's full orientation, which keeps the re-anchor near zero.

## FMU stamp clamp

`px4_odom_callback` re-stamps the `map`->`px4` TF and `/reactor/drone_odom` and `/reactor/drone_pose` with node time when the FMU timestamp skews more than `fmu_stamp_max_skew_s` from now, because uXRCE timesync excursions can pass boot-relative or future stamps through and corrupt tf2 buffers. The test is skew against now rather than monotonicity, since a future stamp is still monotonic. Excursions are counted and warned at a throttle.

---

## Inputs

The reactor subscribes to two VSLAM topics and two PX4 topics.

| Topic | Type | From | Used for |
|---|---|---|---|
| `/visual_slam/vis/slam_odometry` | `nav_msgs/Odometry` | Isaac VSLAM | Raw odometry stream. |
| `/visual_slam/status` | `isaac_ros_visual_slam_interfaces/VisualSlamStatus` | Isaac VSLAM | Tracking quality and state. |
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | PX4 | Pose for reset-alignment comparison. |
| `/fmu/out/estimator_status_flags` | `px4_msgs/EstimatorStatusFlags` | PX4 | EV-fusion detection. |

## Outputs

The reactor publishes the filtered stream plus its gate and health telemetry.

| Topic | Type | Purpose |
|---|---|---|
| `/visual_slam/filt_slam_odometry` | `nav_msgs/Odometry` | Jump-filtered VSLAM odometry; downstream consumers should prefer this. |
| `/reactor/drone_odom` | `nav_msgs/Odometry` | PX4 odom in ROS conventions (FRD to FLU). |
| `/reactor/drone_pose` | `geometry_msgs/PoseStamped` | Same, pose only, for RViz or Foxglove. |
| `/reactor/vio_reset_epoch` | `std_msgs/UInt8` (latched) | Origin-seat epoch. |
| `/reactor/cadence_gated` | `std_msgs/Bool` (latched) | True while VO is withheld for degraded cadence. |
| `/reactor/cadence_gate_count` | `std_msgs/UInt32` (latched) | Cumulative cadence-gate engagements. |
| `/reactor/vo_healthy` | `std_msgs/Bool` (latched) | False on an exhausted re-seat budget or EV publish silence. |

## Services

The reactor hosts one service and calls one.

| Interface | Direction | Type | Purpose |
|---|---|---|---|
| `visual_slam/set_reactor_pose` | hosts | `std_srvs/Trigger` | Force a zero-origin re-injection. |
| `visual_slam/set_slam_pose` | calls | `isaac_ros_visual_slam_interfaces/SetSlamPose` | Reset call to the VSLAM backend. |

## TF

The reactor broadcasts the drone pose as `map`->`px4` and consumes the TF tree for body-frame conversions.

---

## Parameters

Values come from [`config/px4_vslam_reactor.yaml`](config/px4_vslam_reactor.yaml), loaded by [`px4_vslam/launch/vslam.launch.py`](../px4_vslam/launch/vslam.launch.py). The node declares a default for every key, so it runs if the file is absent or omits a key. Angular gates are entered in degrees and converted in the node.

| Parameter | Units | Controls |
|---|---|---|
| `vslam_stabilization_time` | s | Floor of the post-re-seat jump-gate bypass window. |
| `lin_vel_gate` | m/s | Linear-velocity ceiling for jump detection. |
| `ang_vel_gate_dps` | deg/s | Angular-velocity ceiling for jump detection. |
| `VO_rate_lim` + `VO_pos_delta_lim` | s, m | Slow-jump detection; the stamp delta and the position step must both exceed these. |
| `sync_cache_sz` | count | Cache depth for PX4-to-VSLAM time alignment. |
| `align_yaw_deg` | deg | Settle-exit yaw tolerance. |
| `align_pos_m` | m | Settle-exit 3D position tolerance. |
| `set_origin_settle_time` | s | Injection window before the origin is re-injected. |
| `set_pose_max_odom_age` | s | Max age of the PX4 odom sample seeding a re-seat. |
| `fmu_stamp_max_skew_s` | s | Max \|now - FMU stamp\| before re-stamping outputs. |
| `cadence_gate_s` | s | Floor of the lesser-gap tier; gaps below this are nominal. |
| `cadence_gate_hard_s` | s | Single-gap engage threshold. |
| `cadence_gate_sustained_samples` | count | Consecutive lesser gaps that engage. |
| `cadence_release_samples` | count | Consecutive nominal frames that release. |
| `set_pose_busy_timeout_s` | s | Clears a hung `SetSlamPose` call. |
| `reseat_burst_max` | count | Committed jump re-seats allowed in the window. |
| `reseat_burst_window_s` | s | Rolling window for the re-seat budget. |
| `ev_silence_max_s` | s | EV output silence, with frames arriving, before `/reactor/vo_healthy` latches false. |

Edit the YAML, then restart the launch. The workspace is symlink-installed, so this needs no rebuild.

---

## Running

Normally the VSLAM stack launches it:

```bash
ros2 launch px4_vslam vslam.launch.py
```

Standalone, against an already running VSLAM graph:

```bash
ros2 run px4_vslam_reactor vslam_reactor_node
```
