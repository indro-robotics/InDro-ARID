# px4_vslam_reactor

The reactor is the gate between visual SLAM and the PX4 bridge. It watches the incoming SLAM solution, withholds samples carrying tracking jumps or teleports, and re-seats SLAM onto the PX4 pose after a rejection.

The reactor does not fuse IMU data, run a filter, or cross-check against inertial state. It publishes a jump-filtered visual-odometry stream and raises a reset epoch when it seats SLAM on a new origin.

The package installs one executable, `vslam_reactor_node`, started by the [`px4_vslam`](../px4_vslam/) stack launch as node `vslam_reactor`.

---

## Function

The reactor filters the SLAM stream, keeps SLAM anchored to the PX4 solution, and exposes a manual origin reset.

- **Filter.** Velocity and position-delta gates reject teleports and tracking glitches; accepted frames go out on `/visual_slam/filt_slam_odometry`.
- **Re-anchor.** The startup origin seat, a rejected frame, a settle timeout and the manual trigger each call `SetSlamPose` on the SLAM backend, and the reactor keeps injecting the stream until the PX4 estimator agrees with the new pose.
- **Signal.** A committed origin seat bumps `/reactor/vio_reset_epoch`, which `vio_transform` forwards as `VehicleOdometry.reset_counter`.

## Gates

A frame reaches the PX4 bridge only when SLAM reports tracking and no re-seat is in flight. The jump gate runs once PX4 reports EV fusion; before that the reactor seats the origin and then streams tracking frames unfiltered.

**Velocity and jump gate.** A frame is rejected when its linear velocity reaches `lin_vel_gate`, its angular velocity reaches `ang_vel_gate_dps`, or its position step exceeds `VO_pos_delta_lim` while the stamp interval exceeds `VO_rate_lim`. Rejection withholds the frame and triggers an in-flight re-seat onto the FMU pose.

**Post-re-seat bypass.** A committed re-seat opens a window in which the jump gate is bypassed and the comparison baseline is rebased onto every arriving frame, so the pose step the re-seat creates is not judged as a SLAM jump. The window closes once `vslam_stabilization_time` has elapsed and at least two frames stamped after the commit have rebased the baseline, and it is capped at 3 s. Frames stamped before the commit rebase the baseline but are withheld.

**Displacement and settle gate.** After any re-seat the reactor keeps injecting the stream until the SLAM and FMU poses agree within `align_yaw_deg` and `align_pos_m`. An origin settle that runs past `set_origin_settle_time` re-injects the origin. A jump-re-seat settle that runs past it reports an error, abandons alignment and keeps streaming.

**Re-seat burst limit.** Jump re-seats are budgeted at `reseat_burst_max` dispatched calls per rolling `reseat_burst_window_s`. Once the budget is spent, further jump re-seats are blocked until the count decays back under it. Origin injections are not counted against the budget.

A `SetSlamPose` call that never returns suppresses EV publishing; the `set_pose_busy_timeout_s` watchdog clears the stalled call and opens the bypass window.

## VO health

`/reactor/vo_healthy` latches false on either of two conditions and returns to true when both clear.

- Exhausted re-seat budget.
- EV publish silence: no output for longer than `ev_silence_max_s` while SLAM frames are still arriving and PX4 reports EV fusion.

No node in this repository subscribes to the topic, and the reactor takes no action on it.

## Epochs and reset_counter

Only a committed origin seat bumps the epoch on `/reactor/vio_reset_epoch`, and `vio_transform` forwards it into `VehicleOdometry.reset_counter`. A jump re-seat writes the FMU's own pose into SLAM and sends no reset flag. The epoch is published latched at 0 on startup and wraps at 255.

## Origin injection

An origin injection seats SLAM at zero position and zero yaw, keeping the FMU roll and pitch so the map frame stays gravity-aligned. The first one runs on the first tracked frame, before PX4 reports EV fusion. In-flight re-seats keep the FMU's full position and orientation.

## FMU stamp clamp

When the FMU timestamp skews from now by more than `fmu_stamp_max_skew_s`, the `map` to `px4` TF, `/reactor/drone_odom` and `/reactor/drone_pose` are re-stamped with node time. Excursions are counted and logged as `FMU stamp excursion` at a 5 s throttle.

---

## Inputs

The reactor takes odometry and tracking state from SLAM, and pose and fusion state from PX4.

| Topic | Type | From | Used for |
|---|---|---|---|
| `/visual_slam/vis/slam_odometry` | `nav_msgs/Odometry` | Isaac VSLAM | Raw odometry stream. |
| `/visual_slam/status` | `isaac_ros_visual_slam_interfaces/VisualSlamStatus` | Isaac VSLAM | Tracking state. |
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | PX4 | FMU pose for the TF and ROS-frame outputs; re-seat freshness stamp. |
| `/fmu/out/estimator_status_flags` | `px4_msgs/EstimatorStatusFlags` | PX4 | EV-fusion detection (`cs_ev_pos`). |
| `/reactor/drone_odom` | `nav_msgs/Odometry` | this node | Cached at `sync_cache_sz` for re-seat seeding and alignment. |

## Outputs

The reactor publishes the filtered stream, the PX4 pose in ROS conventions, and its epoch and health telemetry.

| Topic | Type | Purpose |
|---|---|---|
| `/visual_slam/filt_slam_odometry` | `nav_msgs/Odometry` | Jump-filtered odometry, consumed by `vio_transform`. |
| `/reactor/drone_odom` | `nav_msgs/Odometry` | PX4 odom in ROS conventions (FRD to FLU). |
| `/reactor/drone_pose` | `geometry_msgs/PoseStamped` | Same pose without the twist. |
| `/reactor/vio_reset_epoch` | `std_msgs/UInt8` (latched) | Origin-seat epoch. |
| `/reactor/vo_healthy` | `std_msgs/Bool` (latched) | False on an exhausted re-seat budget or EV publish silence. |

## Services

The reactor hosts one service and calls one.

| Interface | Direction | Type | Purpose |
|---|---|---|---|
| `/visual_slam/set_reactor_pose` | hosts | `std_srvs/Trigger` | Request a zero-origin injection; false while SLAM is not tracking or a re-seat is in flight. |
| `/visual_slam/set_slam_pose` | calls | `isaac_ros_visual_slam_interfaces/SetSlamPose` | Re-seat call to the SLAM backend. |

Success on `set_reactor_pose` reports acceptance, not completion: the injection is skipped when no PX4 odometry sample is fresher than `set_pose_max_odom_age`.

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
| `reseat_burst_max` | count | Dispatched jump re-seats allowed in the window. |
| `reseat_burst_window_s` | s | Rolling window for the re-seat budget. |
| `ev_silence_max_s` | s | EV output silence, with frames arriving, before `/reactor/vo_healthy` latches false. |

Edit the YAML and restart the stack. The workspace is symlink-installed, so a YAML change needs no rebuild.

---

## Running

The reactor comes up with the SLAM stack launch. A standalone run against an already running SLAM graph loads no YAML and applies the node defaults.

```bash
ros2 run px4_vslam_reactor vslam_reactor_node
```

> Started without the SLAM backend, the node blocks in construction and logs `waiting for visual_slam/set_slam_pose service...` every 2 s until the service is advertised.
