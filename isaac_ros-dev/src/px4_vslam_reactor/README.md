# px4_vslam_reactor

The reactor coordinates the external visual-SLAM source with PX4. It watches the incoming SLAM
solution, rejects bad samples (tracking jumps, teleports, mis-syncs), and re-anchors SLAM onto the
PX4 solution when the two diverge.

All sensor fusion stays in PX4. The reactor does not fuse IMU data, run a filter, or cross-check
against inertial state. It provides a jump-free visual-odometry stream and signals EKF2 when that
stream has moved discontinuously.

The package contributes one node, `vslam_reactor_node`, launched by
[`px4_vslam`](../px4_vslam/).

---

## Function

1. It filters the SLAM odometry stream. Velocity and position-delta gates reject teleports and
   tracking glitches, and the clean stream is republished on `/visual_slam/filt_slam_odometry`.
2. It detects reset misalignment. After a `SetSlamPose` call it keeps injecting the stream until
   the VSLAM pose agrees with the PX4 estimator within tolerance; if the settle window elapses
   without alignment, the origin is re-injected and the window restarts.
3. It hosts a force-reset service that commands a re-injection of the zero origin at any time.

Thresholds are configurable through
[`config/px4_vslam_reactor.yaml`](config/px4_vslam_reactor.yaml).

## Gates

Every VSLAM frame passes through the gate chain. A frame reaches PX4 only when VSLAM reports
tracking and the reactor is not mid-reseat.

- **Velocity and jump gate.** A frame exceeding `lin_vel_gate`, `ang_vel_gate_dps`, or the
  `VO_pos_delta_lim` and `VO_rate_lim` slow-jump pair is rejected, and the rejection triggers an
  in-flight re-seat. Immediately after a committed re-seat the gate is bypassed until the
  comparison baseline has rebased onto post-re-seat frames, which stops a re-seat loop from
  sustaining itself.
- **Displacement and settle gate.** After an origin injection the reactor keeps injecting until
  the VSLAM and FMU poses agree within `align_yaw_deg` and `align_pos_m`, re-injecting if
  `set_origin_settle_time` elapses first.
- **Cadence gate.** VO is withheld from EKF2 while cuVSLAM stamp cadence is degraded. Engagement
  is two-tier: one header-stamp gap at or over `cadence_gate_hard_s`, or
  `cadence_gate_sustained_samples` consecutive gaps in the `cadence_gate_s` to
  `cadence_gate_hard_s` band, where any nominal sample resets the streak.
  `cadence_release_samples` consecutive nominal frames release the gate, and there is no timed
  escape. Stamp gaps over 5 s are treated as stamp anomalies rather than cadence faults. While
  gated, settles are withheld and the settle countdown is paused, but jump detection stays live so
  a genuine jump still re-seats.

A `SetSlamPose` call that never returns would suppress EV publishing indefinitely; the
`set_pose_busy_timeout_s` watchdog clears the hung call.

## VO health

Two conditions latch `/reactor/vo_healthy` false, since neither is recoverable by re-seating.

- **Re-seat burst.** More than `reseat_burst_max` committed jump re-seats inside
  `reseat_burst_window_s` blocks further jump re-seats until the rate decays back under the
  budget. Origin re-injections are not counted.
- **EV publish silence.** No EV output for `ev_silence_max_s` while cuVSLAM frames are still
  arriving and EKF2 is fusing EV.

The topic is latched and carries no consumer on this drone; the reactor never commands a flight
action off it.

## Epochs and reset_counter

Each committed re-seat bumps the epoch on `/reactor/vio_reset_epoch`. `vio_transform` forwards it
into `VehicleOdometry.reset_counter`, telling EKF2 to reset its EV-aided states onto the new pose
instead of gating the discontinuity. The epoch bumps on the `SetSlamPose` success path only, and a
cadence withhold never bumps it.

## Origin injection

The pre-takeoff datum is zero position and zero yaw, retaining the FMU roll and pitch so the frame
stays gravity-aligned. In-flight re-seats keep the FMU's full orientation so the re-anchor stays
near zero.

## FMU stamp clamp

When the FMU timestamp skews from now by more than `fmu_stamp_max_skew_s`, the `map` to `px4` TF
and `/reactor/drone_odom` and `/reactor/drone_pose` are re-stamped with node time. uXRCE timesync
excursions can otherwise pass boot-relative or future stamps through and corrupt tf2 buffers.
Excursions are counted and warned at a throttle.

---

## Inputs

| Topic | Type | From | Used for |
|---|---|---|---|
| `/visual_slam/vis/slam_odometry` | `nav_msgs/Odometry` | Isaac VSLAM | Raw odometry stream. |
| `/visual_slam/status` | `isaac_ros_visual_slam_interfaces/VisualSlamStatus` | Isaac VSLAM | Tracking quality and state. |
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | PX4 | Pose for reset-alignment comparison. |
| `/fmu/out/estimator_status_flags` | `px4_msgs/EstimatorStatusFlags` | PX4 | EV-fusion detection. |

## Outputs

| Topic | Type | Purpose |
|---|---|---|
| `/visual_slam/filt_slam_odometry` | `nav_msgs/Odometry` | Jump-filtered odometry; downstream consumers should prefer this. |
| `/reactor/drone_odom` | `nav_msgs/Odometry` | PX4 odom in ROS conventions (FRD to FLU). |
| `/reactor/drone_pose` | `geometry_msgs/PoseStamped` | Same, pose only, for RViz or Foxglove. |
| `/reactor/vio_reset_epoch` | `std_msgs/UInt8` (latched) | Committed-re-seat epoch. |
| `/reactor/cadence_gated` | `std_msgs/Bool` (latched) | True while VO is withheld for degraded cadence. |
| `/reactor/cadence_gate_count` | `std_msgs/UInt32` (latched) | Cumulative cadence-gate engagements. |
| `/reactor/vo_healthy` | `std_msgs/Bool` (latched) | False on a re-seat burst or EV publish silence. |

## Services

| Interface | Direction | Type | Purpose |
|---|---|---|---|
| `visual_slam/set_reactor_pose` | hosts | `std_srvs/Trigger` | Force a zero-origin re-injection. |
| `visual_slam/set_slam_pose` | calls | `isaac_ros_visual_slam_interfaces/SetSlamPose` | Reset call to the VSLAM backend. |

## TF

The node broadcasts the drone pose as `map` to `px4`, and consumes the TF tree for body-frame
conversions.

---

## Config

[`config/px4_vslam_reactor.yaml`](config/px4_vslam_reactor.yaml) is loaded by
[`px4_vslam/launch/vslam.launch.py`](../px4_vslam/launch/vslam.launch.py). Every key is also
declared in the node, so the node runs if the file is absent.

| Parameter | Units | Controls |
|---|---|---|
| `vslam_stabilization_time` | s | Wait window after a reset before accepting new odometry. |
| `lin_vel_gate` | m/s | Linear-velocity ceiling for jump detection. |
| `ang_vel_gate_dps` | deg/s | Angular-velocity ceiling for jump detection. |
| `VO_rate_lim` + `VO_pos_delta_lim` | s, m | Slow-jump detection; both must hold to reject. |
| `sync_cache_sz` | count | Cache depth for PX4-to-VSLAM time alignment. |
| `align_yaw_deg` | deg | Settle-exit yaw tolerance. |
| `align_pos_m` | m | Settle-exit 3D position tolerance. |
| `set_origin_settle_time` | s | Injection window before the origin is re-injected. |
| `set_pose_max_odom_age` | s | Max age of the PX4 odom sample seeding a re-seat. |
| `fmu_stamp_max_skew_s` | s | Max \|now - FMU stamp\| before re-stamping outputs. |
| `cadence_gate_s` | s | VO header-stamp gap counted as degraded cadence. |
| `cadence_gate_hard_s` | s | Absolute gap that engages the gate on its own. |
| `cadence_gate_sustained_samples` | count | Consecutive degraded gaps that engage the gate. |
| `cadence_release_samples` | count | Consecutive nominal frames that release the gate. |
| `set_pose_busy_timeout_s` | s | Clear a hung re-seat if `SetSlamPose` never returns. |
| `reseat_burst_max` | count | Committed jump re-seats allowed inside the burst window. |
| `reseat_burst_window_s` | s | Rolling window for the burst count. |
| `ev_silence_max_s` | s | EV output silence, with frames arriving, that latches `vo_healthy` false. |

To change a value, edit the YAML and restart the launch. The package is built with
`--symlink-install`, so no rebuild is needed.

---

## Running

The node normally comes up with the VSLAM stack:

```bash
ros2 launch px4_vslam vslam.launch.py
```

Standalone, against an already running VSLAM graph:

```bash
ros2 run px4_vslam_reactor vslam_reactor_node
```

---

## Dependencies

ROS packages are declared in [`package.xml`](package.xml): `isaac_ros_visual_slam` and
`isaac_ros_visual_slam_interfaces`, `px4_vslam`, `px4_msgs`, `std_msgs`, `std_srvs`, `nav_msgs`,
`geometry_msgs`, `sensor_msgs`, `tf2_ros` and `message_filters`. Python dependencies are
`python3-numpy` and `python3-scipy`.
