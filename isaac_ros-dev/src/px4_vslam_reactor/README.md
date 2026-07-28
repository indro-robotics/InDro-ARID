# px4_vslam_reactor

Coordinates the external visual-SLAM source with PX4. Watches the incoming SLAM solution, gates bad samples (tracking jumps, teleports, mis-syncs), and re-anchors SLAM to the PX4 solution when the two diverge.

All sensor fusion is in PX4: the reactor does not fuse IMU data, run a filter, or cross-check against inertial state. It provides a jump-free visual-odometry stream and signals EKF2 when that stream has moved discontinuously.

Launched by the sibling [`px4_vslam`](../px4_vslam/) package. One node: `vslam_reactor_node`.

---

## Function

1. **Filters the SLAM odometry stream.** Velocity and position-delta gates reject teleports and tracking glitches. Clean stream republished on `/visual_slam/filt_slam_odometry`.
2. **Detects reset-misalignment.** After a `SetSlamPose` call, keeps injecting the stream until the VSLAM pose agrees with the PX4 estimator within tolerance; if the settle window elapses without alignment, the origin is re-injected and the window restarts.
3. **Hosts a force-reset service** (`visual_slam/set_reactor_pose`, `std_srvs/Trigger`): commands a re-injection of the zero origin (0 position, 0 yaw, gravity-aligned roll/pitch) at any time.

All thresholds are configurable via [`config/px4_vslam_reactor.yaml`](config/px4_vslam_reactor.yaml).

## Gates

Every VSLAM frame runs through `slam_odom_callback`. A frame is forwarded to PX4 only when VSLAM reports tracking (`vslam_status == 1`) and the reactor is not mid-reseat (`vslam_busy` clear).

- **Velocity / jump gate** (`odom_velocity_gate`). Rejects a frame exceeding `lin_vel_gate`, `ang_vel_gate_dps`, or the `VO_pos_delta_lim`/`VO_rate_lim` slow-jump pair. A rejection triggers an in-flight re-seat.
- **Displacement / settle gate** (`odom_displacement_gate`). After an origin injection, keeps injecting until VSLAM and FMU poses agree within `align_yaw_deg` and `align_pos_m`; re-injects if `set_origin_settle_time` elapses first.
- **Cadence gate** (`_cadence_update`). Withholds VO from EKF2 while cuVSLAM stamp cadence is degraded. Two-tier engage: one header-stamp gap at or over `cadence_gate_hard_s` (0.40 s, EKF2's own arrival de-latch point), **or** `cadence_gate_sustained_samples` (5) consecutive gaps in the `cadence_gate_s`..`cadence_gate_hard_s` band — any nominal sample resets that streak, so lone 150-400 ms gaps are left to EKF2 as valid-late poses. `cadence_release_samples` consecutive nominal frames release (no timed escape). Stamp gaps over 5 s are treated as anomalies (reseed, no engage). While gated all settles are withheld and the settle countdown is paused; jump detection stays live (a genuine jump still re-seats and bumps the epoch). The withhold never bumps the epoch. State latched on `/reactor/cadence_gated`; engagements count on `/reactor/cadence_gate_count`.

A `SetSlamPose` that never returns leaves `vslam_busy` set and suppresses EV forever; the `set_pose_busy_timeout_s` watchdog clears a hung call.

## Epochs and reset_counter

Each committed re-seat bumps the epoch on `/reactor/vio_reset_epoch` (`std_msgs/UInt8`, transient-local). `vio_transform` forwards it into `VehicleOdometry.reset_counter`, telling EKF2 to reset its EV-aided states onto the new pose instead of gating the discontinuity.

The epoch bumps only on the `SetSlamPose` success path: bumping earlier would stamp the new counter onto a pre-reseat pose, and a failed re-seat would consume an epoch. `vslam_busy` must clear on every path (success, refusal, exception) or EV publishing is suppressed indefinitely.

## Origin injection

On init (pre-takeoff datum) the origin is zero position and zero yaw, keeping FMU roll/pitch so the frame stays gravity-aligned: the magnetometer is disabled, so the heading datum is arbitrary, and a 0-yaw origin removes a re-anchor jump EKF2 would reject. In-flight re-seats keep the FMU's full orientation so the re-anchor stays near zero.

## FMU stamp clamp

`px4_odom_callback` re-stamps the `map`->`px4` TF and `/reactor/drone_odom` / `/reactor/drone_pose` with node time when the FMU timestamp skews more than `fmu_stamp_max_skew_s` from now: uXRCE timesync excursions can pass boot-relative or future stamps through and corrupt tf2 buffers. Skew-vs-now, not monotonicity: a future stamp is still monotonic. Normal-path stamps are untouched. Excursions are counted and warned at a throttle.

## Lifecycle invariant

`est_status_sub` (watching `/fmu/out/estimator_status_flags` for `cs_ev_pos`) is never destroyed: fusion detection must be able to re-fire after a re-arm, or the velocity and displacement gates are bypassed on later epochs. On detection it records `last_set_pose_time` so the settle window measures EKF2-fusion time, not FMU downtime.

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
| `/visual_slam/filt_slam_odometry` | `nav_msgs/Odometry` | Jump-filtered VSLAM odometry; downstream consumers should prefer this. |
| `/reactor/drone_odom` | `nav_msgs/Odometry` | PX4 odom in ROS conventions (FRD to FLU); cached for alignment checks. |
| `/reactor/drone_pose` | `geometry_msgs/PoseStamped` | Same, pose only; for RViz or Foxglove. |
| `/reactor/vio_reset_epoch` | `std_msgs/UInt8` (latched) | Committed-re-seat epoch. |
| `/reactor/cadence_gated` | `std_msgs/Bool` (latched) | True while VO is being withheld for degraded cadence. |
| `/reactor/cadence_gate_count` | `std_msgs/UInt32` (latched) | Cumulative cadence-gate engagements; post-flight forensics. |

## Services

| Interface | Direction | Type | Purpose |
|---|---|---|---|
| `visual_slam/set_reactor_pose` | hosts | `std_srvs/Trigger` | Force a zero-origin re-injection. |
| `visual_slam/set_slam_pose` | calls | `isaac_ros_visual_slam_interfaces/SetSlamPose` | Reset call to the VSLAM backend. |

## TF

Broadcasts the drone pose (`map`->`px4`); consumes the TF tree for body-frame conversions.

---

## Config: `config/px4_vslam_reactor.yaml`

Loaded by [`px4_vslam/launch/vslam.launch.py`](../px4_vslam/launch/vslam.launch.py). Defaults are also declared in the node, so it runs if the file is absent.

| Parameter | Units | Controls |
|---|---|---|
| `vslam_stabilization_time` | s | Wait-window after a reset before accepting new odometry. |
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
| `cadence_gate_sustained_samples` | count | Consecutive `cadence_gate_s`..`cadence_gate_hard_s` gaps that engage the gate. |
| `cadence_release_samples` | count | Consecutive nominal frames to release the cadence gate. |
| `set_pose_busy_timeout_s` | s | Clear a hung `vslam_busy` if SetSlamPose never returns. |

To change a value: edit the YAML, rebuild (or re-source install if built with `--symlink-install`), restart the launch.

---

## Running

Normally launched by the VSLAM stack:

```bash
ros2 launch px4_vslam vslam.launch.py
```

Standalone (testing against a running VSLAM graph):

```bash
ros2 run px4_vslam_reactor vslam_reactor_node
```

---

## Dependencies

ROS packages (declared in [`package.xml`](package.xml)): `isaac_ros_visual_slam` + `isaac_ros_visual_slam_interfaces`, `px4_vslam`, `px4_msgs`, `std_msgs`, `std_srvs`, `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `tf2_ros`, `message_filters`.

Python: `python3-numpy`, `python3-scipy`.
