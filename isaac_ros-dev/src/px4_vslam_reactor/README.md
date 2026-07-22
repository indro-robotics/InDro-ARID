# px4_vslam_reactor

Coordinates external visual-SLAM sources from the [`px4_vslam`](../px4_vslam/) and [`isaac_ros_visual_slam`](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam) packages with PX4. Sits in the middle of the external-odometry feedback loop. Watches the incoming SLAM solution, gates bad samples (tracking jumps, teleports, mis-syncs), and re-anchors SLAM to the PX4 solution when the two diverge.

PX4 owns all sensor fusion: the reactor never fuses IMU, never runs its own filter, and never cross-checks against inertial data. Its only job is to hand EKF2 a clean, jump-free visual-odometry stream and to tell EKF2 when the stream has discontinuously moved so EKF2 re-anchors instead of rejecting the jump as an innovation.

Launched alongside the SLAM graph by the sibling [`px4_vslam`](../px4_vslam/) package. Contains one node: `vslam_reactor_node`.

---

## Function

1. **Filters the SLAM odometry stream.** Applies velocity and position-delta gates to reject samples that look like teleports or tracking glitches. Clean stream republished on `/visual_slam/filt_slam_odometry`.
2. **Detects reset-misalignment.** After a `SetSlamPose` call, the reactor keeps injecting the stream until the VSLAM backend's new pose agrees with the PX4 estimator within a configurable yaw and translation tolerance. If the settle window elapses without alignment, the origin is re-injected and the settle window restarts.
3. **Exposes a force-reset service** (`visual_slam/set_reactor_pose`, `std_srvs/Trigger`) so external callers (ground-station buttons or scripts) can command a re-injection of the zero origin (0 position, 0 yaw, gravity-aligned roll/pitch) at any time.

All thresholds (velocity gates, jump detection, alignment tolerances, settle window, stamp-skew clamp, sync cache depth) are configurable via [`config/px4_vslam_reactor.yaml`](config/px4_vslam_reactor.yaml).

---

## Gates

Every VSLAM odometry frame runs through `slam_odom_callback`. A frame is only forwarded to PX4 (`/visual_slam/filt_slam_odometry`) when VSLAM reports tracking (`vslam_status == 1`) and the reactor is not mid-reseat (`vslam_busy` clear).

- **Velocity / jump gate** (`odom_velocity_gate`). Rejects a frame whose implied linear velocity exceeds `lin_vel_gate`, whose angular velocity exceeds `ang_vel_gate_dps`, or whose position step exceeds `VO_pos_delta_lim` across an interval longer than `VO_rate_lim`. A rejected frame is treated as a VSLAM jump and triggers an in-flight re-seat.
- **Displacement / settle gate** (`odom_displacement_gate`). After an origin injection, the reactor keeps injecting the stream until the VSLAM pose and the FMU pose agree within `align_yaw_deg` and `align_pos_m`. If the `set_origin_settle_time` window elapses without alignment, the origin is re-injected and the settle window restarts.

## Epochs and reset_counter

Each committed re-seat bumps `_vio_reset_epoch`, published on `/reactor/vio_reset_epoch` (`std_msgs/UInt8`, transient-local, so a restarted `vio_transform` latches the current value). `vio_transform` forwards it into `VehicleOdometry.reset_counter`, which tells EKF2 to reset its EV-aided states onto the new pose instead of gating the discontinuity as an innovation.

The epoch bumps in `service_response_callback` on the `SetSlamPose` **success** path only. Bumping before the re-seat completes would let `vio_transform` stamp the new `reset_counter` onto a pre-reseat pose (EKF2 re-anchors onto stale data), and a failed re-seat would burn an epoch with no pose change. `vslam_busy` is set at the call site and cleared in the callback's `finally` block on every path (success, service refusal, exception): if it is not cleared, EV publishing is suppressed forever and the reactor wedges.

## Origin injection

On init (the pre-takeoff datum) the reactor injects an origin at zero position and zero yaw, keeping roll and pitch from the FMU so the frame stays gravity-aligned. Zeroing yaw is deliberate: magnetometer is disabled, so the heading datum is arbitrary, and a clean 0-yaw origin removes a re-anchor jump EKF2 would otherwise reject. In-flight re-seats (non-init) keep the FMU's full orientation, so the re-anchor against EKF2's current estimate stays near zero.

## FMU stamp clamp

`px4_odom_callback` re-stamps the `map`->`px4` TF and `/reactor/drone_odom` / `/reactor/drone_pose` with node time when the FMU timestamp skews more than `fmu_stamp_max_skew_s` from now. uXRCE timesync excursions can pass boot-relative or future stamps straight through and poison tf2 buffers for the `px4` frame. The check is skew-vs-now, not a monotonicity guard: a future rogue stamp is still monotonic. Normal-path stamps are left untouched so ordering fidelity is preserved for downstream synchronizers. Excursion events are counted (`_fmu_stamp_excursions`) and warned at a throttle.

## Lifecycle: est_status_sub must never be destroyed

`est_status_callback` watches `/fmu/out/estimator_status_flags` for `cs_ev_pos` to detect when EKF2 starts fusing external vision. An earlier version destroyed this subscription on the first `True`, so fusion could never be re-detected after any re-arm and the velocity and displacement gates were bypassed on all subsequent epochs. The subscription is now kept alive for the node's whole life. On detection it records `last_set_pose_time` so the settle window measures EKF2-fusion time, not FMU downtime.

---

## Inputs

| Topic | Type | From | Used for |
|---|---|---|---|
| `/visual_slam/vis/slam_odometry` | `nav_msgs/Odometry` | Isaac VSLAM | Raw odometry stream (filtered, republished). |
| `/visual_slam/status` | `isaac_ros_visual_slam_interfaces/VisualSlamStatus` | Isaac VSLAM | Tracking quality and state. |
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | PX4 (via uXRCE-DDS) | PX4 pose for reset-alignment comparison. |
| `/fmu/out/estimator_status_flags` | `px4_msgs/EstimatorStatusFlags` | PX4 (via uXRCE-DDS) | Tracks PX4 EV-fusion health. |

## Outputs

| Topic | Type | Purpose |
|---|---|---|
| `/visual_slam/filt_slam_odometry` | `nav_msgs/Odometry` | Jump-filtered VSLAM odometry. Downstream consumers should prefer this. |
| `/reactor/drone_odom` | `nav_msgs/Odometry` | PX4 odom converted to ROS conventions (FRD to FLU), cached for reset-alignment checks. |
| `/reactor/drone_pose` | `geometry_msgs/PoseStamped` | Same as above, pose only. For RViz or Foxglove. |
| `/reactor/vio_reset_epoch` | `std_msgs/UInt8` (latched) | Committed-re-seat epoch. Forwarded by `vio_transform` into `VehicleOdometry.reset_counter`. |

## Services

| Interface | Direction | Type | Purpose |
|---|---|---|---|
| `visual_slam/set_reactor_pose` | **hosts** | `std_srvs/Trigger` | Force a re-injection of the zero origin (0 position, 0 yaw). |
| `visual_slam/set_slam_pose` | calls | `isaac_ros_visual_slam_interfaces/SetSlamPose` | Upstream reset call to the Isaac VSLAM backend. |

## TF

Publishes via `TransformBroadcaster` (drone pose in world frames). Consumes the TF tree for body-frame conversions. The drone's `robot_state_publisher` must be running with links configured correctly.

---

## Config: `config/px4_vslam_reactor.yaml`

Loaded by [`px4_vslam/launch/vslam.launch.py`](../px4_vslam/launch/vslam.launch.py) and applied to the node as standard ROS 2 parameters. Defaults are also declared in the node, so it runs if the file is absent.

| Parameter | Units | Controls |
|---|---|---|
| `vslam_stabilization_time` | s | Wait-window after a reset before accepting new odometry. |
| `lin_vel_gate` | m/s | Instantaneous linear-velocity ceiling for jump detection. |
| `ang_vel_gate_dps` | deg/s | Instantaneous angular-velocity ceiling for jump detection (converted to rad/s in the node). |
| `VO_rate_lim` + `VO_pos_delta_lim` | s, m | Slow-jump detection. Both conditions must hold to reject. |
| `sync_cache_sz` | count | `message_filters` cache depth for PX4-to-VSLAM time alignment. |
| `align_yaw_deg` | deg | Settle-exit yaw-only tolerance to declare EKF2 converged onto the injected origin (converted to rad in the node). |
| `align_pos_m` | m | Settle-exit 3D position tolerance to declare EKF2 converged onto the injected origin. |
| `set_origin_settle_time` | s | How long to keep injecting the stream for EKF2 to converge before re-injecting the origin. |
| `set_pose_max_odom_age` | s | Max age of the cached PX4 odom sample used to seed a re-seat (stale-sample freshness gate). |
| `fmu_stamp_max_skew_s` | s | Max \|now - FMU stamp\| before re-stamping outputs with node time (uXRCE timesync-excursion clamp). |

**To change a value:** edit the YAML, rebuild (or re-source install if built with `--symlink-install`), restart the launch.

---

## Running

Normally launched by the VSLAM stack:

```bash
ros2 launch px4_vslam vslam.launch.py
```

Standalone (for testing against a running VSLAM graph):

```bash
ros2 run px4_vslam_reactor vslam_reactor_node
```

---

## Dependencies

**ROS packages** (declared in [`package.xml`](package.xml)):

- [`isaac_ros_visual_slam`](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam) and `isaac_ros_visual_slam_interfaces`: the SLAM backend exposing `SetSlamPose`, and the msg/srv definitions used by the reactor's subscribers and service client.
- [`px4_vslam`](../px4_vslam/): sibling launch package. Provides the VSLAM graph this reactor supervises.
- `px4_msgs`: PX4 telemetry and command messages via uXRCE-DDS.
- `std_msgs`, `std_srvs`, `nav_msgs`, `geometry_msgs`, `sensor_msgs`: standard messages and services.
- `tf2_ros`: TF broadcaster and listener.
- `message_filters`: time-synchronized subscribers and message caches.

**Python packages:**

- `python3-numpy`, `python3-scipy`: math.
