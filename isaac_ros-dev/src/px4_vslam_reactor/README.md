# px4_vslam_reactor

Coordinates external visual-SLAM sources from the [`px4_vslam`](../px4_vslam/) and [`isaac_ros_visual_slam`](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam) packages with PX4. Sits in the middle of the external-odometry feedback loop. Watches the incoming SLAM solution, gates bad samples (tracking jumps, teleports, mis-syncs), and re-anchors SLAM to the PX4 solution when the two diverge. PX4's EV fusion receives a coherent stream regardless of underlying tracking glitches.

Launched alongside the SLAM graph by the sibling [`px4_vslam`](../px4_vslam/) package. Contains one node: `vslam_reactor_node`.

---

## Function

1. **Filters the SLAM odometry stream.** Applies velocity and position-delta gates to reject samples that look like teleports or tracking glitches. Clean stream republished on `/visual_slam/filt_slam_odometry`.
2. **Detects reset-misalignment.** After a `SetSlamPose` call, the reactor verifies the VSLAM backend's new pose agrees with the PX4 estimator within a configurable rotation and translation tolerance. If not, it retries.
3. **Exposes a force-reset service** (`visual_slam/set_reactor_pose`, `std_srvs/Trigger`) so external callers (state machines, ground-station buttons) can command a re-anchor to the current PX4 pose at any time.

All thresholds (velocity gates, jump detection, stabilization delay, reset-alignment tolerances, sync cache depth) are configurable via [`config/reactor_conf.yaml`](config/reactor_conf.yaml).

---

## Inputs

| Topic | Type | From | Used for |
|---|---|---|---|
| `/visual_slam/vis/slam_odometry` | `nav_msgs/Odometry` | Isaac VSLAM | Raw odometry stream (filtered, republished). |
| `/visual_slam/status` | `isaac_ros_visual_slam_interfaces/VisualSlamStatus` | Isaac VSLAM | Tracking quality and state. |
| `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | PX4 (via uXRCE-DDS) | PX4 pose for reset-alignment comparison. |
| `/fmu/out/estimator_status_flags` | `px4_msgs/EstimatorStatusFlags` | PX4 (via uXRCE-DDS) | Tracks PX4 EV-fusion health. |
| `/px4_state_machine/fmu_lockout` | `std_msgs/Bool` (latched) | flight state machine | Safety lockout. Suspends reset activity. |

## Outputs

| Topic | Type | Purpose |
|---|---|---|
| `/visual_slam/filt_slam_odometry` | `nav_msgs/Odometry` | Jump-filtered VSLAM odometry. Downstream consumers should prefer this. |
| `/reactor/drone_odom` | `nav_msgs/Odometry` | PX4 odom converted to ROS conventions (FRD to FLU), cached for reset-alignment checks. |
| `/reactor/drone_pose` | `geometry_msgs/PoseStamped` | Same as above, pose only. For RViz or Foxglove. |

## Services

| Interface | Direction | Type | Purpose |
|---|---|---|---|
| `visual_slam/set_reactor_pose` | **hosts** | `std_srvs/Trigger` | Force a pose reset to the current PX4 pose. |
| `visual_slam/set_slam_pose` | calls | `isaac_ros_visual_slam_interfaces/SetSlamPose` | Upstream reset call to the Isaac VSLAM backend. |

## TF

Publishes via `TransformBroadcaster` (drone pose in world frames). Consumes the TF tree for body-frame conversions. The drone's `robot_state_publisher` must be running with links configured correctly.

---

## Config: `config/reactor_conf.yaml`

Loaded directly by the node at startup from its own share dir. No launch-file wiring needed: the config is fully self-contained within this package. Every field is documented inline in the YAML.

| Parameter | Units | Controls |
|---|---|---|
| `vslam_stabilization_time` | s | Wait-window after a reset before accepting new odometry. |
| `lin_vel_gate` | m/s | Instantaneous linear-velocity ceiling for jump detection. |
| `ang_vel_gate` | rad/s | Instantaneous angular-velocity ceiling for jump detection. |
| `VO_rate_lim` + `VO_pos_delta_lim` | s, m | Slow-jump detection. Both conditions must hold to reject. |
| `sync_cache_sz` | count | `message_filters` cache depth for PX4-to-VSLAM time alignment. |
| `quat_delta_theta` | rad | Reset-alignment rotation tolerance (default about 5°). |
| `displacement_delta` | m | Reset-alignment translation tolerance. |

**To change a value:** edit the YAML, rebuild (or re-source install if built with `--symlink-install`), restart the launch. On startup the reactor logs `Loaded reactor_conf.yaml (8 tunables)`.

**If the YAML is missing or malformed:** the node warns or errors and falls back to hardcoded defaults, so it still starts.

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
- `ament_index_python`: resolves the package's `share/` path at runtime to locate `reactor_conf.yaml`.

**Python packages:**

- `python3-numpy`, `python3-scipy`, `python3-yaml`: math and YAML config loading.
