# px4_vslam

Launch package for the RealSense-based visual SLAM stack (Intel D43X stereo cameras).
Wraps three RealSense drivers + [Isaac ROS Visual SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam) + a PX4 bridge into a single `ros2 launch`.

- **`vslam.launch.py`**: orchestrates the full stack.
- **`vio_transform`** (C++ node): bridges VSLAM odometry into PX4 over uXRCE-DDS.
- **`config/vslam_config.template.yaml`**: tracked fleet defaults for the RealSense drivers and the SLAM node.
- **`config/vslam_config.yaml`**: the live per-drone file (template + this drone's serials); git-ignored and regenerated.

Pose-correction / SLAM-reset logic lives in the sibling [`px4_vslam_reactor`](../px4_vslam_reactor/) package.

---

## Launch

Normal operation goes through `/arid_supervisor/vslam_enable` (camera-proven gate + landed interlock, see [`arid_supervisor`](../arid_supervisor/README.md)). Direct launch is the unmanaged dev path:

```bash
ros2 launch px4_vslam vslam.launch.py
```

In order, the launch:

1. Waits for `/robot_description` (blocks with `[vslam] Waiting for /robot_description ...` until the drone's `robot_state_publisher` is up).
2. Starts `vslam_container` with three `realsense2_camera::RealSenseNodeFactory` nodes (`left_realsense`, `front_realsense`, `right_realsense`) and `VisualSlamNode` (6-stream stereo multicam: one IR pair per camera).
3. Starts `vslam_reactor_node` (see the reactor README).
4. Starts `vio_transform`.
5. Starts `vslam_sentry` (per-camera stream health + targeted `hardware_reset`, see [`vslam_sentry`](../vslam_sentry/README.md)).

The three camera drivers and `VisualSlamNode` share one `component_container_mt` process for intra-process comms; `vslam_reactor_node`, `vio_transform` and `vslam_sentry` run as separate processes.

---

## Config: `config/vslam_config.yaml`

Template/live split: `vslam_config.template.yaml` is tracked and carries the fleet structure and tunables. `vslam_config.yaml` is the live file, git-ignored, and `scripts/config_realsense.sh` reseeds it from the template on every run while re-splicing this drone's three serials. Tunable edits belong in the template — live-only edits are lost on the next reseed. Both ship blank `serial_no`.

Single YAML keyed by node name: three identical RealSense blocks (`left_`, `front_`, `right_realsense`) and one SLAM block. Each camera streams `infra1` + `infra2` at 640x360x60; colour, depth, IMU, the IR emitter and the rs2 syncer are all off.

```yaml
left_realsense/left_realsense_link:
  ros__parameters:
    serial_no: "<camera-serial>"
    enable_infra1: true
    enable_infra2: true
    enable_color: false
    enable_depth: false
    enable_sync: false
    depth_module: { profile: '640x360x60', emitter_enabled: 0 }

visual_slam_node:
  ros__parameters:
    map_frame:  'map'
    odom_frame: 'odom'
    base_frame: 'base_link'
    imu_frame:  'autopilot'
    num_cameras: 6
    min_num_images: 4
    stale_stream_timeout_ms: 100.0
    camera_optical_frames:
      - 'front_realsense_infra1_optical_frame'
      - 'front_realsense_infra2_optical_frame'
      - 'left_realsense_infra1_optical_frame'
      - 'left_realsense_infra2_optical_frame'
      - 'right_realsense_infra1_optical_frame'
      - 'right_realsense_infra2_optical_frame'
```

`min_num_images: 4` plus `stale_stream_timeout_ms: 100.0` keep VO alive through a full single-camera loss: a stream silent 100 ms in the stamp domain stops blocking set emission, and sets continue on the two surviving stereo pairs. Both apply only after cuVSLAM init — init still needs one `camera_info` from all six streams.

To change anything: edit the template, reseed, rebuild (`colcon build --packages-select px4_vslam --symlink-install`), source install, relaunch. With `--symlink-install`, YAML-only edits need no rebuild.

### Common tweaks

| Setting | Where | When to change |
|---|---|---|
| `serial_no` per camera | realsense blocks | Camera swap / multiple boards; written by `config_realsense.sh`, not by hand |
| `depth_module.profile` | realsense blocks | Trade FPS vs resolution; never 90 fps (that service interval stalls the USB bus) |
| `min_num_images` | `visual_slam_node` | How many of the 6 streams may drop before VO stops |
| `base_frame`, `imu_frame` | `visual_slam_node` | Must match frames in the published TF tree |
| `camera_optical_frames` | `visual_slam_node` | Must match `<camera_name>_infra{1,2}_optical_frame` from the drivers |
| `enable_imu_fusion` | `visual_slam_node` | Keep `false`: the RealSense IMU streams are disabled (`enable_gyro`/`enable_accel`) and the `vio_transform` IMU republisher is commented out, so no IMU reaches `visual_slam/imu`; PX4 owns fusion |

---

## `vio_transform` node

Converts the filtered VSLAM solution into the PX4 visual-odometry message. Forwards the latched `/reactor/vio_reset_epoch` into `VehicleOdometry.reset_counter` so EKF2 re-anchors on a committed re-seat instead of gating the discontinuity.

| Subscribed | Type |
|---|---|
| `/visual_slam/filt_slam_odometry` | `nav_msgs/Odometry` |
| `/visual_slam/status` | `isaac_ros_visual_slam_interfaces/VisualSlamStatus` |
| `/reactor/vio_reset_epoch` | `std_msgs/UInt8` (transient-local) |

| Published | Type |
|---|---|
| `/fmu/in/vehicle_visual_odometry` | `px4_msgs/VehicleOdometry` |

---

## Dependencies

ROS packages (see [`package.xml`](package.xml)): `isaac_ros_visual_slam` + `isaac_ros_visual_slam_interfaces`, `px4_msgs`, tf2 stack, `nav_msgs`, `sensor_msgs`, `std_msgs`, `geometry_msgs`. `realsense2_camera`.

Runtime:

- `robot_state_publisher` publishing `/robot_description` and `/tf_static` with the frames referenced in the YAML.
- uXRCE-DDS client connected to PX4.
- RealSense cameras on USB with serials matching the YAML.
