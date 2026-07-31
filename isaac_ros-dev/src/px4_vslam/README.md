# px4_vslam

This package launches the RealSense-based visual SLAM stack and bridges its solution into PX4. One `ros2 launch` brings up three RealSense drivers, [Isaac ROS Visual SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam) and the PX4 bridge.

- `vslam.launch.py`: the full stack.
- `vio_transform`: C++ node publishing the visual-odometry message to PX4 over uXRCE-DDS.
- `config/vslam_config.template.yaml`: tracked fleet defaults for the drivers and the SLAM node.
- `config/vslam_config.yaml`: the live per-drone file, git-ignored and regenerated.

Pose-correction and SLAM-reset logic is in [`px4_vslam_reactor`](../px4_vslam_reactor/).

---

## Launch

Normal operation goes through `/arid_supervisor/vslam_enable`, which adds the camera-proven gate and the landed interlock (see [`arid_supervisor`](../arid_supervisor/README.md)). Direct launch is the unmanaged development path:

```bash
ros2 launch px4_vslam vslam.launch.py
```

The launch proceeds in order:

1. Waits for `/robot_description`, printing `[vslam] Waiting for /robot_description ...` until `robot_state_publisher` is up.
2. Starts `vslam_container` with three `realsense2_camera::RealSenseNodeFactory` nodes (`left_realsense`, `front_realsense`, `right_realsense`) and `VisualSlamNode` as a 6-stream stereo multicam, one IR pair per camera.
3. Starts `vslam_reactor_node`.
4. Starts `vio_transform`.

The three camera drivers and `VisualSlamNode` share one `component_container_mt` process for intra-process comms. The reactor and `vio_transform` run as separate processes.

---

## Config

`vslam_config.template.yaml` is tracked and carries the fleet structure and tunables. `vslam_config.yaml` is the live file: git-ignored, rewritten from the template by `scripts/config_realsense.sh` on every run, with this drone's three serials re-spliced in. Put tunable edits in the template, since live-only edits are lost at the next reseed. Both ship blank `serial_no`.

The file is a single YAML keyed by node name: three identical RealSense blocks (`left_`, `front_`, `right_realsense`) and one SLAM block. Each camera streams `infra1` and `infra2` at 640x360x60, with colour, depth, IMU, the IR emitter and the rs2 syncer all off.

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

`min_num_images: 4` with `stale_stream_timeout_ms: 100.0` keeps VO running through a full single-camera loss: a stream silent for 100 ms in the stamp domain stops blocking set emission, and sets continue on the two remaining stereo pairs. Both apply only after cuVSLAM initialization, which still needs one `camera_info` from all six streams.

Edit the template, run `config_realsense`, then restart the stack. The workspace is symlink-installed, so a YAML-only change needs no rebuild.

### Common tweaks

These are the keys that change in normal development.

| Setting | Where | When to change |
|---|---|---|
| `serial_no` | realsense blocks | Camera swap; written by `config_realsense.sh`, not by hand. |
| `depth_module.profile` | realsense blocks | Trade FPS against resolution; never 90 fps, that service interval stalls the USB bus. |
| `min_num_images` | `visual_slam_node` | How many of the 6 streams may drop before VO stops. |
| `base_frame`, `imu_frame` | `visual_slam_node` | Must match frames in the published TF tree. |
| `camera_optical_frames` | `visual_slam_node` | Must match `<camera_name>_infra{1,2}_optical_frame` from the drivers. |
| `enable_imu_fusion` | `visual_slam_node` | Keep `false`: no IMU reaches `visual_slam/imu` and PX4 does the fusion. |

---

## `vio_transform`

`vio_transform` converts the filtered VSLAM solution into the PX4 visual-odometry message. It forwards the latched `/reactor/vio_reset_epoch` into `VehicleOdometry.reset_counter`, so EKF2 re-anchors on a committed origin seat instead of gating the discontinuity.

| Subscribed | Type |
|---|---|
| `/visual_slam/filt_slam_odometry` | `nav_msgs/Odometry` |
| `/visual_slam/status` | `isaac_ros_visual_slam_interfaces/VisualSlamStatus` |
| `/reactor/vio_reset_epoch` | `std_msgs/UInt8` (transient-local) |

| Published | Type |
|---|---|
| `/fmu/in/vehicle_visual_odometry` | `px4_msgs/VehicleOdometry` |

---

## Runtime requirements

The launch needs three things present before it can produce odometry.

- `robot_state_publisher` publishing `/robot_description` and `/tf_static` with the frames named in the YAML.
- A uXRCE-DDS client connected to PX4.
- Three RealSense cameras on USB with serials matching the YAML.
