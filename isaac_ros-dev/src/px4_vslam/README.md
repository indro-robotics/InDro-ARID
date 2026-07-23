# px4_vslam

Launch package for the RealSense-based visual SLAM stack (Intel D43X stereo camera).
Wraps the RealSense driver + [Isaac ROS Visual SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam) + a PX4 bridge into a single `ros2 launch`.

- **`vslam.launch.py`**: orchestrates the full stack.
- **`vio_transform`** (C++ node): bridges VSLAM odometry into PX4 over uXRCE-DDS.
- **`config/vslam_config.yaml`**: parameters for the RealSense driver and the SLAM node.

Pose-correction / SLAM-reset logic lives in the sibling [`px4_vslam_reactor`](../px4_vslam_reactor/) package.

---

## Launch

Normal operation goes through `/arid_supervisor/vslam_enable` (camera-proven gate + landed interlock, see [`arid_supervisor`](../arid_supervisor/README.md)). Direct launch is the unmanaged dev path:

```bash
ros2 launch px4_vslam vslam.launch.py
```

In order, the launch:

1. Waits for `/robot_description` (blocks with `[vslam] Waiting for /robot_description ...` until the drone's `robot_state_publisher` is up).
2. Starts `vslam_container` with one `realsense2_camera::RealSenseNodeFactory` node (`front_realsense`) and `VisualSlamNode` (2-stream stereo SLAM on the front IR pair).
3. Starts `vslam_reactor_node` (see the reactor README).
4. Starts `vio_transform`.

The camera driver and `VisualSlamNode` share one `component_container_mt` process for intra-process comms; `vslam_reactor_node` and `vio_transform` run as separate processes.

---

## Config: `config/vslam_config.yaml`

Single YAML keyed by node name: one RealSense block and one SLAM block. Colour is enabled for downstream consumers.

```yaml
front_realsense/front_realsense_link:
  ros__parameters:
    serial_no: "<camera-serial>"
    enable_infra1: true
    enable_infra2: true
    enable_color: true
    depth_module: { profile: '640x360x90' }
    rgb_camera:    { profile: '1280x720x15' }

visual_slam_node:
  ros__parameters:
    map_frame:  'map'
    odom_frame: 'odom'
    base_frame: 'base_link'
    imu_frame:  '<imu-frame>'
    num_cameras: 2
    camera_optical_frames:
      - 'front_realsense_infra1_optical_frame'
      - 'front_realsense_infra2_optical_frame'
```

To change anything: edit the YAML, rebuild (`colcon build --packages-select px4_vslam --symlink-install`), source install, relaunch. With `--symlink-install`, YAML-only edits need no rebuild.

### Common tweaks

| Setting | Where | When to change |
|---|---|---|
| `serial_no` | realsense block | Camera swap / multiple boards |
| `depth_module.profile` / `rgb_camera.profile` | realsense block | Trade FPS vs resolution |
| `base_frame`, `imu_frame` | `visual_slam_node` | Must match frames in the published TF tree |
| `camera_optical_frames` | `visual_slam_node` | Must match `<camera_name>_infra{1,2}_optical_frame` from the driver |
| `enable_imu_fusion` | `visual_slam_node` | Keep `false`: the `vio_transform` IMU republisher is commented out pending re-test, and the RealSense IMU must not bias EKF2 via the visual-odom feedback path |

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
- RealSense camera on USB with serial matching the YAML.
