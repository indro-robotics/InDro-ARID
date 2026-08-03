# px4_vslam

This package launches the RealSense visual SLAM stack and bridges its solution into PX4. One `ros2 launch` brings up the RealSense driver, [Isaac ROS Visual SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam), the reactor and the PX4 bridge.

- `vslam.launch.py`: the stack launch graph.
- `vio_transform`: C++ node publishing the filtered SLAM solution to PX4 over uXRCE-DDS.
- `config/vslam_config.template.yaml`: tracked fleet defaults for the driver and the SLAM node.
- `config/vslam_config.yaml`: the live per-drone file, untracked and regenerated.

Filtering and re-seat logic live in [`px4_vslam_reactor`](../px4_vslam_reactor/).

---

## Launch

Normal operation goes through `/arid_supervisor/vslam_enable`, which adds the camera-proven gate and the landed interlock (see [`arid_supervisor`](../arid_supervisor/README.md)). Direct launch is the unmanaged development path.

```bash
ros2 launch px4_vslam vslam.launch.py
```

The launch runs four steps in order.

1. Waits for `/robot_description`, logging `[vslam] Waiting for /robot_description from host-side arid_description...` until the host `robot_state_publisher` is up.
2. Starts `vslam_container` with the RealSense driver node (`front_realsense`) and `VisualSlamNode`, running two-stream stereo SLAM over the front IR pair.
3. Starts `vslam_reactor_node`.
4. Starts `vio_transform`.

The camera driver and `VisualSlamNode` share one `component_container_mt` process for intra-process comms, while `vslam_reactor_node` and `vio_transform` run as separate processes. Container shutdown allows up to 25 s for the sensor close to complete.

---

## Config

`vslam_config.template.yaml` is tracked and carries the fleet structure and tunables with a blank `serial_no` field. `vslam_config.yaml` is the live per-drone file: untracked, rewritten from the template by `config_realsense` on every run with this drone's serial re-spliced in. Tunable edits belong in the template, since live-only edits are lost at the next reseed.

The file is a single YAML keyed by node name, with one RealSense block (`front_realsense`) and one SLAM block. The camera streams `infra1` and `infra2` at 640x360x60; colour, depth, the IMU, the IR emitter and the rs2 syncer are all off.

```yaml
front_realsense/front_realsense_link:
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
    num_cameras: 2
    min_num_images: 2
    stale_stream_timeout_ms: 100.0
    camera_optical_frames:
      - 'front_realsense_infra1_optical_frame'
      - 'front_realsense_infra2_optical_frame'
```

`min_num_images: 2` requires both IR streams, so no stream loss is survivable, and `stale_stream_timeout_ms: 100.0` has no effect at one camera: dropping a stale stream leaves one image, still below the minimum. Both apply only after cuVSLAM initialization, which still needs one `camera_info` from both streams.

> Do not select a 90 fps profile. The 90 fps USB service interval stalls the bus.

Change a value by editing the template, running `config_realsense --reseed-only`, then relaunching. The workspace is symlink-installed, so a YAML-only change needs no rebuild.

### Common tweaks

These keys change during development.

| Setting | Where | When to change |
|---|---|---|
| `serial_no` | realsense block | Camera swap; written by `config_realsense`, not by hand. |
| `depth_module.profile` | realsense block | Trading FPS against resolution. |
| `min_num_images` | `visual_slam_node` | Minimum streams per set before VO emits. |
| `stale_stream_timeout_ms` | `visual_slam_node` | How long a stream may be silent before it stops blocking set emission. |
| `base_frame`, `imu_frame` | `visual_slam_node` | Must match frames in the published TF tree. |
| `camera_optical_frames` | `visual_slam_node` | Must match `<camera_name>_infra{1,2}_optical_frame` from the driver. |
| `enable_imu_fusion` | `visual_slam_node` | Keep `false`: no IMU reaches `visual_slam/imu`, and PX4 does the fusion. |

---

## vio_transform

`vio_transform` converts the filtered VSLAM solution into the PX4 visual-odometry message, rotating the FLU pose into PX4 FRD and reporting velocity in the body FRD frame. The VSLAM tracking state becomes `quality`, and the latched `/reactor/vio_reset_epoch` becomes `VehicleOdometry.reset_counter`, so EKF2 re-anchors on a committed origin seat instead of gating the discontinuity.

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
- One RealSense camera on USB with a serial matching the YAML.
