# px4_vslam

This package launches the RealSense-based visual SLAM stack and bridges its solution into PX4. One
`ros2 launch` brings up the RealSense driver, [Isaac ROS Visual
SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam), the reactor and the
PX4 bridge.

- **`vslam.launch.py`**: the stack launch graph.
- **`vio_transform`**: C++ node publishing VSLAM odometry to PX4 over uXRCE-DDS.
- **`config/vslam_config.template.yaml`**: tracked fleet config with a blank `serial_no`.
- **`config/vslam_config.yaml`**: the live per-drone file, reseeded from the template by
  `config_realsense`, which splices in this drone's serial.

Pose correction and SLAM re-seat logic live in [`px4_vslam_reactor`](../px4_vslam_reactor/).

---

## Launch

Normal operation goes through `/arid_supervisor/vslam_enable`, which adds the camera-proven gate
and the landed interlock. Direct launch is the unmanaged development path.

```bash
ros2 launch px4_vslam vslam.launch.py
```

In order, the launch:

1. Waits for `/robot_description`, logging `[vslam] Waiting for /robot_description ...` until the
   host `robot_state_publisher` is up.
2. Starts `vslam_container` holding the `front_realsense` driver node and `VisualSlamNode`, doing
   two-stream stereo SLAM on the front IR pair.
3. Starts `vslam_reactor_node`.
4. Starts `vio_transform`.

The camera driver and `VisualSlamNode` share one `component_container_mt` process for
intra-process comms, while `vslam_reactor_node` and `vio_transform` run as separate
processes. The container gets a 25 s SIGTERM grace so the sensor close finishes before SIGKILL.

---

## Config

`config/vslam_config.yaml` is a single YAML keyed by node name, with one RealSense block and one
SLAM block. Only the IR pair streams; colour, depth and the IMU are off.

Edit the template rather than the live file: `config_realsense` overwrites `vslam_config.yaml`
from the template on every run.

```yaml
front_realsense/front_realsense_link:
  ros__parameters:
    serial_no: "<camera-serial>"
    enable_infra1: true
    enable_infra2: true
    enable_color: false
    enable_depth: false
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

> Do not select a 90 fps profile. The 90 fps USB service interval stalls the bus.

To change a value: edit the template, run `config_realsense --reseed-only`, then relaunch. The
package is built with `--symlink-install`, so YAML-only edits need no rebuild.

### Common tweaks

| Setting | Where | When to change |
|---|---|---|
| `serial_no` | realsense block | Camera swap; set it with `config_realsense`, not by hand. |
| `depth_module.profile` | realsense block | Trading FPS against resolution; stay off 90. |
| `base_frame`, `imu_frame` | `visual_slam_node` | Must match frames in the published TF tree. |
| `camera_optical_frames` | `visual_slam_node` | Must match `<camera_name>_infra{1,2}_optical_frame`. |
| `min_num_images` | `visual_slam_node` | `2` requires both IR streams, so no stream loss is survivable. |
| `stale_stream_timeout_ms` | `visual_slam_node` | Inert at one camera: dropping a stale stream leaves 1 below `min_num_images`. |
| `enable_imu_fusion` | `visual_slam_node` | Keep `false`; the RealSense IMU must not bias EKF2 through the visual-odometry path. |

---

## vio_transform

`vio_transform` converts the filtered VSLAM solution into the PX4 visual-odometry message. It
forwards the latched `/reactor/vio_reset_epoch` into `VehicleOdometry.reset_counter` so EKF2
re-anchors on a committed origin seat instead of gating the discontinuity.

| Subscribed | Type |
|---|---|
| `/visual_slam/filt_slam_odometry` | `nav_msgs/Odometry` |
| `/visual_slam/status` | `isaac_ros_visual_slam_interfaces/VisualSlamStatus` |
| `/reactor/vio_reset_epoch` | `std_msgs/UInt8` (latched) |

| Published | Type |
|---|---|
| `/fmu/in/vehicle_visual_odometry` | `px4_msgs/VehicleOdometry` |

---

## Dependencies

ROS packages are declared in [`package.xml`](package.xml): `isaac_ros_visual_slam` and
`isaac_ros_visual_slam_interfaces`, `realsense2_camera`, `px4_msgs`, the tf2 stack, `nav_msgs`,
`sensor_msgs`, `std_msgs` and `geometry_msgs`.

At runtime the launch needs `robot_state_publisher` publishing `/robot_description` and
`/tf_static` with the frames named in the YAML, a uXRCE-DDS client connected to PX4, and the
RealSense on USB with the serial the YAML names.
