# px4_vslam

This package launches the RealSense visual SLAM stack and bridges its solution into PX4. One `ros2 launch` brings up the three RealSense drivers, [Isaac ROS Visual SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam), the reactor and the PX4 bridge.

- `vslam.launch.py`: the stack launch graph.
- `vio_transform`: C++ node publishing the SLAM solution on `/fmu/in/vehicle_visual_odometry`.
- `config/vslam_config.template.yaml`: tracked fleet defaults for the drivers and the SLAM node.
- `config/vslam_config.yaml`: the live per-drone file, untracked and regenerated.

Pose correction and SLAM re-seat logic live in [`px4_vslam_reactor`](../px4_vslam_reactor/).

---

## Requirements

Three things must be up before the launch can produce odometry.

- `arid_description.service`: `/robot_description` plus the `base_link` and `autopilot` frames on `/tf_static`.
- The ARK-OS DDS agent: the `/fmu` topics on the ROS graph.
- Three RealSense cameras on USB, carrying the serials in `vslam_config.yaml`.

---

## Launch

Normal operation goes through `/arid_supervisor/vslam_enable`, which starts this launch and returns only once all three cameras are up (see [`arid_supervisor`](../arid_supervisor/README.md)). The supervisor refuses a bringup while nodes from a direct launch are on the graph.

```bash
ros2 launch px4_vslam vslam.launch.py
```

The launch waits for `/robot_description`, logging `[vslam] Waiting for /robot_description from host-side arid_description...` until the host `robot_state_publisher` is up. On that message it starts `vslam_container`, `vslam_reactor_node` and `vio_transform`.

`vslam_container` is one `component_container_mt` process carrying the three RealSense driver nodes, one per mount, and `VisualSlamNode` running six-stream stereo SLAM over one IR pair per camera. `vslam_reactor_node` and `vio_transform` are separate processes. Container shutdown allows up to 25 s for the sensor close to complete.

---

## Config

`vslam_config.template.yaml` is tracked and carries the fleet structure and tunables with blank `serial_no` fields. `vslam_config.yaml` is the live per-drone file: untracked, rewritten from the template by `config_realsense` on every run with this drone's three serials re-spliced in. Tunable edits belong in the template, since live-only edits are lost at the next reseed.

The file is a single YAML keyed by `<namespace>/<node-name>`, with three RealSense blocks (`left_realsense/left_realsense_link` and the front and right equivalents) and one `visual_slam_node` block. Each camera streams `infra1` and `infra2` at 640x360x60; colour, depth, the IMU, the IR emitter and the rs2 syncer are all off. `VisualSlamNode` runs `num_cameras: 6` over the six optical frames listed in `camera_optical_frames`.

`min_num_images: 4` with `stale_stream_timeout_ms: 100.0` keeps VO running through a full single-camera loss: a stream silent for 100 ms in the stamp domain stops blocking set emission, and sets continue on the two remaining stereo pairs. Both apply only after cuVSLAM initialization, which still needs one `camera_info` from all six streams.

> Do not select a 90 fps profile. The 90 fps USB service interval stalls the bus.

Change a value by editing the template, running `config_realsense --reseed-only`, then relaunching. The workspace is symlink-installed, so a YAML-only change needs no rebuild. The launch argument `camera_config_file` overrides the path to the live file.

### Common tweaks

These keys change during development.

| Setting | Where | When to change |
|---|---|---|
| `serial_no` | realsense blocks | Camera swap; written by `config_realsense`, not by hand. |
| `depth_module.profile` | realsense blocks | Trading FPS against resolution. |
| `min_num_images` | `visual_slam_node` | Minimum streams per set before VO emits. |
| `stale_stream_timeout_ms` | `visual_slam_node` | How long a stream may be silent before it stops blocking set emission. |
| `base_frame`, `imu_frame` | `visual_slam_node` | Must match links in the URDF: `base_link`, `autopilot`. |
| `camera_optical_frames` | `visual_slam_node` | Must match `<camera_name>_infra{1,2}_optical_frame` from the drivers. |
| `enable_imu_fusion` | `visual_slam_node` | Keep `false`: nothing publishes `visual_slam/imu`. |

---

## vio_transform

`vio_transform` converts the filtered SLAM solution into the PX4 visual-odometry message. It rotates the FLU pose into PX4 FRD, reports velocity in the body FRD frame, copies the SLAM tracking state into `quality`, and copies the latched `/reactor/vio_reset_epoch` into `reset_counter`.

| Subscribed | Type |
|---|---|
| `/visual_slam/filt_slam_odometry` | `nav_msgs/Odometry` |
| `/visual_slam/status` | `isaac_ros_visual_slam_interfaces/VisualSlamStatus` |
| `/reactor/vio_reset_epoch` | `std_msgs/UInt8` (transient-local) |

| Published | Type |
|---|---|
| `/fmu/in/vehicle_visual_odometry` | `px4_msgs/VehicleOdometry` |
