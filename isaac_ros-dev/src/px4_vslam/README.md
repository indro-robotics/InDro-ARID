# px4_vslam

Launch package for a RealSense-based visual-inertial SLAM stack. Compatible with Intel RealSense **D43X** and **D45X** series stereo depth cameras. Wraps multiple RealSense drivers + [Isaac ROS Visual SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam) + a PX4 bridge into a single `ros2 launch`.

Contains:

- **`vslam.launch.py`** — orchestrates the full stack (see [Launch](#launch) below).
- **`vio_transform` (C++ node)** — bridges VSLAM odometry into PX4 via the [uXRCE-DDS middleware](https://docs.px4.io/main/en/middleware/uxrce_dds.html).
- **`config/vslam_config.yaml`** — parameter overrides for the RealSense drivers and the Isaac Visual SLAM node.

Pose-correction / SLAM-reset logic lives in the sibling [`px4_vslam_reactor`](../px4_vslam_reactor/) package.

---

## Launch

```bash
ros2 launch px4_vslam vslam.launch.py
```

In order, the launch:

1. **Waits for `/robot_description`** to appear on the ROS graph. The drone's `robot_state_publisher` must be running with its links configured correctly — VSLAM cannot start without the frames it's configured against. The launch blocks here with `[vslam] Waiting for /robot_description ...` until the latched message arrives.
2. **Starts a composable-node container** (`vslam_container`) with:
   - Three `realsense2_camera::RealSenseNodeFactory` nodes (`left_realsense`, `front_realsense`, `right_realsense`).
   - `nvidia::isaac_ros::visual_slam::VisualSlamNode` — 6-camera stereo-multicam SLAM (one IR stereo pair per camera).
3. **Starts `vslam_reactor_node`** — external-odom supervisor (see the reactor's README).
4. **Starts `vio_transform`** — VSLAM → PX4 bridge.

All of the above runs inside one `component_container_mt` process for intra-process comms between the cameras and the SLAM node.

---

## Config: `config/vslam_config.yaml`

Single YAML keyed by node name. Three RealSense blocks (per-camera) and one SLAM block:

```yaml
left_realsense/left_realsense_link:
  ros__parameters:
    serial_no: "<camera-serial>"
    enable_infra1: true
    enable_infra2: true
    depth_module: { profile: '640x360x90' }
    # ...

front_realsense/front_realsense_link:    # similar (colour enabled by default)
right_realsense/right_realsense_link:    # similar

visual_slam_node:
  ros__parameters:
    map_frame:  'map'
    odom_frame: 'odom'
    base_frame: 'base_link'       # must exist in the published TF tree
    imu_frame:  '<imu_frame>'     # must exist in the published TF tree
    num_cameras: 6
    camera_optical_frames:
      - 'front_realsense_infra1_optical_frame'
      - 'front_realsense_infra2_optical_frame'
      - 'left_realsense_infra1_optical_frame'
      - 'left_realsense_infra2_optical_frame'
      - 'right_realsense_infra1_optical_frame'
      - 'right_realsense_infra2_optical_frame'
```

**To change anything:** edit the YAML, rebuild (`colcon build --packages-select px4_vslam --symlink-install`), source install, relaunch. With `--symlink-install`, YAML-only edits take effect on relaunch without a rebuild.

### Common tweaks

| Setting | Where | Why you'd change it |
|---|---|---|
| `serial_no` per camera | realsense blocks | Camera swap / multiple boards |
| `depth_module.profile` / `rgb_camera.profile` | realsense blocks | Trade FPS vs resolution |
| `base_frame`, `imu_frame` | `visual_slam_node` | Must match frames in the published TF tree |
| `camera_optical_frames` | `visual_slam_node` | Must match `<camera_name>_infra{1,2}_optical_frame` produced by the realsense drivers |
| `enable_imu_fusion` | `visual_slam_node` | Turn on IMU-aided VSLAM (requires a valid `imu_frame` and the `vio_transform/imu` topic) |

### RealSense D4XX notes

- The launch uses only the IR stereo pair per camera for SLAM (`enable_infra1/2: true`). Color is left on for `front_realsense` for downstream consumers; the other cameras have it disabled to save bandwidth.
- **IMU fusion is off by default** (`enable_imu_fusion: false`) so the onboard RealSense IMU doesn't indirectly bias the PX4 state estimator via the visual-odom feedback path. Enable only if you understand the coupling.

---

## `vio_transform` node

C++ node that converts the VSLAM solution into the PX4 visual-odometry message and republishes the PX4 IMU for the SLAM node's optional IMU fusion.

| Subscribed | Type |
|---|---|
| `/visual_slam/tracking/odometry` | [`nav_msgs/Odometry`](https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/Odometry.msg) |
| `/visual_slam/status` | [`isaac_ros_visual_slam_interfaces/VisualSlamStatus`](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/main/isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus.msg) |
| `/fmu/out/sensor_combined` | [`px4_msgs/SensorCombined`](https://github.com/PX4/px4_msgs/blob/main/msg/SensorCombined.msg) |

| Published | Type |
|---|---|
| `/fmu/in/vehicle_visual_odometry` | [`px4_msgs/VehicleOdometry`](https://github.com/PX4/px4_msgs/blob/main/msg/VehicleOdometry.msg) |
| `/vio_transform/imu` | [`sensor_msgs/Imu`](https://docs.ros2.org/humble/api/sensor_msgs/msg/Imu.html) |

---

## Dependencies

**ROS packages** (declared in [`package.xml`](package.xml)):
- [`isaac_ros_visual_slam`](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam) + `isaac_ros_visual_slam_interfaces` — the SLAM backend (composable node + message/service definitions).
- `realsense2_camera` — RealSense driver (loaded as composable nodes).
- `px4_msgs`, `tf2`, `tf2_ros`, `nav_msgs`, `sensor_msgs`, `geometry_msgs` — standard messages/transforms.

**Runtime:**
- The drone's `robot_state_publisher` running with links configured correctly — publishes `/robot_description` and `/tf_static` with the frames referenced in `vslam_config.yaml`.
- uXRCE-DDS client running and talking to PX4.
- RealSense cameras enumerated on USB with serial numbers matching the YAML.
