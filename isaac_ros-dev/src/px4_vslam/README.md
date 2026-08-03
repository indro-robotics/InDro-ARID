# px4_vslam

px4_vslam brings up the three-camera [Isaac ROS Visual SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam) stack and bridges its solution into PX4. The `vio_transform` node publishes `px4_msgs/VehicleOdometry` on `/fmu/in/vehicle_visual_odometry`.

---

## Prerequisites

- `arid_description.service` on the host: `/robot_description` latched, `base_link` and `autopilot` on `/tf_static`.
- The uXRCE-DDS agent, for the `/fmu` topics.
- Three RealSense cameras on USB, with the serials held in `vslam_config.yaml`.

---

## Startup

`/arid_supervisor/vslam_enable` (`std_srvs/SetBool`) runs this launch; the container aliases `initialize` and `deinitialize` call it with `true` and `false`.

```bash
ros2 launch px4_vslam vslam.launch.py
```

The launch waits for a latched `/robot_description`, then starts these nodes; shutdown allows the container 25 s to close the cameras.

> A direct launch leaves nodes the supervisor does not own. The next `vslam_enable` reaps them only with landed state proven, and refuses otherwise.

| Node | Package | Function |
| --- | --- | --- |
| `vslam_container` | `rclcpp_components` | `component_container_mt` holding the four composable nodes. |
| `left_realsense_link`, `front_realsense_link`, `right_realsense_link` | `realsense2_camera` | One driver per mount, each in its own namespace. |
| `visual_slam_node` | `isaac_ros_visual_slam` | cuVSLAM over six IR streams. |
| `vslam_reactor` | `px4_vslam_reactor` | See [`px4_vslam_reactor`](../px4_vslam_reactor/README.md). |
| `vio_transform` | `px4_vslam` | PX4 bridge. |

---

## Topics

`<cam>` is `left_realsense`, `front_realsense` or `right_realsense`.

### RealSense drivers

| Published | Type | Content |
| --- | --- | --- |
| `/<cam>/infra1/image_rect_raw` | `sensor_msgs/Image` | IR image, 640x360 at 60 fps, frame `<cam>_infra1_optical_frame`. |
| `/<cam>/infra2/image_rect_raw` | `sensor_msgs/Image` | Second IR imager, frame `<cam>_infra2_optical_frame`. |
| `/<cam>/infra1/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics, stamp and frame matched to the image. |
| `/<cam>/infra2/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics for infra2. |
| `/<cam>/infra1/metadata`, `/<cam>/infra2/metadata` | `realsense2_camera_msgs/Metadata` | Per-frame sensor metadata. |
| `/<cam>/extrinsics/depth_to_infra1`, `depth_to_infra2` | `realsense2_camera_msgs/Extrinsics` | Stream-to-stream extrinsics. |
| `/tf_static` | `tf2_msgs/TFMessage` | Camera link and optical-frame transforms. |

### visual_slam_node

Inputs carry `SENSOR_DATA` QoS, set by `image_qos`.

| Subscribed | Remapped to | Type |
| --- | --- | --- |
| `visual_slam/image_0`, `visual_slam/camera_info_0` | `/front_realsense/infra1/image_rect_raw`, `/front_realsense/infra1/camera_info` | `Image`, `CameraInfo` |
| `visual_slam/image_1`, `visual_slam/camera_info_1` | `/front_realsense/infra2/image_rect_raw`, `/front_realsense/infra2/camera_info` | `Image`, `CameraInfo` |
| `visual_slam/image_2`, `visual_slam/camera_info_2` | `/left_realsense/infra1/image_rect_raw`, `/left_realsense/infra1/camera_info` | `Image`, `CameraInfo` |
| `visual_slam/image_3`, `visual_slam/camera_info_3` | `/left_realsense/infra2/image_rect_raw`, `/left_realsense/infra2/camera_info` | `Image`, `CameraInfo` |
| `visual_slam/image_4`, `visual_slam/camera_info_4` | `/right_realsense/infra1/image_rect_raw`, `/right_realsense/infra1/camera_info` | `Image`, `CameraInfo` |
| `visual_slam/image_5`, `visual_slam/camera_info_5` | `/right_realsense/infra2/image_rect_raw`, `/right_realsense/infra2/camera_info` | `Image`, `CameraInfo` |
| `visual_slam/imu` | `/vio_transform/imu`, which has no publisher | `sensor_msgs/Imu` |
| `/visual_slam/initial_pose` | not remapped | `geometry_msgs/PoseWithCovarianceStamped` |

`/visual_slam/initial_pose` triggers localization in the folder last given to `load_map`.

| Published | Type | Content |
| --- | --- | --- |
| `/visual_slam/status` | `isaac_ros_visual_slam_interfaces/VisualSlamStatus` | Tracker state and timings. |
| `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` | VO pose and twist, `odom` to `base_link`. |
| `/visual_slam/tracking/vo_pose` | `geometry_msgs/PoseStamped` | VO pose. |
| `/visual_slam/tracking/vo_pose_covariance` | `geometry_msgs/PoseWithCovarianceStamped` | VO pose with covariance. |
| `/visual_slam/tracking/vo_path` | `nav_msgs/Path` | VO pose history. |
| `/visual_slam/tracking/slam_path` | `nav_msgs/Path` | SLAM pose history. |
| `/visual_slam/vis/slam_odometry` | `nav_msgs/Odometry` | SLAM pose in `map`, consumed by the reactor. |
| `/visual_slam/trigger_hint` | `geometry_msgs/PoseWithCovarianceStamped` | Empty message after a failed localization, asking for another hint. |
| `/tf` | `tf2_msgs/TFMessage` | `map` to `odom`, `odom` to `base_link`. |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Tracker and driver diagnostics. |

The `status`, `tracking/` and `vis/` topics publish only while a subscriber is attached.

### vio_transform

| Subscribed | Type | QoS | Use |
| --- | --- | --- | --- |
| `/visual_slam/filt_slam_odometry` | `nav_msgs/Odometry` | sensor data, depth 30 | Pose source, published by `px4_vslam_reactor`. |
| `/visual_slam/status` | `isaac_ros_visual_slam_interfaces/VisualSlamStatus` | sensor data, depth 30 | `vo_state` becomes `quality`. |
| `/reactor/vio_reset_epoch` | `std_msgs/UInt8` | reliable, transient local, depth 1 | Becomes `reset_counter`. |

| Published | Type | QoS | When |
| --- | --- | --- | --- |
| `/fmu/in/vehicle_visual_odometry` | `px4_msgs/VehicleOdometry` | reliable, depth 10 | One message per `/visual_slam/filt_slam_odometry` message. |

| `VehicleOdometry` field | Source |
| --- | --- |
| `timestamp`, `timestamp_sample` | Odometry header stamp, in microseconds. |
| `q`, `position` | Pose rotated from FLU into `POSE_FRAME_FRD`. |
| `velocity`, `angular_velocity` | Twist in `VELOCITY_FRAME_BODY_FRD`. |
| `position_variance`, `orientation_variance`, `velocity_variance` | Covariance diagonals, rotated then made positive. |
| `quality` | `vo_state` from `/visual_slam/status`. |
| `reset_counter` | Latched value on `/reactor/vio_reset_epoch`. |

---

## Services

`vio_transform` hosts no services and calls none.

| Hosted | Type | Effect |
| --- | --- | --- |
| `/visual_slam/reset` | `isaac_ros_visual_slam_interfaces/srv/Reset` | Terminates the cuVSLAM tracker. |
| `/visual_slam/set_slam_pose` | `isaac_ros_visual_slam_interfaces/srv/SetSlamPose` | Writes a `map` to `base_link` pose into the tracker; called by the reactor. |
| `/visual_slam/get_all_poses` | `isaac_ros_visual_slam_interfaces/srv/GetAllPoses` | Returns the pose graph. |
| `/visual_slam/save_map` | `isaac_ros_visual_slam_interfaces/srv/FilePath` | Writes the map to the requested folder. |
| `/visual_slam/load_map` | `isaac_ros_visual_slam_interfaces/srv/FilePath` | Records the folder path for later localization. |
| `/visual_slam/localize_in_map` | `isaac_ros_visual_slam_interfaces/srv/LocalizeInMap` | Localizes in a map folder from a pose hint. |
| `/<cam>/device_info` | `realsense2_camera_msgs/srv/DeviceInfo` | Returns device identity and firmware. |
| `/<cam>/hw_reset` | `std_srvs/Trigger` | Hardware-resets that camera; it drops off the bus and re-enumerates. |

---

## Parameters

| Launch argument | Default | Effect |
| --- | --- | --- |
| `camera_config_file` | `config/vslam_config.yaml` | Parameter file loaded by the drivers and the SLAM node. |

### Config files

`vslam_config.template.yaml` is tracked and carries the fleet defaults with blank serials; `config_realsense` rewrites the untracked `vslam_config.yaml` from it, re-splicing this drone's serials. Tunable edits go in the template, since a live-file edit is lost at the next reseed.

```bash
config_realsense --reseed-only
```

The workspace is symlink-installed, so a YAML change needs only a relaunch.

### RealSense blocks

Each camera has one block, keyed `<cam>/<cam>_link`.

| Key | Value | Effect |
| --- | --- | --- |
| `serial_no` | per drone | Binds the block to one camera. Written by `config_realsense`. |
| `camera_name` | `<cam>` | Prefix of the published frame ids. |
| `enable_infra1`, `enable_infra2` | `true` | The two IR streams feeding cuVSLAM. |
| `enable_color`, `enable_depth`, `enable_gyro`, `enable_accel` | `false` | Nothing on this drone subscribes these streams. |
| `enable_sync` | `false` | Frames publish per stream, without the rs2 syncer. |
| `depth_module.profile` | `640x360x60` | IR resolution and frame rate. |
| `depth_module.emitter_enabled` | `0` | IR projector off. |
| `depth_module.enable_auto_exposure` | `true` | Auto exposure on the IR imagers. |
| `rgb_camera.power_line_frequency` | `1` | Colour-stream anti-flicker. Colour is disabled. |
| `initial_reset` | `false` | No hardware reset during driver startup. |

> Do not select a 90 fps profile. The 90 fps USB service interval stalls the bus.

### visual_slam_node block

| Key | Value | Effect |
| --- | --- | --- |
| `num_cameras` | `6` | Number of `image_N` / `camera_info_N` inputs. |
| `min_num_images` | `4` | Images required per set; 4 keeps VO running through the loss of one camera. |
| `stale_stream_timeout_ms` | `100.0` | A stream silent this long in the stamp domain stops blocking set emission. |
| `sync_matching_threshold_ms` | `15.0` | Stamp spread allowed within one image set. |
| `image_jitter_threshold_ms` | `34.0` | Largest frame-to-frame stamp delta cuVSLAM accepts. |
| `image_qos` | `SENSOR_DATA` | QoS of the image and camera_info subscriptions. |
| `rectified_images` | `true` | Inputs are already rectified. |
| `enable_image_denoising` | `false` | No denoising pass on the input images. |
| `enable_localization_n_mapping` | `true` | Runs SLAM; `false` gives visual odometry and no map. |
| `enable_imu_fusion` | `false` | Keep false: nothing publishes `visual_slam/imu`. |
| `map_frame`, `odom_frame` | `map`, `odom` | Frames of the published TF and odometry. |
| `base_frame`, `imu_frame` | `base_link`, `autopilot` | Must match links in the URDF. |
| `camera_optical_frames` | six frames | Must match `<camera_name>_infra{1,2}_optical_frame`, in input order. |
| `enable_slam_visualization`, `enable_landmarks_view`, `enable_observations_view` | `false` | Landmark, observation and pose-graph visualization off. |

Initialization needs one `camera_info` from all six streams; `min_num_images` and `stale_stream_timeout_ms` apply only after that.
