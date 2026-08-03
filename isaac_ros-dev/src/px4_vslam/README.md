# px4_vslam

`px4_vslam` brings up the Isaac ROS Visual SLAM stack over the RealSense infra streams and bridges
its odometry into PX4 as external vision. The bridge is `vio_transform`; the bringup is
`vslam.launch.py`.

## Bringup

Bringup runs through [`arid_supervisor`](../arid_supervisor/README.md) and is idempotent: a running
stack is not restarted and the call returns success. A healthy start takes 14-26 s, and
`initialize` bounds each call at 300 s.

```bash
initialize
```

| Command | Invokes | Result |
| --- | --- | --- |
| `initialize` | `/arid_supervisor/vslam_enable`, `std_srvs/SetBool`, `data: true` | launches `vslam.launch.py`, proves every configured RealSense driver up |
| `deinitialize` | same service, `data: false` | stops the stack, refused unless the drone is landed |

Both print the supervisor's response message, which carries the evidence behind a refusal.

> Do not run `ros2 launch px4_vslam vslam.launch.py` by hand. A second stack claims the same camera
> and node names, and the supervisor refuses the next bringup.

## Launch

`vslam.launch.py` waits for the latched `/robot_description` (`std_msgs/String`, transient local,
reliable) from the host unit `arid_description.service`, then starts the nodes below.

| Node | Package | Function |
| --- | --- | --- |
| `left_realsense_link`, `front_realsense_link`, `right_realsense_link` | `realsense2_camera` | one driver per mount, each in its own namespace |
| `visual_slam_node` | `isaac_ros_visual_slam` | cuVSLAM over the six infra streams |
| `vslam_reactor` | `px4_vslam_reactor` | odometry gate ahead of EKF2 |
| `vio_transform` | `px4_vslam` | PX4 external-vision bridge |

The drivers and `visual_slam_node` load into one `component_container_mt` with a 25 s SIGTERM grace,
the time the sensors take to close.

| Launch argument | Default | Effect |
| --- | --- | --- |
| `camera_config_file` | `config/vslam_config.yaml` in this package | parameters for the drivers and `visual_slam_node` |

> Without `arid_description.service` on the host, the launch stops at the `/robot_description` wait
> and no camera or SLAM node starts.

## Stack topics

`<cam>` is `left_realsense`, `front_realsense` or `right_realsense`. Every image carries the same
timestamp and optical frame id as its `camera_info`. The camera and `visual_slam` topics publish
only while something is subscribed to them.

| cuVSLAM input | Type | Remapped to |
| --- | --- | --- |
| `visual_slam/image_0` | `sensor_msgs/Image` | `front_realsense/infra1/image_rect_raw` |
| `visual_slam/camera_info_0` | `sensor_msgs/CameraInfo` | `front_realsense/infra1/camera_info` |
| `visual_slam/image_1` | `sensor_msgs/Image` | `front_realsense/infra2/image_rect_raw` |
| `visual_slam/camera_info_1` | `sensor_msgs/CameraInfo` | `front_realsense/infra2/camera_info` |
| `visual_slam/image_2` | `sensor_msgs/Image` | `left_realsense/infra1/image_rect_raw` |
| `visual_slam/camera_info_2` | `sensor_msgs/CameraInfo` | `left_realsense/infra1/camera_info` |
| `visual_slam/image_3` | `sensor_msgs/Image` | `left_realsense/infra2/image_rect_raw` |
| `visual_slam/camera_info_3` | `sensor_msgs/CameraInfo` | `left_realsense/infra2/camera_info` |
| `visual_slam/image_4` | `sensor_msgs/Image` | `right_realsense/infra1/image_rect_raw` |
| `visual_slam/camera_info_4` | `sensor_msgs/CameraInfo` | `right_realsense/infra1/camera_info` |
| `visual_slam/image_5` | `sensor_msgs/Image` | `right_realsense/infra2/image_rect_raw` |
| `visual_slam/camera_info_5` | `sensor_msgs/CameraInfo` | `right_realsense/infra2/camera_info` |
| `visual_slam/imu` | `sensor_msgs/Imu` | `vio_transform/imu`, no subscription and no publisher |
| `/visual_slam/initial_pose` | `geometry_msgs/PoseWithCovarianceStamped` | not remapped: pose hint, localizes in the folder last given to `load_map` |

| Published topic | Type | Content |
| --- | --- | --- |
| `/<cam>/infra1/image_rect_raw`, `/<cam>/infra2/image_rect_raw` | `sensor_msgs/Image` | mono8, 640x360 at 60 fps |
| `/<cam>/infra1/camera_info`, `/<cam>/infra2/camera_info` | `sensor_msgs/CameraInfo` | intrinsics, frame `<cam>_infra<n>_optical_frame` |
| `/<cam>/infra1/metadata`, `/<cam>/infra2/metadata` | `realsense2_camera_msgs/Metadata` | per-frame sensor metadata, JSON string |
| `/<cam>/extrinsics/depth_to_infra1`, `/<cam>/extrinsics/depth_to_infra2` | `realsense2_camera_msgs/Extrinsics` | stream-to-stream extrinsics |
| `/visual_slam/status` | `isaac_ros_visual_slam_interfaces/VisualSlamStatus` | `vo_state` 0 unknown, 1 success, 2 failed, plus tracking times |
| `/visual_slam/vis/slam_odometry` | `nav_msgs/Odometry` | cuVSLAM `map`-frame SLAM pose, the reactor's input |
| `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` | `odom`-frame VO pose, twist and covariance |
| `/visual_slam/tracking/vo_pose` | `geometry_msgs/PoseStamped` | `odom`-frame VO pose |
| `/visual_slam/tracking/vo_pose_covariance` | `geometry_msgs/PoseWithCovarianceStamped` | the same pose with its covariance |
| `/visual_slam/tracking/vo_path`, `/visual_slam/tracking/slam_path` | `nav_msgs/Path` | VO and SLAM pose history |
| `/visual_slam/vis/velocity` | `visualization_msgs/MarkerArray` | VO velocity marker |
| `/visual_slam/trigger_hint` | `geometry_msgs/PoseWithCovarianceStamped` | empty message, published when an `initial_pose` localization fails |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | cuVSLAM callback and tracking times |
| `/tf` | `tf2_msgs/TFMessage` | `map` to `odom`, `odom` to `base_link`, from cuVSLAM |
| `/tf_static` | `tf2_msgs/TFMessage` | camera link to optical frames, from the drivers |

The landmark, observation, pose-graph and localizer topics under `/visual_slam/vis/` stay silent
while `enable_slam_visualization` is `false`. `/visual_slam/vis/gravity` stays silent while
`enable_imu_fusion` is `false`.

## Stack services

| Service | Type | Effect |
| --- | --- | --- |
| `/<cam>/hw_reset` | `std_srvs/Trigger` | hardware-resets that camera from inside the driver; the device re-enumerates |
| `/<cam>/device_info` | `realsense2_camera_msgs/DeviceInfo` | returns device identity, firmware and USB descriptor strings |
| `/visual_slam/reset` | `isaac_ros_visual_slam_interfaces/Reset` | destroys the cuVSLAM tracker; it re-initializes on the next `camera_info` from every stream |
| `/visual_slam/set_slam_pose` | `isaac_ros_visual_slam_interfaces/SetSlamPose` | seats the SLAM pose at the supplied `map`-frame pose |
| `/visual_slam/get_all_poses` | `isaac_ros_visual_slam_interfaces/GetAllPoses` | returns up to `max_count` optimized pose-graph poses |
| `/visual_slam/save_map` | `isaac_ros_visual_slam_interfaces/FilePath` | writes the current map to `file_path` |
| `/visual_slam/load_map` | `isaac_ros_visual_slam_interfaces/FilePath` | loads a map from `file_path` |
| `/visual_slam/localize_in_map` | `isaac_ros_visual_slam_interfaces/LocalizeInMap` | loads `map_folder_path` and localizes against `pose_hint` |

`vslam_reactor` hosts `/visual_slam/set_reactor_pose` and is the only caller of
`/visual_slam/set_slam_pose`, both in [`px4_vslam_reactor`](../px4_vslam_reactor/README.md).

## vio_transform

`vio_transform` converts reactor-gated odometry into the PX4 external-vision message.
`/visual_slam/filt_slam_odometry` and `/reactor/vio_reset_epoch` come from
[`px4_vslam_reactor`](../px4_vslam_reactor/README.md), not from cuVSLAM.

| Subscribed topic | Type | Use | QoS |
| --- | --- | --- | --- |
| `/visual_slam/filt_slam_odometry` | `nav_msgs/Odometry` | gated external-vision stream, converted per message | sensor data, depth 30 |
| `/visual_slam/status` | `isaac_ros_visual_slam_interfaces/VisualSlamStatus` | `vo_state`, copied into `quality` | sensor data, depth 30 |
| `/reactor/vio_reset_epoch` | `std_msgs/UInt8` | re-seat epoch, copied into `reset_counter` | reliable, transient local, depth 1 |

| Published topic | Type | When | QoS |
| --- | --- | --- | --- |
| `/fmu/in/vehicle_visual_odometry` | `px4_msgs/VehicleOdometry` | once per `/visual_slam/filt_slam_odometry` message | reliable, depth 10 |

| `VehicleOdometry` field | Source |
| --- | --- |
| `timestamp`, `timestamp_sample` | the input header stamp, in microseconds |
| `position`, `q` | pose rotated from FLU into `pose_frame` `POSE_FRAME_FRD` |
| `velocity`, `angular_velocity` | twist rotated into `velocity_frame` `VELOCITY_FRAME_BODY_FRD` |
| `position_variance`, `orientation_variance`, `velocity_variance` | covariance diagonals, rotated then absolute-valued |
| `quality` | `vo_state` from `/visual_slam/status` |
| `reset_counter` | latched value on `/reactor/vio_reset_epoch` |

The uXRCE-DDS agent carries `/fmu/in/vehicle_visual_odometry` to EKF2. `vio_transform` declares no
parameters, hosts no services and calls none.

> `/reactor/vio_reset_epoch` is RELIABLE and transient local. A best-effort publisher does not
> connect to it, and `reset_counter` never advances.

## Configuration

`config/vslam_config.yaml` is untracked and regenerated from `config/vslam_config.template.yaml` by
`config_realsense`, which splices this aircraft's camera serials back in.

| Driver parameter | Value | Effect |
| --- | --- | --- |
| `serial_no` | the RealSense serial | binds the driver to one camera; the supervisor's bringup gate counts filled serials |
| `camera_name` | `<cam>` | prefix of every frame id the driver publishes |
| `enable_infra1`, `enable_infra2` | `true` | the two streams cuVSLAM consumes |
| `depth_module.profile` | `640x360x60` | infra resolution and frame rate |
| `depth_module.emitter_enabled` | `0` | IR projector off, passive stereo |
| `depth_module.enable_auto_exposure` | `true` | auto exposure on the infra pair |
| `enable_color` | `false` | no color stream |
| `rgb_camera.power_line_frequency` | `1` | anti-flicker option passed to the color sensor |
| `enable_depth` | `false` | no depth stream |
| `enable_gyro`, `enable_accel` | `false` | no IMU streams |
| `enable_sync` | `false` | frames publish on arrival instead of in matched sets |
| `initial_reset` | `false` | no hardware reset when the driver starts |
| `use_intra_process_comms` | `true` | zero-copy image publishers, no compressed transport |

> Do not raise `depth_module.profile` to 90 fps. The 11.1 ms USB service interval latches the
> librealsense clear-halt loop and the stream collapses until the stack restarts.

| Visual SLAM parameter | Value | Effect |
| --- | --- | --- |
| `num_cameras` | `6` | three cameras, tracked as their six infra streams |
| `min_num_images` | `4` | images required per set after init; 4 keeps VO running through the loss of one camera |
| `stale_stream_timeout_ms` | `100.0` | a stream silent this long in the stamp domain stops blocking set emission |
| `sync_matching_threshold_ms` | `15.0` | stamp spread allowed inside one frame set |
| `image_jitter_threshold_ms` | `34.0` | a longer gap between frame sets logs a jitter warning |
| `image_qos` | `SENSOR_DATA` | QoS of the image and `camera_info` subscriptions |
| `rectified_images` | `true` | input treated as rectified, horizontal stereo |
| `enable_image_denoising` | `false` | no denoising pass on the input images |
| `enable_imu_fusion` | `false` | visual only, no `visual_slam/imu` subscription created |
| `imu_frame` | `autopilot` | inert while `enable_imu_fusion` is `false` |
| `enable_localization_n_mapping` | `true` | SLAM and mapping alongside visual odometry |
| `map_frame`, `odom_frame`, `base_frame` | `map`, `odom`, `base_link` | frames of the transforms cuVSLAM publishes |
| `camera_optical_frames` | the six `<cam>_infra<n>_optical_frame` ids, in input order | rig extrinsics, looked up in TF against `base_frame` |
| `enable_slam_visualization`, `enable_landmarks_view`, `enable_observations_view` | `false` | no visualization output |

Initialization needs one `camera_info` from all six streams; `min_num_images` and
`stale_stream_timeout_ms` apply only after that.

Tunables are edited in the template: `config_realsense` overwrites the live file from it on every
run. Reseed the live file after a template edit, then relaunch.

```bash
config_realsense --reseed-only
```

The workspace is built with `--symlink-install`, so a YAML-only change needs no rebuild. The
reactor's gates and tolerances are separate, in
[`px4_vslam_reactor/config/px4_vslam_reactor.yaml`](../px4_vslam_reactor/config/px4_vslam_reactor.yaml).
