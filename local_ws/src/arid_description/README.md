# arid_description

This package holds the ARID quadrotor URDF, its visual mesh and an RViz configuration. The link names it defines are the frame ids the camera and visual-odometry packages stamp and look up.

![ARID drone](doc/arid_description.png)

## Startup

`arid_description.service` runs `display.launch.py` at boot. The launch expands `xacro/arid.xacro` and starts `robot_state_publisher` with the result as its `robot_description` parameter.

## Viewing the model

`display.launch.py` always starts `robot_state_publisher`, so a manual run adds a second publisher of the same description to the graph.

```bash
ros2 launch arid_description display.launch.py rviz:=true
```

| Argument | Default | Effect |
| --- | --- | --- |
| `rviz` | `false` | Starts `rviz2` on `rviz/arid.rviz`: RobotModel from `/robot_description`, TF axes, fixed frame `base_link` |
| `gui` | `false` | Starts `joint_state_publisher_gui`; every joint is fixed, so no slider moves a frame |

## Published topics

| Topic | Type | QoS | Published | Carries |
| --- | --- | --- | --- | --- |
| `/robot_description` | `std_msgs/String` | Reliable, transient local, depth 1 | Once at start | Expanded URDF, held for late subscribers |
| `/tf_static` | `tf2_msgs/TFMessage` | Reliable, transient local, depth 1 | Once at start | The 13 fixed-joint transforms |
| `/tf` | `tf2_msgs/TFMessage` | Reliable, depth 100 | Never | Advertised by `robot_state_publisher`; no joint in the description moves |

## Subscribed topics

| Topic | Type | Used for |
| --- | --- | --- |
| `/joint_states` | `sensor_msgs/JointState` | Movable-joint positions; nothing on the drone publishes it |

## Parameters

| Parameter | Value | Controls |
| --- | --- | --- |
| `robot_description` | Expanded `xacro/arid.xacro` | The URDF `robot_state_publisher` walks into `/tf_static` |

## Frames

`base_footprint` is the root and `base_link` is coincident with it. `base_link` is the airframe origin, the frame cuVSLAM tracks as its `base_frame`, and every link below is fixed to it.

| Frame | Offset from `base_link` (m) | RPY (deg) | Marks | Referenced by |
| --- | --- | --- | --- | --- |
| `autopilot` | 0.042524, 0, 0.100499 | 0, 0, 0 | ARK FMU v6X | - |
| `front_left_propeller_link` | 0.0975, 0.0975, 0 | 0, 0, 0 | Front left rotor hub | - |
| `front_right_propeller_link` | 0.0975, -0.0975, 0 | 0, 0, 0 | Front right rotor hub | - |
| `rear_left_propeller_link` | -0.0975, 0.0975, 0 | 0, 0, 0 | Rear left rotor hub | - |
| `rear_right_propeller_link` | -0.0975, -0.0975, 0 | 0, 0, 0 | Rear right rotor hub | - |
| `front_realsense_link` | 0.09875, 0.0175, 0.092042 | 0, 0, 0 | Front RealSense | `px4_vslam`: `realsense2_camera` base frame |
| `left_realsense_link` | -0.0175, 0.08875, 0.092042 | 0, 0, 90 | Left RealSense | `px4_vslam`: `realsense2_camera` base frame |
| `right_realsense_link` | 0.0175, -0.08875, 0.092042 | 0, 0, -90 | Right RealSense | `px4_vslam`: `realsense2_camera` base frame |
| `top_visual_link` | 0.077, 0, 0.129792 | -90, 0, -90 | Forward CSI camera | `/cam_front/image_raw` header |
| `bottom_visual_link` | 0.025, 0, 0.000842 | 0, 180, 90 | Downward CSI camera | `/cam_down/image_raw` header |
| `flow_link` | 0, -0.0115, -0.008518 | 180, 0, 0 | Optical flow module | - |
| `rangefinder_link` | -0.001322, 0.00992, -0.011096 | 0, 180, 0 | Rangefinder | - |

Each RealSense link name is the `<camera_name>_link` base frame `realsense2_camera` builds from the `camera_name` in `vslam_config.yaml`, so the driver hangs its stream frames beneath it.

> Renaming a link breaks the consumer that stamps or looks it up.

## Geometry

`base_link` carries the only geometry; every other link is an empty frame.

| Element | Definition |
| --- | --- |
| Visual | `meshes/arid_model.stl` scaled 0.0001 from 0.1 mm units, at 0, 0, 0.023842 m, RPY -90, 0, 90 |
| Collision | Box 0.393 x 0.393 x 0.2047 m, at 0, 0, 0.070192 m |
