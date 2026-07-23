# arid_description

Xacro description and meshes for the **ARID** quadrotor by InDro Robotics.

![ARID drone](doc/arid_description.png)

## Contents

- `xacro/arid.xacro`: full robot description (`base_link`, `autopilot`, 4 propellers, 1 RealSense camera, 1 RoboSense LiDAR, 1 down-facing CV camera, optical flow, rangefinder, `base_footprint`)
- `meshes/arid_model.stl`: visual mesh (collision is an inline `<box>`)
- `launch/display.launch.py`: `robot_state_publisher` with optional GUI and RViz
- `rviz/arid.rviz`: default RViz config

## Build and launch

```bash
cd ~/workspaces/local_ws
colcon build --packages-select arid_description --symlink-install
source install/setup.bash
ros2 launch arid_description display.launch.py
```

Publishes:

- `/robot_description`: latched expanded URDF (RViz, Foxglove)
- `/tf_static`: full fixed-joint transform tree

**Auto-start:** `arid_description.service` (installed by `setup.sh`) runs this launch at boot.

### Launch arguments

| Arg | Default | Function |
|-----|---------|----------|
| `rviz` | `false` | RViz with `arid.rviz` preloaded |
| `gui` | `false` | `joint_state_publisher_gui` (no effect: all joints fixed) |

## Xacro structure

**Properties** (tune to rebuild geometry):

| Property | Value | Meaning |
|---|---|---|
| `mesh_scale` | `0.0001` | STL is mm, URDF is meters. |
| `propeller_span` | `0.0975` | Propellers are at `(±span, ±span)`. |
| `realsense_z` | `0.092042` | Z-offset of the front RealSense camera from `base_link`. |

**Macros:**

- `propeller(name, x, y)`: propeller link + fixed joint; 4x (`front_left`, `front_right`, `rear_left`, `rear_right`)
- `realsense(name, x, y, yaw)`: RealSense link at `(x, y, realsense_z)`; 1x (`front`)

Unique links (autopilot, bottom visual camera, flow, rangefinder, rslidar) are inlined to keep their joint names.

## Frames

```
base_footprint
└── base_link
    ├── autopilot
    ├── front_left_propeller_link   front_right_propeller_link
    ├── rear_left_propeller_link    rear_right_propeller_link
    ├── front_realsense_link
    ├── rslidar_link (RoboSense RSAIRY mount frame)
    ├── bottom_visual_link (down-facing CSI CV camera, optical-z along body -z)
    ├── flow_link
    └── rangefinder_link
```

All joints fixed. The RealSense frame is at the left IR lens (`realsense-ros` convention).

## Visualization in Foxglove

With this launch running, start the `foxglove_bridge` alias in another terminal:

```bash
ros2 launch arid_description display.launch.py
foxglove_bridge
```

Connect Studio to `ws://<device-ip>:8765`; URDF on `/robot_description`, panel frame `base_link`. Studio setup: root [README](../../../README.md) Foxglove section.

## License

Apache-2.0: see [LICENSE](LICENSE).
