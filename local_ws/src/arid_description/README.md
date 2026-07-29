# arid_description

This package holds the xacro description and meshes for the ARID quadrotor by InDro Robotics.

![ARID drone](doc/arid_description.png)

- `urdf/arid.xacro`: the full robot description.
- `meshes/arid_model.stl`: the visual mesh; collision is an inline `<box>`.
- `launch/display.launch.py`: `robot_state_publisher`, with optional GUI and RViz.
- `rviz/arid.rviz`: the default RViz config.

`arid_description.service` runs the launch at boot. It publishes `/robot_description` as the latched expanded URDF and `/tf_static` as the full fixed-joint transform tree.

## Launch arguments

The display launch takes two arguments.

| Argument | Default | Function |
|---|---|---|
| `rviz` | `false` | RViz with `arid.rviz` preloaded. |
| `gui` | `false` | `joint_state_publisher_gui`; all joints are fixed, so it has no effect. |

```bash
ros2 launch arid_description display.launch.py rviz:=true
```

## Xacro structure

Three properties drive the geometry.

| Property | Value | Meaning |
|---|---|---|
| `mesh_scale` | `0.0001` | The STL is in millimetres and the URDF is in metres. |
| `propeller_span` | `0.0975` | Propellers sit at `(±span, ±span)`. |
| `realsense_z` | `0.092042` | Z-offset of the RealSense cameras from `base_link`. |

Two macros generate the repeated links: `propeller(name, x, y)` builds a propeller link and fixed joint for `front_left`, `front_right`, `rear_left` and `rear_right`, and `realsense(name, x, y, yaw)` builds a RealSense link at `(x, y, realsense_z)` for `front`, `left` and `right`. The autopilot, the two visual cameras, the flow module and the rangefinder are inlined so their joint names stay fixed.

## Frames

`base_footprint` is the root and `base_link` is its only child. Every remaining link is a fixed-joint child of `base_link`:

- `autopilot`
- `front_left_propeller_link`, `front_right_propeller_link`, `rear_left_propeller_link`, `rear_right_propeller_link`
- `front_realsense_link`, `left_realsense_link`, `right_realsense_link`
- `top_visual_link`, the front-facing CSI camera
- `bottom_visual_link`, the down-facing CSI camera, optical z along body -z
- `flow_link`
- `rangefinder_link`

RealSense frames sit at the left IR lens, following the `realsense-ros` convention.

## Visualization in Foxglove

With the description running, start the Foxglove bridge in another terminal and connect Studio to `ws://<device-ip>:8765`. The URDF is on `/robot_description` with panel frame `base_link`.

```bash
foxglove_bridge
```

## License

The package is released under Apache-2.0; see [LICENSE](LICENSE).
