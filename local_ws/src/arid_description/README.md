# arid_description

This package holds the xacro description, mesh and RViz configuration for the ARID quadrotor by InDro Robotics.

![ARID drone](doc/arid_description.png)

- `xacro/arid.xacro`: the full robot description.
- `meshes/arid_model.stl`: the visual mesh; collision is an inline box of 0.393 x 0.393 x 0.2047 m.
- `launch/display.launch.py`: `robot_state_publisher`, with optional GUI and RViz.
- `rviz/arid.rviz`: the default RViz config, fixed frame `base_link`.

`arid_description.service` runs the launch at boot. It publishes the expanded URDF on `/robot_description` and the fixed-joint transform tree on `/tf_static`, both latched for late subscribers.

## Launch arguments

The display launch takes two arguments.

| Argument | Default | Function |
|---|---|---|
| `rviz` | `false` | Starts RViz with `arid.rviz` preloaded. |
| `gui` | `false` | Starts `joint_state_publisher_gui`. All joints are fixed, so it has no effect. |

The boot service already runs `robot_state_publisher`. To view the model on the NoMachine desktop, start a second instance with RViz.

```bash
ros2 launch arid_description display.launch.py rviz:=true
```

## Xacro structure

Three properties drive the geometry.

| Property | Value | Meaning |
|---|---|---|
| `mesh_scale` | `0.0001` | STL units (0.1 mm) to metres. |
| `propeller_span` | `0.0975` | Rotor hubs sit at `(±span, ±span, 0)`. |
| `realsense_z` | `0.092042` | Height of the front RealSense link above `base_link`. |

Two macros generate the repeated links. `propeller(name, x, y)` builds a link and fixed joint for each of the four rotors, and `realsense(name, x, y, yaw)` places the front camera link at `realsense_z`. The autopilot, the LiDAR, the CSI camera link, the flow module and the rangefinder are declared inline.

## Frames

`base_footprint` is the root, `base_link` is coincident with it, and every remaining link is a fixed-joint child of `base_link`.

```
base_footprint
└── base_link
    ├── autopilot
    ├── front_left_propeller_link
    ├── front_right_propeller_link
    ├── rear_left_propeller_link
    ├── rear_right_propeller_link
    ├── front_realsense_link
    ├── rslidar_link
    ├── bottom_visual_link
    ├── flow_link
    └── rangefinder_link
```

| Frame | Mounting |
|---|---|
| `autopilot` | ARK FMU v6X. |
| `front_realsense_link` | Front D435, facing forward. |
| `rslidar_link` | RSAIRY LiDAR, pitched 27.8 degrees down. |
| `bottom_visual_link` | Down CSI camera, z axis along body -z. |
| `flow_link` | ARK optical flow, bottom pod. |
| `rangefinder_link` | ARK rangefinder, bottom pod. |

The RealSense link name matches the `<camera_name>_link` frame that `realsense2_camera` publishes, so the driver's stream frames attach beneath it. The link itself sits at the left IR imager.

> Frame names are referenced outside this package, by `px4_vslam`, `gst_camera_manager` and `rslidar_coordinator` configuration. Renaming a link breaks the consumer that stamps or looks it up.

## Foxglove

Foxglove Studio renders the model through `foxglove_bridge`. Start the bridge, connect Studio to `ws://<device-ip>:8765`, and load the URDF from `/robot_description` with panel frame `base_link`.

```bash
foxglove_bridge
```

## License

The package is released under Apache-2.0; see [LICENSE](LICENSE).
