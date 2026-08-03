# arid_description

This package holds the xacro description, mesh, display launch and RViz configuration for the ARID quadrotor by InDro Robotics.

![ARID drone](doc/arid_description.png)

- `xacro/arid.xacro`: the full robot description.
- `meshes/arid_model.stl`: the visual mesh.
- `launch/display.launch.py`: `robot_state_publisher`, with optional GUI and RViz.
- `rviz/arid.rviz`: the default RViz config, fixed frame `base_link`.

`arid_description.service` runs the launch at boot. It publishes the expanded URDF on `/robot_description` and the fixed-joint transform tree on `/tf_static`, both latched for late subscribers.

## Launch arguments

The display launch takes two arguments.

| Argument | Default | Function |
|---|---|---|
| `rviz` | `false` | Starts RViz with `arid.rviz` preloaded. |
| `gui` | `false` | Starts `joint_state_publisher_gui`. All joints are fixed, so it has no effect. |

The launch always starts `robot_state_publisher`, so running it while the boot service is active leaves two publishers of the same description on the graph.

```bash
ros2 launch arid_description display.launch.py rviz:=true
```

## Xacro structure

Three properties drive the geometry.

| Property | Value | Meaning |
|---|---|---|
| `mesh_scale` | `0.0001` | STL units (0.1 mm) to metres. |
| `propeller_span` | `0.0975` | Rotor hubs sit at `(±span, ±span, 0)`. |
| `realsense_z` | `0.092042` | Height of the RealSense links above `base_link`. |

Two macros generate the repeated links. `propeller(name, x, y)` builds a link and fixed joint for each of the four rotors, and `realsense(name, x, y, yaw)` places a camera link at `realsense_z` for `front`, `left` and `right`. The autopilot, the two CSI camera links, the flow module and the rangefinder are declared inline.

`base_link` carries the only geometry: the mesh as its visual, a 0.393 x 0.393 x 0.2047 m box as its collision. Every other link is an empty frame.

## Frames

`base_footprint` is the root, `base_link` is coincident with it, and every remaining link is a fixed-joint child of `base_link`.

| Frame | Mounting |
|---|---|
| `autopilot` | ARK FMU v6X. |
| `front_left_propeller_link` | Front left rotor hub. |
| `front_right_propeller_link` | Front right rotor hub. |
| `rear_left_propeller_link` | Rear left rotor hub. |
| `rear_right_propeller_link` | Rear right rotor hub. |
| `front_realsense_link` | Front RealSense, facing forward. |
| `left_realsense_link` | Left RealSense, yawed +90 degrees. |
| `right_realsense_link` | Right RealSense, yawed -90 degrees. |
| `top_visual_link` | Front CSI camera, z axis along body +x. |
| `bottom_visual_link` | Down CSI camera, z axis along body -z. |
| `flow_link` | Optical flow module, below `base_link`. |
| `rangefinder_link` | Rangefinder, below `base_link`. |

Each RealSense link name matches the `<camera_name>_link` frame that `realsense2_camera` publishes, so the driver's stream frames attach beneath it.

> Frame names are referenced outside this package, by `px4_vslam` and `gst_camera_manager` configuration. Renaming a link breaks the consumer that stamps or looks it up.

## Foxglove

Foxglove Studio renders the model through `foxglove_bridge`. Start the bridge, connect Studio to `ws://<device-ip>:8765`, and load the URDF from `/robot_description` with panel frame `base_link`.

```bash
foxglove_bridge
```

## License

The package is released under Apache-2.0; see [LICENSE](LICENSE).
