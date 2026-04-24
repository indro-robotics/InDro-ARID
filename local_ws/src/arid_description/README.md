# arid_description

Xacro description and meshes for the **ARID** quadrotor platform by InDro Robotics.

![ARID drone](doc/arid.png)

## Contents

- `urdf/arid.xacro` — full robot description: `base_link`, `autopilot`, 4 propellers, 3 RealSense cameras, 2 CV cameras, optical flow, rangefinder, and `base_footprint`. Uses xacro macros for the propeller and RealSense groups
- `meshes/arid_model.stl` — visual mesh of the full airframe (collision is a `<box>` primitive defined inline in the xacro)
- `launch/display.launch.py` — starts `robot_state_publisher` with optional `joint_state_publisher_gui` and RViz
- `rviz/arid.rviz` — default RViz display config

## Build & Launch

```bash
cd ~/workspaces/local_ws
colcon build --packages-select arid_description --symlink-install
source install/setup.bash
ros2 launch arid_description display.launch.py
```

`robot_state_publisher` then publishes:

- `/robot_description` — latched `std_msgs/String` with the expanded URDF (consumed by RViz, Foxglove, MoveIt, etc.).
- `/tf_static` — the full fixed-joint transform tree for all sensor / propeller / airframe frames.

**Auto-start on boot:** `arid_description.service` (installed by `setup.sh`) runs this launch file as a systemd unit, so the TF tree is available system-wide without manual steps.

### Launch arguments

| Arg | Default | Description |
|-----|---------|-------------|
| `rviz` | `false` | Launch RViz with `arid.rviz` preloaded |
| `gui` | `false` | Launch `joint_state_publisher_gui` (not useful — all joints are fixed) |

## Xacro structure

[`urdf/arid.xacro`](urdf/arid.xacro) defines the robot with:

**Properties** (tune these to rebuild the geometry at different scales / spans):

| Property | Value | Meaning |
|---|---|---|
| `mesh_scale` | `0.0001` | Multiplier applied to the STL (mesh is natively mm, URDF uses meters). |
| `propeller_span` | `0.0975` | Half the propeller-to-propeller distance along one axis (propellers sit at `±span, ±span`). |
| `realsense_z` | `0.092042` | Z-offset of the three RealSense cameras from `base_link`. |

**Macros:**

- `propeller(name, x, y)` — instantiates a propeller link and its fixed joint. Used 4× for `front_left`, `front_right`, `rear_left`, `rear_right`.
- `realsense(name, x, y, yaw)` — instantiates a RealSense camera link at `(x, y, realsense_z)` with the given yaw. Used 3× for `front`, `left`, `right`.

Unique links (autopilot, top/bottom visual cameras, flow, rangefinder) are inlined to preserve their specific joint names.

## Frames

TF tree rooted at `base_footprint` → `base_link`:

```
base_footprint
└── base_link
    ├── autopilot
    ├── front_left_propeller_link   front_right_propeller_link
    ├── rear_left_propeller_link    rear_right_propeller_link
    ├── front_realsense_link        left_realsense_link        right_realsense_link
    ├── top_visual_link    (front-facing CSI CV camera — optical-z along body +x)
    ├── bottom_visual_link (down-facing CSI CV camera — optical-z along body -z)
    ├── flow_link
    └── rangefinder_link
```

All joints are fixed. RealSense frames sit at the left IR lens per the `realsense-ros` convention.

## Visualization in Foxglove

Requires [Foxglove Studio](https://foxglove.dev/download) (desktop or web) and the `foxglove_bridge` package (`sudo apt install ros-humble-foxglove-bridge`).

```bash
ros2 launch arid_description display.launch.py       # terminal 1: robot_state_publisher
ros2 run foxglove_bridge foxglove_bridge              # terminal 2: WebSocket on port 8765
```

In Foxglove Studio:
1. **Open connection** → `ws://<host>:8765` (use `localhost` if running on the same machine)
2. Add a **3D** panel
3. In the 3D panel settings → **Custom Layers** → **Add → URDF**, topic `/robot_description`
4. Set the panel **Frame** to `base_link`

## License

Apache-2.0 — see [LICENSE](LICENSE).
