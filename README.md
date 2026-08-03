# ARID

ARID is a quadrotor on an NVIDIA Jetson Orin with an ARK PAB carrier and an ARK FMU running PX4.
The Jetson carries two ROS 2 Humble workspaces, `local_ws` on the host and `isaac_ros-dev` inside
the Isaac ROS container. This repository holds both, and the provisioning that turns a stock Jetson
into a flight unit.

## Packages

Every other directory under each `src/` is a submodule.

| `local_ws` | Function |
|---|---|
| [`arid_description`](local_ws/src/arid_description/) | Airframe xacro, published as `/robot_description` and `/tf_static` |
| [`ros_gst_cameras`](local_ws/src/ros_gst_cameras/) | The two CSI camera pipelines, started and stopped over ROS services |
| [`reset_ark_usb`](local_ws/src/reset_ark_usb/) | Hosts `/reset_usb`, the carrier USB power cycle |

| `isaac_ros-dev` | Function |
|---|---|
| [`arid_supervisor`](isaac_ros-dev/src/arid_supervisor/) | Lifecycle gate: camera-proven VSLAM bringup, landed interlock |
| [`px4_vslam`](isaac_ros-dev/src/px4_vslam/) | Multi-camera cuVSLAM bringup; bridges the solution into PX4 as external vision |
| [`px4_vslam_reactor`](isaac_ros-dev/src/px4_vslam_reactor/) | Withholds discontinuous cuVSLAM frames before they reach EKF2 |

`px4_msgs` defines message types only. `PX4-Autopilot`, `isaac_ros_visual_slam` and
`realsense-ros` are InDro forks carrying local patches.

## Sensors

| Sensor | Fit |
|---|---|
| 3x RealSense | Front, left, right; one IR stereo pair each at 640x360x60 into cuVSLAM |
| 2x IMX219 CSI | `cam_front` forward, `cam_down` downward |
| Optical flow, rangefinder | Underside, into PX4 over UAVCAN |

## Boot

Setup installs and enables every unit in `local_ws/services/` and `isaac_ros-dev/services/` except
`reset_usb`, which runs on demand.

| Unit | Provides |
|---|---|
| `start_isaac_docker` | The Isaac ROS container |
| `arid_supervisor` | The supervisor node inside that container |
| `gst_camera_manager` | The two CSI camera pipelines |
| `arid_description` | `robot_state_publisher`: `/robot_description`, `/tf_static` |
| `usb_ros_reset` | The `/reset_usb` service |
| `jetson-clocks` | Maximum clocks and fan |
| `usbfs-memory` | 1000 MB usbfs buffer for the RealSense cameras |

Both CSI pipelines and the VSLAM stack start cold.

## Daily use

A login shell sources `local_ws/install/setup.bash` and defines the ARID aliases. `help` lists
them; run it inside the container for the container set.

### Visual odometry

`initialize` blocks until the supervisor has proven three RealSense up.

```
initialize
status
deinitialize
```

| Command | Invokes | Result |
|---|---|---|
| `initialize` | `/arid_supervisor/vslam_enable` `SetBool{true}` | Launches `vslam.launch.py` behind the camera gate; three attempts, 300 s each |
| `status` | `/arid_supervisor/status` `Trigger` | `vslam: running\|stopped \| land: landed\|airborne\|unknown` |
| `deinitialize` | `/arid_supervisor/vslam_enable` `SetBool{false}` | Stops the launch tree; refused without a landed sample under 3.5 s old |

Bringup waits for `/robot_description`, and each launch writes `run_logs/vslam/vslam.log`.

| Interface | Type | Carries |
|---|---|---|
| `/visual_slam/status` | `isaac_ros_visual_slam_interfaces/VisualSlamStatus` | Tracker state; becomes the `quality` field |
| `/visual_slam/vis/slam_odometry` | `nav_msgs/Odometry` | cuVSLAM SLAM pose in `map`, the reactor's input |
| `/visual_slam/filt_slam_odometry` | `nav_msgs/Odometry` | The pose the reactor admits |
| `/reactor/drone_odom`, `/reactor/drone_pose` | `nav_msgs/Odometry`, `geometry_msgs/PoseStamped` | FMU pose in `map`, child frame `px4` |
| `/reactor/vio_reset_epoch` | `std_msgs/UInt8` | Re-seat count, stamped into `reset_counter`; transient local |
| `/reactor/vo_healthy` | `std_msgs/Bool` | False once re-seats exhaust the burst limit; transient local |
| `/fmu/in/vehicle_visual_odometry` | `px4_msgs/VehicleOdometry` | The admitted pose in FRD, into EKF2 |
| `/visual_slam/set_reactor_pose` | `std_srvs/Trigger` | Seats cuVSLAM on the current FMU pose |
| `/visual_slam/set_slam_pose` | `isaac_ros_visual_slam_interfaces/SetSlamPose` | The seat the reactor commands, on a jump or that trigger |

> Never select a 90 fps RealSense profile. That USB service interval stalls the bus.

Detail is in [`arid_supervisor`](isaac_ros-dev/src/arid_supervisor/README.md),
[`px4_vslam`](isaac_ros-dev/src/px4_vslam/README.md) and
[`px4_vslam_reactor`](isaac_ros-dev/src/px4_vslam_reactor/README.md).

`config_realsense` writes each camera's serial into `vslam_config.yaml`. Run it after a camera
swap.

### Cameras

Both pipelines stream 1920x1080 `mono8` at 15 fps, uncalibrated until `cam_calibrate` runs.

| Command | Invokes | Result |
|---|---|---|
| `cam_front_start` | `/gst_camera_manager/cam_front` `SetBool{true}` | Forks `gst_cam_node`; `/cam_front/image_raw`, `/cam_front/image_raw/compressed` and `/cam_front/camera_info` appear, stamped `top_visual_link` |
| `cam_down_start` | `/gst_camera_manager/cam_down` `SetBool{true}` | The same three topics under `/cam_down`, stamped `bottom_visual_link` |
| `cam_front_stop`, `cam_down_stop` | The same service, `SetBool{false}` | Subprocess terminated, topics stop, `alive` false |
| `cam_stop` | Both pipeline services, `SetBool{false}` | Both pipelines stopped |
| `cam_front_status`, `cam_down_status` | `/gst_camera_manager/<name>/status` `Trigger` | `<name> RUNNING (pid=N)` or `<name> STOPPED` |
| `cam_front_alive`, `cam_down_alive` | Echoes `/gst_camera_manager/<name>/alive` | Latched `std_msgs/Bool`, false after 5 s without a frame |
| `cam_refresh` | `/gst_camera_manager/refresh` `Trigger` | Stops both pipelines, re-reads `pipelines.yaml`, rebuilds the per-pipeline services |
| `ver_cv_cams [front\|down\|both]` | `SetBool{true}`, then the compressed topic | Live window on the NoMachine desktop, `q` quits; pipelines it started are stopped on exit |
| no alias | `/gst_camera_manager/status_all` `Trigger` | One `[RUNNING]` or `[STOPPED]` line per pipeline |
| no alias | `/gst_camera_manager/stop_all` `Trigger` | Every running pipeline terminated |

The manager logs to `isaac_ros-dev/run_logs/gst_camera_manager/`; pipeline fields are in
[`ros_gst_cameras`](local_ws/src/ros_gst_cameras/README.md).

### USB reset

[`reset_usb`](local_ws/src/reset_ark_usb/README.md) powers the ARK PAB hub at USB location 1-2 off
and on, then pulses `gpiochip0` line 85, the standalone USB3 port. In the container it calls
`/reset_usb` (`std_srvs/Trigger`), which starts the same unit. Allow 20 s for re-enumeration.

> The power cycle reboots the FMU and drops all three RealSense off the bus. Call it only with the
> aircraft disarmed on the ground.

### Health

`local_test` checks the host units, aliases, Foxglove bridge, both CSI pipelines and the supervisor
services, and leaves the pipelines stopped.

### Visualization

Every shell and service runs `ROS_DOMAIN_ID=23` with `ROS_LOCALHOST_ONLY=1`, so a remote machine
reaches the graph only through `foxglove_bridge` on `ws://<device-ip>:8765`. The URDF is on
`/robot_description` with panel frame `base_link`.

## Development

Work happens on the drone over SSH; both workspaces build in place. `isaac_ros-dev/` bind-mounts
into the container at `/workspaces/isaac_ros-dev`.

| Command | Effect |
|---|---|
| `colcon_local` | Builds `local_ws` on the host; stops its services first, restarts them after |
| `colcon_isaac` | Builds `isaac_ros-dev` in the container; calls `deinitialize` first, restarts the supervisor |
| `build_isaac` | Builds the container image and enters it |
| `start_isaac`, `stop_isaac` | Starts or stops the container |
| `run_isaac`, `isaac_bash` | Starts and enters the container, or opens a shell in the running one |
| `clean_local`, `clean_isaac` | Removes `build`, `install` and `log` from that workspace |
| `rosdep_local`, `rosdep_isaac` | Installs the workspace dependencies |
| `colcon_isaac`, in the container | Builds the workspace alone, without the teardown and restart |
| `vslam`, in the container | Launches the stack without the supervisor |

### Submodules

`update_submods` checks the LIVE set against its tracked branch and the PINNED set against its tag.
`--update` moves the LIVE set to the branch tips.

| Submodule | Class |
|---|---|
| `local_ws/auxiliary/PX4-Autopilot` | LIVE, branch `PX4-InDro` |
| `isaac_ros-dev/src/realsense-ros` | LIVE, branch `v4.51.1` |
| `isaac_ros-dev/src/isaac_ros_visual_slam` | LIVE, branch `v3.2-14` |
| `isaac_ros-dev/src/px4_msgs` | LIVE, branch `release/1.15` |
| `isaac_ros-dev/src/isaac_ros_common` | PINNED, tag `v3.2-14` |
| `isaac_ros-dev/src/isaac_ros_nitros` | PINNED, tag `v3.2-14` |
| `isaac_ros-dev/src/isaac_ros_image_pipeline` | PINNED, tag `v3.2-14` |
| `isaac_ros-dev/src/px4-ros2-interface-lib` | PINNED, tag `1.4.0` |

## Setup menu

`setup` opens a menu of individual tasks. These are the ones with no alias.

| Option | Task |
|---|---|
| Full setup | Provisioning, below |
| Camera focus | Streams the CSI pipelines and the Foxglove bridge for a live focus pass |
| Install ARK-OS | Clones and runs the ARK-OS installer |
| Install ROS2 | ROS 2 Humble, from the ARK-OS tree; install ARK-OS first |
| Uninstall | Removes the units, rules, bashrc block and container, keeping the repo |

Building the Isaac workspace from the menu needs the container running, and is the route for a
workspace never built before: `colcon_isaac` calls `deinitialize`, which needs the supervisor on
the graph.

## Camera calibration

[`cam_calibrate front|down`](local_ws/auxiliary/camera_calibration/README.md) runs the ROS
calibrator against the live pipeline, applies the result on the pipeline's next start, and needs a
connected NoMachine session.

## Provisioning

Setup runs from `~/workspaces` and refuses to start with uninitialised submodules.

```
git clone --recurse-submodules <repo-url> ~/workspaces
cd ~/workspaces && ./setup.sh --full
```

`--full` and menu option 1 run the same provisioning: a questionnaire up front, then every step
unattended. Each step detects its own state, so a re-run configures only what is missing, and build
prompts default to skip. Setup reboots after an ARK-OS, ROS 2, JetPack or CSI-overlay install, and
again before the smoke test when a build ran; the next terminal prompts to resume.

| Flag | Runs |
|---|---|
| `--full` | The questionnaire, then the whole provisioning |
| `--resume` | The continuation a reboot armed |
| `--continue` | The tail after an abort, skipping checkpointed sections |
| `--help` | The flag list |

The questionnaire takes the Wi-Fi credentials and a ZeroTier network id; `wifi` and `zt_join`
change them later, and the drone is a member of exactly one ZeroTier network. Provisioning does not
calibrate the CSI cameras.

## Firmware

`local_ws/auxiliary/PX4-Autopilot` carries airframe `4026_arid_quad_v1_2`. A prebuilt image ships
at `local_ws/auxiliary/px4_compiled/arid.px4`.

```
cd local_ws/auxiliary/PX4-Autopilot && make ark_fmu-v6x_default
```

The build lands at `build/ark_fmu-v6x_default/ark_fmu-v6x_default.px4`. Flash a `.px4` from the
ARK-OS web interface, then select the airframe from a MAVLink shell.

```
param set SYS_AUTOSTART 4026
param save
reboot
```

## Layout

| Path | Contents |
|---|---|
| `setup.sh` | Menu, questionnaire and orchestration |
| `setup/` | Steps: `io.sh` helpers, `system.sh` host, `network.sh` ZeroTier, `ark.sh`, `container.sh` |
| `scripts/` | Host scripts behind the aliases |
| `local_ws/` | Host workspace, its units, firmware and calibration |
| `isaac_ros-dev/` | Container workspace, its units and container scripts |
| `log/`, `run_logs/` | Setup logs, node logs |
