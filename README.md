# ARID

ARID is an indoor quadrotor: an NVIDIA Jetson Orin on an ARK PAB carrier, an ARK FMU running
PX4, and ROS 2 Humble in two workspaces, `local_ws` on the host and `isaac_ros-dev` inside the
Isaac ROS container. This repository holds both, and the provisioning that turns a stock Jetson
into a flight unit.

## Sensors

| Sensor | Fit |
|---|---|
| RealSense D43x | Front; IR stereo into cuVSLAM |
| RoboSense RSAIRY LiDAR | Ethernet on `enP8p1s0` |
| IMX477 CSI | Down camera |
| ARK optical flow and rangefinder | Bottom pod |

## Packages

| `local_ws` | Function |
|---|---|
| [`ros_gst_cameras`](local_ws/src/ros_gst_cameras/) | `gst_camera_manager` and the `gst_cam_node` it spawns per pipeline |
| [`rslidar_coordinator`](local_ws/src/rslidar_coordinator/) | Runs the RoboSense SDK node as a managed subprocess |
| [`reset_ark_usb`](local_ws/src/reset_ark_usb/) | Hosts `/reset_usb`, the carrier USB power cycle |
| [`arid_description`](local_ws/src/arid_description/) | Airframe xacro and its static transform tree |

| `isaac_ros-dev` | Function |
|---|---|
| [`arid_supervisor`](isaac_ros-dev/src/arid_supervisor/) | Lifecycle gate: camera-proven VSLAM bringup, landed teardown interlock |
| [`px4_vslam`](isaac_ros-dev/src/px4_vslam/) | cuVSLAM bringup; bridges the odometry into PX4 as external vision |
| [`px4_vslam_reactor`](isaac_ros-dev/src/px4_vslam_reactor/) | Withholds discontinuous cuVSLAM frames before they reach EKF2 |

Every other directory under either `src/` is a submodule.

## Boot

Setup installs the unit files from `local_ws/services/` and `isaac_ros-dev/services/`, and
enables all but the on-demand `reset_usb.service`.

| Unit | Provides |
|---|---|
| `start_isaac_docker` | The Isaac ROS container |
| `arid_supervisor` | The supervisor node inside that container |
| `gst_camera_manager` | The CSI pipeline manager |
| `rslidar_coordinator` | The LiDAR supervisor |
| `arid_description` | `robot_state_publisher` and the transform tree |
| `usb_ros_reset` | The `/reset_usb` service |
| `jetson-clocks` | Maximum clocks and fan |
| `usbfs-memory` | 1000 MB usbfs buffer for the RealSense |

The camera pipeline, the LiDAR driver and VSLAM start on request.

## Daily use

A login shell sources `local_ws/install/setup.bash` and defines the ARID aliases. `help` lists
them; run it inside the container for the container set.

### Visual odometry

| Command | Invokes | Result |
|---|---|---|
| `initialize` | `/arid_supervisor/vslam_enable`, `SetBool` true | Camera-proven bringup, up to 300 s per attempt, three attempts |
| `status` | `/arid_supervisor/status`, `Trigger` | VSLAM running or stopped, and the land state |
| `deinitialize` | `/arid_supervisor/vslam_enable`, `SetBool` false | Teardown, refused unless the drone is landed |

Bringup launches the front RealSense, `visual_slam_node`, `vslam_reactor` and `vio_transform`,
writing `run_logs/vslam/vslam.log`.

| Topic | Type | Carries |
|---|---|---|
| `/visual_slam/filt_slam_odometry` | `nav_msgs/Odometry` | The VO frames the reactor admits |
| `/fmu/in/vehicle_visual_odometry` | `px4_msgs/VehicleOdometry` | That solution, as PX4 external vision |
| `/reactor/vio_reset_epoch` | `std_msgs/UInt8` | Re-seat epoch, latched; becomes `reset_counter` |
| `/reactor/vo_healthy` | `std_msgs/Bool` | Latched false once jump re-seats exhaust the burst limit |
| `/reactor/drone_odom` | `nav_msgs/Odometry` | The FMU's own solution in FLU |
| `/reactor/drone_pose` | `geometry_msgs/PoseStamped` | The same pose, without the twist |

`config_realsense` writes the camera's serial into `vslam_config.yaml`; run it after a camera
swap. In the container, `vslam` launches the same graph without the gate or the interlock.

### Camera

| Command | Invokes | Result |
|---|---|---|
| `cam_down_start`, `cam_down_stop` | `/gst_camera_manager/cam_down`, `SetBool` true or false | Spawns or terminates `gst_cam_node`; frames stamped `bottom_visual_link` |
| `cam_down_status` | `/gst_camera_manager/cam_down/status`, `Trigger` | `cam_down RUNNING (pid=...)` or `cam_down STOPPED` |
| `cam_down_alive` | Reads `/gst_camera_manager/cam_down/alive` | Latched `Bool`, false 5 s after the last `camera_info` |
| `cam_refresh` | `/gst_camera_manager/refresh`, `Trigger` | Stops running pipelines, then re-reads `pipelines.yaml` |

| Topic | Type | Carries |
|---|---|---|
| `/cam_down/image_raw` | `sensor_msgs/Image` | 1080x1080 GRAY8 at 15 fps, cropped from 1920x1080 |
| `/cam_down/image_raw/compressed` | `sensor_msgs/CompressedImage` | The same frames, through `image_transport` |
| `/cam_down/camera_info` | `sensor_msgs/CameraInfo` | Calibration, one per frame |

`ver_cv_cams` shows the live feed on the NoMachine desktop; `q` quits.

### LiDAR

| Command | Invokes | Result |
|---|---|---|
| `rslidar_start`, `rslidar_stop` | `/rslidar_coordinator/enable`, `SetBool` true or false | Spawns `rslidar_sdk_node`, or SIGTERMs its process group |
| `rslidar_status` | `/rslidar_coordinator/status`, `Trigger` | `RUNNING (pid=...)` or `STOPPED` |
| `rslidar_alive` | Reads `/rslidar_coordinator/alive` | Latched `Bool`, false 5 s after the last cloud |
| `rslidar_restart` | `/rslidar_coordinator/restart`, `Trigger` | Terminate, then spawn |

| Topic | Type | Carries |
|---|---|---|
| `/rslidar_points` | `sensor_msgs/PointCloud2` | The scan, in `rslidar_link`, timestamped at end of scan |

The subprocess starts whether or not the LiDAR is reachable, and nothing restarts it when it
exits: `rslidar_restart` is the recovery.

### USB reset

`reset_usb` power-cycles the ARK PAB hub and the standalone USB3 port. The container alias calls
`/reset_usb` (`std_srvs/srv/Trigger`) for the same effect.

> The power cycle reboots the FMU. Never call it in flight.

### Health

`local_test` checks the host services, the aliases, the bridge socket and a full `cam_down` and
LiDAR lifecycle, leaving both stopped.

### Visualization

Every shell and service runs `ROS_DOMAIN_ID=23` with `ROS_LOCALHOST_ONLY=1`, so a remote machine
reaches the graph only through a bridge: `foxglove_bridge` on `ws://<device-ip>:8765`, from
either shell. The URDF is latched on `/robot_description`.

## Development

Work happens on the drone over SSH; both workspaces build in place. `isaac_ros-dev/` bind-mounts
into the container at `/workspaces/isaac_ros-dev`.

| Command | Builds |
|---|---|
| `colcon_local` | `local_ws` on the host; stops its services first, restarts them after |
| `colcon_isaac` | `isaac_ros-dev` in the container; deinitializes VSLAM first, restarts the supervisor |
| `build_isaac` | The container image |

`start_isaac`, `stop_isaac` and `isaac_bash` control and enter the container.

### Submodules

`update_submods` syncs every submodule to its pin and verifies it: branch-tracked entries against
their branch, tagged entries against their tag. `--update` advances both. `PX4-Autopilot`,
`isaac_ros_visual_slam` and `realsense-ros` are InDro forks carrying local patches.

## Setup menu

`setup` opens a menu of individual tasks. These are the ones with no alias.

| Option | Task |
|---|---|
| 1 Full setup | Provisioning, below |
| 5 CSI overlay | Applies the 2-lane IMX477 jetson-io overlay; requires a reboot |
| 10 Camera focus | Streams `cam_down` and the bridge for a live focus pass; `q` stops both |
| 14 Install ARK-OS | Clones and runs the ARK-OS installer |
| 15 Install ROS2 | ROS 2 Humble, from the ARK-OS tree; install ARK-OS first |
| 17 Uninstall | Removes the units, network profiles, rules, bashrc block and container |

The IMX477 reaches `sensor-id=0` only after option 5 and a reboot.

### LiDAR link

| Command | Action |
|---|---|
| `config_lidar` | Rewrites the `rslidar` profile and dispatcher to the addresses the LiDAR carries |
| `lidar_diag` | Walks link, profile, address, ARP probe and cloud rate; sudo adds a packet sniff |

Run `config_lidar` after a LiDAR swap. Undetected, the fallback stays at `192.168.1.102/24` to
`192.168.1.200`; on link-up the dispatcher ARP-probes for 8 s, then falls back to DHCP.

### Camera calibration

`cam_calibrate` runs the ROS calibrator against the live `cam_down` pipeline and needs a
connected NoMachine session. The included board
(`local_ws/auxiliary/camera_calibration/calibration_pattern/calib_pattern.pdf`) is 10x7 squares
of 50 mm; the script prompts for the measured square size, or set `SIZE` and `SQUARE`. It writes
`cam_down.yaml`: set `calibration: "cam_down"` in `pipelines.yaml`, then restart the pipeline.

## Provisioning

Setup runs from `~/workspaces` and refuses to start with uninitialised submodules.

```
git clone --recurse-submodules <repo-url> ~/workspaces
cd ~/workspaces && ./setup.sh --full
```

`--full` and menu option 1 run the same provisioning: a questionnaire whose prompts all default
to skip, then every step unattended, each configuring only what is missing. The run reboots at
most twice; open a terminal after each and answer the resume prompt. `./setup.sh --help` lists
the remaining flags.

> Setup over the wired port defers the LiDAR network step. Finish it over Wi-Fi with
> `config_lidar`.

The questionnaire takes a Wi-Fi network and a ZeroTier network id; `wifi` and `zt_join` change
them later. The drone is a member of exactly one ZeroTier network.

## Firmware

The PX4 fork is on branch `PX4-InDro` and carries the `4026_arid_quad_v1_2` airframe; a built
image is kept at `local_ws/auxiliary/px4_compiled/arid.px4`.

```
cd local_ws/auxiliary/PX4-Autopilot && make ark_fmu-v6x_default
```

The build lands at `build/ark_fmu-v6x_default/ark_fmu-v6x_default.px4`. Flash it from the ARK-OS
web interface, then select the airframe from a MAVLink shell.

```
param set SYS_AUTOSTART 4026
param save
reboot
```

## Layout

| Path | Contents |
|---|---|
| `setup.sh` | Menu, questionnaire and orchestration |
| `setup/` | Steps: `io.sh` helpers, `system.sh` host, `network.sh`, `lidar.sh`, `ark.sh`, `container.sh` |
| `scripts/` | Host scripts behind the aliases |
| `local_ws/` | Host workspace, its units, firmware and calibration |
| `isaac_ros-dev/` | Container workspace, its units and container scripts |
| `archive/`, `.docs/archive.zip` | Archived records |
| `log/`, `run_logs/` | Setup logs, node logs |
