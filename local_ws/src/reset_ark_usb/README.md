# reset_ark_usb

reset_ark_usb hosts `/reset_usb`, the ROS 2 trigger for the ARK PAB carrier USB power cycle.

> The power cycle reboots the FMU and re-enumerates the RealSense cameras. Call it only with the
> aircraft disarmed on the ground.

## Node

`usb_ros_reset.service` runs `reset_usb_service` on the host at boot, from the `local_ws` overlay on
`ROS_DOMAIN_ID=23` with `ROS_LOCALHOST_ONLY=1`. The node creates no publishers, subscriptions,
clients or parameters.

## Service

| Service | Type | Effect |
|---|---|---|
| `/reset_usb` | `std_srvs/srv/Trigger` | Starts the host unit `reset_usb.service`, returns when it exits |

| Response | Value |
|---|---|
| `success` | `true` when `sudo /bin/systemctl start reset_usb.service` exits zero |
| `message`, success | `USB reset triggered:` and the systemctl stdout |
| `message`, failure | `Failed:` and the stderr, or `Failed to run the reset script:` |

```bash
ros2 service call /reset_usb std_srvs/srv/Trigger "{}"
```

The unit is `Type=oneshot`, so a `uhubctl` or `gpioset` failure returns `success: false`. USB
re-enumeration and FMU boot continue after the call returns.

## Effect

| Stage | Action |
|---|---|
| `reset_usb.service` | Runs `scripts/usb_reset.sh` as root |
| `uhubctl -l 1-2` | Powers the ARK PAB hub off, then on |
| `gpioset gpiochip0 85=0` | 1 s open-drain pulse on the standalone USB3 port |

Allow 20 s for re-enumeration before treating a camera as missing.

## Callers

| Caller | Invocation |
|---|---|
| `reset_usb` alias, container shell | Checks `/reset_usb` is listed, then calls it |
| `reset_usb` alias, host shell | Runs `scripts/usb_reset.sh` directly, without this node |
| `arid_supervisor` | One call at the vslam bringup pre-check, one in its single recovery, each when fewer RealSense enumerate than configured |

## Troubleshooting

| Symptom | Cause |
|---|---|
| `/reset_usb` absent from `ros2 service list` | `usb_ros_reset.service` is not running, or the caller is not on `ROS_DOMAIN_ID=23` |
| `/reset_usb` absent for the length of a `colcon_local` build | The build stops `usb_ros_reset.service` and starts it again on completion |
| `success: false`, `message` reports a `sudo` password failure | The sudoers rule is missing. Re-run `setup.sh` |
| `success: false`, `message` reports `reset_usb.service` not found | The unit is not installed. Re-run `setup.sh` |
| `success: false`, `message` reports a `uhubctl` failure | `uhubctl` is not installed, or it finds no hub at location 1-2 |
