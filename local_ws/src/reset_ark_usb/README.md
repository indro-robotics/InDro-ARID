# reset_ark_usb

This package hosts `/reset_usb`, the ROS 2 service that power-cycles the USB ports on the ARK PAB carrier. The service starts `reset_usb.service`, which runs `scripts/usb_reset.sh`: `uhubctl` power-cycles the hub at location 1-2, then `gpioset` holds `gpiochip0` line 85, the standalone USB3 port, low for one second. `usb_ros_reset.service` starts the node on the host at boot with `ROS_DOMAIN_ID=23` and `ROS_LOCALHOST_ONLY=1`. Callers on the host and inside the Isaac container reach the same service.

> `/reset_usb` also reboots the flight controller. Never call it in flight.

## Service interface

The node exposes one service and declares no parameters.

| Service | Type |
|---|---|
| `/reset_usb` | `std_srvs/srv/Trigger` |

```bash
ros2 service call /reset_usb std_srvs/srv/Trigger "{}"
```

`success` is true when `sudo /bin/systemctl start reset_usb.service` returns 0. Otherwise `message` carries the `systemctl` stderr, or the `OSError` text if `sudo` could not be launched. The console entry point is `reset_usb_service`.

## Effect

The call returns once `usb_reset.sh` has finished, so a `uhubctl` or `gpioset` failure comes back as `success: false`. The flight controller reboots with the hub. All three RealSense drop off the bus and re-enumerate; allow about 20 s before treating a camera as missing.

## Troubleshooting

The node reports a failed reset as `success: false` with the cause in `message`.

| Symptom | Cause |
|---|---|
| `/reset_usb` is absent from `ros2 service list` | `usb_ros_reset.service` is not running, or the caller is not on `ROS_DOMAIN_ID=23`. |
| `/reset_usb` is absent for the length of a `colcon_local` build | The build stops `usb_ros_reset.service` and starts it again on completion. |
| `success: false`, `message` reports a `sudo` password failure | The sudoers rule is missing. Re-run `setup.sh`. |
| `success: false`, `message` reports `reset_usb.service` not found | The unit is not installed. Re-run `setup.sh`. |
| `success: false`, `message` reports a `uhubctl` failure | `uhubctl` is not installed, or it finds no hub at location 1-2. |

## License

The package is released under Apache-2.0.
