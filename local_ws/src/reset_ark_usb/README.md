# reset_ark_usb

This package hosts `/reset_usb`, the ROS 2 service that hardware-resets the USB ports on the ARK PAB Orin carrier. The service starts `reset_usb.service`, which runs `scripts/usb_reset.sh`: `uhubctl` power-cycles the carrier hub and `gpioset` power-cycles the standalone USB3 port on `gpiochip0` line 85. The hub topology and that GPIO line are specific to the ARK PAB Orin carrier. `usb_ros_reset.service` starts the node on the host at boot with `ROS_DOMAIN_ID=23` and `ROS_LOCALHOST_ONLY=1`, so callers on the host and inside the Isaac container reach the same service.

> `/reset_usb` also resets the flight controller. Never call it in flight.

## Service interface

The node exposes one service and declares no parameters.

| Service | Type | Response |
|---|---|---|
| `/reset_usb` | `std_srvs/srv/Trigger` | `success: bool`, `message: string` carrying the `systemctl` output |

```bash
ros2 service call /reset_usb std_srvs/srv/Trigger "{}"
```

The console entry point is `reset_usb_service`.

```bash
ros2 run reset_ark_usb reset_usb_service
```

## Effect

A successful call power-cycles the USB hub and reboots the flight controller. All three RealSense drop off the bus and re-enumerate. Allow about 20 s for enumeration to finish before treating a camera as missing.

## Troubleshooting

Five symptoms account for most failures.

| Symptom | Likely cause |
|---|---|
| `/reset_usb` is absent from `ros2 service list` | `usb_ros_reset.service` is down, or the caller is not on `ROS_DOMAIN_ID=23`. |
| The call blocks indefinitely | The sudoers rule is missing, so a hidden password prompt is waiting. |
| `success: false`, `a password is required` | The same missing sudoers rule. |
| `success: false`, `Unit reset_usb.service not found` | The unit is not installed. Re-run `setup.sh`. |
| The ports do not reset | `uhubctl` cannot detect the hub. Check `uhubctl -l`, permissions and kernel modules. |

## License

The package is released under Apache-2.0.
