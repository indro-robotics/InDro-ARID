# reset_ark_usb

This package hosts `/reset_usb`, the ROS 2 service that hardware-resets the USB ports on the ARK PAB Orin carrier. The service starts `reset_usb.service`, which runs `scripts/usb_reset.sh`: `uhubctl` power-cycles the hub and `gpioset` pulses the FMU reset line. `usb_ros_reset.service` runs the node at boot.

> `/reset_usb` also resets the flight controller. Never call it in flight.

## Service interface

The node exposes one service.

| Service | Type | Response |
|---|---|---|
| `/reset_usb` | `std_srvs/srv/Trigger` | `success: bool`, `message: string` carrying systemctl output |

```bash
ros2 service call /reset_usb std_srvs/srv/Trigger "{}"
```

The hub topology and the reset GPIO in `scripts/usb_reset.sh` are specific to the ARK PAB Orin carrier.

## Troubleshooting

Four symptoms account for most failures.

| Symptom | Likely cause |
|---|---|
| The call blocks indefinitely | The sudoers rule is missing, so a hidden password prompt is waiting. |
| `success: false`, `a password is required` | The same missing sudoers rule. |
| `success: false`, `Unit reset_usb.service not found` | The unit is not installed. Re-run `setup.sh`. |
| The ports do not reset | `uhubctl` cannot detect the hub. Check `uhubctl -l`, permissions and kernel modules. |

## License

The package is released under Apache-2.0.
