# reset_ark_usb

This package exposes `/reset_usb`, a ROS 2 service that hardware-resets the USB ports on the ARK
PAB Orin carrier. The node starts `reset_usb.service`, which runs `scripts/usb_reset.sh`:
`uhubctl` power-cycles the hub and `gpioset` pulses the FMU reset line. The hub topology and the
reset GPIO in that script are specific to the ARK PAB carrier.

`usb_ros_reset.service` starts the node at boot.

> `/reset_usb` also resets the flight controller. Never call it in flight.

## Service interface

| Service | Type | Response |
|---|---|---|
| `/reset_usb` | `std_srvs/srv/Trigger` | `success: bool`, `message: string` carrying systemctl output. |

```bash
ros2 service call /reset_usb std_srvs/srv/Trigger "{}"
```

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| The call blocks indefinitely | The sudoers rule is missing and sudo is waiting on a hidden password prompt. |
| `success: false`, `a password is required` | The same missing sudoers rule. |
| `success: false`, `Unit reset_usb.service not found` | The unit is not installed; re-run `setup.sh`. |
| The ports do not reset | `uhubctl` cannot see the hub; check `uhubctl -l`, permissions and kernel modules. |

## License

Apache-2.0
