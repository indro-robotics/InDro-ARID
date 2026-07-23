# reset_ark_usb

ROS 2 node exposing `/reset_usb`: hardware-resets the USB ports on the **ARK PAB Orin carrier** by starting `reset_usb.service`, which runs `scripts/usb_reset.sh` (`uhubctl` power-cycles the hub, `gpioset` pulses the FMU reset line).

> `/reset_usb` also resets the flight controller. Never call it in flight.

## Service interface

| Service | Type | Response |
|---|---|---|
| `/reset_usb` | `std_srvs/srv/Trigger` | `success: bool`, `message: string` (systemctl stdout/stderr) |

```bash
ros2 service call /reset_usb std_srvs/srv/Trigger "{}"
```

Programmatic (Python):

```python
from std_srvs.srv import Trigger

client = self.create_client(Trigger, '/reset_usb')
client.wait_for_service(timeout_sec=5.0)
future = client.call_async(Trigger.Request())
rclpy.spin_until_future_complete(self, future)
```

## Prerequisites

All installed by the workspace `setup.sh`:

- `reset_usb.service` (started on demand, not enabled at boot)
- sudoers rule for passwordless `systemctl start`; without it the call blocks on a hidden password prompt
- `uhubctl` and `gpioset` on the host
- ARK PAB Orin carrier: hub topology and reset GPIO in `scripts/usb_reset.sh` are board-specific

## Build and run

```bash
cd ~/workspaces/local_ws
colcon build --packages-select reset_ark_usb --symlink-install
source install/setup.bash
ros2 run reset_ark_usb reset_usb_service
```

`usb_ros_reset.service` starts this node at boot.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Call blocks indefinitely | Sudoers rule missing; hidden password prompt. |
| `success: false`, `a password is required` | Same: sudoers rule missing. |
| `success: false`, `Unit reset_usb.service not found` | Service not installed. Re-run `setup.sh`. |
| Ports do not reset | `uhubctl` cannot detect the hub. Check `uhubctl -l`, permissions, kernel modules. |

## License

Apache-2.0
