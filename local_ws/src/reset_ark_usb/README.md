# reset_ark_usb

ROS 2 node that exposes a service to hardware-reset the USB ports on the **ARK PAB Orin carrier board**. Internally wraps a systemd unit (`reset_usb.service`) that shells out to `uhubctl` to cycle the USB hub.

---

## How it works

```
ros2 service call /reset_usb std_srvs/srv/Trigger "{}"
             │
             ▼
  ResetUsbService node (this package)
             │  sudo /bin/systemctl start reset_usb.service
             ▼
      reset_usb.service (systemd)
             │  runs scripts/usb_reset.sh
             ▼
          uhubctl  →  USB hub power-cycled
```

The node is intentionally thin — it's the glue between a ROS service and the privileged `systemctl` call. The actual USB reset logic lives in `scripts/usb_reset.sh` at the workspace root.

---

## Service interface

| Service | Type | Request | Response |
|---|---|---|---|
| `/reset_usb` | `std_srvs/srv/Trigger` | *(empty)* | `success: bool`, `message: string` |

- On success: `message` contains `systemctl` stdout (typically empty).
- On failure: `message` contains `systemctl` stderr.

### CLI usage

```bash
ros2 service call /reset_usb std_srvs/srv/Trigger "{}"
```

Expected response on success:
```
response:
std_srvs.srv.Trigger_Response(success=True, message='USB reset triggered: ')
```

### Programmatic use (Python)

```python
from std_srvs.srv import Trigger

client = self.create_client(Trigger, '/reset_usb')
client.wait_for_service(timeout_sec=5.0)
future = client.call_async(Trigger.Request())
rclpy.spin_until_future_complete(self, future)
# future.result().success, future.result().message
```

---

## Prerequisites

These are set up by the workspace-level `setup.sh` — included here for reference / portability.

1. **`reset_usb.service` installed + enabled** (`local_ws/services/reset_usb.service`).
2. **Sudoers rule** allowing the `jetson` user to run `systemctl start reset_usb.service` without a password prompt (otherwise the subprocess call hangs waiting for input).
3. **`uhubctl`** installed on the host, with the PAB carrier's USB hub accessible to it.
4. **ARK PAB Orin carrier** — the hub topology / port numbering in `scripts/usb_reset.sh` is specific to this board. Other carriers will need their own reset script.

---

## Build & run

```bash
cd ~/workspaces/local_ws
colcon build --packages-select reset_ark_usb --symlink-install
source install/setup.bash
ros2 run reset_ark_usb reset_usb_service
```

Or let the `usb_ros_reset.service` systemd unit start it automatically on boot (installed by `setup.sh`).

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Service call hangs forever | Sudoers rule missing — `systemctl` prompting for password invisibly. |
| `success: false` with `sudo: a password is required` | Same as above. |
| `success: false` with `Unit reset_usb.service not found` | `reset_usb.service` not installed — re-run `setup.sh`. |
| Ports don't actually reset | `uhubctl` can't see the hub — check `uhubctl -l` lists the PAB hub; permissions / kernel modules. |

---

## License

Apache-2.0
