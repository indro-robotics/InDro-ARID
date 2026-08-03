#!/usr/bin/env python3
# Hosts /reset_usb; usb_ros_reset.service runs it on the host at boot.
# The unit it starts power-cycles the ARK PAB hub: the FMU reboots and the RealSense cameras
# drop off the bus. A call in flight reboots the flight controller.

import subprocess
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class ResetUsbService(Node):
    def __init__(self):
        super().__init__('reset_usb_service')
        self.srv = self.create_service(
            Trigger,
            'reset_usb',
            self.reset_callback)
        
    def reset_callback(self, request, response):
        try:
            # sudo matches the NOPASSWD rule in /etc/sudoers.d/<user>_systemctl by literal path;
            # a path not listed there prompts for a password and fails with no tty.
            result = subprocess.run(
                ['sudo', '/bin/systemctl', 'start', 'reset_usb.service'],
                check=True,
                capture_output=True,
                text=True
            )
            # The unit is oneshot, so this returns when the power cycle ends, not when the FMU
            # has booted and the cameras have re-enumerated.
            response.success = True
            response.message = f"USB reset triggered: {result.stdout}"
        except subprocess.CalledProcessError as e:
            response.success = False
            response.message = f"Failed: {e.stderr}"
        except OSError as e:
            response.success = False
            response.message = f"Failed to run the reset script: {e}"
        return response

def main():
    rclpy.init()
    node = ResetUsbService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
