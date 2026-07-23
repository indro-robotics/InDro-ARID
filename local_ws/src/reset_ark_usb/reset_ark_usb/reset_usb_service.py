#!/usr/bin/env python3

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
            # Absolute /bin/systemctl path: the sudoers NOPASSWD rule matches on it.
            result = subprocess.run(
                ['sudo', '/bin/systemctl', 'start', 'reset_usb.service'],
                check=True,
                capture_output=True,
                text=True
            )
            response.success = True
            response.message = f"USB reset triggered: {result.stdout}"
        except subprocess.CalledProcessError as e:
            response.success = False
            response.message = f"Failed: {e.stderr}"
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
