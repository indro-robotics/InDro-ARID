import rclpy
import subprocess
from rclpy.node import Node
from std_srvs.srv import Trigger
from gst_camera_interfaces.srv import ControlService 

class CameraSwitcher(Node):
    def __init__(self):
        super().__init__('cam_switcher')
        self.declare_parameter('service_name', '')
        self.service_name = self.get_parameter('service_name').value
        
        self.serving = False
        self.kill_flag = False
        self.service_list = ['cam_down_2k_10',
                             'cam_down_2k_20', 
                             'cam_down_2k_30', 
                             'cam_front_4k_5', 
                             'cam_front_4k_10',
                             'cam_front_4k_21']

        self.start_srv = self.create_service(
            ControlService,
            'start_camera',
            self.start_callback)

        self.stop_srv = self.create_service(
            ControlService,
            'stop_camera',
            self.stop_callback)
            
        self.get_logger().info("Camera Switcher ready")

    def stop_current_service(self):
        for service_name in self.service_list:
            check_cmd = ['systemctl', 'is-active', f'{service_name}.service']
            try:
                status = subprocess.check_output(check_cmd).decode().strip()
                if status == 'active':
                    self.get_logger().info(f"Stopping {service_name}")
                    try:
                        self.control_service('stop', service_name)
                    except RuntimeError as e:
                        self.get_logger().warning(f"Stop failed for {service_name}, attempting kill: {e}")
                        try:
                            self.control_service('kill', service_name)
                        except RuntimeError as e2:
                            self.get_logger().error(f"Kill failed for {service_name}: {e2}")
            except subprocess.CalledProcessError:
                # Service is not active; nothing to do
                continue
        self.serving = False

    def stop_callback(self, request, response):
        self.stop_current_service()
        response.success = True
        response.message = f"Stopped all camera services."
        return response

    def start_callback(self, request, response):
        try:
            if not request.service_name:
                raise ValueError("Service name required")

            # Stop current service if running
            if self.serving:
                self.stop_current_service()

            # Start new service
            self.control_service('start', request.service_name)
            
            self.serving = True
            response.success = True
            response.message = f"Started {request.service_name}"

        except Exception as e:
            self.get_logger().error(f"Start failed: {str(e)}")
            response.success = False
            response.message = str(e)
            
        return response
    
    def control_service(self, action, service_name):
        try:
            cmd = ['sudo', 'systemctl', action, f'{service_name}.service'] # passwordless sudo
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE)
                
            stdout, stderr = process.communicate()
            
            if process.returncode != 0:
                if action == 'stop':
                    self.kill_flag = True
                raise RuntimeError(stderr.decode())   

        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Service control error: {e}")
            raise

def main(args=None):
    rclpy.init(args=args)
    switcher = CameraSwitcher()
    rclpy.spin(switcher)
    switcher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
