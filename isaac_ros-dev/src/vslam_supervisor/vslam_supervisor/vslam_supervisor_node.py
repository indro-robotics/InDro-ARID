"""Lifecycle supervisor for the px4_vslam stack.

Hosts a single `std_srvs/SetBool` service:
  /vslam_supervisor/vslam_enable

`true`  spawns `ros2 launch px4_vslam vslam.launch.py` as a managed subprocess.
`false` requires a fresh `VehicleLandDetected.landed == True` sample, then sends
SIGTERM to the launch's process group with a SIGKILL fallback.

Per-subprocess stdout/stderr is appended to /tmp/vslam_supervisor/vslam.log.
"""

import os
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool

from px4_msgs.msg import VehicleLandDetected


LAND_FRESH_S = 2.0
TERM_WAIT_S = 5.0
LOG_DIR = '/tmp/vslam_supervisor'


class _Stack:
    def __init__(self, name, launch_pkg, launch_file):
        self.name = name
        self.launch_pkg = launch_pkg
        self.launch_file = launch_file
        self.proc = None

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self):
        os.makedirs(LOG_DIR, exist_ok=True)
        log_path = os.path.join(LOG_DIR, f'{self.name}.log')
        log = open(log_path, 'ab', buffering=0)
        self.proc = subprocess.Popen(
            ['ros2', 'launch', self.launch_pkg, self.launch_file],
            stdout=log, stderr=subprocess.STDOUT, preexec_fn=os.setsid,
        )
        return log_path

    def stop(self):
        if not self.alive():
            self.proc = None
            return
        pgid = os.getpgid(self.proc.pid)
        try:
            os.killpg(pgid, signal.SIGTERM)
        except ProcessLookupError:
            self.proc = None
            return
        deadline = time.monotonic() + TERM_WAIT_S
        while time.monotonic() < deadline:
            if self.proc.poll() is not None:
                self.proc = None
                return
            time.sleep(0.1)
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            self.proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            pass
        self.proc = None


class VslamSupervisor(Node):
    def __init__(self):
        super().__init__('vslam_supervisor')

        self.vslam = _Stack('vslam', 'px4_vslam', 'vslam.launch.py')

        self._lock = threading.Lock()
        self._landed = None
        self._landed_at = 0.0

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected',
            self._land_cb, px4_qos,
        )
        self.create_service(SetBool, '~/vslam_enable', self._vslam_cb)

        self.get_logger().info('vslam_supervisor up - vslam_enable available')

    def _land_cb(self, msg):
        self._landed = bool(msg.landed)
        self._landed_at = time.monotonic()

    def _landed_fresh(self):
        if self._landed is None:
            return None
        if time.monotonic() - self._landed_at > LAND_FRESH_S:
            return None
        return self._landed

    def _gate_landed(self, resp, action):
        state = self._landed_fresh()
        if state is None:
            resp.success = False
            resp.message = f'land state unknown (no recent PX4 telemetry); cannot {action}'
            return False
        if not state:
            resp.success = False
            resp.message = f'drone not landed; land before {action}'
            return False
        return True

    def _vslam_cb(self, req, resp):
        with self._lock:
            if req.data:
                if self.vslam.alive():
                    resp.success = True
                    resp.message = 'vslam already running'
                    return resp
                log = self.vslam.start()
                resp.success = True
                resp.message = f'vslam launching (log: {log})'
                self.get_logger().info(resp.message)
                return resp

            if not self.vslam.alive():
                resp.success = True
                resp.message = 'vslam already stopped'
                return resp

            if not self._gate_landed(resp, 'disabling vslam'):
                self.get_logger().warn(resp.message)
                return resp

            self.vslam.stop()
            resp.success = True
            resp.message = 'vslam stopped'
            self.get_logger().info(resp.message)
            return resp

    def shutdown(self):
        with self._lock:
            self.vslam.stop()


def main():
    rclpy.init()
    node = VslamSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
