"""Supervisor for the RoboSense RSAIRY LiDAR.

Manages an rslidar_sdk_node child process via SetBool/Trigger services and
publishes a latched /alive Bool based on PointCloud2 frame flow.

Idle at boot: the SDK subprocess does not spawn until /rslidar_coordinator/enable
is called with data=true.
"""

import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import PointCloud2


SDK_PKG = 'rslidar_sdk'
SDK_EXEC = 'rslidar_sdk_node'
CLOUD_TOPIC = '/rslidar_points'


class RslidarCoordinator(Node):
    def __init__(self):
        super().__init__('rslidar_coordinator')

        self.declare_parameter('alive_threshold', 5.0)
        self.declare_parameter('terminate_grace', 3.0)
        self.alive_threshold = float(self.get_parameter('alive_threshold').value)
        self.terminate_grace = float(self.get_parameter('terminate_grace').value)

        share_dir = get_package_share_directory('rslidar_coordinator')
        self.config_path = os.path.join(share_dir, 'config', 'rslidar.yaml')

        self.proc: subprocess.Popen | None = None
        self.proc_pgid: int | None = None  # captured at spawn (setsid → pgid == pid)
        self.last_frame_time: float | None = None
        self.alive_state = False

        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.alive_pub = self.create_publisher(Bool, '~/alive', latched_qos)
        self._publish_alive(False)

        self.create_service(SetBool, '~/enable', self._handle_enable)
        self.create_service(Trigger, '~/status', self._handle_status)
        self.create_service(Trigger, '~/restart', self._handle_restart)

        sensor_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(PointCloud2, CLOUD_TOPIC, self._cloud_cb, sensor_qos)

        self.create_timer(0.5, self._tick)

        self.get_logger().info(
            f'rslidar_coordinator ready. config_path={self.config_path}'
        )
        self.get_logger().info(
            f'alive_threshold={self.alive_threshold:.1f}s. '
            f'Call /rslidar_coordinator/enable {{data: true}} to start the LiDAR.'
        )

    def _publish_alive(self, state: bool) -> None:
        if state != self.alive_state:
            self.alive_state = state
            self.alive_pub.publish(Bool(data=state))

    def _cloud_cb(self, _msg: PointCloud2) -> None:
        self.last_frame_time = time.monotonic()

    def _tick(self) -> None:
        if self.proc is None:
            self._publish_alive(False)
            return

        rc = self.proc.poll()
        if rc is not None:
            self.get_logger().error(f'{SDK_EXEC} died (exit={rc})')
            self.proc = None
            self.last_frame_time = None
            self._publish_alive(False)
            return

        if self.last_frame_time is None:
            self._publish_alive(False)
            return

        stale_for = time.monotonic() - self.last_frame_time
        if stale_for > self.alive_threshold:
            if self.alive_state:
                self.get_logger().warn(
                    f'No frames on {CLOUD_TOPIC} for {stale_for:.2f}s '
                    f'(threshold {self.alive_threshold:.1f}s)'
                )
            self._publish_alive(False)
        else:
            if not self.alive_state:
                self.get_logger().info(f'Frames flowing on {CLOUD_TOPIC}.')
            self._publish_alive(True)

    def _spawn(self) -> tuple[bool, str]:
        if self.proc is not None and self.proc.poll() is None:
            return True, f'already running (pid={self.proc.pid})'

        cmd = [
            'ros2', 'run', SDK_PKG, SDK_EXEC,
            '--ros-args', '-p', f'config_path:={self.config_path}',
        ]
        try:
            # New session/process group so we can later signal the whole chain
            # (ros2 wrapper + rslidar_sdk_node) via killpg, even if the wrapper
            # doesn't forward SIGTERM to its child.
            self.proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
        except Exception as e:
            self.proc = None
            self.proc_pgid = None
            return False, f'spawn failed: {e}'

        # setsid in the child makes pgid == child pid; cache so we can still
        # signal the group after the immediate child (ros2 wrapper) exits.
        self.proc_pgid = self.proc.pid
        # Startup grace: first alive_threshold seconds after spawn don't count as stalled.
        self.last_frame_time = time.monotonic()
        self.get_logger().info(f'Spawned {SDK_EXEC} (pid={self.proc.pid}, pgid={self.proc_pgid})')
        return True, f'started (pid={self.proc.pid})'

    def _wait_pgroup_empty(self, pgid: int, timeout: float) -> bool:
        """Poll a process group until it contains no live processes.

        Returns True if the group drained within `timeout` seconds, False
        otherwise. Uses signal 0 (existence check, no signal delivered) —
        raises ProcessLookupError once no process in the group remains.

        Note: zombie processes still count as group members for `killpg(0)`
        purposes until their parent wait()s on them. The immediate child
        (the `ros2 run` wrapper) becomes a zombie under THIS coordinator
        when it exits, so we must poll `self.proc.poll()` inside the loop —
        that's what reaps it. Without that, the group never appears empty
        while the zombie wrapper lingers.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            # Reap our direct child if it has terminated (non-blocking).
            if self.proc is not None:
                self.proc.poll()
            try:
                os.killpg(pgid, 0)
            except ProcessLookupError:
                return True
            time.sleep(0.1)
        return False

    def _terminate(self) -> tuple[bool, str]:
        if self.proc is None or self.proc.poll() is not None:
            self.proc = None
            self.proc_pgid = None
            self.last_frame_time = None
            self._publish_alive(False)
            return True, 'already stopped'

        pid = self.proc.pid
        pgid = self.proc_pgid if self.proc_pgid is not None else pid
        self.get_logger().info(f'Terminating {SDK_EXEC} (pid={pid}, pgid={pgid})')

        # 1) SIGTERM the whole process group (ros2 wrapper + forked binary).
        try:
            os.killpg(pgid, signal.SIGTERM)
        except ProcessLookupError:
            pass  # group already empty — fall through to cleanup

        # 2) Wait for the WHOLE GROUP to drain, not just self.proc. The wrapper
        #    exits quickly on SIGTERM, but rslidar_sdk_node may take a couple of
        #    seconds to acknowledge if it's mid-loop in ERRCODE_MSOPTIMEOUT.
        drained = self._wait_pgroup_empty(pgid, self.terminate_grace)
        if not drained:
            self.get_logger().warn(
                f'process group {pgid} not empty after {self.terminate_grace:.1f}s; '
                f'sending SIGKILL'
            )
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            drained = self._wait_pgroup_empty(pgid, 2.0)
            if not drained:
                self.get_logger().error(
                    f'process group {pgid} still alive after SIGKILL; abandoning handle'
                )
                self.proc = None
                self.proc_pgid = None
                self.last_frame_time = None
                self._publish_alive(False)
                return False, 'failed to terminate (group still alive)'

        # 3) Reap the direct child to clean up its zombie entry (group is empty
        #    by now, but the kernel still holds the exit status until wait()).
        try:
            self.proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            pass  # group is gone; zombie will be reaped by init eventually

        self.proc = None
        self.proc_pgid = None
        self.last_frame_time = None
        self._publish_alive(False)
        return True, f'stopped (pid={pid})'

    def _handle_enable(self, req: SetBool.Request, resp: SetBool.Response):
        ok, msg = self._spawn() if req.data else self._terminate()
        resp.success = ok
        resp.message = msg
        return resp

    def _handle_status(self, _req, resp: Trigger.Response):
        if self.proc is not None and self.proc.poll() is None:
            resp.message = f'RUNNING (pid={self.proc.pid})'
        else:
            resp.message = 'STOPPED'
        resp.success = True
        return resp

    def _handle_restart(self, _req, resp: Trigger.Response):
        self._terminate()
        ok, msg = self._spawn()
        resp.success = ok
        resp.message = f'restarted ({msg})' if ok else f'restart failed ({msg})'
        return resp

    def shutdown(self) -> None:
        self._terminate()


def _sigterm_handler(*_):
    raise KeyboardInterrupt()


def main(args=None):
    rclpy.init(args=args)
    node = RslidarCoordinator()

    # Convert SIGTERM (from systemd stop) into KeyboardInterrupt so the finally
    # block runs and we terminate the SDK subprocess cleanly.
    signal.signal(signal.SIGTERM, _sigterm_handler)

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
