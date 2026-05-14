#!/usr/bin/env python

import os
import yaml
import signal
import subprocess
from datetime import datetime
from pathlib import Path
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import CameraInfo
from rclpy.qos import (QoSProfile,
                       QoSReliabilityPolicy,
                       QoSHistoryPolicy,
                       QoSDurabilityPolicy,
                       qos_profile_sensor_data)


MAX_LOG_FILES = 20
DEFAULT_ALIVE_THRESHOLD = 5.0   # seconds, used when a pipeline omits alive_threshold
WATCHDOG_PERIOD = 0.5           # seconds, how often to check liveness

ALIVE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)

# QoS to use on camera_info subscription when the pipeline is configured with `reliable: true`.
# Matches gst_cam_node's reliable-volatile publisher in that mode.
CAMERA_INFO_QOS_RELIABLE = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5
)


def _parse_reliable(raw):
    """Normalize the YAML `reliable` field. Accepts bool, str ('true'/'false'/''),
    or missing. Anything except truthy-true maps to False."""
    if isinstance(raw, bool):
        return raw
    if isinstance(raw, str):
        return raw.strip().lower() == 'true'
    return False


####################################################################################################
# GST CAMERA MANAGER ###############################################################################
class GstCameraManager(Node):
    def __init__(self):
        super().__init__('gst_camera_manager')

        self.pipelines         = {}   # name -> dict (gst_pipeline, calibration, topic, frame_id, encoding, alive_threshold, reliable)
        self.processes         = {}   # name -> Popen or None
        self.log_files         = {}   # name -> open file handle or None
        self.alive_pubs        = {}   # name -> Publisher<Bool>
        self.alive_state       = {}   # name -> bool (last published alive value; kept in sync with _publish_alive)
        self.alive_thresholds  = {}   # name -> float seconds
        self.reliable_flags    = {}   # name -> bool (parsed from YAML `reliable`)
        self.last_frame_time   = {}   # name -> rclpy.time.Time (set on each camera_info callback)
        self.info_subs         = {}   # name -> Subscription<CameraInfo>
        self.srv_handles       = {}   # name -> service handles (keep alive)
        self.process_lock      = Lock()

        self.launch_env = os.environ.copy()
        # Aravis is built from source — ensure the GStreamer plugin is always found
        # regardless of whether GST_PLUGIN_PATH is set in the calling shell.
        aravis_gst = '/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0'
        existing = self.launch_env.get('GST_PLUGIN_PATH', '')
        self.launch_env['GST_PLUGIN_PATH'] = (
            aravis_gst + ':' + existing if existing else aravis_gst
        )

        pkg_share = Path(get_package_share_directory('gst_camera_manager'))
        self.log_root   = pkg_share / 'logs'
        self.calib_root = pkg_share / 'config' / 'calibrations'

        config_path = pkg_share / 'config' / 'pipelines.yaml'
        self._load_config(config_path)
        self._create_publishers()
        self._create_info_subscriptions()
        self._create_services()
        self._create_watchdog()

        self.get_logger().info('GstCameraManager ready — %d pipeline(s) registered' % len(self.pipelines))

    ################################################################################################
    def _load_config(self, config_path):
        try:
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)
            for name, info in data.get('pipelines', {}).items():
                self.pipelines[name] = info
                self.processes[name] = None
                self.log_files[name] = None
                self.alive_state[name] = False
                self.alive_thresholds[name] = float(info.get('alive_threshold', DEFAULT_ALIVE_THRESHOLD))
                self.reliable_flags[name] = _parse_reliable(info.get('reliable'))
                self.last_frame_time[name] = None
                self.get_logger().info(
                    'Registered pipeline: %s  (topic: /%s/image_raw, alive_threshold=%.2fs, qos=%s)' % (
                        name, info.get('topic', name), self.alive_thresholds[name],
                        'RELIABLE' if self.reliable_flags[name] else 'BEST_EFFORT'))
        except Exception as e:
            self.get_logger().error('Failed to load config: %s' % str(e))

    ################################################################################################
    def _create_publishers(self):
        for name in self.pipelines:
            pub = self.create_publisher(Bool, 'gst_camera_manager/%s/alive' % name, ALIVE_QOS)
            self.alive_pubs[name] = pub
            msg = Bool()
            msg.data = False
            pub.publish(msg)

    ################################################################################################
    def _publish_alive(self, name, alive):
        try:
            msg = Bool()
            msg.data = alive
            self.alive_pubs[name].publish(msg)
            self.alive_state[name] = alive
        except Exception:
            pass

    ################################################################################################
    def _create_info_subscriptions(self):
        # Subscribe to each pipeline's camera_info topic. The subscription persists for the
        # node's lifetime; it sits idle until the subprocess starts publishing and is the
        # signal the watchdog uses to prove frames are flowing.
        cbg = MutuallyExclusiveCallbackGroup()
        for name in self.pipelines:
            topic = self.pipelines[name].get('topic', name)
            info_topic = '/%s/camera_info' % topic
            qos = CAMERA_INFO_QOS_RELIABLE if self.reliable_flags[name] else qos_profile_sensor_data
            sub = self.create_subscription(
                CameraInfo,
                info_topic,
                lambda msg, n=name: self._on_camera_info(n, msg),
                qos,
                callback_group=cbg,
            )
            self.info_subs[name] = sub

    ################################################################################################
    def _on_camera_info(self, name, _msg):
        # Single-key dict write — atomic under the GIL, no lock needed.
        self.last_frame_time[name] = self.get_clock().now()

    ################################################################################################
    def _create_services(self):
        for name in self.pipelines:
            cbg_ctrl   = MutuallyExclusiveCallbackGroup()
            cbg_status = MutuallyExclusiveCallbackGroup()

            ctrl_srv = self.create_service(
                SetBool,
                'gst_camera_manager/%s' % name,
                lambda req, res, n=name: self._handle_pipeline_srv(req, res, n),
                callback_group=cbg_ctrl
            )
            status_srv = self.create_service(
                Trigger,
                'gst_camera_manager/%s/status' % name,
                lambda req, res, n=name: self._handle_status_srv(req, res, n),
                callback_group=cbg_status
            )
            self.srv_handles[name] = (ctrl_srv, status_srv)

        cbg_all = MutuallyExclusiveCallbackGroup()
        self.status_all_srv = self.create_service(
            Trigger, 'gst_camera_manager/status_all',
            self._handle_status_all_srv, callback_group=cbg_all)

        cbg_stop = MutuallyExclusiveCallbackGroup()
        self.stop_all_srv = self.create_service(
            Trigger, 'gst_camera_manager/stop_all',
            self._handle_stop_all_srv, callback_group=cbg_stop)

    ################################################################################################
    def _create_watchdog(self):
        cbg = MutuallyExclusiveCallbackGroup()
        self.watchdog_timer = self.create_timer(WATCHDOG_PERIOD, self._watchdog_tick, callback_group=cbg)

    ################################################################################################
    def _watchdog_tick(self):
        now = self.get_clock().now()
        with self.process_lock:
            for name, proc in list(self.processes.items()):
                if proc is None:
                    continue

                # 1. Process-death check: subprocess exited → not alive, cleanup, log.
                if proc.poll() is not None:
                    exit_code = proc.returncode
                    self.processes[name] = None
                    self._close_log(name)
                    self._publish_alive(name, False)
                    self.get_logger().warn('Pipeline crashed: %s (exit=%d)' % (name, exit_code))
                    continue

                # 2. Frame-flow check: no camera_info message within alive_threshold → stalled.
                threshold = self.alive_thresholds.get(name, DEFAULT_ALIVE_THRESHOLD)
                last = self.last_frame_time.get(name)
                if last is None:
                    continue  # not yet set — grace period is initialized at enable time
                elapsed = (now - last).nanoseconds / 1e9
                currently_alive = elapsed <= threshold
                if currently_alive != self.alive_state.get(name, False):
                    self._publish_alive(name, currently_alive)
                    if currently_alive:
                        self.get_logger().info('Pipeline %s: frames resumed' % name)
                    else:
                        self.get_logger().warn(
                            'Pipeline %s: stalled — %.2fs since last frame (threshold %.2fs)' % (
                                name, elapsed, threshold))

    ################################################################################################
    def _build_command(self, name):
        info      = self.pipelines[name]
        pipeline  = info['gst_pipeline'].replace('\n', ' ')
        calib     = info.get('calibration', name)
        topic     = info.get('topic', name)
        frame_id  = info.get('frame_id', name + '_frame')
        encoding  = info.get('encoding', '')
        compress  = 'true' if info.get('compress', True) else 'false'
        reliable  = 'true' if self.reliable_flags.get(name, False) else 'false'
        calib_url = 'file://' + str(self.calib_root / (calib + '.yaml'))

        # Escape inner double-quotes so the shell doesn't split the pipeline string
        # when it contains features="..." (e.g. aravissrc features="PixelFormat=Mono8 ...")
        pipeline_escaped = pipeline.replace('"', '\\"')

        cmd = (
            'ros2 run gst_cam_node gst_cam_node --ros-args'
            ' -p gst_pipeline:="%s"'
            ' -p camera_topic:="%s"'
            ' -p frame_id:="%s"'
            ' -p camera_info_path:="%s"'
        ) % (pipeline_escaped, topic, frame_id, calib_url)
        # Only emit `encoding` when set — empty quoted string collapses through
        # the shell and ROS 2's arg parser rejects bare `-p encoding:=`.
        if encoding:
            cmd += ' -p encoding:="%s"' % encoding
        cmd += ' -p compress:=%s -p reliable:=%s' % (compress, reliable)
        return cmd

    ################################################################################################
    def _open_log(self, name):
        log_dir = self.log_root / name
        log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_path  = log_dir / ('%s_%s.log' % (name, timestamp))
        existing  = sorted(log_dir.glob('%s_*.log' % name))
        while len(existing) >= MAX_LOG_FILES:
            existing.pop(0).unlink(missing_ok=True)
        f = open(log_path, 'w')
        self.log_files[name] = f
        self.get_logger().info('Logging %s -> %s' % (name, log_path))
        return f

    ################################################################################################
    def _close_log(self, name):
        f = self.log_files.get(name)
        if f is not None:
            try:
                f.close()
            except Exception:
                pass
            self.log_files[name] = None

    ################################################################################################
    def _is_running(self, name):
        proc = self.processes.get(name)
        return proc is not None and proc.poll() is None

    ################################################################################################
    def _handle_status_srv(self, request, response, name):
        with self.process_lock:
            if self._is_running(name):
                response.success = True
                response.message = '%s RUNNING (pid=%d)' % (name, self.processes[name].pid)
            else:
                response.success = False
                response.message = '%s STOPPED' % name
        return response

    ################################################################################################
    def _handle_status_all_srv(self, request, response):
        lines = []
        with self.process_lock:
            for name in self.pipelines:
                if self._is_running(name):
                    lines.append('  [RUNNING] %s  (pid=%d)' % (name, self.processes[name].pid))
                else:
                    lines.append('  [STOPPED] %s' % name)
        response.success = True
        response.message = '\n' + '\n'.join(lines)
        return response

    ################################################################################################
    def _handle_stop_all_srv(self, request, response):
        stopped = []
        for name in list(self.pipelines.keys()):
            if self._is_running(name):
                self._disable_pipeline(name)
                stopped.append(name)
        response.success = True
        response.message = 'stopped: %s' % (', '.join(stopped) if stopped else 'nothing running')
        return response

    ################################################################################################
    def _handle_pipeline_srv(self, request, response, name):
        if request.data:
            response.success, response.message = self._enable_pipeline(name)
        else:
            response.success, response.message = self._disable_pipeline(name)
        return response

    ################################################################################################
    def _enable_pipeline(self, name):
        with self.process_lock:
            proc = self.processes[name]
            if proc is not None and proc.poll() is None:
                msg = '%s already running (pid=%d)' % (name, proc.pid)
                self.get_logger().info(msg)
                self._publish_alive(name, True)
                return True, msg

            command  = self._build_command(name)
            log_file = self._open_log(name)
            try:
                proc = subprocess.Popen(
                    command,
                    shell=True,
                    executable='/bin/bash',
                    stdout=log_file,
                    stderr=log_file,
                    env=self.launch_env,
                    preexec_fn=os.setsid
                )
                self.processes[name] = proc
                # Seed the frame timestamp so the first alive_threshold seconds after launch
                # act as a grace period (watchdog won't mark as stalled during startup).
                self.last_frame_time[name] = self.get_clock().now()
                self._publish_alive(name, True)
                msg = '%s started (pid=%d)' % (name, proc.pid)
                self.get_logger().info('Started %s' % name)
                return True, msg
            except Exception as e:
                self._close_log(name)
                self.get_logger().error('Failed to start %s: %s' % (name, str(e)))
                return False, str(e)

    ################################################################################################
    def _disable_pipeline(self, name):
        with self.process_lock:
            proc = self.processes[name]
            if proc is None or proc.poll() is not None:
                self.processes[name] = None
                self.last_frame_time[name] = None
                self._close_log(name)
                self._publish_alive(name, False)
                return True, '%s already stopped' % name
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=5.0)
                self.processes[name] = None
                self.last_frame_time[name] = None
                self._close_log(name)
                self._publish_alive(name, False)
                return True, '%s stopped' % name
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass
                self.processes[name] = None
                self.last_frame_time[name] = None
                self._close_log(name)
                self._publish_alive(name, False)
                return True, '%s force-killed' % name
            except Exception as e:
                self.get_logger().error('Failed to stop %s: %s' % (name, str(e)))
                return False, str(e)

    ################################################################################################
    def shutdown_all(self):
        self.get_logger().info('GstCameraManager shutting down — stopping all pipelines')
        for name in list(self.pipelines.keys()):
            self._disable_pipeline(name)


####################################################################################################
# MAIN #############################################################################################
def main(args=None):
    rclpy.init(args=args)
    node = GstCameraManager()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    def _sigterm_handler(signum, frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, _sigterm_handler)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
