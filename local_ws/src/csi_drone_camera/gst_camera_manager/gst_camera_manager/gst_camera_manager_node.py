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
from rclpy.qos import (QoSProfile,
                       QoSReliabilityPolicy,
                       QoSHistoryPolicy,
                       QoSDurabilityPolicy)


MAX_LOG_FILES = 20

ALIVE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)


####################################################################################################
# GST CAMERA MANAGER ###############################################################################
class GstCameraManager(Node):
    def __init__(self):
        super().__init__('gst_camera_manager')

        self.pipelines    = {}   # name -> dict (gst_pipeline, calibration, topic, frame_id, encoding)
        self.processes    = {}   # name -> Popen or None
        self.log_files    = {}   # name -> open file handle or None
        self.alive_pubs   = {}   # name -> Publisher<Bool>
        self.srv_handles  = {}   # name -> service handles (keep alive)
        self.process_lock = Lock()

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
                self.get_logger().info('Registered pipeline: %s  (topic: /%s/image_raw)' % (
                    name, info.get('topic', name)))
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
        except Exception:
            pass

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
        self.watchdog_timer = self.create_timer(3.0, self._watchdog_tick, callback_group=cbg)

    ################################################################################################
    def _watchdog_tick(self):
        with self.process_lock:
            for name, proc in self.processes.items():
                if proc is not None and proc.poll() is not None:
                    exit_code = proc.returncode
                    self.processes[name] = None
                    self._close_log(name)
                    self._publish_alive(name, False)
                    self.get_logger().warn('Pipeline crashed: %s (exit=%d)' % (name, exit_code))

    ################################################################################################
    def _build_command(self, name):
        info      = self.pipelines[name]
        pipeline  = info['gst_pipeline'].replace('\n', ' ')
        calib     = info.get('calibration', name)
        topic     = info.get('topic', name)
        frame_id  = info.get('frame_id', name + '_frame')
        encoding  = info.get('encoding', 'bgr8')
        compress  = 'true' if info.get('compress', True) else 'false'
        calib_url = 'file://' + str(self.calib_root / (calib + '.yaml'))

        # Escape inner double-quotes so the shell doesn't split the pipeline string
        # when it contains features="..." (e.g. aravissrc features="PixelFormat=Mono8 ...")
        pipeline_escaped = pipeline.replace('"', '\\"')

        return (
            'ros2 run gst_cam_node gst_cam_node --ros-args'
            ' -p gst_pipeline:="%s"'
            ' -p camera_topic:="%s"'
            ' -p frame_id:="%s"'
            ' -p camera_info_path:="%s"'
            ' -p encoding:="%s"'
            ' -p compress:=%s'
        ) % (pipeline_escaped, topic, frame_id, calib_url, encoding, compress)

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
                self._close_log(name)
                self._publish_alive(name, False)
                return True, '%s already stopped' % name
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=5.0)
                self.processes[name] = None
                self._close_log(name)
                self._publish_alive(name, False)
                return True, '%s stopped' % name
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass
                self.processes[name] = None
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
