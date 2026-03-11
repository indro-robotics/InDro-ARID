#!/usr/bin/env python

import os
import time
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
from std_msgs.msg import Bool, Empty
from std_srvs.srv import SetBool, Trigger
from rclpy.qos import (QoSProfile,
                       QoSReliabilityPolicy,
                       QoSHistoryPolicy,
                       QoSDurabilityPolicy)


MAX_LOG_FILES        = 20
WATCHDOG_INTERVAL    = 3.0   # seconds — must match create_timer call below
MONITOR_RESTART_SECS = 10.0  # restart a monitored pipeline if Hz not reached within this time

# Latched QoS — any subscriber gets the last value immediately on connect
ALIVE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)

# Best-effort for monitor topic subscriptions (camera_info) — no need to queue
MONITOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)


####################################################################################################
# NODE MANAGER #####################################################################################
class NodeManager(Node):
    def __init__(self):
        super().__init__('node_manager')

        self.pipelines    = {}      # name -> command string
        self.processes    = {}      # name -> Popen or None
        self.log_files    = {}      # name -> open file handle or None
        self.alive_pubs   = {}      # name -> Publisher<Bool>
        self.alive_state  = {}      # name -> bool (last published, for change detection)
        self.process_lock = Lock()
        self.srv_handles  = {}      # name -> service handle tuple (keep alive)

        # Per-pipeline optional topic monitoring
        self.monitor_topics    = {}    # name -> topic string or None
        self.min_hz            = {}    # name -> float or None
        self.restart_timeout   = {}    # name -> float (seconds before auto-restart if Hz not reached)
        self.msg_counts        = {}    # name -> int (reset each watchdog tick)
        self.monitor_subs      = {}    # name -> Subscription or None
        self.start_times       = {}    # name -> float (time.time() when last spawned)

        # Freeze the environment at startup — every child process gets exactly this
        self.launch_env = os.environ.copy()

        # Log root lives inside the installed package share directory
        self.log_root = Path(get_package_share_directory('node_manager')) / 'logs'

        config_path = Path(get_package_share_directory('node_manager')) / 'config' / 'pipelines.yaml'
        self._load_config(config_path)
        self._create_publishers()
        self._create_services()
        self._create_watchdog()

        self.get_logger().info('NodeManager ready — %d pipeline(s) registered' % len(self.pipelines))

    ################################################################################################
    def _load_config(self, config_path):
        try:
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)

            pipelines = data.get('pipelines', {})
            for name, info in pipelines.items():
                self.pipelines[name]      = info['command']
                self.processes[name]      = None
                self.log_files[name]      = None
                self.alive_state[name]    = False
                self.monitor_topics[name]  = info.get('monitor_topic', None)
                self.min_hz[name]          = info.get('min_hz', None)
                self.restart_timeout[name] = info.get('restart_timeout', MONITOR_RESTART_SECS)
                self.msg_counts[name]      = 0
                self.monitor_subs[name]    = None
                self.start_times[name]     = 0.0

                if self.monitor_topics[name]:
                    self.get_logger().info(
                        'Registered pipeline: %s  (monitor: %s @ %.1f Hz)' % (
                            name, self.monitor_topics[name], self.min_hz[name]))
                else:
                    self.get_logger().info('Registered pipeline: %s' % name)

        except Exception as e:
            self.get_logger().error('Failed to load config: %s' % str(e))

    ################################################################################################
    def _create_publishers(self):
        # One latched Bool topic per pipeline — any node can subscribe and immediately
        # get the current alive state without making a service call
        for name in self.pipelines:
            pub = self.create_publisher(Bool, 'node_manager/%s/alive' % name, ALIVE_QOS)
            self.alive_pubs[name] = pub

            # Publish initial state (stopped) so late subscribers get something right away
            msg = Bool()
            msg.data = False
            pub.publish(msg)

    ################################################################################################
    def _publish_alive(self, name, alive):
        # Only publish on state change — reduces chatter on latched topic
        if self.alive_state.get(name) == alive:
            return
        try:
            self.alive_state[name] = alive
            msg = Bool()
            msg.data = alive
            self.alive_pubs[name].publish(msg)
        except Exception:
            pass

    ################################################################################################
    def _create_monitor_sub(self, name):
        """Subscribe to the pipeline's monitor_topic to count incoming messages.
        Called after the process spawns. The watchdog tick measures Hz over each 3s window
        and promotes alive=True once the threshold is met."""
        topic = self.monitor_topics.get(name)
        if not topic:
            return
        cbg = MutuallyExclusiveCallbackGroup()
        sub = self.create_subscription(
            Empty,
            topic,
            lambda msg, n=name: self._monitor_cb(n),
            MONITOR_QOS,
            callback_group=cbg
        )
        self.monitor_subs[name] = sub
        self.msg_counts[name]   = 0
        self.get_logger().info('Monitoring %s on %s (min %.1f Hz)' % (
            name, topic, self.min_hz[name]))

    ################################################################################################
    def _monitor_cb(self, name):
        self.msg_counts[name] += 1

    ################################################################################################
    def _destroy_monitor_sub(self, name):
        sub = self.monitor_subs.get(name)
        if sub is not None:
            try:
                self.destroy_subscription(sub)
            except Exception:
                pass
            self.monitor_subs[name] = None
            self.msg_counts[name]   = 0

    ################################################################################################
    def _create_services(self):
        for name in self.pipelines:
            cbg_ctrl   = MutuallyExclusiveCallbackGroup()
            cbg_status = MutuallyExclusiveCallbackGroup()

            ctrl_srv = self.create_service(
                SetBool,
                'node_manager/%s' % name,
                lambda req, res, n=name: self._handle_pipeline_srv(req, res, n),
                callback_group=cbg_ctrl
            )
            status_srv = self.create_service(
                Trigger,
                'node_manager/%s/status' % name,
                lambda req, res, n=name: self._handle_status_srv(req, res, n),
                callback_group=cbg_status
            )
            self.srv_handles[name] = (ctrl_srv, status_srv)
            self.get_logger().info('Service ready: /node_manager/%s  |  /node_manager/%s/status' % (name, name))

        # Single service to query all pipeline states at once
        cbg_all = MutuallyExclusiveCallbackGroup()
        self.status_all_srv = self.create_service(
            Trigger,
            'node_manager/status_all',
            self._handle_status_all_srv,
            callback_group=cbg_all
        )
        self.get_logger().info('Service ready: /node_manager/status_all')

        # Single service to stop all pipelines at once
        cbg_stop_all = MutuallyExclusiveCallbackGroup()
        self.stop_all_srv = self.create_service(
            Trigger,
            'node_manager/stop_all',
            self._handle_stop_all_srv,
            callback_group=cbg_stop_all
        )
        self.get_logger().info('Service ready: /node_manager/stop_all')

    ################################################################################################
    def _create_watchdog(self):
        cbg_watchdog = MutuallyExclusiveCallbackGroup()
        self.watchdog_timer = self.create_timer(
            WATCHDOG_INTERVAL,
            self._watchdog_tick,
            callback_group=cbg_watchdog
        )

    ################################################################################################
    def _watchdog_tick(self):
        restart_needed = []

        with self.process_lock:
            for name, proc in self.processes.items():

                # Crash detection — process died on its own
                if proc is not None and proc.poll() is not None:
                    exit_code = proc.returncode
                    self.processes[name] = None
                    self._destroy_monitor_sub(name)
                    self._close_log(name)
                    self._publish_alive(name, False)
                    self.get_logger().warn('Pipeline crashed: %s (exit code %d)' % (name, exit_code))
                    continue

                # Hz monitoring for running monitored pipelines
                if self.monitor_topics.get(name) and proc is not None:
                    hz        = self.msg_counts[name] / WATCHDOG_INTERVAL
                    self.msg_counts[name] = 0   # reset window for next tick
                    threshold = self.min_hz[name]

                    if hz >= threshold:
                        self._publish_alive(name, True)
                    else:
                        self._publish_alive(name, False)
                        elapsed = time.time() - self.start_times.get(name, 0.0)
                        self.get_logger().info(
                            '%s: %.1f Hz < %.1f Hz threshold — not ready (%.1fs since start)' % (
                                name, hz, threshold, elapsed))
                        if elapsed > self.restart_timeout[name]:
                            restart_needed.append(name)

        # Restart outside the lock — _disable/_enable each acquire process_lock internally
        for name in restart_needed:
            self.get_logger().warn(
                'Auto-restarting %s — failed to reach %.1f Hz within %.0fs' % (
                    name, self.min_hz[name], self.restart_timeout[name]))
            self._disable_pipeline(name)
            self._enable_pipeline(name)

    ################################################################################################
    def _open_log(self, name):
        log_dir = self.log_root / name
        log_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_path  = log_dir / ('%s_%s.log' % (name, timestamp))

        # Prune oldest logs beyond MAX_LOG_FILES
        existing = sorted(log_dir.glob('%s_*.log' % name))
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
                if self.monitor_topics.get(name):
                    # If already alive, republish True immediately (caller may have reset
                    # pipeline_active=False externally; bypass change-detection by clearing
                    # alive_state first).  If not yet alive, leave it to the watchdog.
                    if self.alive_state.get(name):
                        self.alive_state[name] = None
                        self._publish_alive(name, True)
                else:
                    self._publish_alive(name, True)
                return True, msg

            command  = self.pipelines[name]
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
                self.processes[name]   = proc
                self.start_times[name] = time.time()

                if self.monitor_topics.get(name):
                    # alive stays False — watchdog will set True once Hz threshold is met
                    self._create_monitor_sub(name)
                else:
                    self._publish_alive(name, True)

                msg = '%s started (pid=%d)' % (name, proc.pid)
                self.get_logger().info('Started %s: %s' % (name, command))
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
                self._destroy_monitor_sub(name)
                self._close_log(name)
                self._publish_alive(name, False)
                msg = '%s already stopped' % name
                self.get_logger().info(msg)
                return True, msg

            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=5.0)
                self.processes[name] = None
                self._destroy_monitor_sub(name)
                self._close_log(name)
                self._publish_alive(name, False)
                msg = '%s stopped' % name
                self.get_logger().info(msg)
                return True, msg

            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass
                self.processes[name] = None
                self._destroy_monitor_sub(name)
                self._close_log(name)
                self._publish_alive(name, False)
                msg = '%s force-killed (SIGTERM timeout)' % name
                self.get_logger().warn(msg)
                return True, msg

            except Exception as e:
                self.get_logger().error('Failed to stop %s: %s' % (name, str(e)))
                return False, str(e)

    ################################################################################################
    def shutdown_all(self):
        self.get_logger().info('NodeManager shutting down — stopping all pipelines')
        for name in list(self.pipelines.keys()):
            self._disable_pipeline(name)


####################################################################################################
# MAIN #############################################################################################
def main(args=None):
    rclpy.init(args=args)
    node = NodeManager()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # ros2 launch sends SIGTERM (not SIGINT) to child nodes on Ctrl+C.
    # Python's default SIGTERM kills the process immediately, bypassing the finally block.
    # Re-raise as KeyboardInterrupt so shutdown_all() always runs.
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
