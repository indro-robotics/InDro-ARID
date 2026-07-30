"""arid_supervisor - always-on lifecycle manager for the ARID VSLAM stack.

Services (under /arid_supervisor):
  ~/vslam_enable (std_srvs/SetBool)
    true : camera-proven bringup - USB pre-check (orphaned /dev/bus/usb nodes pruned, camera
           nodes repaired to root:plugdev 0666), log-watch gate (the front camera's
           "RealSense Node Is Up!" tag, fail-fast on the image_transport plugin-load race,
           40 s backstop bounds everything else), ONE reset_usb recovery cycle. Blocks ~15 s
           healthy, ~3 min worst.
    false: landed-gated teardown - SIGINT the process group, wait for drain, escalate
           to SIGTERM/SIGKILL only on stall.
  ~/status (std_srvs/Trigger): success = vslam running.

Idempotent both ways; double-spawn impossible (single-threaded executor queues calls,
self._lock preserves this under any executor).

Unowned vslam trees - a direct `ros2 launch px4_vslam vslam.launch.py`, or an orphan a
crashed supervisor left holding the camera - are reaped on BOTH paths, but only with a
fresh landed proof; airborne or unknown refuses instead. Same rule on shutdown: a
supervisor stop while provably airborne leaves the stack flying.

Subscribes /vslam_sentry/healthy: on the unhealthy edge it runs a bounded camera-node repair
burst (repair only, never prune) so a camera re-enumerated mid-flight onto a stale devnum
becomes reopenable without waiting for the next bringup.
Subprocess log: /workspaces/isaac_ros-dev/run_logs/<name>/<name>.log, truncated per launch.
"""

import glob
import os
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

from px4_msgs.msg import VehicleLandDetected


LAND_FRESH_S = 3.5   # survives one dropped 1 Hz keepalive; 2.0 read a flying drone as unknown
# Per-stack clean-shutdown grace before any hard kill; SIGINT lets nodes release their DDS shm.
SIGINT_GRACE_S = {'vslam': 25.0}
DEFAULT_SIGINT_GRACE_S = 15.0
TERM_WAIT_S = 5.0   # SIGTERM grace after the SIGINT window, before SIGKILL
LOG_DIR = '/workspaces/isaac_ros-dev/run_logs'

RS_VID = '8086'                           # Intel RealSense USB vendor id
# D43X-family PIDs; pin the exact one via the rs_usb_pids ROS param once confirmed
# on hardware (cat /sys/bus/usb/devices/*/idProduct).
DEFAULT_RS_PIDS = ['0b07', '0b3a', '0b3d', '0b64', '0b5c']

VSLAM_CONFIG = '/workspaces/isaac_ros-dev/src/px4_vslam/config/vslam_config.yaml'


def _cam_count_from_config(path=VSLAM_CONFIG, fallback=1):
    """Physical cameras = *_realsense sections carrying a serial - the same rule
    vslam_sentry applies to the same file. NOT visual_slam num_cameras (2 = the one
    camera's two IR streams). Unprovisioned/unreadable config -> fallback."""
    try:
        import yaml as _yaml
        with open(path) as f:
            root = _yaml.safe_load(f)
        n = sum(1 for k, v in root.items()
                if '_realsense/' in str(k)
                and isinstance(v, dict)
                and v.get('ros__parameters', {}).get('serial_no'))
        return n if n > 0 else fallback
    except Exception:
        return fallback


CAM_COUNT = _cam_count_from_config()      # ONE physical front RealSense on this airframe
CAM_UP_MARKER = 'RealSense Node Is Up!'
# The up-marker only proves the driver found its device. A class_loader race kills the
# image_transport publishers while every marker still prints - gate passes, VO at 0 Hz.
CAM_PLUGIN_ERR = 'no factory exists'
# Evidence only, never a verdict: the fork retries this case every reconnect_timeout.
CAM_ERR_MARKER = 'Error starting device'
CAM_GATE_BACKSTOP_S = 40.0   # healthy bringup completes in 14-26 s
CAM_GATE_POLL_S = 0.25
USB_REENUM_WAIT_S = 20.0     # /reset_usb: ~5 s power cycle + ~10 s re-enumeration
# Mid-flight camera-node repair: a sentry hw_reset can re-enumerate a camera onto a devnum
# whose /dev node was left root:root by a dropped hotplug event. On the /vslam_sentry/healthy
# False edge, repair (never prune) across the recovery window so the driver's open-retry
# succeeds; the burst self-terminates on recovery or at the deadline.
REPAIR_BURST_PERIOD_S = 5.0
REPAIR_BURST_MAX_S = 70.0    # just past vslam_sentry reset_verify_s (60 s)
RESET_USB_TIMEOUT_S = 30.0   # subprocess `ros2 service call /reset_usb` hard cap
RESP_MSG_MAX = 500           # SetBool response clip; full evidence always in the node log
LEGACY_SCAN_TIMEOUT_S = 20.0  # `ros2 node list --no-daemon` fresh-discovery hard cap


def _proc_descendants(root_pid):
    # Walks descendants, not the process group: setsid children leave the group but not the tree.
    kids = {}
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/stat') as f:
                ppid = int(f.read().rsplit(')', 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            continue
        kids.setdefault(ppid, []).append(int(pid))
    out, todo = [], [int(root_pid)]
    while todo:
        p = todo.pop()
        out.append(p)
        todo.extend(kids.get(p, []))
    return out


def _pgid_members(pgid):
    # Live members of this group (pgrp field of /proc/<pid>/stat). Confirms real membership
    # before any killpg on a REMEMBERED pgid: a drained pgid handed on by pid reuse must
    # never be signalled.
    members = []
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/stat') as f:
                pgrp = int(f.read().rsplit(')', 1)[1].split()[2])
        except (OSError, ValueError, IndexError):
            continue
        if pgrp == pgid:
            members.append(int(pid))
    return members


def _mapped_shm(pids):
    # Excludes domain-global fastrtps_port segments: co-mapped by every DDS participant, never reclaim.
    segs = set()
    for pid in pids:
        try:
            with open(f'/proc/{pid}/maps') as f:
                for line in f:
                    i = line.find('/dev/shm/')
                    if i == -1:
                        continue
                    path = line[i:].split()[0]
                    base = os.path.basename(path)
                    for pre in ('fastrtps_', 'sem.fastrtps_'):
                        if base.startswith(pre):
                            if not base[len(pre):].startswith('port'):
                                segs.add(path)
                            break
        except OSError:
            continue
    return segs


def _group_alive(pgid):
    # True if any process is still in this process group.
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def _reap_groups(groups, grace):
    # Reaps orphaned setsid groups that remain after a mid-teardown parent death.
    alive = [g for g in groups if _group_alive(g)]
    if not alive:
        return []
    for g in alive:
        try:
            os.killpg(g, signal.SIGINT)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + grace
    while time.monotonic() < deadline and any(_group_alive(g) for g in alive):
        time.sleep(0.3)
    for g in alive:
        if _group_alive(g):
            try:
                os.killpg(g, signal.SIGKILL)
            except ProcessLookupError:
                pass
    return alive


def sweep_stack_shm(owned, logger=None):
    """Reclaim only the torn-down stack's own GUID segments that no live process still maps."""
    if not owned:
        return 0
    held = _mapped_shm(int(p) for p in os.listdir('/proc') if p.isdigit())
    removed = []
    for seg in owned:
        if seg in held or not os.path.exists(seg):
            continue
        base = os.path.basename(seg)
        # GUID names are unique, so these globs cannot match another participant's live segment.
        for path in glob.glob(f'/dev/shm/{base}*') + glob.glob(f'/dev/shm/sem.{base}*'):
            try:
                os.remove(path)
                removed.append(os.path.basename(path))
            except OSError:
                pass
    if removed and logger is not None:
        logger.warn('reclaimed %d orphaned shm segment(s): %s' % (len(removed), ', '.join(removed)))
    return len(removed)


def _usb_rs_devices(pids):
    # sysfs serial is the USB descriptor serial, NOT the librealsense camera serial:
    # evidence only, never compare across the two.
    devs = []
    for d in sorted(glob.glob('/sys/bus/usb/devices/*/')):
        try:
            with open(os.path.join(d, 'idVendor')) as f:
                vid = f.read().strip()
            with open(os.path.join(d, 'idProduct')) as f:
                pid = f.read().strip()
        except OSError:
            continue
        if vid != RS_VID or pid not in pids:
            continue
        try:
            with open(os.path.join(d, 'serial')) as f:
                serial = f.read().strip()
        except OSError:
            serial = '?'
        devs.append(f'{os.path.basename(d.rstrip("/"))} {vid}:{pid} serial={serial}')
    return devs


def _prune_orphan_nodes(logger=None):
    # Bringup-only, quiescent bus: the container's private /dev accumulates nodes because its
    # udevd drops REMOVE events under the uevent storm a USB reset generates. A node whose
    # (bus, devnum) has no backing device in sysfs is unreachable and, left in place, poisons
    # that devnum for the next device that enumerates onto it. Never called in flight.
    live = set()
    for d in sorted(glob.glob('/sys/bus/usb/devices/*/')):
        try:
            with open(os.path.join(d, 'busnum')) as f:
                bus = '%03d' % int(f.read().strip())
            with open(os.path.join(d, 'devnum')) as f:
                dev = '%03d' % int(f.read().strip())
        except (OSError, ValueError):
            continue
        live.add((bus, dev))
    if not live:
        return 0  # no sysfs evidence: every node would look orphaned - never prune blind
    orphans = [p for p in sorted(glob.glob('/dev/bus/usb/*/*'))
               if tuple(p.split('/')[-2:]) not in live]
    if not orphans:
        return 0
    try:
        subprocess.run(['sudo', 'rm', '-f'] + orphans, timeout=15.0)
    except (OSError, subprocess.SubprocessError) as exc:
        if logger is not None:
            logger.warn('orphan node prune failed (%s: %s)' % (type(exc).__name__, exc))
        return 0
    if logger is not None:
        logger.warn('pruned %d orphaned /dev/bus/usb node(s): %s'
                    % (len(orphans), ', '.join(orphans)))
    return len(orphans)


def _repair_camera_nodes(pids, logger=None):
    # The container's /dev is a private tmpfs. A hotplug ADD that lands on a devnum still
    # holding a node from a prior device cannot mknod over it, so the camera inherits that
    # node's root:root ownership and libusb_open fails EACCES (RS2_USB_STATUS_ACCESS) for the
    # non-root user. Restore root:plugdev 0666 on any RealSense node this user cannot open.
    # Bringup-only: the cameras are enumerated and not yet held here, so this races nothing.
    repaired = []
    for d in sorted(glob.glob('/sys/bus/usb/devices/*/')):
        try:
            with open(os.path.join(d, 'idVendor')) as f:
                if f.read().strip() != RS_VID:
                    continue
            with open(os.path.join(d, 'idProduct')) as f:
                if f.read().strip() not in pids:
                    continue
            with open(os.path.join(d, 'busnum')) as f:
                bus = '%03d' % int(f.read().strip())
            with open(os.path.join(d, 'devnum')) as f:
                dev = '%03d' % int(f.read().strip())
        except (OSError, ValueError):
            continue
        node = '/dev/bus/usb/%s/%s' % (bus, dev)
        if os.path.exists(node) and not os.access(node, os.R_OK | os.W_OK):
            repaired.append(node)

    if not repaired:
        return 0
    try:
        subprocess.run(['sudo', 'chown', 'root:plugdev'] + repaired, timeout=15.0)
        subprocess.run(['sudo', 'chmod', '0666'] + repaired, timeout=15.0)
    except (OSError, subprocess.SubprocessError) as exc:
        if logger is not None:
            logger.warn('camera node repair failed (%s: %s)' % (type(exc).__name__, exc))
        return 0

    if logger is not None:
        logger.warn('repaired %d camera node(s) to root:plugdev 0666: %s'
                    % (len(repaired), ', '.join(repaired)))
    return len(repaired)


def _distinct_cam_ups(buf):
    # Counts unique per-camera node tags (last [tag] before the marker), not raw marker
    # occurrences: one camera re-emitting after a reconnect must never satisfy CAM_COUNT.
    tags = set()
    for line in buf.splitlines():
        if CAM_UP_MARKER not in line:
            continue
        pre = line.split(CAM_UP_MARKER, 1)[0]
        i, j = pre.rfind('['), pre.rfind(']')
        tags.add(pre[i + 1:j] if 0 <= i < j else pre.strip())
    return len(tags)


def _squash(text):
    # Multi-line evidence -> single ' | '-joined line for a SetBool response message.
    return ' | '.join(line.strip() for line in text.splitlines() if line.strip())


def _clip(msg, limit=RESP_MSG_MAX):
    msg = _squash(msg)
    if len(msg) <= limit:
        return msg
    return msg[:limit] + ' ...[truncated; full detail in supervisor log]'


class _Stack:
    def __init__(self, name, launch_pkg, launch_file, logger):
        self.name = name
        self.launch_pkg = launch_pkg
        self.launch_file = launch_file
        self.logger = logger
        self.proc = None
        self._pgid = None        # remembered group - outlives a dead leader for teardown
        self.started_at = None

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self):
        stack_dir = os.path.join(LOG_DIR, self.name)
        os.makedirs(stack_dir, exist_ok=True)
        log_path = os.path.join(stack_dir, f'{self.name}.log')
        log = open(log_path, 'wb', buffering=0)
        self.proc = subprocess.Popen(
            ['ros2', 'launch', self.launch_pkg, self.launch_file],
            stdout=log, stderr=subprocess.STDOUT, preexec_fn=os.setsid,
        )
        self._pgid = self.proc.pid   # setsid makes the leader its own group leader
        self.started_at = time.monotonic()
        return log_path

    def stop(self):
        if not self.alive():
            # `ros2 launch` can die (OOM/segfault) while its component_container children
            # SURVIVE in the same group, still holding the RealSense and its DDS shm.
            # Abandoning them makes every next bringup collide - reap the remembered group.
            pgid = self._pgid
            self.proc = None
            self._pgid = None
            if pgid is None:
                return
            members = _pgid_members(pgid)
            if not members:
                return
            self.logger.warn('%s: leader dead - reaping %d surviving pgid member(s)'
                             % (self.name, len(members)))
            descendants = []
            for m in members:
                descendants.extend(_proc_descendants(m))
            owned = _mapped_shm(descendants)
            groups = {pgid}
            for pid in descendants:
                try:
                    groups.add(os.getpgid(pid))
                except ProcessLookupError:
                    pass
            grace = SIGINT_GRACE_S.get(self.name, DEFAULT_SIGINT_GRACE_S)
            if not self._dead_leader_signal_and_wait(pgid, signal.SIGINT, grace):
                if not self._dead_leader_signal_and_wait(pgid, signal.SIGTERM, TERM_WAIT_S):
                    # Membership re-confirmed immediately before the hard kill.
                    if _pgid_members(pgid):
                        try:
                            os.killpg(pgid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
            reaped = _reap_groups(groups, TERM_WAIT_S)
            if reaped:
                self.logger.warn('%s: reaped %d straggler process group(s)' % (self.name, len(reaped)))
            sweep_stack_shm(owned, self.logger)
            return
        try:
            pgid = os.getpgid(self.proc.pid)
        except ProcessLookupError:
            # Leader vanished between alive() and here - retry via the dead-leader path so
            # any surviving group member is still reaped.
            self.proc = None
            return self.stop()
        # Snapshot owned shm + every group (incl. setsid pipelines) before the kill.
        descendants = _proc_descendants(self.proc.pid)
        owned = _mapped_shm(descendants)
        groups = set()
        for pid in descendants:
            try:
                groups.add(os.getpgid(pid))
            except ProcessLookupError:
                pass
        grace = SIGINT_GRACE_S.get(self.name, DEFAULT_SIGINT_GRACE_S)
        # SIGINT first: nodes shut down cleanly and release their DDS shm; escalate only on stall.
        if not self._signal_and_wait(pgid, signal.SIGINT, grace):
            if not self._signal_and_wait(pgid, signal.SIGTERM, TERM_WAIT_S):
                try:
                    os.killpg(pgid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                try:
                    self.proc.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    pass
        self.proc = None
        self._pgid = None
        reaped = _reap_groups(groups, TERM_WAIT_S)
        if reaped:
            self.logger.warn('%s: reaped %d straggler process group(s)' % (self.name, len(reaped)))
        sweep_stack_shm(owned, self.logger)

    def _signal_and_wait(self, pgid, sig, timeout):
        # Wait for the WHOLE group, not just ros2 launch: returning early interrupts the
        # RealSense destructor still releasing the camera USB (dirty camera on next init).
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return True
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.proc.poll()   # reap ros2 launch so a dead leader doesn't hold the group open
            if not _group_alive(pgid):
                return True
            time.sleep(0.2)
        self.proc.poll()
        return not _group_alive(pgid)

    def _dead_leader_signal_and_wait(self, pgid, sig, timeout):
        # No self.proc to poll, and membership is re-confirmed via /proc immediately before
        # the killpg: a remembered pgid whose group has drained may have been recycled.
        if not _pgid_members(pgid):
            return True
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return True
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not _pgid_members(pgid):
                return True
            time.sleep(0.2)
        return not _pgid_members(pgid)


class AridSupervisor(Node):
    def __init__(self):
        super().__init__('arid_supervisor')

        self.vslam = _Stack('vslam', 'px4_vslam', 'vslam.launch.py', self.get_logger())

        # ROS param so the exact PID can be pinned on-hardware without a code change.
        self.rs_usb_pids = list(self.declare_parameter('rs_usb_pids', DEFAULT_RS_PIDS).value)
        self.get_logger().info(
            'front RealSense USB match: VID %s PID one of %s'
            % (RS_VID, ', '.join(self.rs_usb_pids)))

        self._lock = threading.Lock()
        self._landed = None
        self._landed_at = 0.0

        # Event-driven mid-flight camera-node repair (see REPAIR_BURST_* above).
        self._repair_on_unhealthy = bool(
            self.declare_parameter('repair_on_unhealthy', True).value)
        self._repair_timer = None
        self._repair_deadline = 0.0
        self._repair_active = False

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
        self.create_service(Trigger, '~/status', self._status_cb)

        if self._repair_on_unhealthy:
            # Match vslam_sentry's latched publisher (reliable + transient_local).
            sentry_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.create_subscription(
                Bool, '/vslam_sentry/healthy', self._sentry_health_cb, sentry_qos)

        self.get_logger().info('arid_supervisor up - vslam_enable + status available')

    def _sentry_health_cb(self, msg):
        # False = a camera is failing to recover; a sentry hw_reset may have re-enumerated it
        # onto a devnum whose /dev node is root:root. Repair (never prune) across the recovery
        # window. True cancels the burst. An escape here is re-raised out of spin().
        try:
            if msg.data:
                self._end_repair_burst()
            elif not self._repair_active:
                self._begin_repair_burst()
        except Exception as exc:                                   # noqa: BLE001
            self.get_logger().warn(
                'sentry-health handler failed (%s: %s)' % (type(exc).__name__, exc))

    def _begin_repair_burst(self):
        self._end_repair_burst()   # destroy any prior timer (safe: not from within a tick)
        self._repair_deadline = time.monotonic() + REPAIR_BURST_MAX_S
        self._repair_active = True
        self._repair_timer = self.create_timer(REPAIR_BURST_PERIOD_S, self._repair_tick)
        self.get_logger().warn(
            'sentry unhealthy - camera-node repair burst started (<= %.0fs)' % REPAIR_BURST_MAX_S)

    def _end_repair_burst(self):
        self._repair_active = False
        if self._repair_timer is not None:
            self._repair_timer.cancel()
            try:
                self.destroy_timer(self._repair_timer)
            except Exception:                                     # noqa: BLE001
                pass
            self._repair_timer = None

    def _repair_tick(self):
        # Repair ONLY - never prune in flight (delete can race a mid-enumeration device;
        # chmod/chown cannot disturb an already-open stream). Guarded: an escape would be
        # re-raised out of spin().
        try:
            _repair_camera_nodes(self.rs_usb_pids, self.get_logger())
        except Exception as exc:                                  # noqa: BLE001
            self.get_logger().warn(
                'camera-node repair tick failed (%s: %s)' % (type(exc).__name__, exc))
        if time.monotonic() >= self._repair_deadline:
            self._repair_active = False
            if self._repair_timer is not None:
                self._repair_timer.cancel()

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

    def _status_cb(self, req, resp):
        with self._lock:
            vslam = self.vslam.alive()
            landed = self._landed_fresh()
        resp.success = vslam
        land = 'landed' if landed else ('airborne' if landed is False else 'unknown')
        resp.message = f'vslam: {"running" if vslam else "stopped"} | land: {land}'
        return resp

    def _vslam_cb(self, req, resp):
        with self._lock:
            if req.data:
                if self.vslam.alive():
                    up_s = int(time.monotonic() - (self.vslam.started_at or time.monotonic()))
                    resp.success = True
                    resp.message = (f'vslam already running (up {up_s}s, '
                                    f'{CAM_COUNT}/{CAM_COUNT} camera at bringup)')
                    # Log the no-op: an unlogged idempotent return is invisible to post-run forensics.
                    self.get_logger().info('vslam_enable(true) idempotent no-op: ' + resp.message)
                    return resp
                return self._vslam_enable_gated(resp)

            # A dead leader whose children survive still holds the camera; only an empty
            # tree is 'already stopped' (self.vslam.stop() reaps the remembered group).
            survivors = bool(self.vslam._pgid and _pgid_members(self.vslam._pgid))
            if not self.vslam.alive() and not survivors:
                # An unowned tree (an airborne-preserved orphan, or a direct launch) must
                # not get a false 'already stopped' success: landed -> reap it here.
                legacy = self._legacy_vslam_nodes()
                stray = self._unowned_tree_pids()
                if legacy or stray:
                    if self._landed_fresh() is True:
                        self._reap_unowned_trees()
                        resp.success = True
                        resp.message = ('unowned vslam trees reaped ('
                                        + ', '.join(legacy or [str(p) for p in stray]) + ')')
                        self.get_logger().warn(resp.message)
                        return resp
                    resp.success = False
                    resp.message = _clip(
                        'unowned vslam trees present (' + ', '.join(legacy or [str(p) for p in stray])
                        + ') and land state not proven - land first')
                    self.get_logger().warn(resp.message)
                    return resp
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

    # ------------------------------------------------------------------
    # Camera-proven vslam bringup (runs under self._lock, from _vslam_cb).

    def _vslam_enable_gated(self, resp):
        # Blocks the single-threaded executor for the whole bringup; other callbacks queue behind it.
        legacy = self._legacy_vslam_nodes()
        stray = self._unowned_tree_pids()
        # A dead leader's children carry none of REAP_PATTERN, so only the remembered
        # pgid can find them; without this they survive the reap and hold the camera.
        survivors = bool(self.vslam._pgid and _pgid_members(self.vslam._pgid))
        if legacy or stray or survivors:
            # Kill-before-spinup: landed-proven -> reap the unowned trees and carry on into a
            # fresh gated bringup, so `initialize` over an orphan just works.
            if self._landed_fresh() is True:
                self.get_logger().warn(
                    'vslam trees with no live owner (graph: ' + ', '.join(legacy) +
                    '; pids: ' + str(stray) + f'; dead-leader survivors: {survivors})'
                    ' - landed, reaping before bringup')
                if survivors:
                    self.vslam.stop()
                self._reap_unowned_trees()
                time.sleep(10.0)  # DDS forget
                legacy = self._legacy_vslam_nodes()
            if legacy or self._unowned_tree_pids():
                msg = ('refusing vslam bringup: vslam nodes already on the ROS graph but NOT '
                       'managed by this supervisor (legacy direct '
                       "'ros2 launch px4_vslam vslam.launch.py'?): " + ', '.join(legacy)
                       + '. Landed-proven trees are reaped automatically; this refusal means '
                         'land state is unknown/airborne or the reap could not clear them.')
                self.get_logger().error(msg)
                resp.success = False
                resp.message = _clip(msg)
                return resp

        ok, msg = self._usb_precheck()
        if not ok:
            self.get_logger().error(msg)
            resp.success = False
            resp.message = _clip(msg)
            return resp

        # An unhandled raise here would leave a launched-but-unproven stack that the next
        # enable(true) falsely no-ops as 'already running 1/1'.
        try:
            log = self.vslam.start()
            self.get_logger().info(
                f'vslam launching (log: {log}); gating on {CAM_COUNT}x RealSense bringup')
            ok, elapsed, report1 = self._watch_vslam_log(log)
            if ok:
                resp.success = True
                resp.message = f'vslam up: {CAM_COUNT}/{CAM_COUNT} camera in {elapsed:.0f}s'
                self.get_logger().info(resp.message)
                return resp
            self.get_logger().error('vslam camera gate FAIL (attempt 1/2):\n' + report1)

            # ONE recovery, no ladder. No land gate: pre-mission bringup, camera already unusable.
            self.vslam.stop()
            # Proportional recovery: a bus power cycle costs ~20 s of re-enumeration and only
            # helps a camera that is ABSENT. Every camera still enumerated means the bringup
            # lost a driver-side claim or an image_transport plugin load; the driver retries
            # the claim itself and a relaunch clears both.
            devs = _usb_rs_devices(self.rs_usb_pids)
            bus_cycled = len(devs) < CAM_COUNT
            if not bus_cycled:
                self.get_logger().warn(
                    'recovery: %d/%d RealSense still enumerated - claim-side failure; '
                    'relaunching without /reset_usb (single attempt)' % (len(devs), CAM_COUNT))
            else:
                self.get_logger().warn(
                    'recovery: only %d/%d RealSense enumerated - /reset_usb + relaunch '
                    '(single attempt); devices: %s'
                    % (len(devs), CAM_COUNT, '; '.join(devs) or '(none)'))
                reset_ok, _ = self._reset_usb()
                if not reset_ok:
                    self.get_logger().warn('/reset_usb failed - relaunching on the un-cycled bus anyway')
                if not self._wait_usb_rs():
                    devs = _usb_rs_devices(self.rs_usb_pids)
                    self.get_logger().warn(
                        'only %d/%d RealSense on USB after /reset_usb - relaunching anyway; devices: %s'
                        % (len(devs), CAM_COUNT, '; '.join(devs) or '(none)'))
            _repair_camera_nodes(self.rs_usb_pids, self.get_logger())
            log = self.vslam.start()
            self.get_logger().info(f'vslam relaunched (log: {log}); re-running camera gate')
            ok, elapsed, report2 = self._watch_vslam_log(log)
            if ok:
                resp.success = True
                how = 'reset_usb + relaunch' if bus_cycled else 'relaunch, no bus cycle'
                resp.message = (f'vslam up: {CAM_COUNT}/{CAM_COUNT} camera in {elapsed:.0f}s '
                                f'(after one recovery: {how})')
                self.get_logger().info(resp.message)
                return resp
            self.get_logger().error('vslam camera gate FAIL (attempt 2/2):\n' + report2)
            self.vslam.stop()
            stage2 = 'post-reset_usb' if bus_cycled else 'post-relaunch, bus NOT cycled'
            tail = ('' if bus_cycled else
                    ' The camera stayed enumerated, so no bus cycle was attempted; call '
                    '/reset_usb and retry if a camera is wedged rather than unclaimed.')
            full = ('vslam camera bringup failed twice (single-recovery policy); stack stopped. '
                    '=== FAILURE 1 (initial) === ' + _squash(report1)
                    + f' === FAILURE 2 ({stage2}) === ' + _squash(report2) + tail)
            resp.success = False
            resp.message = _clip(full)
            return resp
        except Exception as exc:  # noqa: BLE001 - deliberate catch-all: explicit failure over a false 'already running 1/1'
            err = f'{type(exc).__name__}: {exc}'
            self.get_logger().error(
                'vslam bringup aborted by unexpected exception - stopping the unproven stack: ' + err)
            cleanup = 'stack stopped'
            try:
                self.vslam.stop()
            except Exception as stop_exc:
                # Drop tracking so the next enable(true) cannot no-op; survivors then trip the
                # unowned-tree guard, which reaps them when landed and refuses otherwise.
                self.vslam.proc = None
                self.vslam._pgid = None
                cleanup = (f'stack stop ALSO failed ({type(stop_exc).__name__}: {stop_exc}); '
                           'tracking dropped - survivors go to the unowned-tree guard')
                self.get_logger().error(cleanup)
            resp.success = False
            resp.message = _clip(f'vslam bringup aborted by unexpected exception ({err}); {cleanup}')
            return resp

    # Same qualified pattern as container_scripts/reap_stack.sh (keep in sync). The
    # container runs --pid=host, so this is matched against HOST cmdlines too: it must stay
    # fully qualified to 'ros2 launch px4_vslam vslam.launch.py' - the host-side
    # rslidar_coordinator, rslidar_sdk, gst_camera_manager and arid_description units are
    # separate services and must survive.
    REAP_PATTERN = r'ros2 launch px4_vslam vslam\.launch\.py'

    def _unowned_tree_pids(self):
        # Process-level scan: immune to the DDS discovery race that can hide a fresh orphan
        # from `ros2 node list`.
        try:
            out = subprocess.run(['pgrep', '-f', self.REAP_PATTERN],
                                 capture_output=True, text=True, timeout=10)
        except (OSError, subprocess.TimeoutExpired):
            return []
        pids = [int(p) for p in out.stdout.split() if p.strip().isdigit()]
        owned = {g for g in (self.vslam._pgid,) if g}
        return [p for p in pids if p not in owned]

    def _reap_unowned_trees(self):
        # Group-SIGINT the unowned launch trees, drain, group-SIGKILL. Callers MUST hold the
        # landed proof.
        pids = self._unowned_tree_pids()
        if not pids:
            return True
        self.get_logger().warn(f'reaping unowned launch trees: {pids}')
        for p in pids:
            try:
                os.killpg(p, signal.SIGINT)
            except (ProcessLookupError, PermissionError):
                try:
                    os.kill(p, signal.SIGINT)
                except OSError:
                    pass
        deadline = time.monotonic() + 25.0
        while time.monotonic() < deadline:
            if not any(self._group_has_members(p) for p in pids):
                self.get_logger().info('unowned trees drained')
                return True
            time.sleep(1.0)
        for p in pids:
            try:
                os.killpg(p, signal.SIGKILL)
            except OSError:
                pass
        self.get_logger().warn('unowned trees SIGKILLed after 25s')
        return True

    @staticmethod
    def _group_has_members(pgid):
        try:
            return subprocess.run(['pgrep', '-g', str(pgid)],
                                  capture_output=True, timeout=5).returncode == 0
        except (OSError, subprocess.TimeoutExpired):
            return False

    def _legacy_vslam_nodes(self):
        # Only called when self.vslam is not alive, so any vslam node on the graph is foreign.
        # --no-daemon forces fresh discovery; on CLI failure returns [] (introspection must
        # never block bringup).
        try:
            out = subprocess.run(
                ['ros2', 'node', 'list', '--no-daemon'],
                capture_output=True, text=True, timeout=LEGACY_SCAN_TIMEOUT_S,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.get_logger().warn(
                f'legacy-vslam graph check skipped ({type(exc).__name__}: {exc})')
            return []
        return [n.strip() for n in out.stdout.splitlines()
                if 'visual_slam' in n or 'vslam_container' in n]

    def _usb_precheck(self):
        # A camera absent from USB cannot be fixed by launching drivers: one /reset_usb, then fail.
        # Bus is quiescent here (pre-launch), so it is safe to clear orphaned nodes before
        # repairing the camera nodes that survive.
        _prune_orphan_nodes(self.get_logger())
        _repair_camera_nodes(self.rs_usb_pids, self.get_logger())
        devs = _usb_rs_devices(self.rs_usb_pids)
        self.get_logger().info(
            'usb pre-check: %d/%d RealSense (VID %s) on the bus'
            % (len(devs), CAM_COUNT, RS_VID))
        if len(devs) >= CAM_COUNT:
            return True, ''
        self.get_logger().warn(
            'only %d/%d RealSense on USB - devices: %s; issuing one /reset_usb'
            % (len(devs), CAM_COUNT, '; '.join(devs) or '(none)'))
        reset_ok, reset_out = self._reset_usb()
        if not reset_ok:
            return False, (
                'usb pre-check: only %d/%d RealSense on USB and /reset_usb failed (%s) - cannot '
                'recover; not launching vslam. devices: %s'
                % (len(devs), CAM_COUNT, reset_out, '; '.join(devs) or '(none)'))
        if self._wait_usb_rs():
            # Re-enumeration hands the cameras new devnums, so re-check the new nodes.
            _repair_camera_nodes(self.rs_usb_pids, self.get_logger())
            self.get_logger().info(
                'usb pre-check: %d/%d RealSense back after /reset_usb' % (CAM_COUNT, CAM_COUNT))
            return True, ''
        devs = _usb_rs_devices(self.rs_usb_pids)
        return False, (
            'usb pre-check: still %d/%d RealSense (VID %s) on USB after /reset_usb - camera(s) '
            'absent from the bus (dead VBUS/cable/port); not launching vslam. devices: %s'
            % (len(devs), CAM_COUNT, RS_VID, '; '.join(devs) or '(none)'))

    def _wait_usb_rs(self):
        # Poll for re-enumeration after /reset_usb (power cycle ~5 s + enumeration ~10 s).
        deadline = time.monotonic() + USB_REENUM_WAIT_S
        while True:
            if len(_usb_rs_devices(self.rs_usb_pids)) >= CAM_COUNT:
                return True
            if time.monotonic() >= deadline:
                return len(_usb_rs_devices(self.rs_usb_pids)) >= CAM_COUNT
            time.sleep(2.0)

    def _reset_usb(self):
        # ros2 CLI subprocess, NOT an rclpy client: a sync client call inside this service
        # callback deadlocks the single-threaded executor.
        # SAFETY: /reset_usb power-cycles the ARK PAB USB hub the RealSense is on and pulses
        # the FMU reset line (GPIO85); pre-mission bringup only, drone disarmed on the ground.
        try:
            out = subprocess.run(
                ['ros2', 'service', 'call', '/reset_usb', 'std_srvs/srv/Trigger', '{}'],
                capture_output=True, text=True, timeout=RESET_USB_TIMEOUT_S,
            )
            text = (out.stdout + out.stderr).strip()
            ok = out.returncode == 0 and 'success=True' in text
        except (OSError, subprocess.TimeoutExpired) as exc:
            text, ok = f'{type(exc).__name__}: {exc}', False
        if not ok:
            # Fallback: start the host unit directly over the mounted D-Bus socket
            # (polkit authorizes uid 1000); covers a down usb_ros_reset.service.
            try:
                out = subprocess.run(
                    ['systemctl', 'start', 'reset_usb.service'],
                    capture_output=True, text=True, timeout=RESET_USB_TIMEOUT_S,
                )
                fb = (out.stdout + out.stderr).strip()
                if out.returncode == 0:
                    ok, text = True, 'systemctl fallback: reset_usb.service started'
                else:
                    text += ' | systemctl fallback: ' + fb
            except (OSError, subprocess.TimeoutExpired) as exc:
                text += f' | systemctl fallback: {type(exc).__name__}: {exc}'
        if ok:
            self.get_logger().info('/reset_usb: ok')
        else:
            self.get_logger().error('/reset_usb: FAILED - ' + _clip(text, 300))
        return ok, _squash(text)

    def _watch_vslam_log(self, log_path):
        # Tail the per-launch-truncated log so counts are scoped to this generation.
        # Returns (ok, elapsed_s, failure_report); report is '' on success.
        start = time.monotonic()
        buf = ''
        try:
            f = open(log_path, 'r', errors='replace')
        except OSError as exc:
            return False, 0.0, f'GATE FAIL: cannot open vslam log {log_path}: {exc}'
        with f:
            while True:
                buf += f.read()
                elapsed = time.monotonic() - start
                if CAM_PLUGIN_ERR in buf and 'user interrupted with ctrl-c' not in buf:
                    # image_transport publishers failed to construct: the up-marker will
                    # still print but no image ever flows. A ctrl-c line means these are the
                    # PREVIOUS generation's plugin-UNLOAD stragglers on the truncated log.
                    return False, elapsed, self._gate_report(
                        buf, f"'{CAM_PLUGIN_ERR}' in vslam log "
                             '(image_transport plugin load failed - publishers dead)', elapsed)
                ups = _distinct_cam_ups(buf)
                if ups >= CAM_COUNT:
                    return True, elapsed, ''
                if not self.vslam.alive():
                    return False, elapsed, self._gate_report(
                        buf, f'vslam launch process exited during bringup ({ups}/{CAM_COUNT} up)',
                        elapsed)
                if elapsed >= CAM_GATE_BACKSTOP_S:
                    return False, elapsed, self._gate_report(
                        buf, f'backstop: only {ups}/{CAM_COUNT} up after '
                             f'{int(CAM_GATE_BACKSTOP_S)}s', elapsed)
                time.sleep(CAM_GATE_POLL_S)

    def _gate_report(self, buf, reason, elapsed):
        # Verbatim driver evidence: error lines, cameras that did come up, last WARN/ERROR lines.
        lines = buf.splitlines()
        err = [l for l in lines if CAM_ERR_MARKER in l]
        up = [l for l in lines if CAM_UP_MARKER in l]
        warn = [l for l in lines if '[WARN]' in l or '[ERROR]' in l][-5:]

        def block(title, ls):
            body = '\n'.join('  ' + l for l in ls) if ls else '  (none)'
            return f'-- {title}:\n{body}'

        return (f'GATE FAIL after {elapsed:.1f}s: {reason}\n'
                + block(f"'{CAM_ERR_MARKER}' lines (verbatim, incl. exception text)", err) + '\n'
                + block(f"cameras that DID come up ('{CAM_UP_MARKER}')", up) + '\n'
                + block('last WARN/ERROR lines', warn))

    def shutdown(self):
        with self._lock:
            # A supervisor stop while PROVABLY flying leaves vslam running; landed or
            # unknown tears down. ExecStopPost applies the same gate on the crash path.
            if self._landed_fresh() is False:
                self.get_logger().error(
                    'supervisor stopping while AIRBORNE - leaving vslam running '
                    '(land, then deinitialize/initialize)')
                return
            self.vslam.stop()


def main():
    rclpy.init()
    node = AridSupervisor()

    # Treat SIGTERM (systemctl stop / ExecStop) like Ctrl+C so the finally tears down child stacks instead of orphaning them.
    def _terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _terminate)

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
