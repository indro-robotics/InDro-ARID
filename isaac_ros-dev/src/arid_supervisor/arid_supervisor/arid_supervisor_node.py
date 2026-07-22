"""arid_supervisor - always-on lifecycle gate for the ARID VSLAM stack.

Hosts a single `std_srvs/SetBool` service:
  /arid_supervisor/vslam_enable

`true`  -> ros2-launch the vslam stack as a managed subprocess.
`false` -> SIGINT the stack's process group and wait for the whole group to drain so every
node releases its resources and DDS shm; escalate to SIGTERM then SIGKILL only on stall.

vslam_enable=true is CAMERA-PROVEN: it returns success only once all 3 RealSense (front/
left/right) are actually up, so EVERY caller (initialize.sh, manual bringup) inherits the
same protection:
  1. USB pre-check - the 3 RealSense (VID 8086, one of the D43X PIDs) on
     /sys/bus/usb/devices; short -> one /reset_usb + re-enumeration wait + recheck;
     still short -> success=false carrying the per-device USB evidence (stack never
     launched).
  2. Log-watch gate on the (per-launch-truncated) vslam log - SUCCESS once 3 DISTINCT
     cameras (unique node tags) emit "RealSense Node Is Up!",
     FAIL-FAST on "Error starting device" (terminal per camera - upstream retry patch
     reverted), 40 s backstop for silent hangs.
  3. ONE recovery on gate failure - teardown, /reset_usb, respawn, re-watch. Second
     failure -> stack stopped, success=false carrying BOTH verbatim error sets (clipped
     to ~500 chars; full detail in this node's log).
Consequence: the handler BLOCKS ~15 s (healthy) up to ~3 min (double failure). This node
spins under the default single-threaded executor, so the land-detect subscription QUEUES
behind an in-flight bringup; /reset_usb is invoked as a `ros2 service call` SUBPROCESS
because a synchronous rclpy client call from inside a service callback would deadlock that
executor (see _reset_usb).

Idempotency + reentrancy:
  - vslam_enable=true with the supervisor-owned stack alive: NO-OP (stack untouched),
    success=true, message "vslam already running (up <N>s, 3/3 cameras at bringup)".
  - vslam_enable=false with nothing running: success=true no-op.
  - Double-spawn is impossible: every callback of this node sits in the node's default
    MutuallyExclusiveCallbackGroup under the default SingleThreadedExecutor, so a second
    enable(true) can never interleave with an in-flight bringup - it queues on the
    executor (client just waits), and self._lock keeps that guarantee even if the
    executor model ever changes. Once the first bringup returns, the queued call lands
    in the already-running no-op.
  - LEGACY vslam (a direct `ros2 launch px4_vslam vslam.launch.py` outside the
    supervisor): detected pre-spawn via `ros2 node list` (visual_slam/vslam_container on
    the graph while self.vslam is not ours) and REFUSED with the colliding node names -
    spawning over it would only fail later, mid-gate, on node/camera collisions.

Interlock:
  - vslam_enable=false requires VehicleLandDetected.landed == True (within LAND_FRESH_S of
    the last sample). Refused with success=false otherwise.

Logs the vslam subprocess's latest run (truncated per launch) to
/workspaces/isaac_ros-dev/run_logs/<name>/<name>.log.
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
from std_srvs.srv import SetBool, Trigger

from px4_msgs.msg import VehicleLandDetected


LAND_FRESH_S = 2.0
# Per-stack clean-shutdown grace before any hard kill; SIGINT lets nodes release their DDS shm.
SIGINT_GRACE_S = {'vslam': 25.0}
DEFAULT_SIGINT_GRACE_S = 15.0
TERM_WAIT_S = 5.0   # SIGTERM grace after the SIGINT window, before SIGKILL
LOG_DIR = '/workspaces/isaac_ros-dev/run_logs'

# Camera-proven vslam bringup (the gate every vslam_enable caller inherits).
RS_VID = '8086'                           # Intel RealSense USB vendor id
# RealSense D43X-series product id(s). The exact PID differs by variant (D435 0b07,
# D435i 0b3a, D435if 0b3d, D405 0b5c, ...) and MUST be confirmed on the drone; the
# `rs_usb_pids` ROS param below defaults to the D43X-family set and is matched against
# VID 8086. Confirm on hardware with `cat /sys/bus/usb/devices/*/idProduct` (VID is 8086).
DEFAULT_RS_PIDS = ['0b07', '0b3a', '0b3d', '0b64', '0b5c']
CAM_COUNT = 3                             # THREE physical RealSense (front/left/right)
CAM_UP_MARKER = 'RealSense Node Is Up!'
CAM_ERR_MARKER = 'Error starting device'
CAM_GATE_BACKSTOP_S = 40.0   # healthy bringup lands in 14-26 s
CAM_GATE_POLL_S = 0.25
USB_REENUM_WAIT_S = 20.0     # /reset_usb: ~5 s power cycle + ~10 s re-enumeration
RESET_USB_TIMEOUT_S = 30.0   # subprocess `ros2 service call /reset_usb` hard cap
RESP_MSG_MAX = 500           # SetBool response clip; full evidence always in the node log
LEGACY_SCAN_TIMEOUT_S = 20.0  # `ros2 node list --no-daemon` fresh-discovery hard cap


def _proc_descendants(root_pid):
    # Live pids in the process tree rooted at root_pid (inclusive). setsid children live in
    # a different process group, but are still descendants, so this finds them too.
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


def _mapped_shm(pids):
    # Per-participant Fast-DDS GUID segments mapped by these pids. EXCLUDES the domain-global
    # fastrtps_port<N> segments: those are co-mapped by every participant on the DDS domain
    # (other local services, the host PX4 agent), so a stack teardown must never reclaim them.
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
    # SIGINT then (after grace) SIGKILL each still-alive group. Reaps orphaned setsid
    # descendants that survive if their parent dies mid-teardown.
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
    """Reclaim only the GUID segments a torn-down stack owned that no live process still maps.

    `owned` is the per-participant segments recorded from the stack's tree before the kill
    (port segments excluded), so this never touches anything outside the stack. Relies on every
    DDS participant being uid-1000-readable, which holds in this single-uid deployment.
    """
    if not owned:
        return 0
    held = _mapped_shm(int(p) for p in os.listdir('/proc') if p.isdigit())
    removed = []
    for seg in owned:
        if seg in held or not os.path.exists(seg):
            continue
        base = os.path.basename(seg)
        # GUID ring orphaned -> reclaim it + its _el / sem companions (same dead participant).
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
    # Evidence lines for every RealSense on the bus, via /sys/bus/usb/devices (visible
    # in-container; lsusb is not installed here - verified). Device dirs carry idVendor/
    # idProduct; interface dirs (2-1:1.0) don't, and are skipped by the reads failing.
    # `pids` is the set of accepted D43X product ids (matched against VID 8086).
    # NOTE: the sysfs serial is the USB descriptor serial, NOT the librealsense camera
    # serial - evidence only, never compare across the two.
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


def _distinct_cam_ups(buf):
    # DISTINCT cameras up, not raw marker occurrences: one camera re-emitting the marker
    # (driver reconnect/re-init) must never fake CAM_COUNT. Real line shape
    # (run_logs/vslam/vslam.log):
    #   [component_container_mt-2] [INFO] [<stamp>] [left_realsense.left_realsense_link]: RealSense Node Is Up!
    # The LAST [tag] before the marker is the per-camera node tag
    # (left_/front_/right_realsense namespaces) - count unique tags. Tagless fallback:
    # dedupe on the whole prefix (never MORE than raw occurrences).
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
        self.started_at = None   # time.monotonic() of the live proc's spawn (uptime reporting)

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
        self.started_at = time.monotonic()
        return log_path

    def stop(self):
        if not self.alive():
            self.proc = None
            return
        try:
            pgid = os.getpgid(self.proc.pid)
        except ProcessLookupError:
            self.proc = None
            return
        # Snapshot the process tree before the kill: the per-participant shm it owns, and every
        # process group in it - incl. any setsid pipeline groups, which the main group kill
        # never reaches directly.
        descendants = _proc_descendants(self.proc.pid)
        owned = _mapped_shm(descendants)
        groups = set()
        for pid in descendants:
            try:
                groups.add(os.getpgid(pid))
            except ProcessLookupError:
                pass
        grace = SIGINT_GRACE_S.get(self.name, DEFAULT_SIGINT_GRACE_S)
        # SIGINT first: ros2 launch + every rclcpp/rclpy node (cuVSLAM, RealSense) shut down
        # cleanly and release their shm. Escalate only if it stalls.
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
        # Reap any captured group still alive - guarantees no orphaned pipeline survives even if
        # a child was killed mid-teardown. Then reclaim only this stack's leaked shm.
        reaped = _reap_groups(groups, TERM_WAIT_S)
        if reaped:
            self.logger.warn('%s: reaped %d straggler process group(s)' % (self.name, len(reaped)))
        sweep_stack_shm(owned, self.logger)

    def _signal_and_wait(self, pgid, sig, timeout):
        # True once the whole group has drained, not just ros2 launch (self.proc): ros2 launch
        # exits in ~1-2s while component_container_mt still runs the RealSense destructor releasing
        # the camera USB. Returning early let _reap_groups SIGINT that cleanup -> dirty camera on next init.
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


class AridSupervisor(Node):
    def __init__(self):
        super().__init__('arid_supervisor')

        self.vslam = _Stack('vslam', 'px4_vslam', 'vslam.launch.py', self.get_logger())

        # Accepted RealSense PIDs (matched against VID 8086). ROS param so the exact
        # D43X product id can be pinned on-hardware without a code change.
        self.rs_usb_pids = list(self.declare_parameter('rs_usb_pids', DEFAULT_RS_PIDS).value)
        self.get_logger().info(
            'RealSense USB match: VID %s PID one of %s'
            % (RS_VID, ', '.join(self.rs_usb_pids)))

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
        self.create_service(Trigger, '~/status', self._status_cb)

        self.get_logger().info('arid_supervisor up - vslam_enable + status available')

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
        # Side-effect-free health query: VSLAM stack running + freshest land state.
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
                    # Idempotent no-op: stack untouched; alive() re-verified the tracked
                    # process. The cameras were PROVEN 3/3 at bringup (the gate is the only
                    # path to a running supervisor-owned vslam).
                    up_s = int(time.monotonic() - (self.vslam.started_at or time.monotonic()))
                    resp.success = True
                    resp.message = (f'vslam already running (up {up_s}s, '
                                    f'{CAM_COUNT}/{CAM_COUNT} cameras at bringup)')
                    # Journal evidence for the no-op path - otherwise an idempotent return
                    # is invisible to post-run forensics (same class as the unobservable
                    # launch-service defect from the IDLE-stall RCA).
                    self.get_logger().info('vslam_enable(true) idempotent no-op: ' + resp.message)
                    return resp
                return self._vslam_enable_gated(resp)

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

    # ------------------------------------------------------------------
    # Camera-proven vslam bringup (runs under self._lock, from _vslam_cb).

    def _vslam_enable_gated(self, resp):
        # USB pre-check -> spawn -> log-watch gate -> ONE recovery (teardown + /reset_usb +
        # respawn + re-watch). success only with CAM_COUNT cameras proven up; success=false
        # carries the verbatim evidence. Blocks the single-threaded executor for the whole
        # bringup (~15 s healthy, ~3 min worst) - the land-detect subscription queues behind it.
        legacy = self._legacy_vslam_nodes()
        if legacy:
            msg = ('refusing vslam bringup: vslam nodes already on the ROS graph but NOT '
                   'owned by this supervisor (legacy direct '
                   "'ros2 launch px4_vslam vslam.launch.py'?): " + ', '.join(legacy)
                   + '. Spawning over it would collide on node names/cameras. Stop that '
                     'stack (Ctrl+C its ros2 launch / kill its process group), wait ~10s '
                     'for DDS to forget it, then retry.')
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

        # Exception safety: the whole spawn -> watch -> recovery sequence runs under this
        # try. Without it, an unexpected raise (log watch, /reset_usb subprocess, USB
        # re-enum poll) propagates out through _vslam_cb leaving self.vslam
        # launched-but-UNPROVEN - and the next enable(true) would hit the alive() no-op
        # and FALSELY report 'vslam already running (... 3/3 cameras at bringup)' for a
        # stack that never passed the gate.
        try:
            log = self.vslam.start()
            self.get_logger().info(
                f'vslam launching (log: {log}); gating on {CAM_COUNT}x RealSense bringup')
            ok, elapsed, report1 = self._watch_vslam_log(log)
            if ok:
                resp.success = True
                resp.message = f'vslam up: {CAM_COUNT}/{CAM_COUNT} cameras in {elapsed:.0f}s'
                self.get_logger().info(resp.message)
                return resp
            self.get_logger().error('vslam camera gate FAIL (attempt 1/2):\n' + report1)

            # SINGLE recovery, once - no ladder. No land gate here: this is pre-mission bringup
            # at the caller's explicit enable request and the cameras are already unusable.
            self.get_logger().warn('recovery: vslam teardown + /reset_usb + relaunch (single attempt)')
            self.vslam.stop()
            reset_ok, _ = self._reset_usb()
            if not reset_ok:
                self.get_logger().warn('/reset_usb failed - relaunching on the un-cycled bus anyway')
            if not self._wait_usb_rs():
                devs = _usb_rs_devices(self.rs_usb_pids)
                self.get_logger().warn(
                    'only %d/%d RealSense on USB after /reset_usb - relaunching anyway; devices: %s'
                    % (len(devs), CAM_COUNT, '; '.join(devs) or '(none)'))
            log = self.vslam.start()
            self.get_logger().info(f'vslam relaunched (log: {log}); re-running camera gate')
            ok, elapsed, report2 = self._watch_vslam_log(log)
            if ok:
                resp.success = True
                resp.message = (f'vslam up: {CAM_COUNT}/{CAM_COUNT} cameras in {elapsed:.0f}s '
                                '(after one reset_usb recovery)')
                self.get_logger().info(resp.message)
                return resp
            self.get_logger().error('vslam camera gate FAIL (attempt 2/2):\n' + report2)
            self.vslam.stop()
            full = ('vslam camera bringup failed twice (single-recovery policy); stack stopped. '
                    '=== FAILURE 1 (initial) === ' + _squash(report1)
                    + ' === FAILURE 2 (post-reset_usb) === ' + _squash(report2))
            resp.success = False
            resp.message = _clip(full)
            return resp
        except Exception as exc:  # noqa: BLE001 - deliberate catch-all: honest failure beats a false 'already running 3/3'
            err = f'{type(exc).__name__}: {exc}'
            self.get_logger().error(
                'vslam bringup aborted by unexpected exception - stopping the unproven stack: ' + err)
            cleanup = 'stack stopped'
            try:
                self.vslam.stop()
            except Exception as stop_exc:
                # Even the teardown failed: drop tracking so the next enable(true) cannot
                # no-op as 'already running 3/3'; any survivors then trip the pre-spawn
                # legacy-node guard, which refuses with the colliding node names.
                self.vslam.proc = None
                cleanup = (f'stack stop ALSO failed ({type(stop_exc).__name__}: {stop_exc}); '
                           'tracking dropped - survivors will be refused by the legacy-node guard')
                self.get_logger().error(cleanup)
            resp.success = False
            resp.message = _clip(f'vslam bringup aborted by unexpected exception ({err}); {cleanup}')
            return resp

    def _legacy_vslam_nodes(self):
        # Pre-spawn guard against a vslam stack this supervisor does NOT own (legacy direct
        # `ros2 launch px4_vslam vslam.launch.py`). Spawning over one only fails later,
        # mid-gate, on duplicate node names / double-claimed cameras - refuse up front
        # instead, with the colliding node names. Only called when self.vslam is not alive,
        # so ANY visual_slam/vslam_container node on the graph is foreign. `--no-daemon`
        # forces fresh discovery (no ros2cli daemon state under systemd). A freshly-crashed
        # stack can linger on the graph until its DDS lease expires - the refusal message
        # says to wait ~10s and retry. On CLI failure returns [] - an introspection tool
        # must never block bringup.
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
        # A dead camera VBUS means the camera is simply ABSENT from USB - no point launching
        # the drivers. One /reset_usb attempt, then honest failure with per-device evidence.
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
            self.get_logger().info(
                'usb pre-check: %d/%d RealSense back after /reset_usb' % (CAM_COUNT, CAM_COUNT))
            return True, ''
        devs = _usb_rs_devices(self.rs_usb_pids)
        return False, (
            'usb pre-check: still %d/%d RealSense (VID %s) on USB after /reset_usb - camera(s) '
            'absent from the bus (dead VBUS/cable/port); not launching vslam. devices: %s'
            % (len(devs), CAM_COUNT, RS_VID, '; '.join(devs) or '(none)'))

    def _wait_usb_rs(self):
        # Post-/reset_usb the power cycle takes ~5 s and re-enumeration ~10 s more; poll up
        # to USB_REENUM_WAIT_S for all CAM_COUNT RealSense to reappear.
        deadline = time.monotonic() + USB_REENUM_WAIT_S
        while True:
            if len(_usb_rs_devices(self.rs_usb_pids)) >= CAM_COUNT:
                return True
            if time.monotonic() >= deadline:
                return len(_usb_rs_devices(self.rs_usb_pids)) >= CAM_COUNT
            time.sleep(2.0)

    def _reset_usb(self):
        # DESIGN CHOICE - deadlock avoidance: this node spins via rclpy.spin() (default
        # SingleThreadedExecutor, default mutually-exclusive callback group). A synchronous
        # rclpy client call to /reset_usb from inside this service callback can never see
        # its response (the executor is blocked right here), i.e. it DEADLOCKS. Rather than
        # move the whole node to a MultiThreadedExecutor (which would change the concurrency
        # model of every flight-critical callback), /reset_usb - a host-side
        # std_srvs/Trigger on the shared DDS graph - is invoked via the ros2 CLI in a
        # subprocess: crude, isolated, deadlock-free. Same env as the `ros2 launch` Popen
        # this node already relies on.
        # SAFETY: ARID's /reset_usb (reset_ark_usb -> uhubctl + GPIO85) power-cycles the ARK
        # PAB USB hub that the RealSense cameras are on. Acceptable HERE ONLY - pre-mission vslam
        # bringup, drone disarmed on the ground. NEVER issue it later in the mission lifecycle.
        try:
            out = subprocess.run(
                ['ros2', 'service', 'call', '/reset_usb', 'std_srvs/srv/Trigger', '{}'],
                capture_output=True, text=True, timeout=RESET_USB_TIMEOUT_S,
            )
            text = (out.stdout + out.stderr).strip()
            ok = out.returncode == 0 and 'success=True' in text
        except (OSError, subprocess.TimeoutExpired) as exc:
            text, ok = f'{type(exc).__name__}: {exc}', False
        if ok:
            self.get_logger().info('/reset_usb: ok')
        else:
            self.get_logger().error('/reset_usb: FAILED - ' + _clip(text, 300))
        return ok, _squash(text)

    def _watch_vslam_log(self, log_path):
        # Event-race gate: tail-follow the vslam log from offset 0 (the _Stack.start open()
        # truncates it per launch, so counts are scoped to this generation). Poll every
        # CAM_GATE_POLL_S:
        #   SUCCESS   - CAM_COUNT DISTINCT cameras emitted CAM_UP_MARKER (_distinct_cam_ups)
        #   FAIL-FAST - the moment CAM_ERR_MARKER appears (with the upstream 5x-retry patch
        #               reverted, a startup claim collision is TERMINAL for that camera -
        #               no point waiting for the backstop), or the launch process dies.
        #   BACKSTOP  - CAM_GATE_BACKSTOP_S for silent hangs that neither succeed nor error.
        # Returns (ok, elapsed_s, failure_report) - report is '' on success.
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
                if CAM_ERR_MARKER in buf:
                    return False, elapsed, self._gate_report(
                        buf, f"'{CAM_ERR_MARKER}' in vslam log "
                             '(terminal per camera - retry patch reverted)', elapsed)
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
                             f'{int(CAM_GATE_BACKSTOP_S)}s (silent hang, no driver error)', elapsed)
                time.sleep(CAM_GATE_POLL_S)

    def _gate_report(self, buf, reason, elapsed):
        # Verbatim failure evidence so the operator sees WHAT failed: the driver's terminal
        # "Error starting device: <exception>" lines, which cameras DID come up, and the
        # last 5 WARN/ERROR lines.
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
