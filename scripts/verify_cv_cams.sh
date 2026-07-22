#!/bin/bash
# verify_cv_cams.sh - live visual check of the two CV cameras (IMX219 CSI), through the
# gst_camera_manager:
#   - front IMX219 (CSI, sensor-id=0)    -> pipeline cam_front -> /cam_front/image_raw/compressed
#   - downward IMX219 (CSI, sensor-id=1) -> pipeline cam_down  -> /cam_down/image_raw/compressed
#
# Starts the selected manager pipeline(s) and streams the published COMPRESSED image topic in a
# cv2 window at the pipeline's native rate. The manager is the single camera consumer, so there
# is no contention. Click the window or press any key in THIS TERMINAL to advance ('q' quits).
# Pipelines that were stopped are left stopped again on exit (left as found).
#
# Run any time:  ver_cv_cams [front|down|both]   or   bash scripts/verify_cv_cams.sh [front|down|both]
# With no argument both feeds are shown; pass 'front' or 'down' to check only one.
# Requires gst_camera_manager.service running, a connected NoMachine session, cv2 (OpenCV) + rclpy.

set -u
ulimit -c 0 2>/dev/null || true   # no core files if a probe's cv2/Qt aborts on a half-ready display

# Output helpers.
if [[ -t 1 ]]; then
    RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[1;34m'; BOLD='\033[1m'; NC='\033[0m'
else
    RED=''; GREEN=''; YELLOW=''; BLUE=''; BOLD=''; NC=''
fi
step() { echo -e "\n${BLUE}${BOLD}==> $*${NC}"; }
ok()   { echo -e "  ${GREEN}[OK]${NC}   $*"; }
warn() { echo -e "  ${YELLOW}[WARN]${NC} $*"; }
err()  { echo -e "  ${RED}[ERROR]${NC} $*" >&2; }

# Operator-facing lines that precede a blocking wait must bypass setup's `tee` (which block-buffers
# stdout), or they stay invisible until the wait ends. Write them straight to the controlling terminal.
TTY="/dev/tty"; { : > "${TTY}"; } 2>/dev/null || TTY="/dev/stdout"
tnote() { printf '%b\n' "$*" > "${TTY}"; }

# Optional selector ($1): which camera(s) to check. Default = both (back-compatible).
WHICH="both"
case "${1:-}" in
    front|top|cam_front*)          WHICH="front" ;;
    down|bottom|cam_down*)         WHICH="down" ;;
    ""|both|all)                   WHICH="both" ;;
    *) echo "  unknown camera selector '${1}', checking both" ;;
esac
case "${WHICH}" in
    front) step "CV camera feed verification (front IMX219)" ;;
    down)  step "CV camera feed verification (downward IMX219)" ;;
    *)     step "CV camera feed verification (front + down IMX219)" ;;
esac

# cv2 + rclpy are required; the display can be waited for (open NoMachine after starting).
python3 -c 'import cv2' >/dev/null 2>&1 || { err "cv2 (OpenCV) not importable - cannot display the feed"; exit 1; }
python3 -c 'import rclpy, sensor_msgs.msg' >/dev/null 2>&1 || { err "rclpy / sensor_msgs not importable - cannot subscribe to the camera topics"; exit 1; }

# The user's REAL authority (where NoMachine/nxagent writes the live cookie). We NEVER
# modify it: a stale cookie there makes gnome-session fail ("Invalid MIT-MAGIC-COOKIE-1
# key") and the whole NoMachine desktop will not start. cv2 instead uses our own
# throwaway file.
REAL_XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"
CAM_XAUTH="$(mktemp /tmp/arid_camxauth.XXXXXX)"
export XAUTHORITY="${CAM_XAUTH}"

# The pipeline is left stopped on exit if THIS script started it; also restore terminal + xauth.
STARTED_PIPES=()
cleanup() {
    stty sane 2>/dev/null || true
    local p
    for p in "${STARTED_PIPES[@]:-}"; do
        [[ -n "$p" ]] && ros2 service call "/gst_camera_manager/${p}" std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1 || true
    done
    rm -f "${CAM_XAUTH}"
}
trap cleanup EXIT

# Copy the display's live cookie out of the real authority and re-key it under the current
# hostname into our temp authority. Handles the jetson hostname rename without ever touching
# ~/.Xauthority.
apply_xauth() {
    command -v xauth >/dev/null 2>&1 || return 0
    local dnum="${DISPLAY##*:}"; dnum="${dnum%%.*}"
    local ck; ck=$(XAUTHORITY="${REAL_XAUTH}" xauth list 2>/dev/null | grep -m1 ":${dnum}[[:space:]]" | awk '{print $3}')
    if [[ -n "${ck}" ]]; then
        : > "${CAM_XAUTH}"
        xauth -f "${CAM_XAUTH}" add "$(hostname)/unix:${dnum}" MIT-MAGIC-COOKIE-1 "${ck}" 2>/dev/null || true
        xauth -f "${CAM_XAUTH}" add ":${dnum}" MIT-MAGIC-COOKIE-1 "${ck}" 2>/dev/null || true
    fi
    xhost +local: >/dev/null 2>&1 || true
}

# True only if cv2 can ACTUALLY open a window on $1. Run in a wrapped subprocess so a Qt/X
# abort is contained and silent: prevents the core dump when a NoMachine socket exists but
# the session is not yet ready/authed.
probe_display() {
    timeout 8 bash -c 'DISPLAY=$1 XAUTHORITY=$2 python3 -c "import cv2; cv2.namedWindow(\"_p\"); cv2.destroyAllWindows()"; exit $?' \
        _ "$1" "${XAUTHORITY}" >/dev/null 2>&1
}

# An ATTACHED NoMachine viewer shows as an ESTABLISHED connection on the NX port (4000). The
# virtual desktop X server (e.g. :1005) stays up with no viewer attached, so this is what
# tells us the operator is actually connected and watching.
NX_PORT=4000
nx_attached() {
    ss -tn state established 2>/dev/null | awk '{print $3}' | grep -q ":${NX_PORT}$"
}

# Set DISPLAY to a cv2-usable X display, detected adaptively (no hard-coded number). Try the
# inherited $DISPLAY first, then each /tmp/.X11-unix socket highest-number-first. NoMachine
# sessions are :1000+ and increment, so the newest (the one you just connected) wins over a
# stale or physical lower-numbered display.
find_ready_display() {
    local n
    if [[ -n "${DISPLAY:-}" ]]; then apply_xauth; probe_display "${DISPLAY}" && return 0; fi
    for n in $(ls /tmp/.X11-unix/X* 2>/dev/null | sed 's#.*/X##' | sort -rn); do
        export DISPLAY=":${n}"
        apply_xauth
        probe_display "${DISPLAY}" && return 0
    done
    return 1
}

# Ready = a NoMachine viewer is actually connected AND cv2 can open on its display.
display_ready() { nx_attached && find_ready_display; }

# Wait until a NoMachine session is connected; continue automatically once it is. Enter skips.
if ! display_ready; then
    if [[ ! -t 0 ]]; then warn "no NoMachine session and no terminal to prompt - skipping"; exit 0; fi
    warn "No NoMachine session connected."
    _ip=$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -vE '^(127\.|169\.254\.|172\.(1[6-9]|2[0-9]|3[01])\.)' | head -1)
    tnote "  Connect a NoMachine session to ${USER}@${_ip:-this host} - verification starts automatically"
    tnote "  once you are connected.  (press Enter to skip)"
    _nm_waited=0
    while ! display_ready; do
        if read -r -t 2 _ <"${TTY}" 2>/dev/null; then warn "camera verification skipped"; exit 0; fi
        _nm_waited=$((_nm_waited+2))
        if (( _nm_waited >= ${NM_WAIT_S:-180} )); then warn "no NoMachine session after ${NM_WAIT_S:-180}s - skipping camera verification"; exit 0; fi
    done
fi
ok "NoMachine connected - using display ${DISPLAY}"

# ROS sourcing (for the manager services + topic subscription). ROS/ament setup files reference
# unbound vars under `set -u`, so disable -u during sourcing, then restore.
[[ -f /opt/ros/humble/setup.bash ]] || { err "ROS 2 Humble not found"; exit 1; }
set +u; source /opt/ros/humble/setup.bash; set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"

# Ensure the camera manager is up; start its systemd service if it is not. The manager's
# normal state is running (gst_camera_manager.service, enabled). It is left running afterward.
ensure_manager() {
    ros2 service list 2>/dev/null | grep -q '^/gst_camera_manager/' && return 0
    if systemctl is-active --quiet gst_camera_manager.service 2>/dev/null; then
        echo "  gst_camera_manager.service active - waiting for it to register..."
    else
        warn "gst_camera_manager not running - starting gst_camera_manager.service"
        sudo systemctl start gst_camera_manager.service 2>/dev/null || true
    fi
    local i
    for i in $(seq 1 30); do
        ros2 service list 2>/dev/null | grep -q '^/gst_camera_manager/' && { ok "gst_camera_manager is up"; return 0; }
        sleep 1
    done
    return 1
}
ensure_manager || { err "gst_camera_manager could not be started - cannot stream the camera topics"; exit 1; }

# Is the pipeline currently streaming?
pipe_running() { timeout 6 ros2 service call "/gst_camera_manager/$1/status" std_srvs/srv/Trigger 2>/dev/null | grep -q 'success=True'; }

# True iff the topic delivers a frame within $2 seconds AND is STILL delivering ~4 s later. An
# IMX219 brought up cold right after a reboot (nvargus doing its first CSI init, or the Isaac
# container briefly holding sensor-id=0) commonly emits a single frame then stalls; a one-shot
# `--once` check then declares it live and the window stays blank. Requiring two frames a few
# seconds apart rejects that half-alive state.
stream_sustained() {   # $1 = compressed topic, $2 = first-frame budget (s)
    timeout "$2" ros2 topic echo --once --qos-reliability best_effort "$1" >/dev/null 2>&1 || return 1
    timeout 4  ros2 topic echo --once --qos-reliability best_effort "$1" >/dev/null 2>&1
}

# Wait for a sustained stream on $1, restarting pipeline $2 once if the first (generous) window is
# silent. A fixed short single-shot check false-fails a cold camera after a reboot - nvargus first
# CSI bring-up routinely exceeds 10 s - so warm up, and kick a wedged pipeline once ("started but
# never streams") before giving up. Returns 0 once live, 1 otherwise.
wait_for_stream() {    # $1 = compressed topic, $2 = pipe
    stream_sustained "$1" 22 && return 0
    tnote "  ${2}: no sustained stream in ~22s (cold start) - restarting the pipeline once and waiting..."
    timeout 10 ros2 service call "/gst_camera_manager/$2" std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
    sleep 2
    timeout 10 ros2 service call "/gst_camera_manager/$2" std_srvs/srv/SetBool "{data: true}"  >/dev/null 2>&1
    stream_sustained "$1" 18
}

# run_feed: start the manager pipeline (if not already running), stream its compressed topic in
# a cv2 window, advance on a TERMINAL keypress. Returns 0 once a frame is shown, 3 if none
# arrives. The viewer is fed via process substitution so its stdin stays attached to the
# terminal (for the keypress). Subshell + trailing `exit $?` + stderr to /dev/null keeps a
# stray cv2/Qt abort quiet.
run_feed() {                       # $1 = label, $2 = pipe, $3 = compressed topic
    local label="$1" pipe="$2" topic="$3"
    # ensure_manager only confirmed the manager NODE is up; its per-pipeline services register
    # gradually after a (re)boot and ros2's discovery daemon cache lags, so wait for THIS pipeline's
    # service instead of bailing in the startup race (otherwise it skips here, then the pipeline
    # starts fine seconds later).
    local i svc=0
    for i in $(seq 1 15); do
        ros2 service list 2>/dev/null | grep -q "/gst_camera_manager/${pipe}\$" && { svc=1; break; }
        sleep 1
    done
    (( svc )) || return 2   # pipeline service never showed; caller reports "manager still starting"

    local was=0; pipe_running "${pipe}" && was=1
    ros2 service call "/gst_camera_manager/${pipe}" std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
    (( was )) || STARTED_PIPES+=("${pipe}")     # only stop on exit what we started

    # Check the stream is ALIVE (a frame actually arrives) before opening a window, so a disconnected
    # camera is reported and skipped instead of leaving a blank window with no feedback. best_effort
    # QoS matches the sensor_data publisher.
    tnote "  ${label}: pipeline started - waiting for a live stream (a cold start after a reboot can take ~40s)..."
    if ! wait_for_stream "${topic}" "${pipe}"; then
        return 3   # no frames after warm-up + one restart; caller reports the camera-specific cause
    fi
    ok "${label}: stream is live"

    tnote "  Opening the feed on the NoMachine display - please wait a moment for the window to appear..."
    ( DISPLAY="${DISPLAY}" XAUTHORITY="${XAUTHORITY}" \
        python3 <(cat <<'PY'
import sys, os, time, select
import numpy as np, cv2
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage

label = sys.argv[1]; topic = sys.argv[2]
MAXW   = 960
GIVEUP = 20.0         # seconds with no frame EVER -> pipeline not producing, stop trying
GRACE  = 2.0          # after the window first shows, ignore advance keys this long so the remote
                      # window has time to open/render and is not skipped by a stray key

latest = {"buf": None, "n": 0, "t": 0.0}
def cb(msg):
    latest["buf"] = bytes(msg.data); latest["n"] += 1; latest["t"] = time.time()

rclpy.init()
node = rclpy.create_node("vercam_view")
# BEST_EFFORT subscriber is compatible with a reliable OR best-effort publisher.
qos = QoSProfile(depth=1); qos.reliability = ReliabilityPolicy.BEST_EFFORT; qos.history = HistoryPolicy.KEEP_LAST
node.create_subscription(CompressedImage, topic, cb, qos)

# cbreak the terminal so a single keypress here (no Enter) advances; restored on exit.
try: import termios, tty
except Exception: termios = None
is_tty = bool(termios) and sys.stdin.isatty(); old_term = None
if is_tty:
    try: old_term = termios.tcgetattr(sys.stdin.fileno()); tty.setcbreak(sys.stdin.fileno())
    except Exception: is_tty = False
def term_key():
    if not is_tty: return ""
    if select.select([sys.stdin], [], [], 0)[0]:
        try: return os.read(sys.stdin.fileno(), 64).decode("utf-8", "ignore")
        except OSError: return ""
    return ""

def banner(text):
    img = np.zeros((360, 640), np.uint8)
    cv2.putText(img, text, (16, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, 255, 2)
    cv2.imshow(label, img)

cv2.namedWindow(label, cv2.WINDOW_AUTOSIZE)
banner("starting...")
shown = False; rc = 3; last_n = -1; start_t = time.time(); shown_t = 0.0
try:
    while True:
        rclpy.spin_once(node, timeout_sec=0.03)
        now = time.time()
        if latest["buf"] is not None and latest["n"] != last_n:
            last_n = latest["n"]
            im = cv2.imdecode(np.frombuffer(latest["buf"], np.uint8), cv2.IMREAD_GRAYSCALE)
            if im is not None and im.size:
                if im.shape[1] > MAXW:
                    s = MAXW / float(im.shape[1]); im = cv2.resize(im, (MAXW, int(im.shape[0] * s)))
                if not shown:
                    shown = True; rc = 0; shown_t = now
                    if is_tty:
                        try:
                            while select.select([sys.stdin], [], [], 0)[0]: os.read(sys.stdin.fileno(), 64)
                        except OSError: pass
                cv2.imshow(label, im)
        elif not shown and int(now * 2) % 2 == 0:
            banner("waiting for frames...")
        if not shown and now - start_t > GIVEUP:
            break
        win_key = cv2.waitKey(1)
        if shown and (now - shown_t) >= GRACE:
            tk = term_key()
            if ("q" in tk.lower()) or win_key in (ord("q"), ord("Q")):
                rc = 10; break
            if tk or (not is_tty and win_key != -1):
                break
except Exception:
    pass
finally:
    if old_term is not None:
        try: termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_term)
        except Exception: pass
    try: cv2.destroyAllWindows()
    except Exception: pass
    try: node.destroy_node(); rclpy.shutdown()
    except Exception: pass
sys.exit(rc)
PY
) "$label" "$topic"
      exit $? ) 2>/dev/null
}

# -- Downward IMX219 (cam_down pipeline, sensor-id=1) --------------------------
if [[ "${WHICH}" == both || "${WHICH}" == down ]]; then
    step "Downward IMX219 (cam_down) - live; press any key to continue, q to quit"
    run_feed "DOWNWARD IMX219 (cam_down)" cam_down /cam_down/image_raw/compressed; rc=$?
    case "$rc" in
        0)  ok "cam_down: stream OK" ;;
        10) exit 0 ;;
        2)  warn "cam_down: gst_camera_manager pipeline not registered yet (still starting) - skipped" ;;
        *)  warn "cam_down: no live frames - IMX219 absent / nvargus-daemon issue, or another consumer holds sensor-id=1 (window not opened)" ;;
    esac
fi

# -- Front IMX219 (cam_front pipeline, sensor-id=0) ----------------------------
if [[ "${WHICH}" == both || "${WHICH}" == front ]]; then
    step "Front IMX219 (cam_front) - live; press any key to continue, q to quit"
    run_feed "FRONT IMX219 (cam_front)" cam_front /cam_front/image_raw/compressed; rc=$?
    case "$rc" in
        0)  ok "cam_front: stream OK" ;;
        10) exit 0 ;;
        2)  warn "cam_front: gst_camera_manager pipeline not registered yet (still starting) - skipped" ;;
        *)  warn "cam_front: no live frames - IMX219 absent / nvargus-daemon issue, or another consumer holds sensor-id=0 (window not opened)" ;;
    esac
fi

exit 0
