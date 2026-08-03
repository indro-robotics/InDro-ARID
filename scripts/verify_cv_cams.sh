#!/bin/bash
# verify_cv_cams.sh - live visual check of the downward IMX477 CSI camera (alias: ver_cv_cams).
# Requires gst_camera_manager.service, a NoMachine session, cv2 + rclpy.
# A pipeline this script started is stopped again on exit; one already running is left alone.

set -u
ulimit -c 0 2>/dev/null || true   # no core files if a probe's cv2/Qt aborts on a half-ready display

if [[ -t 1 ]]; then
    RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[1;34m'; BOLD='\033[1m'; NC='\033[0m'
else
    RED=''; GREEN=''; YELLOW=''; BLUE=''; BOLD=''; NC=''
fi
step() { echo -e "\n${BLUE}${BOLD}==> $*${NC}"; }
ok()   { echo -e "  ${GREEN}[OK]${NC}   $*"; }
warn() { echo -e "  ${YELLOW}[WARN]${NC} $*"; }
err()  { echo -e "  ${RED}[ERROR]${NC} $*" >&2; }

# setup.sh pipes this script through `tee`, which block-buffers stdout: a line printed before a
# blocking wait stays invisible until the wait ends unless it goes straight to the terminal.
TTY="/dev/tty"; { : > "${TTY}"; } 2>/dev/null || TTY="/dev/stdout"
tnote() { printf '%b\n' "$*" > "${TTY}"; }

step "CV camera feed verification (downward IMX477)"

python3 -c 'import cv2' >/dev/null 2>&1 || { err "cv2 (OpenCV) not importable - cannot display the feed"; exit 1; }
python3 -c 'import rclpy, sensor_msgs.msg' >/dev/null 2>&1 || { err "rclpy / sensor_msgs not importable - cannot subscribe to the camera topic"; exit 1; }

# NEVER write to the real ~/.Xauthority: a stale cookie there breaks the whole NoMachine desktop.
REAL_XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"
CAM_XAUTH="$(mktemp /tmp/arid_camxauth.XXXXXX)"
export XAUTHORITY="${CAM_XAUTH}"

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

# Both the hostname/unix and the bare :N entry are added, so a hostname rename does not
# invalidate the cookie.
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

# Wrapped in a subprocess: Qt aborts the process outright when a NoMachine socket exists but the
# session is not yet ready.
probe_display() {
    timeout 8 bash -c 'DISPLAY=$1 XAUTHORITY=$2 python3 -c "import cv2; cv2.namedWindow(\"_p\"); cv2.destroyAllWindows()"; exit $?' \
        _ "$1" "${XAUTHORITY}" >/dev/null 2>&1
}

# Attached viewer = ESTABLISHED on the NX port; the virtual X server stays up with no
# viewer attached, so a live display is not proof anyone is watching.
NX_PORT=4000
nx_attached() {
    ss -tn state established 2>/dev/null | awk '{print $3}' | grep -q ":${NX_PORT}$"
}

# Sockets are tried highest-number-first: NoMachine session numbers start at :1000 and increment,
# so the newest session wins over a stale one.
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

display_ready() { nx_attached && find_ready_display; }

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

# ament's setup files read unbound variables: sourcing them under `set -u` aborts the script.
[[ -f /opt/ros/humble/setup.bash ]] || { err "ROS 2 Humble not found"; exit 1; }
set +u; source /opt/ros/humble/setup.bash; set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"

# A service started here is left running on exit, unlike the pipeline.
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
ensure_manager || { err "gst_camera_manager could not be started - cannot stream the camera topic"; exit 1; }

pipe_running() { timeout 6 ros2 service call "/gst_camera_manager/$1/status" std_srvs/srv/Trigger 2>/dev/null | grep -q 'success=True'; }

# A cold IMX477 commonly emits one frame and then stalls, which a single --once check misreads
# as a live stream, so a second frame is required ~4 s later.
stream_sustained() {   # $1 = compressed topic, $2 = first-frame budget (s)
    timeout "$2" ros2 topic echo --once --qos-reliability best_effort "$1" >/dev/null 2>&1 || return 1
    timeout 4  ros2 topic echo --once --qos-reliability best_effort "$1" >/dev/null 2>&1
}

# Cold nvargus bring-up routinely exceeds 10 s, so a stalled pipeline is restarted once before
# the check is called a failure.
wait_for_stream() {    # $1 = compressed topic, $2 = pipe
    stream_sustained "$1" 22 && return 0
    tnote "  ${2}: no sustained stream in ~22s (cold start) - restarting the pipeline once and waiting..."
    timeout 10 ros2 service call "/gst_camera_manager/$2" std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
    sleep 2
    timeout 10 ros2 service call "/gst_camera_manager/$2" std_srvs/srv/SetBool "{data: true}"  >/dev/null 2>&1
    stream_sustained "$1" 18
}

# The viewer is fed by process substitution so stdin stays on the terminal for the advance
# keypress; the subshell keeps a stray cv2/Qt abort off the operator's screen.
run_feed() {                       # $1 = label, $2 = pipe, $3 = compressed topic
    local label="$1" pipe="$2" topic="$3"
    # Per-pipeline services register gradually after boot, well after the manager's own services.
    local i svc=0
    for i in $(seq 1 15); do
        ros2 service list 2>/dev/null | grep -q "/gst_camera_manager/${pipe}\$" && { svc=1; break; }
        sleep 1
    done
    (( svc )) || return 2   # pipeline service never appeared

    local was=0; pipe_running "${pipe}" && was=1
    ros2 service call "/gst_camera_manager/${pipe}" std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1
    (( was )) || STARTED_PIPES+=("${pipe}")

    # Wait for a frame before opening the window: a dead camera then reports as dead rather than
    # as a blank window.
    tnote "  ${label}: pipeline started - waiting for a live stream (a cold start after a reboot can take ~40s)..."
    if ! wait_for_stream "${topic}" "${pipe}"; then
        return 3   # no frames after warm-up and one restart
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
GIVEUP = 20.0         # seconds with no frame at all before giving up
GRACE  = 2.0          # seconds after the first frame in which advance keys are ignored, so a
                      # stray keypress cannot skip the window the operator is meant to look at

latest = {"buf": None, "n": 0, "t": 0.0}
def cb(msg):
    latest["buf"] = bytes(msg.data); latest["n"] += 1; latest["t"] = time.time()

rclpy.init()
node = rclpy.create_node("vercam_view")
# BEST_EFFORT subscriber is compatible with a reliable OR best-effort publisher.
qos = QoSProfile(depth=1); qos.reliability = ReliabilityPolicy.BEST_EFFORT; qos.history = HistoryPolicy.KEEP_LAST
node.create_subscription(CompressedImage, topic, cb, qos)

# cbreak so a single keypress advances without Enter.
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

step "Downward IMX477 (cam_down) - live; press any key to continue, q to quit"
run_feed "DOWNWARD IMX477 (cam_down)" cam_down /cam_down/image_raw/compressed; rc=$?
case "$rc" in
    0)  ok "cam_down: stream OK" ;;
    10) exit 0 ;;
    2)  warn "cam_down: gst_camera_manager pipeline not registered yet (still starting) - skipped" ;;
    *)  warn "cam_down: no live frames - IMX477 absent / nvargus-daemon issue, or another consumer holds sensor-id=0 (window not opened)" ;;
esac

exit 0
