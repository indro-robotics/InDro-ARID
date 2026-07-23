#!/bin/bash
# Calibrate a CSI pipeline with the ROS camera_calibration tool.
#
# Usage:  camera_calibrate.sh front|down
#
# Targets:
#   front = cam_front (sensor-id 0, topic /cam_front)
#   down  = cam_down  (sensor-id 1, topic /cam_down)
#
# Saves to the calibration store AND applies to the live pipeline calibration file
# (takes effect on the next pipeline restart).
#
# Env overrides (skip the board prompt):
#   SIZE     interior corners WxH        (default 9x6 = 10x7 squares)
#   SQUARE   square side in metres       (default 0.050)
#   ROS_DOMAIN_ID                        (default 23)
#
# The calibrator opens a GUI window: connect a NoMachine session first.

set -u
ulimit -c 0 2>/dev/null || true   # no core files if a display probe's cv2/Qt aborts
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${WORKSPACES:-$(cd "${SCRIPT_DIR}/../../../.." && pwd)}"
VENV_DIR="${SCRIPT_DIR}/calib_env"
GST_CALIB="${WS}/local_ws/src/ros_gst_cameras/gst_camera_manager/config/calibrations"
STORE="${WS}/local_ws/auxiliary/camera_calibration/camera_calibrations"

if [[ -t 1 ]]; then GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
else GREEN=''; YELLOW=''; RED=''; NC=''; fi
ok()   { echo -e "  ${GREEN}[OK]${NC}   $*"; }
warn() { echo -e "  ${YELLOW}[WARN]${NC} $*"; }
err()  { echo -e "  ${RED}[ERROR]${NC} $*" >&2; }
is_yes() { local a="${1//[^A-Za-z]/}"; case "${a,,}" in y|yes) return 0;; *) return 1;; esac; }

case "${1:-}" in
    front|cam_front)  PIPE="cam_front"; TOPIC="/cam_front/image_raw"; NS="/cam_front"; STORE_DIR="${STORE}/cam_front"; STORE_FILE="cam_front.yaml"; LIVE="${GST_CALIB}/cam_front.yaml" ;;
    down|cam_down)    PIPE="cam_down";  TOPIC="/cam_down/image_raw";  NS="/cam_down";  STORE_DIR="${STORE}/cam_down";  STORE_FILE="cam_down.yaml";  LIVE="${GST_CALIB}/cam_down.yaml" ;;
    *) err "usage: $(basename "$0") front|down"; exit 2 ;;
esac

# Prompt via /dev/tty: under setup.sh's tee pipe, a `read -p` prompt never reaches the operator.
TTY="/dev/tty"; { : > "${TTY}"; } 2>/dev/null || TTY="/dev/stderr"
ask() { printf '%s' "$1" > "${TTY}"; IFS= read -r REPLY < "${TTY}" || REPLY=""; }

# Refuse non-terminal runs: a backgrounded calibrator window cannot be driven or closed.
[[ -t 0 ]] || { err "calibration must be run from an interactive terminal"; exit 6; }

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"

# cameracalibrator's --size is INTERIOR CORNERS = squares - 1 per dimension.
if [[ -z "${SIZE:-}" || -z "${SQUARE:-}" ]] && [[ -t 0 ]]; then
    ans=""
    ask "Use a custom calibration board? (y/n, Enter = included 10x7-square / 50 mm board): "; ans="${REPLY}"
    if is_yes "${ans}"; then
        cols=""; rows=""; mm=""
        ask "  Columns (number of squares across): "; cols="${REPLY}"
        ask "  Rows (number of squares down): ";      rows="${REPLY}"
        ask "  Square size in mm: ";                  mm="${REPLY}"
        cols="${cols//[^0-9]/}"; rows="${rows//[^0-9]/}"; mm="${mm//[^0-9.]/}"
        if [[ "$cols" =~ ^[0-9]+$ && "$rows" =~ ^[0-9]+$ && "$mm" =~ ^[0-9]+(\.[0-9]+)?$ && "$cols" -ge 2 && "$rows" -ge 2 ]]; then
            SIZE="$((cols-1))x$((rows-1))"
            SQUARE="$(awk "BEGIN{printf \"%.6f\", ${mm}/1000}")"
            ok "custom board: ${cols}x${rows} squares -> ${SIZE} corners, ${mm} mm squares"
        else
            warn "incomplete / invalid board entry - using the included default"
        fi
    fi
fi
SIZE="${SIZE:-9x6}"; SQUARE="${SQUARE:-0.050}"
echo "Calibrating ${PIPE}: pipeline '${PIPE}', topic ${TOPIC}, board ${SIZE} corners @ ${SQUARE} m"

# numpy<2 venv: Humble cv_bridge breaks under numpy 2.x.
if [[ ! -x "${VENV_DIR}/bin/python3" ]]; then
    echo "[calib] creating venv at ${VENV_DIR}"
    python3 -m venv "${VENV_DIR}" --system-site-packages
fi
NUMPY_MAJOR=$(PYTHONNOUSERSITE=1 "${VENV_DIR}/bin/python3" -c "import numpy;print(numpy.__version__.split('.')[0])" 2>/dev/null || echo 0)
if [[ "${NUMPY_MAJOR}" -ge 2 || "${NUMPY_MAJOR}" -eq 0 ]]; then
    echo "[calib] installing numpy<2 into venv"
    PYTHONNOUSERSITE=1 "${VENV_DIR}/bin/pip" install "numpy<2" -q
fi

# Temporary X authority: ~/.Xauthority is never modified.
command -v python3 >/dev/null 2>&1 && python3 -c 'import cv2' >/dev/null 2>&1 \
    || { err "cv2 (OpenCV) not importable - the calibrator GUI needs it"; exit 1; }

REAL_XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"
CAM_XAUTH="$(mktemp /tmp/arid_camxauth.XXXXXX)"
export XAUTHORITY="${CAM_XAUTH}"

STARTED=0
stop_cam() { (( STARTED )) && ros2 service call "/gst_camera_manager/${PIPE}" std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1 || true; }
cleanup()  { stop_cam; rm -f "${CAM_XAUTH:-}"; }
trap cleanup EXIT

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

probe_display() {
    timeout 8 bash -c 'DISPLAY=$1 XAUTHORITY=$2 python3 -c "import cv2; cv2.namedWindow(\"_p\"); cv2.destroyAllWindows()"; exit $?' \
        _ "$1" "${XAUTHORITY}" >/dev/null 2>&1
}

NX_PORT=4000
nx_attached() { ss -tn state established 2>/dev/null | awk '{print $3}' | grep -q ":${NX_PORT}$"; }
find_ready_display() {
    local n
    if [[ -n "${DISPLAY:-}" ]]; then apply_xauth; probe_display "${DISPLAY}" && return 0; fi
    for n in $(ls /tmp/.X11-unix/X* 2>/dev/null | sed 's#.*/X##' | sort -rn); do
        export DISPLAY=":${n}"; apply_xauth; probe_display "${DISPLAY}" && return 0
    done
    return 1
}
display_ready() { nx_attached && find_ready_display; }

if ! display_ready; then
    if [[ ! -t 0 ]]; then err "no NoMachine session and no terminal to prompt - connect NoMachine and rerun"; exit 5; fi
    warn "No NoMachine session connected."
    _ip=$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -vE '^(127\.|169\.254\.|172\.(1[6-9]|2[0-9]|3[01])\.)' | head -1)
    echo "  The calibrator opens a GUI window - connect a NoMachine session to ${USER}@${_ip:-this host}."
    echo "  Calibration starts automatically once you are connected. (press Enter to cancel)"
    _nm_waited=0
    while ! display_ready; do
        if read -r -t 2 _ 2>/dev/null; then warn "calibration cancelled - no display"; exit 0; fi
        _nm_waited=$((_nm_waited+2))
        if (( _nm_waited >= ${NM_WAIT_S:-180} )); then warn "no NoMachine session after ${NM_WAIT_S:-180}s - skipping calibration"; exit 0; fi
    done
fi
ok "NoMachine connected - using display ${DISPLAY}"

[[ -f /opt/ros/humble/setup.bash ]] || { err "ROS 2 Humble not found"; exit 1; }
set +u; source /opt/ros/humble/setup.bash; set -u

ensure_manager() {
    ros2 service list 2>/dev/null | grep -q '^/gst_camera_manager/' && return 0
    if systemctl is-active --quiet gst_camera_manager.service 2>/dev/null; then
        echo "[calib] gst_camera_manager.service active - waiting for it to register..."
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
ensure_manager || { err "gst_camera_manager could not be started"; exit 1; }
if ! ros2 service list 2>/dev/null | grep -q "/gst_camera_manager/${PIPE}\$"; then
    err "pipeline service /gst_camera_manager/${PIPE} not found in the manager config"
    exit 1
fi

start_pipe()      { ros2 service call "/gst_camera_manager/${PIPE}" std_srvs/srv/SetBool "{data: true}"  >/dev/null 2>&1 && STARTED=1; }
stop_pipe()       { ros2 service call "/gst_camera_manager/${PIPE}" std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1; STARTED=0; }
wait_for_frames() { local n; for n in $(seq 1 "${1:-12}"); do timeout 2 ros2 topic echo --once "${TOPIC}" >/dev/null 2>&1 && return 0; done; return 1; }

# Stop first: this run must be the sole consumer of the Argus pipeline.
stop_pipe; sleep 2
echo "[calib] starting ${PIPE}..."
start_pipe
echo "[calib] waiting for frames on ${TOPIC}..."
if ! wait_for_frames 12; then
    warn "no frames yet - restarting the pipeline once"
    stop_pipe; sleep 3; start_pipe
    wait_for_frames 15 || { err "no frames on ${TOPIC} - camera not streaming (check that the Isaac container is not holding this sensor-id)"; exit 3; }
fi
ok "camera streaming"

rm -f /tmp/ost.yaml /tmp/calibrationdata.tar.gz
printf '%s\n' "[calib] launching cameracalibrator - the window can take a while to open on the NoMachine display, please wait..." > "${TTY}"
printf '%s\n' "[calib] when it appears: move the board through the frame, then Calibrate -> Commit" > "${TTY}"
PYTHONNOUSERSITE=1 "${VENV_DIR}/bin/python3" \
    /opt/ros/humble/lib/camera_calibration/cameracalibrator \
    --size "${SIZE}" --square "${SQUARE}" \
    --ros-args --remap "image:=${TOPIC}" --remap "camera:=${NS}" || true

# "Commit" writes /tmp/ost.yaml; "Save" writes /tmp/calibrationdata.tar.gz.
if [[ ! -f /tmp/ost.yaml && -f /tmp/calibrationdata.tar.gz ]]; then
    tar -xzf /tmp/calibrationdata.tar.gz -C /tmp/ ost.yaml 2>/dev/null || true
fi
if [[ ! -f /tmp/ost.yaml ]]; then
    warn "no calibration produced - did you click Calibrate then Commit / Save?"
    exit 4
fi

mkdir -p "${STORE_DIR}"
ts=$(date +%Y%m%d_%H%M%S)
cp /tmp/ost.yaml "${STORE_DIR}/${STORE_FILE%.yaml}_${ts}.yaml"
cp /tmp/ost.yaml "${STORE_DIR}/${STORE_FILE}"
ok "saved -> ${STORE_DIR}/${STORE_FILE}"
mkdir -p "$(dirname "${LIVE}")"; cp /tmp/ost.yaml "${LIVE}"
ok "applied -> ${LIVE}"

echo "[calib] done. Set calibration: \"${PIPE}\" in pipelines.yaml, then restart the ${PIPE} pipeline (${PIPE}_stop; ${PIPE}_start) to pick up the new calibration."
