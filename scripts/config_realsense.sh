#!/bin/bash
# config_realsense.sh - pin the three RealSense serials to their front/left/right mounts
# in vslam_config.yaml, identified via a live feed per camera.

set -u

is_yes() { local a="${1//[^A-Za-z]/}"; case "${a,,}" in y|yes) return 0 ;; *) return 1 ;; esac; }

WORKSPACES="${WORKSPACES:-/home/jetson/workspaces}"
VSLAM_CONFIG="${WORKSPACES}/isaac_ros-dev/src/px4_vslam/config/vslam_config.yaml"
# vslam_config.yaml is untracked and is overwritten from the template on every run. The three
# serials are the only content carried across; any other hand edit to it is lost. Tunables
# belong in vslam_config.template.yaml.
VSLAM_TEMPLATE="${VSLAM_CONFIG%.yaml}.template.yaml"
RESEED_ONLY=0
[[ "${1:-}" == "--reseed-only" ]] && RESEED_ONLY=1
_SERIALS=()
[ -f "${VSLAM_CONFIG}" ] && mapfile -t _SERIALS < <(grep -oE 'serial_no: "[0-9]+"' "${VSLAM_CONFIG}" | grep -oE '[0-9]+')
if (( RESEED_ONLY )) && [ "${#_SERIALS[@]}" -ne 3 ]; then
    cp "${VSLAM_TEMPLATE}" "${VSLAM_CONFIG}"
    echo "WARNING: vslam_config.yaml had ${#_SERIALS[@]}/3 serials - reseeded from template" >&2
    echo "         with BLANK serials; run 'config_realsense' to reassign." >&2
    exit 0
fi
cp "${VSLAM_TEMPLATE}" "${VSLAM_CONFIG}"
if [ "${#_SERIALS[@]}" -eq 3 ]; then
    python3 - "${VSLAM_CONFIG}" "${_SERIALS[@]}" <<'PYSPLICE'
import re, sys
p, serials = sys.argv[1], sys.argv[2:]
s = open(p).read()
it = iter(serials)
s = re.sub(r'serial_no: ""', lambda m: 'serial_no: "%s"' % next(it), s, count=3)
open(p, 'w').write(s)
PYSPLICE
fi
if (( RESEED_ONLY )); then
    echo "vslam_config.yaml reseeded from template (serials preserved)"
    exit 0
fi
MOUNTS=(left front right)

# rs-enumerate-devices and pyrealsense2 live under /opt/ros/humble. ROS setup.bash expands
# unguarded variables, so nounset stays off across the source.
set +u
[[ -f /opt/ros/humble/setup.bash ]] && source /opt/ros/humble/setup.bash 2>/dev/null || true
set -u

if [[ -t 1 ]]; then
    RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[1;34m'; BOLD='\033[1m'; NC='\033[0m'
else
    RED=''; GREEN=''; YELLOW=''; BLUE=''; BOLD=''; NC=''
fi
step() { echo -e "\n${BLUE}${BOLD}==> $*${NC}"; }
ok()   { echo -e "  ${GREEN}[OK]${NC}   $*"; }
warn() { echo -e "  ${YELLOW}[WARN]${NC} $*"; }
skip() { echo -e "  [SKIP]  $*"; }
err()  { echo -e "  ${RED}[ERROR]${NC} $*" >&2; }

# setup.sh runs this under `exec > >(tee ...)`, where a `read -p` prompt block-buffers in the pipe
# and never reaches the operator, so prompts and replies go through the controlling terminal.
TTY="/dev/tty"; { : > "${TTY}"; } 2>/dev/null || TTY="/dev/stderr"
ask() { printf '%s' "$1" > "${TTY}"; IFS= read -r REPLY < "${TTY}" || REPLY=""; }

HAVE_PYRS=0; HAVE_CV2=0; HAVE_DISP=0; FEED=0
# The import text is kept: the usual failure is a native .so that cannot load its dependencies,
# not a missing module, and only the error names the missing library.
PYRS_ERR=$(python3 -c 'import pyrealsense2' 2>&1) && HAVE_PYRS=1 || true
CV2_ERR=$(python3 -c 'import cv2' 2>&1)           && HAVE_CV2=1 || true

# Captured before the retry loop below starts overwriting XAUTHORITY.
REAL_XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"

# nxagent keeps running after the viewer disconnects, so an X display is not proof anyone is
# watching. An ESTABLISHED connection on the NX port is.
NX_PORT=4000
nx_attached() { ss -tn state established 2>/dev/null | awk '{print $3}' | grep -q ":${NX_PORT}$"; }

ensure_display() {
    HAVE_DISP=0
    nx_attached || return 1
    command -v xauth >/dev/null 2>&1 || return 1
    local cands=() d s ck cam
    [[ -n "${DISPLAY:-}" ]] && cands+=("${DISPLAY##*:}")
    for s in $(ls /tmp/.X11-unix/X* 2>/dev/null); do cands+=("$(basename "$s" | sed -E 's/^X([0-9]+).*/\1/')"); done
    for d in "${cands[@]}"; do
        d="${d%%.*}"
        ck=$(XAUTHORITY="${REAL_XAUTH}" xauth list 2>/dev/null | grep -m1 ":${d}[[:space:]]" | awk '{print $3}')
        [[ -n "${ck}" ]] || continue
        # The cookie is re-keyed into a throwaway file. Never write ~/.Xauthority: a stale cookie
        # there breaks the whole NoMachine desktop.
        cam="$(mktemp /tmp/arid_camxauth.XXXXXX)"
        xauth -f "${cam}" add "$(hostname)/unix:${d}" MIT-MAGIC-COOKIE-1 "${ck}" 2>/dev/null || true
        xauth -f "${cam}" add ":${d}" MIT-MAGIC-COOKIE-1 "${ck}" 2>/dev/null || true
        # xhost connects with the cookie, so success proves the display is live and local access
        # is now held. Nothing cheaper distinguishes a live display from a stale socket.
        if DISPLAY=":${d}" XAUTHORITY="${cam}" xhost +local: >/dev/null 2>&1; then
            export DISPLAY=":${d}" XAUTHORITY="${cam}"; _CAM_XAUTH="${cam}"; HAVE_DISP=1; return 0
        fi
        rm -f "${cam}"
    done
    return 1
}
ensure_display || true

FEED_PID=""
close_feed() {
    [[ -z "${FEED_PID}" ]] && return 0
    kill "${FEED_PID}" 2>/dev/null
    local i; for i in 1 2 3 4 5 6; do kill -0 "${FEED_PID}" 2>/dev/null || break; sleep 0.5; done
    kill -9 "${FEED_PID}" 2>/dev/null
    wait "${FEED_PID}" 2>/dev/null
    FEED_PID=""
}
trap 'close_feed; rm -f "${_CAM_XAUTH:-}"' EXIT

open_feed() {              # $1 = camera serial
    [[ "${FEED}" -eq 1 ]] || return 0
    DISPLAY="${DISPLAY}" python3 - "$1" >/dev/null 2>&1 <<'PY' &
import sys, numpy as np, cv2, pyrealsense2 as rs
serial = sys.argv[1]
pipe = rs.pipeline(); cfg = rs.config(); cfg.enable_device(serial)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipe.start(cfg)
try:
    while True:
        f = pipe.wait_for_frames(2000).get_color_frame()
        if not f: continue
        cv2.imshow("Identify this camera (serial %s)" % serial, np.asanyarray(f.get_data()))
        if cv2.waitKey(30) == 27: break
finally:
    try: pipe.stop()
    except Exception: pass
    cv2.destroyAllWindows()
PY
    FEED_PID=$!
    sleep 2
}

write_serial() {           # $1 = mount (left|front|right), $2 = camera serial
    local mount="$1" serial="$2" tmp; tmp=$(mktemp)
    awk -v blk="^${mount}_realsense/" -v sn="${serial}" '
        $0 ~ blk { inblk=1 }
        inblk && /^[A-Za-z0-9_]+\// && $0 !~ blk { inblk=0 }
        inblk && /^[[:space:]]*serial_no:/ && !done {
            match($0, /^[[:space:]]*serial_no:[[:space:]]*/)
            indent = substr($0, 1, RLENGTH); cmt = ""
            if (match($0, /#.*/)) cmt = "               " substr($0, RSTART)
            print indent "\"" sn "\"" (cmt=="" ? "" : cmt); done=1; next
        }
        { print }
    ' "${VSLAM_CONFIG}" > "${tmp}" && mv "${tmp}" "${VSLAM_CONFIG}"
}

# librealsense claims the device over libusb rather than /dev/video*, so without this udev rule the
# node stays 0664 root:root and enumeration returns zero devices to a non-root user. That surfaces
# as a blank serial_no rather than an error.
ensure_libusb_rules() {
    local rs_rules="/etc/udev/rules.d/99-realsense-libusb.rules"
    if [[ -f "${rs_rules}" ]]; then
        ok "${rs_rules} present"
    else
        warn "installing ${rs_rules} (same content setup_permissions writes)"
        sudo tee "${rs_rules}" > /dev/null << 'EOL'
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", MODE:="0666", GROUP:="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3a", MODE:="0666", GROUP:="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3d", MODE:="0666", GROUP:="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5c", MODE:="0666", GROUP:="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b64", MODE:="0666", GROUP:="plugdev"
KERNEL=="iio*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3a", MODE:="0777", GROUP:="plugdev"
KERNEL=="iio*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5c", MODE:="0777", GROUP:="plugdev"
EOL
        sudo udevadm control --reload-rules && sudo udevadm trigger
        sleep 3
        ok "rules installed, reloaded and triggered"
    fi
    id -nG | grep -qw plugdev || warn "${USER} is not in 'plugdev' - log out and back in for the group to apply"
}

realsense_assign() {
    step "RealSense serial assignment"

    ensure_libusb_rules

    [[ -f "${VSLAM_CONFIG}" ]] || { warn "${VSLAM_CONFIG} not found - skipping RealSense assignment"; return 0; }
    local m missing=()
    for m in "${MOUNTS[@]}"; do
        grep -qE "^${m}_realsense/" "${VSLAM_CONFIG}" || missing+=("${m}_realsense")
    done
    (( ${#missing[@]} )) && { warn "vslam_config.yaml missing block(s): ${missing[*]} - skipping"; return 0; }

    local USB_OUT COUNT=0
    USB_OUT=$(lsusb -d 8086: 2>/dev/null | grep -i realsense || true)
    [[ -n "${USB_OUT}" ]] && COUNT=$(echo "${USB_OUT}" | grep -c .)
    [[ "${COUNT}" -ne 3 ]] && { warn "${COUNT} RealSense on USB - exactly 3 required to assign serials; skipping"; return 0; }
    ok "3 RealSense cameras detected"

    local GO
    if [[ -n "${ARID_RS_ASSIGN:-}" ]]; then GO="${ARID_RS_ASSIGN}"
    elif [[ "${#_SERIALS[@]}" -eq 3 ]]; then
        ok "existing serials carried through reseed: ${_SERIALS[*]}"
        ask "  Assign RealSense cameras? (y/n, Enter = keep existing): "; GO="${REPLY}"
        is_yes "${GO}" || { skip "keeping existing serials; template refreshed"; return 0; }
    else
        ask "  Assign RealSense cameras? (y/n, Enter = skip): "; GO="${REPLY}"
    fi
    is_yes "${GO}" || { skip "RealSense serial assignment skipped; vslam_config.yaml unchanged"; return 0; }

    FEED=0
    if (( HAVE_PYRS && HAVE_CV2 && HAVE_DISP )); then FEED=1
    else
        if (( ! HAVE_CV2 )); then
            err "cv2 (OpenCV) not importable: ${CV2_ERR##*$'\n'}"
            err "Usual cause: a missing native lib (e.g. libGL.so.1) or a ~/.local install under a different user."
            exit 2
        fi
        if (( ! HAVE_PYRS )); then
            err "pyrealsense2 not importable: ${PYRS_ERR##*$'\n'}"
            err "Usual cause: a missing native lib or a wheel built for a different python than the host's."
            exit 2
        fi
        if (( ! HAVE_DISP )); then
            # Loopback, link-local and the docker bridge are excluded so the address printed is
            # the one a NoMachine client can reach.
            local _ip _w
            _ip=$(hostname -I 2>/dev/null | tr ' ' '\n' \
                | grep -vE '^(127\.|169\.254\.|172\.(1[6-9]|2[0-9]|3[01])\.)' | head -1)
            local _nm_wait_s="${NM_WAIT_S:-180}"
            printf '\n  Connect a NoMachine session to %s@%s now to assign RealSense serials (waiting up to %ss)...\n' \
                "${USER}" "${_ip:-this host}" "${_nm_wait_s}" > "${TTY}"
            for _w in $(seq 1 "$(( _nm_wait_s / 5 ))"); do ensure_display && break; sleep 5; done
            if (( ! HAVE_DISP )); then
                warn "no NoMachine session connected in time - run 'config_realsense' over NoMachine later"
                exit 0
            fi
            ok "NoMachine display detected"
            FEED=1
        fi
    fi

    local -a SERIALS=()
    if (( HAVE_PYRS )); then
        mapfile -t SERIALS < <(python3 - <<'PY'
import pyrealsense2 as rs
print("\n".join(sorted(d.get_info(rs.camera_info.serial_number) for d in rs.context().query_devices())))
PY
)
    elif command -v rs-enumerate-devices >/dev/null 2>&1; then
        # The awk anchors "Serial Number" at line start so "Asic Serial Number" is not counted too.
        mapfile -t SERIALS < <(rs-enumerate-devices 2>/dev/null \
            | awk -F: '/^[[:space:]]*Serial Number[[:space:]]*:/ {gsub(/[ \t]/,"",$2); print $2}' \
            | sort -u | grep -E '^[0-9]+$')
    else
        warn "cannot enumerate serials (need pyrealsense2 or rs-enumerate-devices) - skipping"; return 0
    fi
    [[ "${#SERIALS[@]}" -ne 3 ]] && { warn "enumerated ${#SERIALS[@]} serial(s) - exactly 3 required; skipping"; return 0; }
    ok "serials: ${SERIALS[*]}"

    if (( FEED )); then
        ok "NoMachine connected - opening live feeds"
    fi

    local -A POS=( [1]=front [2]=left [3]=right )
    local ORDER=(1 2 3)
    local -A ASSIGN TAKEN
    local sn num remaining opts pick

    for sn in "${SERIALS[@]}"; do
        echo ""
        remaining=()
        for num in "${ORDER[@]}"; do [[ -z "${TAKEN[$num]:-}" ]] && remaining+=("${num}"); done
        if (( ${#remaining[@]} == 1 )); then
            num="${remaining[0]}"; ASSIGN["${POS[$num]}"]="${sn}"; TAKEN[$num]=1
            ok "auto-assigned last position: ${POS[$num]} (${num}) = ${sn}"; continue
        fi
        open_feed "${sn}"
        (( FEED )) || warn "no preview - identify serial ${sn} physically"
        opts=""; for num in "${remaining[@]}"; do opts+="${POS[$num]} (${num})   "; done
        while true; do
            ask "  Which position is this feed?   ${opts}: "; pick="${REPLY}"
            pick="${pick//[[:space:]]/}"
            if [[ -n "${POS[$pick]:-}" && -z "${TAKEN[$pick]:-}" ]]; then
                ASSIGN["${POS[$pick]}"]="${sn}"; TAKEN[$pick]=1; ok "${POS[$pick]} (${pick}) = ${sn}"; break
            fi
            printf '    enter one of: %s\n' "${opts}" > "${TTY}"
        done
        close_feed
    done

    step "Writing serials to vslam_config.yaml"
    for m in "${MOUNTS[@]}"; do
        if [[ -n "${ASSIGN[$m]:-}" ]]; then write_serial "${m}" "${ASSIGN[$m]}"; ok "${m}_realsense <- ${ASSIGN[$m]}"
        else warn "${m}_realsense: no serial assigned - left unchanged"; fi
    done
}

realsense_assign
exit 0
