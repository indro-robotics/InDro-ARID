#!/bin/bash
# Auto-detect the single front RealSense serial via rs-enumerate-devices on the
# host (provided by ros-humble-librealsense2), write it into vslam_config.yaml.
# Assumes exactly one RealSense is connected (this drone's hardware constraint).

set -u

WORKSPACES="${WORKSPACES:-/home/jetson/workspaces}"
VSLAM_CONFIG="${WORKSPACES}/isaac_ros-dev/src/px4_vslam/config/vslam_config.yaml"

# rs-enumerate-devices lives in /opt/ros/humble/bin (apt ros-humble-librealsense2).
# Source ROS if PATH doesn't already have it (e.g. invoked from setup.sh before
# the user's interactive bash sources /opt/ros/humble/setup.bash).
command -v rs-enumerate-devices >/dev/null 2>&1 \
    || { [[ -f /opt/ros/humble/setup.bash ]] && source /opt/ros/humble/setup.bash; }

# ───────────────── output helpers ─────────────────
if [[ -t 1 ]]; then
    BLUE='\033[1;34m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'
else
    BLUE=''; GREEN=''; YELLOW=''; RED=''; CYAN=''; BOLD=''; NC=''
fi
hdr()  { echo -e "\n${BLUE}${BOLD}================================================================================${NC}"; echo -e "${BLUE}${BOLD}$*${NC}"; echo -e "${BLUE}${BOLD}================================================================================${NC}"; }
step() { echo -e "\n${BLUE}${BOLD}── $* ──${NC}"; }
what() { echo -e "${CYAN}WHAT:${NC}    $*"; }
why()  { echo -e "${CYAN}WHY:${NC}     $*"; }
raw()  { echo -e "${CYAN}RAW:${NC}"; echo "$1" | sed 's/^/         /'; }
ok()   { echo -e "${GREEN}RESULT:  [OK]    $*${NC}"; }
warn() { echo -e "${YELLOW}RESULT:  [WARN]  $*${NC}"; }
fail() { echo -e "${RED}RESULT:  [FAIL]  $*${NC}"; }
fix()  { echo -e "${CYAN}FIX:${NC}     $*"; }
note() { echo -e "         $*"; }

hdr "Front RealSense Serial Auto-Configuration"
echo ""
echo "Target: ${VSLAM_CONFIG}"

# ───────────────── 1. yaml present ─────────────────
step "1. vslam_config.yaml present"
if [[ ! -f "${VSLAM_CONFIG}" ]]; then
    fail "${VSLAM_CONFIG} not found"
    fix  "ensure isaac_ros-dev/src/px4_vslam is checked out"
    exit 1
fi
if ! grep -qE '^[[:space:]]*serial_no:' "${VSLAM_CONFIG}"; then
    fail "no serial_no: line in ${VSLAM_CONFIG}"
    exit 1
fi
ok "found, has a serial_no: line"

# ───────────────── 2. host-side USB probe ─────────────────
step "2. USB probe (host-side)"
what     "lsusb for Intel ID 8086 — RealSense vendor."
why      "Fail fast before invoking rs-enumerate-devices, which may hang briefly when no camera is enumerable."
USB_OUT=$(lsusb -d 8086: 2>/dev/null | grep -i realsense || true)
if [[ -z "${USB_OUT}" ]]; then
    fail "no RealSense on USB"
    fix  "plug in the camera and verify with 'lsusb -d 8086:'"
    exit 1
fi
raw "${USB_OUT}"
COUNT=$(echo "${USB_OUT}" | wc -l)
if [[ "${COUNT}" -gt 1 ]]; then
    warn "${COUNT} RealSense cameras detected — this script assumes exactly one"
    note "the first serial returned by rs-enumerate-devices will be written"
else
    ok "1 RealSense on USB"
fi

# ───────────────── 3. rs-enumerate-devices available ─────────────────
step "3. rs-enumerate-devices on PATH"
if ! command -v rs-enumerate-devices >/dev/null 2>&1; then
    fail "rs-enumerate-devices not found"
    fix  "sudo apt-get install ros-humble-librealsense2    # provides /opt/ros/humble/bin/rs-enumerate-devices"
    exit 1
fi
ok "$(which rs-enumerate-devices)"

# ───────────────── 4. serial via rs-enumerate-devices ─────────────────
step "4. Query RealSense serial"
what     "rs-enumerate-devices; parse the first 'Serial Number' field (the librealsense ID, not the USB iSerial)."
why      "realsense2_camera matches on librealsense's Serial Number. lsusb's iSerial reports the ASIC serial instead, which won't match."
SERIAL=$(rs-enumerate-devices 2>/dev/null \
    | awk -F: '/Serial Number/ {gsub(/[ \t]/,"",$2); print $2; exit}')
if [[ -z "${SERIAL}" ]]; then
    fail "could not extract serial from rs-enumerate-devices output"
    fix  "run 'isaac_bash' then 'rs-enumerate-devices' and inspect the output manually"
    exit 1
fi
if [[ ! "${SERIAL}" =~ ^[0-9]+$ ]]; then
    fail "extracted value '${SERIAL}' is not a numeric serial"
    exit 1
fi
ok "serial: ${SERIAL}"

# ───────────────── 5. write to yaml ─────────────────
step "5. Update serial_no in vslam_config.yaml"
what     "Replace the value on the serial_no: line; preserve indentation and any trailing comment."
CURRENT=$(grep -E '^[[:space:]]*serial_no:' "${VSLAM_CONFIG}" | head -1)
note "before: ${CURRENT}"

# Preserve indentation and inline comment. Pattern: <indent>serial_no:<gap><value><optional space + # comment>
# Replace only the value, keeping the comment.
sed -i -E "s|^([[:space:]]*serial_no:[[:space:]]*)\"[^\"]*\"(.*)$|\1\"${SERIAL}\"\2|" "${VSLAM_CONFIG}"

NEW=$(grep -E '^[[:space:]]*serial_no:' "${VSLAM_CONFIG}" | head -1)
note "after:  ${NEW}"

# Verify the written value matches.
WRITTEN=$(echo "${NEW}" | grep -oE '"[0-9]+"' | tr -d '"' | head -1)
if [[ "${WRITTEN}" == "${SERIAL}" ]]; then
    ok "wrote serial ${SERIAL}"
else
    fail "verification failed; expected ${SERIAL}, file now contains '${WRITTEN}'"
    exit 1
fi

# ───────────────── summary ─────────────────
hdr "Summary"
echo "  RealSense serial:  ${SERIAL}"
echo "  Written to:        ${VSLAM_CONFIG}"
echo ""
echo "  Next: rebuild px4_vslam if it's already been built, or just relaunch vslam."
echo ""
exit 0
