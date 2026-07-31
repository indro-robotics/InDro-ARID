#!/bin/bash
# config_realsense.sh - detect the front RealSense serial and write it into vslam_config.yaml.
# Serial query: pyrealsense2 wheel preferred, rs-enumerate-devices fallback.
# Never apt-install librealsense on the host: the container stack links the RSUSB build at /usr/local.
# Assumes exactly one RealSense is connected.

set -u

WORKSPACES="${WORKSPACES:-/home/jetson/workspaces}"
VSLAM_CONFIG="${WORKSPACES}/isaac_ros-dev/src/px4_vslam/config/vslam_config.yaml"
# vslam_config.yaml is untracked and REGENERABLE: template (fleet structure/tunables)
# + serial (this drone). Reseed from the template on every run so template updates
# propagate; an existing serial is captured first and re-spliced.
VSLAM_TEMPLATE="${VSLAM_CONFIG%.yaml}.template.yaml"
# --reseed-only: template refresh + serial re-splice, no probing, then exit. Setup runs
# it unconditionally so new template keys reach provisioned drones; a missing/partial
# config is left to the detect flow.
RESEED_ONLY=0
[[ "${1:-}" == "--reseed-only" ]] && RESEED_ONLY=1
_SERIALS=()
[ -f "${VSLAM_CONFIG}" ] && mapfile -t _SERIALS < <(grep -oE 'serial_no: "[0-9]+"' "${VSLAM_CONFIG}" | grep -oE '[0-9]+')
if (( RESEED_ONLY )) && [ "${#_SERIALS[@]}" -ne 1 ]; then
    # Seed anyway so a fresh clone / wiped config still gets a usable file with the
    # current template keys; warn loudly because the serials are NOT restorable here.
    cp "${VSLAM_TEMPLATE}" "${VSLAM_CONFIG}"
    echo "WARNING: vslam_config.yaml had ${#_SERIALS[@]}/1 serials - reseeded from template" >&2
    echo "         with BLANK serials; run 'config_realsense' to reassign." >&2
    exit 0
fi
cp "${VSLAM_TEMPLATE}" "${VSLAM_CONFIG}"
if [ "${#_SERIALS[@]}" -eq 1 ]; then
    python3 - "${VSLAM_CONFIG}" "${_SERIALS[@]}" <<'PYSPLICE'
import re, sys
p, serials = sys.argv[1], sys.argv[2:]
s = open(p).read()
it = iter(serials)
s = re.sub(r'serial_no: ""', lambda m: 'serial_no: "%s"' % next(it), s, count=1)
open(p, 'w').write(s)
PYSPLICE
fi
if (( RESEED_ONLY )); then
    echo "vslam_config.yaml reseeded from template (serial preserved)"
    exit 0
fi

# Source ROS if PATH lacks rs-enumerate-devices (setup.sh invokes this before the
# user's shell sources ROS).
# set +u around the source: ament's setup files read unbound variables, so under `set -u`
# this line aborts the whole script with "AMENT_TRACE_SETUP_FILES: unbound variable" - which is
# why setup.sh only ever reported "config_realsense failed" and left serial_no blank.
if ! command -v rs-enumerate-devices >/dev/null 2>&1 && [[ -f /opt/ros/humble/setup.bash ]]; then
    set +u
    source /opt/ros/humble/setup.bash
    set -u
fi

# Output helpers
if [[ -t 1 ]]; then
    BLUE='\033[1;34m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'
else
    BLUE=''; GREEN=''; YELLOW=''; RED=''; CYAN=''; BOLD=''; NC=''
fi
hdr()  { echo -e "\n${BLUE}${BOLD}================================================================================${NC}"; echo -e "${BLUE}${BOLD}$*${NC}"; echo -e "${BLUE}${BOLD}================================================================================${NC}"; }
step() { echo -e "\n${BLUE}${BOLD}-- $* --${NC}"; }
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

step "2. USB probe (host-side)"
what     "lsusb for Intel ID 8086 (RealSense vendor)."
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
    warn "${COUNT} RealSense cameras detected; this script assumes exactly one"
    note "the first serial returned by rs-enumerate-devices will be written"
else
    ok "1 RealSense on USB"
fi

step "3. Serial-query tooling (pyrealsense2 preferred, rs-enumerate-devices fallback)"
HAVE_PYRS=0
python3 -c 'import pyrealsense2' >/dev/null 2>&1 && HAVE_PYRS=1
if (( ! HAVE_PYRS )) && ! command -v rs-enumerate-devices >/dev/null 2>&1; then
    what "Neither pyrealsense2 nor rs-enumerate-devices found; best-effort pip install of the self-contained pyrealsense2 wheel."
    PIP_BREAK=""
    python3 -m pip install --help 2>/dev/null | grep -q -- '--break-system-packages' && PIP_BREAK="--break-system-packages"
    python3 -m pip install pyrealsense2 ${PIP_BREAK} >/dev/null 2>&1 || true
    python3 -c 'import pyrealsense2' >/dev/null 2>&1 && HAVE_PYRS=1
fi
if (( HAVE_PYRS )); then
    ok "pyrealsense2 importable"
elif command -v rs-enumerate-devices >/dev/null 2>&1; then
    ok "$(which rs-enumerate-devices)"
else
    fail "no serial-query tool available"
    fix  "python3 -m pip install pyrealsense2    # self-contained wheel, no apt librealsense"
    exit 1
fi

step "3b. Host libusb access (udev rules)"
what     "librealsense claims the device over libusb, not just /dev/video*. Without a rule the node is 0664 root:root."
why      "setup installs pyrealsense2 from a pip wheel, which ships no udev rules, and librealsense's own source install is not part of provisioning. Missing rules present as 'no devices' even though lsusb sees the camera."
RS_RULES="/etc/udev/rules.d/99-realsense-libusb.rules"
if [[ -f "${RS_RULES}" ]]; then
    ok "${RS_RULES} present"
else
    what "installing ${RS_RULES} (same content setup_permissions writes)"
    sudo tee "${RS_RULES}" > /dev/null << 'EOL'
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

step "4. Query RealSense serial"
what     "First librealsense 'Serial Number' (the librealsense ID, not the USB iSerial)."
why      "realsense2_camera matches on librealsense's Serial Number. lsusb's iSerial reports the ASIC serial instead, which won't match."
if (( HAVE_PYRS )); then
    SERIAL=$(python3 - <<'PYRS' 2>/dev/null
import pyrealsense2 as rs
ctx = rs.context()
devs = ctx.query_devices()
print(devs[0].get_info(rs.camera_info.serial_number) if devs.size() else "")
PYRS
)
else
    SERIAL=$(rs-enumerate-devices 2>/dev/null \
        | awk -F: '/Serial Number/ {gsub(/[ \t]/,"",$2); print $2; exit}')
fi
if [[ -z "${SERIAL}" ]]; then
    fail "could not extract a RealSense serial"
    fix  "run 'isaac_bash' then 'rs-enumerate-devices' inside the container and inspect manually"
    exit 1
fi
if [[ ! "${SERIAL}" =~ ^[0-9]+$ ]]; then
    fail "extracted value '${SERIAL}' is not a numeric serial"
    exit 1
fi
ok "serial: ${SERIAL}"

step "5. Update serial_no in vslam_config.yaml"
what     "Replace the value on the serial_no: line; preserve indentation and any trailing comment."
CURRENT=$(grep -E '^[[:space:]]*serial_no:' "${VSLAM_CONFIG}" | head -1)
note "before: ${CURRENT}"

# Replace only the value, preserving indentation and any inline comment.
sed -i -E "s|^([[:space:]]*serial_no:[[:space:]]*)\"[^\"]*\"(.*)$|\1\"${SERIAL}\"\2|" "${VSLAM_CONFIG}"

NEW=$(grep -E '^[[:space:]]*serial_no:' "${VSLAM_CONFIG}" | head -1)
note "after:  ${NEW}"

WRITTEN=$(echo "${NEW}" | grep -oE '"[0-9]+"' | tr -d '"' | head -1)
if [[ "${WRITTEN}" == "${SERIAL}" ]]; then
    ok "wrote serial ${SERIAL}"
else
    fail "verification failed; expected ${SERIAL}, file now contains '${WRITTEN}'"
    exit 1
fi

hdr "Summary"
echo "  RealSense serial:  ${SERIAL}"
echo "  Written to:        ${VSLAM_CONFIG}"
echo ""
echo "  Next: rebuild px4_vslam if it has already been built, or relaunch vslam."
echo ""
exit 0
