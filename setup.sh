#!/bin/bash
# ARID workspace setup.
# Usage: ./setup.sh [--full|--resume|--continue|--help]
# Every step is idempotent; safe to re-run any time.
#
# Domain steps live under setup/*.sh. This file is orchestration only.
set -euo pipefail

# Source domain libraries
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP_DIR="${SCRIPT_DIR}/setup"

# io.sh first (defines failure, quit_handler, helpers).
# shellcheck source=setup/io.sh
source "${SETUP_DIR}/io.sh"
for _lib in system network ark lidar container; do
    # shellcheck disable=SC1090
    source "${SETUP_DIR}/${_lib}.sh"
done
unset _lib

# Global state
STEPS_RUN=()
STEPS_SKIPPED=()
DID_BUILD=0         # gates the pre-smoke reboot: set when any build ran this session
INSTALL_GROUP_REBOOT=0  # gates the install reboot: set by ark_os when ARK-OS / ROS2 / JetPack installed
RUN_FULL=0          # --full: skip the menu and run the whole setup
RUN_RESUME=0        # --resume: continue setup after a reboot
FORCE_CONTINUE=0    # --continue: re-enter the setup tail with no stage marker

# Single source of truth for the Foxglove launch (alias + camera-focus both use it).
FOXGLOVE_LAUNCH="ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"

# Pre-collected answers (full setup only). PRE=1 means "use these, do not prompt mid-run".
PRE=0
PRE_HOST=""; PRE_PASS=""
PRE_WIFI=""; PRE_WIFI_SSID=""; PRE_WIFI_PASS=""
PRE_NOMACHINE=""; PRE_PX4=""; PRE_ZTNET=""
PRE_ARK=""; PRE_ARK_ROS2=""; PRE_JETPACK=""; PRE_POST_ARK_REBOOT=""
PRE_REALSENSE=""; PRE_VERIFY=""; PRE_FOCUS=""
PRE_BUILD_ISAAC=""; PRE_COLCON=""; PRE_LOCAL_WS=""
PRE_SMOKE=""; PRE_REBOOT=""

# Constants
USERNAME="jetson"
HOME_DIR="/home/${USERNAME}"
BASHRC_FILE="${HOME_DIR}/.bashrc"
WORKSPACES="${HOME_DIR}/workspaces"
LOCAL_WS="${WORKSPACES}/local_ws"
ISAAC_ROS_WS="${WORKSPACES}/isaac_ros-dev"
PX4_DIR="${LOCAL_WS}/auxiliary/PX4-Autopilot"
POLKIT_RULE_FILE="/etc/polkit-1/rules.d/10-reset-usb.rules"
SUDOERS_FILE="/etc/sudoers.d/${USERNAME}_systemctl"
REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel)"
export WORKSPACES LOCAL_WS ISAAC_ROS_WS

# Traps (handlers in setup/io.sh)
trap 'failure ${LINENO}' ERR
trap quit_handler INT

# One continuous log per session: a resume reuses the path pinned in ~/.arid_setup_log.
LOG_DIR="${SCRIPT_DIR}/log"
mkdir -p "${LOG_DIR}"

# Keep only the newest $2 logs of prefix $1 in LOG_DIR (e.g. _prune_logs setup_log 10).
_prune_logs() {
    local files
    mapfile -t files < <(ls -1t "${LOG_DIR}/${1}_"*.log 2>/dev/null)
    (( ${#files[@]} > $2 )) && rm -f -- "${files[@]:$2}"
    return 0
}

if [[ " $* " == *" --resume "* || " $* " == *" --continue "* ]] && [[ -s "${HOME_DIR}/.arid_setup_log" ]]; then
    LOG_FILE="$(cat "${HOME_DIR}/.arid_setup_log" 2>/dev/null)"
fi
LOG_FILE="${LOG_FILE:-${LOG_DIR}/setup_log_$(date +%Y%m%d_%H%M%S).log}"
: >> "${LOG_FILE}" 2>/dev/null || LOG_FILE="${LOG_DIR}/setup_log_$(date +%Y%m%d_%H%M%S).log"
: >> "${LOG_FILE}"   # ensure it exists on disk so the prune counts it
exec > >(tee -a "${LOG_FILE}") 2>&1
echo "Logging to ${LOG_FILE}"
_prune_logs setup_log 10

# Argument parsing
parse_args() {
    for arg in "$@"; do
        case "$arg" in
            --full|-y)    RUN_FULL=1 ;;
            --resume)     RUN_RESUME=1 ;;
            --continue)   RUN_RESUME=1; FORCE_CONTINUE=1 ;;
            --help|-h)
                cat <<EOH
Usage: $0 [--full|--resume|--continue|--help]
  (no args)   Interactive menu: full setup or individual tools.
  --full      Run the questionnaire, then the full setup unattended.
  --resume    Continue setup after a reboot it required.
  --continue  Re-enter the setup tail; finished steps are skipped.
All steps auto-detect their current state and install / configure only what is
missing. Safe to re-run any time.
EOH
                exit 0 ;;
            *) err "Unknown argument: $arg"; exit 1 ;;
        esac
    done
}

# Sanity checks
preflight() {
    step "Sanity checks"

    [[ $EUID -eq 0 ]] && { err "Do not run as root."; exit 1; }
    command -v git >/dev/null || { err "git not found."; exit 1; }

    # Cache credentials up front so the ARK-OS clone authenticates once and the rest of
    # setup reuses it instead of re-prompting mid-run.
    git config --global credential.helper "cache --timeout=604800"

    if git -C "$REPO_ROOT" submodule status 2>/dev/null | grep -q "^-"; then
        err "Uninitialized submodules detected."
        err "Run: git submodule update --init --recursive"
        exit 1
    fi

    # Initialised submodules whose working tree was emptied still match their pin, so the
    # status check above misses them; they are restored in the Git step.
    local _p _emptied=()
    while IFS= read -r _p; do
        [[ -d "$REPO_ROOT/$_p" ]] || continue
        [[ "$(git -C "$REPO_ROOT/$_p" ls-tree -r --name-only HEAD 2>/dev/null | wc -l)" -gt 0 ]] || continue
        if [[ -z "$(find "$REPO_ROOT/$_p" -type f -not -path '*/.git/*' -print -quit 2>/dev/null)" ]]; then
            _emptied+=("$_p")
        fi
    done < <(git -C "$REPO_ROOT" config -f .gitmodules --get-regexp 'path' 2>/dev/null | awk '{print $2}')
    (( ${#_emptied[@]} )) && warn "emptied submodule(s): ${_emptied[*]} - will be restored in the Git step"

    ok "Sanity checks passed"
}

# Front RealSense serial -> vslam_config.yaml.
setup_realsense() {
    step "Front RealSense serial → vslam_config.yaml"

    # Reseed FIRST, unconditionally, before any camera / dependency / answer gate: an
    # updating drone must receive new template keys even when the assignment is skipped.
    bash "${WORKSPACES}/scripts/config_realsense.sh" --reseed-only || true

    # config_realsense.sh needs pyrealsense2 to query the serial by device.
    ensure_pip_pkg "pyrealsense2" "pyrealsense2" "" || warn "pyrealsense2 install failed - config_realsense degrades to rs-enumerate"
    # pip-satisfied is not the same as importable: an aarch64 wheel can install yet fail to load.
    python3 -c 'import pyrealsense2' >/dev/null 2>&1 \
        || warn "pyrealsense2 installed but not importable on the host - config_realsense will use rs-enumerate"

    if (( PRE )) && ! is_yes "${PRE_REALSENSE:-}"; then
        skip "serial assignment declined - template reseeded only"
        STEPS_SKIPPED+=("realsense")
        return
    fi

    if ! lsusb -d 8086: 2>/dev/null | grep -qi realsense; then
        warn "no RealSense on USB; skipping serial auto-detection"
        warn "after plugging it in, run 'config_realsense' manually"
        STEPS_SKIPPED+=("realsense")
        return
    fi

    if bash "${WORKSPACES}/scripts/config_realsense.sh"; then
        STEPS_RUN+=("realsense")
        ok "config_realsense succeeded"
    else
        warn "config_realsense failed; run it manually once the issue is resolved"
        STEPS_SKIPPED+=("realsense")
    fi
}

# Live cam_down feed check (questionnaire-driven).
verify_cameras() {
    step "Camera feed verification"

    if [[ "${PRE_VERIFY:-}" == "skip" ]]; then
        skip "camera verification (declined)"
        STEPS_SKIPPED+=("verify_cameras")
        return 0
    fi
    if (( PRE )) && ! is_yes "${PRE_VERIFY:-}"; then
        STEPS_SKIPPED+=("verify_cameras")
        return 0
    fi
    if (( ! PRE )) && ! ask_yn "  Verify the camera feed now? (y/n, Enter = skip): " n; then
        skip "camera verification skipped; run 'ver_cv_cams' manually later"
        STEPS_SKIPPED+=("verify_cameras")
        return 0
    fi

    if bash "${WORKSPACES}/scripts/verify_cv_cams.sh"; then
        STEPS_RUN+=("verify_cameras")
    else
        prompt_failure_action "camera verification incomplete" \
            "Likely cv2/rclpy not importable or no NoMachine session connected. Run 'ver_cv_cams' after fixing."
        STEPS_SKIPPED+=("verify_cameras")
    fi
}

verify_cameras_menu() {
    bash "${WORKSPACES}/scripts/verify_cv_cams.sh"
}

calibrate_cameras() {
    step "Camera calibration"
    local cal="${WORKSPACES}/local_ws/auxiliary/camera_calibration/camera_calibration_auto/camera_calibrate.sh"
    [[ -x "$cal" ]] || { skip "camera_calibrate.sh not found"; return; }
    bash "$cal" || warn "calibration did not complete"
}

# Foxglove + live cam_down feed; the operator tunes the lens focus by watching it.
camera_focus() {
    step "Camera focus"

    set +u
    source /opt/ros/humble/setup.bash 2>/dev/null
    [[ -f "${LOCAL_WS}/install/setup.bash" ]] && source "${LOCAL_WS}/install/setup.bash"
    set -u
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"

    if ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
        warn "foxglove_bridge not installed - run 'sudo apt install ros-humble-foxglove-bridge'"
        return
    fi
    # Focus is interactive; without a controlling terminal this would spin the camera up
    # unattended and never be told to stop.
    if [[ ! -t 0 ]]; then
        warn "no controlling terminal - skipping (run menu option 9 from a terminal)"
        return
    fi

    if ! ros2 service list 2>/dev/null | grep -q '^/gst_camera_manager/'; then
        if systemctl is-active --quiet gst_camera_manager.service 2>/dev/null; then
            echo "  gst_camera_manager.service active - waiting for it to register..."
        else
            warn "gst_camera_manager not running - starting gst_camera_manager.service"
            sudo systemctl start gst_camera_manager.service 2>/dev/null || true
        fi
        local i; for i in $(seq 1 30); do
            ros2 service list 2>/dev/null | grep -q '^/gst_camera_manager/' && break
            sleep 1
        done
    fi
    ros2 service list 2>/dev/null | grep -q '^/gst_camera_manager/' \
        || { warn "gst_camera_manager unavailable - aborting"; return; }

    # Stop first for a clean slate + let the sensor release: a pipeline started while the
    # camera is briefly held reports "started" but never streams.
    echo "  Starting cam_down..."
    ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1 || true
    sleep 2
    ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1 || true

    # A cold argus start can emit ONE frame then stall; a one-shot check would call that
    # "streaming" and leave the Foxglove panel blank. Require two frames, restart once.
    _stream_sustained() {   # $1 = topic, $2 = first-frame budget (s)
        timeout "$2" ros2 topic echo --once --qos-reliability best_effort "$1" >/dev/null 2>&1 || return 1
        timeout 4  ros2 topic echo --once --qos-reliability best_effort "$1" >/dev/null 2>&1
    }
    if _stream_sustained /cam_down/image_raw/compressed 22; then
        ok "cam_down streaming"
    else
        echo "    cam_down: no sustained stream yet (cold start) - restarting the pipeline once..."
        timeout 10 ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1 || true
        sleep 2
        timeout 10 ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool "{data: true}"  >/dev/null 2>&1 || true
        _stream_sustained /cam_down/image_raw/compressed 18 && ok "cam_down streaming" \
            || warn "cam_down: no live stream after warm-up + restart - sensor held (Isaac container sensor-id=0) or absent"
    fi

    # Own process group via job control (set -m): with `setsid cmd &`, $! is the exited
    # wrapper and every group-kill misses - the bridge leaked. Under -m, $! IS the leader.
    echo "  Starting Foxglove bridge..."
    set -m
    # shellcheck disable=SC2086
    ${FOXGLOVE_LAUNCH} </dev/null >/tmp/arid_focus_foxglove.log 2>&1 &
    local fox_pgid=$!
    set +m
    echo "${fox_pgid}" > /tmp/arid_focus_fox.pgid
    # Ctrl+C at the focus prompt lands in the global INT trap, which knows nothing about
    # this pgid. RETURN fires on every way out of the function; the teardown below clears
    # the trap first so the normal path does not run it twice.
    # shellcheck disable=SC2064
    trap "rm -f /tmp/arid_focus_fox.pgid
          kill -KILL -- -${fox_pgid} 2>/dev/null || true
          ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool '{data: false}' >/dev/null 2>&1 || true" RETURN
    sleep 3

    # setup runs under `exec > >(tee)`, which block-buffers stdout: as the last output
    # before a blocking read these lines would stay invisible. Write them to the terminal.
    local fox_port; fox_port=$(grep -oE 'port:=[0-9]+' <<<"${FOXGLOVE_LAUNCH}" | grep -oE '[0-9]+')
    fox_port="${fox_port:-8765}"
    local TTY=/dev/tty; { : >"${TTY}"; } 2>/dev/null || TTY=/dev/stdout
    {
        echo ""
        echo -e "  ${BOLD}Open Foxglove Studio -> Open connection -> Foxglove WebSocket. Connect to:${NC}"
        # Real NICs only - docker/bridge/virtual IPs are not reachable from a laptop.
        local ip found=0
        for ip in $(ip -4 -o addr show scope global 2>/dev/null \
                | awk '$2 !~ /^(docker|br-|veth|l4tbr|usb|virbr)/ {print $4}' | cut -d/ -f1); do
            echo -e "    ${GREEN}ws://${ip}:${fox_port}${NC}"; found=1
        done
        (( found )) || echo -e "  ${YELLOW}[WARN]${NC} no routable IP detected - check 'ip addr' (port ${fox_port})"
        echo "  Add an Image panel for: /cam_down/image_raw/compressed"
        echo ""
        echo "  Adjust focus in Foxglove, then press q to stop and return to the menu."
    } > "${TTY}"
    # Read from the terminal when we have one, else stdin (TTY fell back to /dev/stdout,
    # which is write-only and would end the wait instantly).
    if [[ "${TTY}" == "/dev/tty" ]]; then exec 7<"${TTY}"; else exec 7<&0; fi
    # Discard anything typed during startup so an early keypress does not exit immediately.
    while read -rsn1 -t 0.1 _ <&7 2>/dev/null; do :; done
    local key=""
    while [[ "$key" != "q" && "$key" != "Q" ]]; do read -rn1 key <&7 || break; done
    exec 7<&-

    trap - RETURN
    rm -f /tmp/arid_focus_fox.pgid
    echo "  Stopping Foxglove bridge and cam_down..."
    kill -INT  -- -"${fox_pgid}" 2>/dev/null || true
    sleep 1
    kill -KILL -- -"${fox_pgid}" 2>/dev/null || true
    ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1 || true
    ok "stopped"
}

# Summary
print_summary() {
    echo ""
    echo -e "${BOLD}======================================${NC}"
    echo -e "${BOLD}  Setup Complete${NC}"
    echo -e "${BOLD}======================================${NC}"
    echo -e "  Log:   ${LOG_FILE}"

    if [[ ${#STEPS_RUN[@]} -gt 0 ]]; then
        echo ""
        echo -e "  ${GREEN}Completed:${NC}"
        for s in "${STEPS_RUN[@]}"; do echo "    - ${s}"; done
    fi

    if [[ ${#STEPS_SKIPPED[@]} -gt 0 ]]; then
        echo ""
        echo -e "  ${YELLOW}Skipped:${NC}"
        for s in "${STEPS_SKIPPED[@]}"; do echo "    - ${s}"; done
    fi

    echo -e "${BOLD}======================================${NC}"
}

# Front-load every decision so the run is unattended; steps consult PRE_* instead of prompting.
collect_answers() {
    step "Setup questionnaire - answer once; setup then runs without further prompts"
    PRE=1

    if [[ -f "${HOME_DIR}/.arid_provisioned" ]]; then
        echo "  Already provisioned - hostname / password unchanged."
    else
        ask_yn "  Set hostname? (y/n, Enter = arid): " n \
            && { read -r -p "    Hostname: " PRE_HOST || PRE_HOST=""; }
        if ask_yn "  Set password? (y/n, Enter = leave unchanged): " n; then
            read -r -s -p "    Password: " PRE_PASS || PRE_PASS=""; echo ""
        fi
    fi

    if nmcli -t -f TYPE,STATE device status 2>/dev/null | grep -q '^wifi:connected'; then
        local cur
        cur=$(nmcli -t -f NAME,TYPE connection show --active 2>/dev/null \
            | awk -F: '/:802-11-wireless$/{print $1; exit}')
        ok "Wi-Fi: connected${cur:+ to '${cur}'}"
    else
        warn "Wi-Fi: not connected"
    fi
    if ask_yn "  Connect to Wi-Fi? (y/n, Enter = skip): " n; then
        read -r -p "    SSID: " PRE_WIFI_SSID || PRE_WIFI_SSID=""
        read -r -s -p "    Password (empty = open): " PRE_WIFI_PASS || PRE_WIFI_PASS=""; echo ""
        PRE_WIFI=yes
    else
        PRE_WIFI=skip
    fi

    # Never offer a NoMachine reinstall from inside a NoMachine session: it drops this session.
    if is_inside_nomachine; then
        PRE_NOMACHINE=skip
    elif dpkg -s nomachine >/dev/null 2>&1 || [[ -x /usr/NX/bin/nxserver ]]; then
        ask_yn "  Reinstall NoMachine? (y/n, Enter = skip): " n && PRE_NOMACHINE=yes || PRE_NOMACHINE=skip
    else
        PRE_NOMACHINE=install
    fi

    # ARK-OS - only ask to reinstall if already present; otherwise it installs unconditionally.
    if ark_installed; then
        ask_yn "  Reinstall ARK-OS? (y/n, Enter = skip): " n && PRE_ARK=yes || PRE_ARK=skip
    else
        PRE_ARK=install
    fi

    # ROS2 - isolated; ask to reinstall if present, else install. A failed install leaves
    # /opt/ros/humble and even its setup.bash behind, so test the ros2 executable.
    if [[ -x /opt/ros/humble/bin/ros2 ]]; then
        ask_yn "  Reinstall ROS2? (y/n, Enter = skip): " n && PRE_ARK_ROS2=yes || PRE_ARK_ROS2=skip
    else
        PRE_ARK_ROS2=install
    fi

    # JetPack: offer reinstall if present, else install it. Installing forces a post-ARK reboot.
    local jp; jp=$(detect_jetpack_version)
    if [[ -n "$jp" ]]; then
        ask_yn "  ${jp} detected. Reinstall JetPack during ARK-OS? (y/n, Enter = skip): " n && PRE_JETPACK=yes || PRE_JETPACK=skip
    else
        warn "JetPack not detected - ARK-OS will install it."
        PRE_JETPACK=yes
    fi
    if [[ "$PRE_JETPACK" == "yes" ]]; then
        PRE_POST_ARK_REBOOT=yes
        ok "JetPack will be installed - the unit reboots afterward to activate it, then setup resumes."
    else
        PRE_POST_ARK_REBOOT=no
    fi

    # Validated here so a typo is caught at questionnaire time; setup never switches
    # networks (that is zt_join's job).
    if command -v zerotier-cli >/dev/null 2>&1 \
       && sudo zerotier-cli -j listnetworks 2>/dev/null | grep -q '"nwid"'; then
        PRE_ZTNET="skip"
        echo "  ZeroTier: already on a network - skipping (use 'zt_join' to switch)"
    else
        while true; do
            read -r -p "  ZeroTier network id to join (16 hex chars, Enter = skip): " PRE_ZTNET || PRE_ZTNET=""
            PRE_ZTNET="${PRE_ZTNET// /}"
            [[ -z "${PRE_ZTNET}" ]] && PRE_ZTNET="skip" && break
            [[ "${PRE_ZTNET}" =~ ^[0-9a-fA-F]{16}$ ]] && break
            warn "  invalid network id (need 16 hex chars, or Enter to skip)"
        done
    fi

    if command -v arm-none-eabi-gcc >/dev/null 2>&1; then
        PRE_PX4=present
    else
        ask_yn "  Install PX4 toolchain? (y/n, Enter = no): " n && PRE_PX4=yes || PRE_PX4=no
    fi

    ask_yn "  Assign RealSense cameras? (y/n, Enter = skip): " n \
        && PRE_REALSENSE=yes || PRE_REALSENSE=skip

    ask_yn "  Verify the camera feed at the end? (y/n, Enter = skip): " n \
        && PRE_VERIFY=yes || PRE_VERIFY=skip

    ask_yn "  Focus the down camera after verification? (y/n, Enter = skip): " n \
        && PRE_FOCUS=yes || PRE_FOCUS=skip

    echo "  Isaac builds are long - run over NoMachine (survives disconnect) or skip and"
    echo "  run 'build_isaac' / 'colcon_isaac' later."
    local q
    if docker image inspect isaac_ros_dev-aarch64 >/dev/null 2>&1; then
        q="  Rebuild the Isaac container? (y/n, Enter = skip): "
    else
        q="  Build the Isaac container? (y/n, Enter = skip): "
    fi
    if ask_yn "${q}" n; then
        PRE_BUILD_ISAAC=yes
        touch "${HOME_DIR}/.arid_pending_build_isaac"
    else
        PRE_BUILD_ISAAC=skip
        rm -f "${HOME_DIR}/.arid_pending_build_isaac"
    fi

    # Isaac workspace colcon, whenever an image exists or is queued.
    if [[ "${PRE_BUILD_ISAAC}" == "yes" ]] || docker image inspect isaac_ros_dev-aarch64 >/dev/null 2>&1; then
        if [[ -f "${ISAAC_ROS_WS}/install/setup.bash" ]]; then
            # Enter = rebuild: pulled packages and C++ ship nothing until colcon runs, and
            # skip-by-default once left a launch pointing at an unbuilt package.
            ask_yn "  Rebuild the Isaac workspace? (y/n, Enter = rebuild): " y && PRE_COLCON=yes || PRE_COLCON=skip
        else
            ask_yn "  Build the Isaac workspace? (y/n, Enter = yes): " y && PRE_COLCON=yes || PRE_COLCON=skip
        fi
    else
        PRE_COLCON=skip
    fi

    if [[ -d "${LOCAL_WS}/install" ]]; then
        ask_yn "  Rebuild the local workspace? (y/n, Enter = skip): " n && PRE_LOCAL_WS=yes || PRE_LOCAL_WS=skip
    else
        ask_yn "  Build the local workspace? (y/n, Enter = yes): " y && PRE_LOCAL_WS=yes || PRE_LOCAL_WS=skip
    fi

    ask_yn "  Run the smoke test at the end? (y/n, Enter = y): " y && PRE_SMOKE=yes || PRE_SMOKE=skip

    # The smoke test is a clean-boot validation, so setup reboots before it when anything
    # built. Answer n to run it inline instead and finish without a reboot.
    ask_yn "  Reboot before the smoke test? (y/n, Enter = y): " y && PRE_REBOOT=yes || PRE_REBOOT=no

    ok "Answers recorded - running unattended."

    _persist_questionnaire
}

# Persist PRE_* for run_resume (mode 600). Secrets (PRE_PASS, PRE_WIFI_PASS) are deliberately
# NOT persisted: they gate steps that only run before the reboot.
_persist_questionnaire() {
    local f="${HOME_DIR}/.arid_questionnaire"
    umask 077
    {
        echo "# Generated by setup.sh collect_answers - sourced by run_resume."
        echo "PRE=1"
        for var in PRE_HOST PRE_WIFI PRE_WIFI_SSID PRE_NOMACHINE PRE_PX4 PRE_ZTNET \
                   PRE_ARK PRE_ARK_ROS2 PRE_JETPACK PRE_POST_ARK_REBOOT \
                   PRE_REALSENSE PRE_VERIFY PRE_FOCUS \
                   PRE_BUILD_ISAAC PRE_COLCON PRE_LOCAL_WS PRE_SMOKE PRE_REBOOT; do
            printf '%s=%q\n' "$var" "${!var-}"
        done
    } > "$f"
}

# Undo the host-side state setup.sh creates. Never called from run_full_setup or run_resume.
setup_uninstall() {
    echo ""
    echo -e "${RED}${BOLD}============== ARID uninstall ==============${NC}"
    echo "  This will stop and remove every systemd unit installed by setup.sh,"
    echo "  delete /etc/sudoers.d/jetson_systemctl, the polkit rule, the rslidar"
    echo "  NetworkManager profiles + dispatcher, the rslidar sysctl drop-in, the"
    echo "  ARID block in ~/.bashrc, the docker patches inside isaac_ros_common, every"
    echo "  setup.sh sentinel file, and the Isaac container + image."
    echo ""
    echo -e "${BOLD}Left in place:${NC} the repo clone, hostname, password, group memberships,"
    echo "  apt-mark holds, ROS 2 Humble, JetPack, the Docker engine itself, chrony, uhubctl,"
    echo "  and any other state setup.sh did not write."
    echo -e "${RED}${BOLD}============================================${NC}"
    local ans
    read -r -p "Type 'yes' to proceed: " ans || ans=""
    if [[ "${ans}" != "yes" ]]; then
        echo "  Aborted."
        return 0
    fi

    step "Stop + disable + remove systemd units"
    # Derived from the shipped unit files so this list cannot drift from setup_systemd's,
    # plus the pre-rename supervisor unit older drones may still carry.
    local units=() u con
    mapfile -t units < <(arid_units_shipped)
    units+=(vslam_supervisor.service)
    for u in "${units[@]}"; do
        sudo -n systemctl stop    "$u" 2>/dev/null || true
        sudo -n systemctl disable "$u" 2>/dev/null || true
        sudo rm -f "/etc/systemd/system/$u" 2>/dev/null || true
    done
    sudo rm -f /etc/systemd/system.conf.d/10-arid-ros-env.conf 2>/dev/null || true
    sudo systemctl daemon-reexec 2>/dev/null || true
    sudo systemctl daemon-reload 2>/dev/null || true
    for u in "${units[@]}"; do sudo -n systemctl reset-failed "$u" 2>/dev/null || true; done
    ok "systemd units removed: ${units[*]}"

    step "Remove NetworkManager LiDAR config"
    for con in rslidar dev; do
        nmcli -t -f NAME connection show 2>/dev/null | grep -qxF "$con" \
            && sudo nmcli connection delete "$con" >/dev/null 2>&1 || true
    done
    sudo rm -f "${RSLIDAR_DISPATCHER}" 2>/dev/null || true
    ok "NM profiles + dispatcher removed"

    step "Remove sysctl drop-in"
    sudo rm -f "${RSLIDAR_SYSCTL}" 2>/dev/null || true
    sudo sysctl --system >/dev/null 2>&1 || true
    ok "sysctl drop-in removed"

    step "Remove sudoers, polkit, udev"
    sudo rm -f "${SUDOERS_FILE}" 2>/dev/null || true
    sudo rm -f "${POLKIT_RULE_FILE}" 2>/dev/null || true
    sudo rm -f /etc/udev/rules.d/52-usb.rules /etc/udev/rules.d/99-gpio.rules 2>/dev/null || true
    sudo udevadm control --reload-rules 2>/dev/null || true
    ok "Sudoers + polkit + udev rules removed"

    step "Strip ARID block from ~/.bashrc"
    sed -i '/# BEGIN ARID SETUP/,/# END ARID SETUP/d' "$BASHRC_FILE"
    ok "bashrc cleaned"

    step "Un-skip-worktree config + docker patches"
    local marked f
    marked=$(git -C "$REPO_ROOT" ls-files -v 2>/dev/null | awk '/^S /{sub(/^S /, ""); print}' || true)
    while IFS= read -r f; do
        [[ -n "$f" ]] || continue
        git -C "$REPO_ROOT" update-index --no-skip-worktree -- "$f" 2>/dev/null || true
    done <<< "$marked"
    if [[ -e "${ISAAC_ROS_WS}/src/isaac_ros_common/.git" ]]; then
        git -C "${ISAAC_ROS_WS}/src/isaac_ros_common" update-index --no-skip-worktree \
            scripts/.isaac_ros_common-config \
            scripts/run_dev.sh 2>/dev/null || true
        git -C "${ISAAC_ROS_WS}/src/isaac_ros_common" checkout -- \
            scripts/.isaac_ros_common-config \
            scripts/run_dev.sh 2>/dev/null || true
        rm -f "${ISAAC_ROS_WS}/src/isaac_ros_common/docker/Dockerfile.arid" 2>/dev/null || true
        rm -f "${ISAAC_ROS_WS}/src/isaac_ros_common/docker/scripts/arid_env.sh" 2>/dev/null || true
    fi
    ok "skip-worktree marks cleared + isaac_ros_common patches reverted"

    step "Remove sentinels"
    rm -f \
        "${HOME_DIR}/.arid_provisioned" \
        "${HOME_DIR}/.arid_pending_build_isaac" \
        "${HOME_DIR}/.arid_resume_setup" \
        "${HOME_DIR}/.arid_setup_continue" \
        "${HOME_DIR}/.arid_run_smoke" \
        "${HOME_DIR}/.arid_did_build" \
        "${HOME_DIR}/.arid_progress" \
        "${HOME_DIR}/.arid_ark_os_installed" \
        "${HOME_DIR}/.arid_resume.lock" \
        "${HOME_DIR}/.arid_setup_log" \
        "${HOME_DIR}/.arid_questionnaire" 2>/dev/null || true
    ok "sentinel files removed"

    step "Remove Isaac container + image"
    docker rm -f isaac_ros_dev-aarch64-container 2>/dev/null || true
    docker rmi -f isaac_ros_dev-aarch64 2>/dev/null || true
    docker images --format '{{.Repository}}:{{.Tag}}' 2>/dev/null \
        | grep -E '^isaac_ros_dev-aarch64' \
        | xargs -r docker rmi -f 2>/dev/null || true
    ok "container + image removed"

    echo ""
    echo -e "${GREEN}${BOLD}Uninstall complete.${NC}"
    echo "  Open a new shell to drop the ARID environment from your current bashrc."
    echo "  The repo clone, hostname, and password are unchanged."
}

# Recover the Docker + supervisor boot units and wait for the supervisor to advertise, so the
# smoke test sees a healthy graph on a cold boot. Idempotent; safe to re-run.
_recover_docker_supervisor() {
    if ! docker image inspect isaac_ros_dev-aarch64 >/dev/null 2>&1; then
        warn "Isaac container image not available - skipping supervisor wait."
        warn "Re-run 'build_isaac' once the underlying issue is resolved, then:"
        warn "sudo systemctl restart start_isaac_docker.service arid_supervisor.service"
        return 0
    fi

    # reset-failed clears systemd's StartLimit counter so on-failure backoff cannot suppress
    # a manual start; stop then start flushes in-flight restart timers.
    step "Recovering Docker + supervisor units (if not yet running)"
    sudo -n systemctl reset-failed start_isaac_docker.service arid_supervisor.service 2>/dev/null || true
    sudo -n systemctl stop  start_isaac_docker.service arid_supervisor.service 2>/dev/null || true
    sudo -n systemctl start start_isaac_docker.service 2>/dev/null || warn "start_isaac_docker.service start failed"

    # The supervisor needs the in-container workspace built.
    local w ws_built=0
    for w in $(seq 1 10); do
        docker exec -u admin isaac_ros_dev-aarch64-container \
            test -f /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null && { ws_built=1; break; }
        sleep 2
    done
    if (( ws_built == 0 )); then
        warn "Isaac workspace not built (no install/setup.bash) - skipping supervisor; run 'colcon_isaac'."
        return 0
    fi
    sudo -n systemctl start arid_supervisor.service 2>/dev/null || warn "arid_supervisor.service start failed"

    step "Waiting for arid_supervisor to advertise (up to 90 s)"
    set +u
    [[ -f /opt/ros/humble/setup.bash ]] && source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=23
    set -u
    local i ok_sup=0
    for i in $(seq 1 18); do
        if timeout 5 ros2 service list 2>/dev/null | grep -qx /arid_supervisor/vslam_enable; then
            ok_sup=1; break
        fi
        (( i % 3 == 0 )) && echo "    waiting... (${i}/18, $((i * 5)) s elapsed)"
        sleep 5
    done
    if (( ok_sup )); then
        ok "supervisor service on the graph"
    else
        warn "supervisor did not advertise within 90 s - smoke test Section 6 may report failures"
    fi
}

# Final post-install gate: services active, aliases, cam_down + rslidar lifecycles, supervisor graph.
run_smoke_test() {
    step "Running local smoke test"
    local rc=0
    # Separate smoke-test log alongside the setup log; output still streams to the main log + screen.
    local smoke_log="${LOG_DIR}/smoke_test_log_$(date +%Y%m%d_%H%M%S).log"
    echo "  Smoke test log: ${smoke_log}"
    printf 'ARID smoke test - %s\n\n' "$(date '+%Y-%m-%d %H:%M:%S')" > "${smoke_log}"
    if bash "${WORKSPACES}/scripts/local_test.sh" 2>&1 | tee -a "${smoke_log}"; then rc=0; else rc=${PIPESTATUS[0]}; fi
    _prune_logs smoke_test_log 10
    if (( rc == 0 )); then
        STEPS_RUN+=("smoke_test")
        ok "Smoke test passed"
    else
        STEPS_RUN+=("smoke_test (failures)")
        warn "Smoke test reported failures - see output above"
    fi
    return "${rc}"
}

# Orchestration
run_full_setup() {
    preflight
    printf '%s\n' "${LOG_FILE}" > "${HOME_DIR}/.arid_setup_log"   # pin this session's log across its reboot
    # Fresh run: nothing checkpointed, no build recorded (a stale .arid_did_build would
    # otherwise force the pre-smoke reboot even when nothing builds this run).
    rm -f "${HOME_DIR}/.arid_progress" "${HOME_DIR}/.arid_did_build"
    collect_answers
    first_boot
    setup_power
    disable_updates
    enable_user_linger
    clean_nvidia_desktop
    ensure_wifi
    nomachine
    ark_os            # ARK-OS + ROS2 + JetPack; everything below depends on /opt/ros/humble
    _run_setup_tail   # checkpointed steps; a resume re-enters here
}

# Run one tail section, then checkpoint it - a resume skips finished sections.
run_step() {
    local name="$1"
    grep -qxF "${name}" "${HOME_DIR}/.arid_progress" 2>/dev/null && { skip "${name} already done"; return 0; }
    "${name}"
    echo "${name}" >> "${HOME_DIR}/.arid_progress"
}

# PHASE A - install + host config. Every section is checkpointed, so a resume never
# re-runs or re-prompts a finished one.
_run_phase_a_install() {
    # ARK's dds-agent install (Fast-DDS) locks /usr/local + include + lib to 0700; reopen for non-root.
    sudo chmod -R a+rX /usr/local 2>/dev/null || true
    run_step setup_repos
    run_step setup_apt_packages
    run_step enable_clock_sync   # after apt: chrony must be installed before it can replace timesyncd
    # A ZeroTier failure is not a provisioning failure (join later via zt_join); never abort.
    run_step setup_zerotier || true
    run_step setup_px4_deps
    run_step setup_git
    run_step setup_docker_patches
    run_step setup_skip_worktree
    run_step setup_bashrc
    run_step setup_permissions
    run_step setup_uhubctl
    run_step setup_lidar_sysctl
    run_step setup_lidar_network
    run_step setup_ros_workspace   # BUILD: host colcon of local_ws
    # Docker must precede setup_systemd: a mid-run abort would otherwise leave the boot
    # unit pointing at an absent daemon.
    run_step setup_docker
    run_step setup_systemd
    run_step setup_realsense
}

# PHASE B - container builds + live checks. Not checkpointed: these are the steps an
# operator re-runs deliberately.
_run_phase_b_build() {
    _run_build_isaac_if_queued || true   # BUILD: Isaac container image
    colcon_isaac_step || true            # BUILD: in-container colcon
    verify_cameras
    if is_yes "${PRE_FOCUS:-}"; then camera_focus || true; fi
}

_run_setup_tail() {
    _run_phase_a_install

    # Install reboot: exactly once at the end of Phase A, iff ARK-OS / ROS2 / JetPack installed
    # this run. It is armed with .arid_setup_continue, so the resume re-enters the tail; run_step
    # skips every finished Phase A section (ark_os runs before the tail and is never re-entered),
    # INSTALL_GROUP_REBOOT stays 0 on that pass, and the run falls through to Phase B - no loop.
    if (( INSTALL_GROUP_REBOOT )); then
        _do_reboot "${HOME_DIR}/.arid_setup_continue" && return 0
    fi

    _run_phase_b_build
    _finish_with_smoke_test
}

# The smoke test validates a clean boot, so it runs after a reboot whenever anything built.
# Nothing built (or the operator declined the reboot) -> run it inline and finish.
_finish_with_smoke_test() {
    # .arid_did_build is the persisted "a build actually ran" signal; it survives the reboot.
    [[ -f "${HOME_DIR}/.arid_did_build" ]] && DID_BUILD=1

    print_summary

    if (( DID_BUILD )) && [[ "${PRE_SMOKE:-}" != "skip" ]] && [[ "${PRE_REBOOT:-yes}" != "no" ]]; then
        _do_reboot "${HOME_DIR}/.arid_run_smoke" && return 0
    fi

    if [[ "${PRE_SMOKE:-}" == "skip" ]]; then
        step "Smoke test skipped (per questionnaire)"
    else
        _recover_docker_supervisor
        run_smoke_test || true
    fi
    rm -f "${HOME_DIR}/.arid_setup_log" "${HOME_DIR}/.arid_did_build"   # session done
    echo ""
    echo -e "${BOLD}Setup complete.${NC}"
}

# Arm the resume hook + the stage marker the resume reads, then reboot. Returns 0 on a reboot
# the caller should `return` after; non-zero (rolled back) means continue inline.
_do_reboot() {
    local marker="$1"
    echo ""
    echo -e "${YELLOW}${BOLD}Rebooting in a few seconds. Setup will resume on the next terminal session.${NC}"
    sync; sleep 3   # render the message before reboot
    # Install the resume hook first, else a reboot leaves an armed flag with no bashrc hook.
    ensure_resume_hook
    touch "${HOME_DIR}/.arid_resume_setup" "${marker}"
    if ! sudo -n reboot 2>/dev/null && ! sudo reboot; then
        warn "sudo reboot failed - rolling back resume flags; continuing inline"
        rm -f "${HOME_DIR}/.arid_resume_setup" "${marker}"
        return 1
    fi
}

# Resume after a reboot. Stage markers disambiguate where to re-enter:
#   .arid_setup_continue -> re-enter the tail (run_step skips finished Phase A steps); armed by
#                           the install reboot at the end of Phase A and by --continue.
#   .arid_run_smoke      -> the pre-smoke reboot: recovery + smoke test only.
# Neither -> a manual `--resume`: camera verification, queued builds, recovery, smoke test.
run_resume() {
    # flock serialises racing resume terminals; the lock file is deliberately never
    # removed (flock binds the inode).
    local lock="${HOME_DIR}/.arid_resume.lock"
    if ! exec 9>"${lock}" 2>/dev/null; then
        warn "could not open resume lock at ${lock} - continuing without serialisation"
    elif ! flock -n 9; then
        # Return non-zero so the bashrc hook leaves the resume flag for whoever is running.
        echo "Another resume is already in progress in another terminal - exiting."
        return 1
    fi

    # Restore PRE_* answers; missing file = manual --resume, run the default flow.
    if [[ -r "${HOME_DIR}/.arid_questionnaire" ]]; then
        # shellcheck disable=SC1090
        set +u
        source "${HOME_DIR}/.arid_questionnaire" || true
        set -u
    fi

    # Clear the marker BEFORE running so that if the pre-smoke reboot fires inside the tail
    # we resume via .arid_run_smoke, not here.
    if [[ -f "${HOME_DIR}/.arid_setup_continue" ]] || (( FORCE_CONTINUE )); then
        step "Continuing setup after reboot"
        PRE=1
        rm -f "${HOME_DIR}/.arid_setup_continue"
        _run_setup_tail
        return 0
    fi

    local post_reboot_smoke=0
    if [[ -f "${HOME_DIR}/.arid_run_smoke" ]]; then
        step "Continuing setup after reboot - running the smoke test"
        PRE=1
        post_reboot_smoke=1
        rm -f "${HOME_DIR}/.arid_run_smoke"
    fi

    # Phase B already ran before the pre-smoke reboot - don't redo it.
    if (( post_reboot_smoke )); then
        :
    elif (( ${PRE:-0} )) && ! is_yes "${PRE_VERIFY:-}"; then
        step "Continuing setup - camera verification skipped (per questionnaire)"
    else
        step "Continuing setup - camera verification"
        if bash "${WORKSPACES}/scripts/verify_cv_cams.sh"; then
            ok "camera verification complete"
        else
            warn "camera verification incomplete - run 'ver_cv_cams' to retry"
        fi
    fi

    if (( ! post_reboot_smoke )); then
        if is_yes "${PRE_FOCUS:-}"; then camera_focus || true; fi
        _run_build_isaac_if_queued || true
        if docker image inspect isaac_ros_dev-aarch64 >/dev/null 2>&1; then
            sudo -n systemctl reset-failed start_isaac_docker.service 2>/dev/null || true
            sudo -n systemctl start start_isaac_docker.service 2>/dev/null || true
            PRE=1
            colcon_isaac_step || true
        fi
    fi

    _recover_docker_supervisor

    local smoke_rc=0
    if [[ "${PRE_SMOKE:-}" == "skip" ]]; then
        step "Smoke test skipped (per questionnaire)"
    else
        run_smoke_test || smoke_rc=$?
    fi

    # Keep the questionnaire on failure so a retried resume sees the same answers.
    if (( smoke_rc == 0 )); then
        rm -f "${HOME_DIR}/.arid_questionnaire" "${HOME_DIR}/.arid_setup_log" \
              "${HOME_DIR}/.arid_did_build"
    fi

    echo ""
    echo -e "${BOLD}Setup complete.${NC}"
    return "${smoke_rc}"
}

# Interactive menu; a non-zero tool exit returns to the menu rather than aborting.
# 'q' clears every resume hook so quitting does not auto-resume on the next terminal.
menu() {
    while true; do
        echo ""
        echo -e "${BOLD}============== ARID setup ==============${NC}"
        echo "  1) Full setup"
        echo "  2) Smoke test"
        echo "  3) RealSense assignment"
        echo "  4) Camera feed check"
        echo "  5) LiDAR diagnostic"
        echo "  6) LiDAR auto-detect"
        echo "  7) Wi-Fi connect"
        echo "  8) Camera calibration"
        echo "  9) Camera focus"
        echo "  10) Build Isaac container"
        echo "  11) Build Isaac workspace"
        echo "  12) Build local workspace"
        echo "  13) Install ARK-OS"
        echo "  14) Install ROS2"
        echo "  15) ZeroTier join/switch"
        echo "  16) Uninstall"
        echo "  q) Quit"
        echo -e "${BOLD}========================================${NC}"
        echo "  (Ctrl+C ends setup at any time)"
        local choice
        read -r -p "  Select: " choice || choice="q"
        case "${choice}" in
            1) run_full_setup; break ;;
            2) run_smoke_test                                    || true ;;
            3) bash "${WORKSPACES}/scripts/config_realsense.sh"  || true ;;
            4) verify_cameras_menu                               || true ;;
            5) bash "${WORKSPACES}/scripts/lidar_diag.sh"        || true ;;
            6) sudo bash "${WORKSPACES}/scripts/config_lidar.sh" || true ;;
            7) bash "${WORKSPACES}/scripts/wifi.sh"              || true ;;
            8) calibrate_cameras                                 || true ;;
            9) camera_focus                                      || true ;;
            10) build_isaac_step                                 || true ;;
            11) colcon_isaac_step                                || true ;;
            12) setup_ros_workspace                              || true ;;
            13) menu_install_ark                                 || true ;;
            14) menu_install_ros2                                || true ;;
            15) PRE=0 setup_zerotier                             || true ;;
            16) setup_uninstall                                  || true ;;
            q|Q) echo "  Quit."; cleanup_user_exit; break ;;
            *) warn "invalid selection: '${choice}'" ;;
        esac
    done
}

main() {
    exec 8>"${HOME_DIR}/.arid_setup.lock"
    flock -n 8 || { err "another setup.sh is already running"; exit 1; }
    parse_args "$@"
    if (( RUN_RESUME )); then
        run_resume
    elif (( RUN_FULL )); then
        run_full_setup
    else
        menu
    fi
}

main "$@"
