#!/bin/bash
# ARID workspace setup.
# Usage: ./setup.sh [--full|--resume|--continue|--help]
# Every step is idempotent; safe to re-run any time.
#
# Domain steps live under setup/*.sh. This file is orchestration only.
set -euo pipefail

# SOURCE DOMAIN LIBRARIES
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP_DIR="${SCRIPT_DIR}/setup"

# io.sh first (defines failure, quit_handler, helpers).
# shellcheck source=setup/io.sh
source "${SETUP_DIR}/io.sh"
for _lib in system network ark container; do
    # shellcheck disable=SC1090
    source "${SETUP_DIR}/${_lib}.sh"
done
unset _lib

# GLOBAL STATE
STEPS_RUN=()
STEPS_SKIPPED=()
DID_BUILD=0         # reboot gate: set when any build (local_ws, Isaac image, colcon_isaac) ran
INSTALL_GROUP_REBOOT=0  # install reboot gate: set by ark_os when ARK-OS / ROS2 / JetPack installed
RUN_FULL=0          # --full: skip the menu and run the whole setup
RUN_RESUME=0        # --resume: continue setup after a reboot
FORCE_CONTINUE=0    # --continue: re-enter the setup tail with no marker file

# Single source of truth for the Foxglove launch (alias + camera-focus both use it).
FOXGLOVE_LAUNCH="ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"

# Pre-collected answers (full setup only). PRE=1 means "use these, do not prompt mid-run".
PRE=0
PRE_HOST=""; PRE_PASS=""
PRE_WIFI=""; PRE_WIFI_SSID=""; PRE_WIFI_PASS=""
PRE_NOMACHINE=""; PRE_PX4=""; PRE_ZTNET=""
PRE_ARK=""; PRE_ARK_ROS2=""; PRE_JETPACK=""; PRE_POST_ARK_REBOOT=""
PRE_REALSENSE=""; PRE_VERIFY=""; PRE_FOCUS=""; PRE_SMOKE=""
PRE_BUILD_ISAAC=""; PRE_COLCON=""; PRE_LOCAL_WS=""; PRE_REBOOT=""

# CONSTANTS
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

# TRAPS  (functions defined in setup/io.sh)
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

# ARGUMENT PARSING
parse_args() {
    for arg in "$@"; do
        case "$arg" in
            --full|-y)    RUN_FULL=1 ;;
            --resume)     RUN_RESUME=1 ;;
            --continue)   RUN_RESUME=1; FORCE_CONTINUE=1 ;;   # re-enter the tail, no marker needed
            --help|-h)
                cat <<EOH
Usage: $0 [--full|--resume|--continue|--help]
  (no args)   Interactive menu: full setup or individual tools.
  --full      Run the questionnaire, then the full setup unattended.
  --resume    Continue setup after a reboot it armed.
  --continue  Re-enter the setup tail (finished steps are skipped) after an abort.
All steps auto-detect their current state and install / configure only what is
missing. Safe to re-run any time.
EOH
                exit 0 ;;
            *) err "Unknown argument: $arg"; exit 1 ;;
        esac
    done
}

# SANITY CHECKS
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

    # Initialised but emptied working trees (files deleted while HEAD still matches the pin)
    # aren't flagged above; they are restored by update_submods.sh in the Git step.
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

# RealSense serial assignment (front/left/right)
setup_realsense() {
    step "RealSense serial mapping (front/left/right)"

    # Reseed FIRST, unconditionally (before the camera/dep/skip gates): an updating drone
    # must receive new template keys or camera-loss tolerance reverts to stock.
    bash "${WORKSPACES}/scripts/config_realsense.sh" --reseed-only || true

    # config_realsense.sh needs pyrealsense2 to stream each camera by serial.
    ensure_pip_pkg "pyrealsense2" "pyrealsense2" "" || warn "pyrealsense2 install failed - config_realsense degrades to rs-enumerate"
    # pip-satisfied is not the same as importable: an aarch64 wheel can install yet fail to load.
    python3 -c 'import pyrealsense2' >/dev/null 2>&1 \
        || warn "pyrealsense2 installed but not importable on the host - config_realsense will skip"

    local n
    n=$(lsusb -d 8086: 2>/dev/null | grep -ic realsense || true)
    if [[ "${n}" -eq 0 ]]; then
        warn "no RealSense on USB; skipping serial assignment"
        warn "after plugging the cameras in, run 'config_realsense' manually"
        STEPS_SKIPPED+=("realsense")
        return
    fi
    [[ "${n}" -ne 3 ]] && warn "found ${n} RealSense on USB (expected 3) - continuing anyway"

    # Pass the questionnaire answer through so config_realsense.sh does not re-prompt.
    local rs_env=()
    (( PRE )) && rs_env=(ARID_RS_ASSIGN="${PRE_REALSENSE}")
    local rc=0
    env "${rs_env[@]}" bash "${WORKSPACES}/scripts/config_realsense.sh" || rc=$?
    if (( rc == 0 )); then
        STEPS_RUN+=("realsense"); ok "config_realsense completed"
    elif (( rc == 2 )); then
        # Name the dep that actually failed (mirror config_realsense's ROS-sourced import env).
        local _miss
        _miss=$(bash -c '[[ -f /opt/ros/humble/setup.bash ]] && source /opt/ros/humble/setup.bash 2>/dev/null
            m=(); python3 -c "import cv2" 2>/dev/null || m+=(cv2)
            python3 -c "import pyrealsense2" 2>/dev/null || m+=(pyrealsense2); echo "${m[*]}"')
        warn "RealSense host dep not importable: ${_miss:-cv2/pyrealsense2} - run 'config_realsense' after fixing"
        STEPS_SKIPPED+=("realsense")
    else
        warn "config_realsense did not complete; run 'config_realsense' manually later"
        STEPS_SKIPPED+=("realsense")
    fi
}

# Camera feed verification (full-setup step): both IMX219 feeds unless declined.
verify_cameras() {
    step "Camera feed verification"

    local ans
    if (( PRE )); then ans="${PRE_VERIFY}"; else ask_yn "  Verify the camera feeds (front + down)? (y/n, Enter = skip): " n && ans=yes || ans=skip; fi
    if ! is_yes "${ans}"; then
        skip "camera verification skipped; run 'ver_cv_cams' manually later"
        STEPS_SKIPPED+=("verify_cameras")
        return
    fi

    if bash "${WORKSPACES}/scripts/verify_cv_cams.sh" both; then
        STEPS_RUN+=("verify_cameras")
    else
        prompt_failure_action "camera verification incomplete" \
            "Likely cv2/rclpy not importable or no NoMachine session connected. Run 'ver_cv_cams' after fixing."
        STEPS_SKIPPED+=("verify_cameras")
    fi
}

# Camera feed check (menu option): discrete front / down / both selection.
verify_cameras_menu() {
    step "Camera feed check"
    echo "  Check which camera?   1) front (IMX219)   2) down (IMX219)   3) both   (q/Enter = back to menu)"
    local pick
    read -rn1 -p "  Select: " pick || pick=""; echo ""
    pick="${pick//[^123]/}"
    [[ -z "$pick" ]] && { skip "camera check skipped"; return; }
    local sel; case "$pick" in 1) sel=front ;; 2) sel=down ;; 3) sel=both ;; esac
    bash "${WORKSPACES}/scripts/verify_cv_cams.sh" "$sel" || warn "camera check incomplete"
}

# Camera calibration (menu option): venv-isolated cameracalibrator against a live pipeline.
calibrate_cameras() {
    step "Camera calibration"
    local which=""
    while [[ "${which}" != "front" && "${which}" != "down" ]]; do
        read -r -p "  Calibrate which camera? (front/down, Enter = down): " which || which=""
        case "${which,,}" in
            ""|d|down)  which="down" ;;
            f|front)    which="front" ;;
            *) warn "choose front or down"; which="" ;;
        esac
    done
    bash "${WORKSPACES}/local_ws/auxiliary/camera_calibration/camera_calibration_auto/camera_calibrate.sh" "${which}"
}

# CAMERA FOCUS (menu option). Stream the selected IMX219 pipeline(s) + the Foxglove bridge so
# the lenses can be focused live in Foxglove Studio. Any key stops it and returns to the menu.
camera_focus() {
    step "Camera focus"

    # ROS env (ament setup files reference unbound vars under set -u; source with it disabled).
    set +u
    source /opt/ros/humble/setup.bash 2>/dev/null
    [[ -f "${LOCAL_WS}/install/setup.bash" ]] && source "${LOCAL_WS}/install/setup.bash"
    set -u
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"

    if ! ros2 pkg prefix foxglove_bridge >/dev/null 2>&1; then
        warn "foxglove_bridge not installed - run 'sudo apt install ros-humble-foxglove-bridge'"
        return
    fi
    # Focus is interactive - you watch the feed while turning the lens - so it needs a terminal
    # to read the stop key from; without one the pipelines would spin up and never be torn down.
    if [[ ! -t 0 ]]; then
        warn "camera focus: no controlling terminal - skipping (run the menu option from a shell)"
        return
    fi

    # Ensure the camera manager is up (start its service if needed).
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

    local -a cams=()
    if (( PRE )); then
        cams=(cam_front cam_down)
    else
        local pick=""
        while [[ -z "${pick}" ]]; do
            read -r -p "  Focus which camera? (front/down/both, Enter = both): " pick || pick=""
            case "${pick,,}" in
                ""|b|both)         cams=(cam_front cam_down); pick=both ;;
                f|front|cam_front) cams=(cam_front);          pick=front ;;
                d|down|cam_down)   cams=(cam_down);           pick=down ;;
                *) warn "choose front, down or both"; pick="" ;;
            esac
        done
    fi

    # Stop first for a clean slate and let the sensor release: a pipeline started while its
    # camera is briefly held reports "started" but never streams.
    echo "  Starting ${cams[*]}..."
    local c
    for c in "${cams[@]}"; do
        ros2 service call "/gst_camera_manager/${c}" std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1 || true
    done
    sleep 2
    for c in "${cams[@]}"; do
        ros2 service call "/gst_camera_manager/${c}" std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1 || true
    done

    # Require a SUSTAINED stream: a cold argus start can emit ONE frame and then stall, which a
    # single-shot check calls "streaming" while the Foxglove panel stays blank.
    _stream_sustained() {   # $1 = topic, $2 = first-frame budget (s)
        timeout "$2" ros2 topic echo --once --qos-reliability best_effort "$1" >/dev/null 2>&1 || return 1
        timeout 4  ros2 topic echo --once --qos-reliability best_effort "$1" >/dev/null 2>&1
    }
    cam_live() {            # $1 = pipeline
        _stream_sustained "/$1/image_raw/compressed" 22 && return 0
        echo "    $1: no sustained stream yet (cold start) - restarting the pipeline once and waiting..."
        timeout 10 ros2 service call "/gst_camera_manager/$1" std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
        sleep 2
        timeout 10 ros2 service call "/gst_camera_manager/$1" std_srvs/srv/SetBool "{data: true}"  >/dev/null 2>&1
        _stream_sustained "/$1/image_raw/compressed" 18
    }
    for c in "${cams[@]}"; do
        cam_live "$c" && ok "${c} streaming" \
            || warn "${c}: no live stream after warm-up + restart - sensor absent or held by another consumer; its Foxglove panel will be blank"
    done

    # Own process group via job control (set -m): with 'setsid CMD &', $! is the exited wrapper
    # and every group-kill misses - the bridge leaked. Under -m, $! IS the group leader.
    echo "  Starting Foxglove bridge..."
    set -m
    # shellcheck disable=SC2086
    ${FOXGLOVE_LAUNCH} </dev/null >/tmp/arid_focus_foxglove.log 2>&1 &
    local fox_pgid=$!
    set +m
    echo "${fox_pgid}" > /tmp/arid_focus_fox.pgid
    # Ctrl+C at the focus prompt lands in the global INT trap, which knows nothing about this
    # pgid. RETURN fires on every way out of this function; the normal teardown clears the trap
    # first so it cannot run twice.
    local _stop_cams=""
    for c in "${cams[@]}"; do
        _stop_cams+="ros2 service call /gst_camera_manager/${c} std_srvs/srv/SetBool '{data: false}' >/dev/null 2>&1 || true
"
    done
    # The trap clears itself: a RETURN trap is shell-global, so without that it would fire
    # again when the caller returns.
    # shellcheck disable=SC2064
    trap "rm -f /tmp/arid_focus_fox.pgid
          kill -KILL -- -${fox_pgid} 2>/dev/null || true
          ${_stop_cams}trap - RETURN" RETURN
    sleep 3

    local fox_port; fox_port=$(grep -oE 'port:=[0-9]+' <<<"${FOXGLOVE_LAUNCH}" | grep -oE '[0-9]+')
    fox_port="${fox_port:-8765}"
    # Write the instructions and read the key on the controlling terminal directly: setup runs
    # under 'exec > >(tee)', which block-buffers, so these lines would otherwise stay invisible
    # until the blocking read is torn down.
    local TTY=/dev/tty; { : >"${TTY}"; } 2>/dev/null || TTY=/dev/stdout
    {
        echo ""
        echo -e "  ${BOLD}Open Foxglove Studio -> Open connection -> Foxglove WebSocket. Connect to:${NC}"
        local ip found=0
        for ip in $(ip -4 -o addr show scope global 2>/dev/null \
                | awk '$2 !~ /^(docker|br-|veth|l4tbr|usb|virbr)/ {print $4}' | cut -d/ -f1); do
            echo -e "    ${GREEN}ws://${ip}:${fox_port}${NC}"; found=1
        done
        (( found )) || echo -e "  ${YELLOW}[WARN]${NC} no routable IP detected - check 'ip addr' (port ${fox_port})"
        for c in "${cams[@]}"; do echo "  Add an Image panel for: /${c}/image_raw/compressed"; done
        echo ""
        echo "  Adjust focus in Foxglove, then press any key to stop."
    } > "${TTY}"
    # Discard anything typed during the startup delay so an early keypress doesn't exit immediately.
    while read -rsn1 -t 0.1 _ <"${TTY}" 2>/dev/null; do :; done
    read -rn1 _ <"${TTY}" || true

    trap - RETURN
    rm -f /tmp/arid_focus_fox.pgid
    echo "  Stopping Foxglove bridge and ${cams[*]}..."
    kill -INT  -- -"${fox_pgid}" 2>/dev/null || true
    sleep 1
    kill -KILL -- -"${fox_pgid}" 2>/dev/null || true
    for c in "${cams[@]}"; do
        ros2 service call "/gst_camera_manager/${c}" std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1 || true
    done
    ok "stopped"
}

# SUMMARY
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

# Undo the host-side state setup.sh creates. Never called from run_full_setup or run_resume.
setup_uninstall() {
    echo ""
    echo -e "${RED}${BOLD}============== ARID uninstall ==============${NC}"
    echo "  This will stop and remove every systemd unit installed by setup.sh,"
    echo "  delete /etc/sudoers.d/jetson_systemctl, the polkit rule, the apt drop-in,"
    echo "  the ARID block in ~/.bashrc, the docker patches inside isaac_ros_common,"
    echo "  every setup.sh sentinel file, and the Isaac container + image."
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
    # Same source as the install list (shipped_units), plus the pre-rename supervisor unit.
    local units=()
    mapfile -t units < <(shipped_units)
    units+=(vslam_supervisor.service)
    local u
    for u in "${units[@]}"; do
        sudo -n systemctl stop    "$u" 2>/dev/null || true
        sudo -n systemctl disable "$u" 2>/dev/null || true
        sudo rm -f "/etc/systemd/system/$u" 2>/dev/null || true
    done
    sudo rm -f /etc/systemd/system.conf.d/10-arid-ros-env.conf 2>/dev/null || true
    sudo systemctl daemon-reexec 2>/dev/null || true
    sudo systemctl daemon-reload 2>/dev/null || true
    for u in "${units[@]}"; do sudo -n systemctl reset-failed "$u" 2>/dev/null || true; done
    ok "${#units[@]} unit(s) removed"

    step "Remove apt drop-in"
    sudo rm -f /etc/apt/apt.conf.d/99-arid-disable-auto-updates 2>/dev/null || true
    ok "apt drop-in removed"

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
    local marked
    while IFS= read -r marked; do
        [[ -z "$marked" ]] && continue
        git -C "$REPO_ROOT" update-index --no-skip-worktree "$marked" 2>/dev/null || true
    done < <(git -C "$REPO_ROOT" ls-files -v | sed -n 's/^S //p')
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
        "${HOME_DIR}/.arid_bashrc.lock" \
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

    # The supervisor needs an in-container install to exec.
    local _w ws_built=0
    for _w in $(seq 1 10); do
        docker exec -u admin isaac_ros_dev-aarch64-container test -f /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null && { ws_built=1; break; }
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
    # Require BOTH services: they advertise in sequence, so a mid-snapshot sees only vslam_enable.
    local i sup_list ok_sup=0
    for i in $(seq 1 18); do
        sup_list=$(timeout 5 ros2 service list 2>/dev/null || true)
        if echo "${sup_list}" | grep -qx /arid_supervisor/vslam_enable \
           && echo "${sup_list}" | grep -qx /arid_supervisor/status; then
            ok_sup=1; break
        fi
        (( i % 3 == 0 )) && echo "    waiting... (poll ${i}/18, $((i * 5)) s elapsed)"
        sleep 5
    done
    if (( ok_sup )); then
        ok "supervisor services on the graph"
    else
        warn "supervisor did not advertise within 90 s - smoke test Section 6 may report failures"
    fi
}

# Final post-install gate: services, aliases, both IMX219 pipelines, supervisor graph.
run_smoke_test() {
    step "Running local smoke test"
    local rc=0
    # Separate smoke-test log; output still streams to the main log + screen.
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

# Front-load every decision so the run is unattended; steps consult PRE_* instead of prompting.
collect_answers() {
    step "Setup questionnaire - answer once; setup then runs without further prompts"
    PRE=1

    if [[ -f "${HOME_DIR}/.arid_provisioned" ]]; then
        echo "  Already provisioned - hostname / password unchanged."
    else
        ask_yn "  Set hostname? (y/n, Enter = arid): " n && { read -r -p "    Hostname: " PRE_HOST || PRE_HOST=""; }
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

    # Never offer NoMachine when setup runs inside a NoMachine session: a reinstall drops it.
    if is_inside_nomachine; then
        PRE_NOMACHINE=skip
    elif dpkg -s nomachine >/dev/null 2>&1 || [[ -x /usr/NX/bin/nxserver ]]; then
        ask_yn "  Upgrade NoMachine? (y/n, Enter = skip): " n && PRE_NOMACHINE=yes || PRE_NOMACHINE=skip
    else
        PRE_NOMACHINE=install
    fi

    # ARK-OS - only ask to reinstall if already present; otherwise it installs unconditionally.
    if ark_installed; then
        ask_yn "  Reinstall ARK-OS? (y/n, Enter = skip): " n && PRE_ARK=yes || PRE_ARK=skip
    else
        PRE_ARK=install
    fi

    # ROS2 - isolated; ask to reinstall if present, else install. A bare /opt/ros/humble
    # directory is left behind by a partial install, so test the setup script itself.
    if [[ -f /opt/ros/humble/setup.bash ]]; then
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

    ask_yn "  Verify the camera feeds (front + down) at the end? (y/n, Enter = skip): " n \
        && PRE_VERIFY=yes || PRE_VERIFY=skip

    ask_yn "  Focus both CSI cameras after verification (Foxglove)? (y/n, Enter = skip): " n \
        && PRE_FOCUS=yes || PRE_FOCUS=skip

    # Local workspace colcon: rebuild if built, else build.
    if [[ -d "${LOCAL_WS}/install" ]]; then
        ask_yn "  Rebuild the local workspace? (y/n, Enter = skip): " n && PRE_LOCAL_WS=yes || PRE_LOCAL_WS=skip
    else
        ask_yn "  Build the local workspace? (y/n, Enter = yes): " y && PRE_LOCAL_WS=yes || PRE_LOCAL_WS=skip
    fi

    echo "  Isaac builds are long - run over NoMachine (survives a disconnect) or skip and"
    echo "  run 'build_isaac' / 'colcon_isaac' later."
    local _q
    if docker image inspect isaac_ros_dev-aarch64 >/dev/null 2>&1; then _q="  Rebuild the Isaac container? (y/n, Enter = skip): "; else _q="  Build the Isaac container? (y/n, Enter = skip): "; fi
    if ask_yn "${_q}" n; then
        PRE_BUILD_ISAAC=yes
        touch "${HOME_DIR}/.arid_pending_build_isaac"
    else
        PRE_BUILD_ISAAC=skip
        rm -f "${HOME_DIR}/.arid_pending_build_isaac"
    fi
    # Isaac workspace colcon (when an image exists or is queued).
    if [[ "${PRE_BUILD_ISAAC}" == "yes" ]] || docker image inspect isaac_ros_dev-aarch64 >/dev/null 2>&1; then
        if [[ -f "${ISAAC_ROS_WS}/install/setup.bash" ]]; then
            # Enter = rebuild: pulled packages ship nothing until colcon runs, and a launch
            # referencing an unbuilt package is the failure that skip-by-default produced.
            ask_yn "  Rebuild the Isaac workspace? (y/n, Enter = rebuild): " y && PRE_COLCON=yes || PRE_COLCON=skip
        else
            ask_yn "  Build the Isaac workspace? (y/n, Enter = yes): " y && PRE_COLCON=yes || PRE_COLCON=skip
        fi
    else
        PRE_COLCON=skip
    fi

    ask_yn "  Run the smoke test at the end? (y/n, Enter = y): " y && PRE_SMOKE=yes || PRE_SMOKE=skip
    ask_yn "  Reboot before the smoke test if anything was built? (y/n, Enter = y): " y && PRE_REBOOT=yes || PRE_REBOOT=no

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
                   PRE_REALSENSE PRE_VERIFY PRE_FOCUS PRE_SMOKE \
                   PRE_BUILD_ISAAC PRE_COLCON PRE_LOCAL_WS PRE_REBOOT; do
            printf '%s=%q\n' "$var" "${!var-}"
        done
    } > "$f"
}

# ORCHESTRATION
run_full_setup() {
    preflight
    printf '%s\n' "${LOG_FILE}" > "${HOME_DIR}/.arid_setup_log"   # pin this session's log across its reboot
    # Fresh run: no checkpointed sections, and no build recorded yet (a stale .arid_did_build
    # would otherwise force a reboot when nothing builds this run).
    rm -f "${HOME_DIR}/.arid_progress" "${HOME_DIR}/.arid_did_build"
    collect_answers
    setup_power
    first_boot
    ensure_wifi
    nomachine
    enable_user_linger
    clean_nvidia_desktop
    disable_updates
    ark_os            # ARK-OS + ROS2 + JetPack; everything below depends on /opt/ros/humble
    _run_setup_tail   # checkpointed steps; --resume / --continue re-enter here
}

# Run one tail section, then checkpoint it - a resume skips finished sections instead of redoing them.
run_step() {
    local name="$1"
    grep -qxF "${name}" "${HOME_DIR}/.arid_progress" 2>/dev/null && { skip "${name} already done"; return 0; }
    "${name}"
    echo "${name}" >> "${HOME_DIR}/.arid_progress"
}

# PHASE A - install + host configuration. Each section is checkpointed via run_step so a
# resume never re-runs (or re-prompts) it.
_run_phase_a_install() {
    # ARK's dds-agent install (Fast-DDS) locks /usr/local + include + lib to 0700; reopen for non-root.
    sudo chmod -R a+rX /usr/local 2>/dev/null || true
    run_step setup_repos
    run_step setup_apt_packages
    run_step enable_clock_sync          # after apt: chrony must exist before it is enabled
    run_step setup_zerotier || true     # a ZT failure is not a provisioning failure
    run_step setup_px4_deps
    run_step setup_git
    run_step setup_docker_patches
    run_step setup_skip_worktree
    run_step setup_bashrc
    run_step setup_permissions
    run_step setup_uhubctl
    run_step setup_ros_workspace        # BUILD: host colcon of local_ws
    # Docker engine + NVIDIA runtime must precede setup_systemd, so a mid-run abort + reboot
    # cannot leave the boot service pointing at an absent daemon.
    run_step setup_docker
    run_step setup_systemd
    run_step setup_realsense
}

# PHASE B - builds + live camera steps.
_run_phase_b_build() {
    _run_build_isaac_if_queued || true   # BUILD: Isaac container image
    colcon_isaac_step || true            # BUILD: in-container colcon; self-gates on image + PRE_COLCON
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

# One reboot, condition-driven: it fires right before the smoke test iff a build ran and the
# smoke test will run - a clean boot is what the smoke test validates. Otherwise the smoke
# test runs inline and the run ends with no reboot.
_finish_with_smoke_test() {
    # The persisted "a build actually ran" signal; each build step touches it.
    [[ -f "${HOME_DIR}/.arid_did_build" ]] && DID_BUILD=1

    print_summary

    if (( DID_BUILD )) && [[ "${PRE_SMOKE:-}" != "skip" ]] && ! is_no "${PRE_REBOOT:-yes}"; then
        _do_reboot "${HOME_DIR}/.arid_run_smoke" && return 0
    fi

    if [[ "${PRE_SMOKE:-}" == "skip" ]]; then
        step "Smoke test skipped (per questionnaire)"
    else
        _recover_docker_supervisor
        run_smoke_test || true
    fi
    (( DID_BUILD )) && echo -e "${YELLOW}A reboot is still recommended before flying.${NC}"
    # Session done: fresh log next run, and clear the build flag so a later --continue
    # does not inherit it and reboot for nothing.
    rm -f "${HOME_DIR}/.arid_setup_log" "${HOME_DIR}/.arid_did_build"
    echo ""
    echo -e "${BOLD}Setup complete.${NC}"
}

# Arm the resume hook + the given stage marker, then reboot. Returns 0 on a reboot the caller
# should 'return' after; non-zero (rolled back) means continue inline.
_do_reboot() {
    local marker="$1"
    echo ""
    echo -e "${YELLOW}${BOLD}Rebooting in a few seconds. Setup resumes on the next terminal session.${NC}"
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

# INTERACTIVE MENU. Individual tasks return to the menu on failure. 'q' clears every resume
# hook so quitting does not auto-resume on the next terminal.
menu() {
    while true; do
        echo ""
        echo -e "${BOLD}============== ARID setup ==============${NC}"
        echo "  1) Full setup"
        echo "  2) Smoke test"
        echo "  3) RealSense assignment"
        echo "  4) Camera feed check"
        echo "  5) Wi-Fi connect"
        echo "  6) Camera calibration"
        echo "  7) Camera focus"
        echo "  8) Build Isaac container"
        echo "  9) Build Isaac workspace"
        echo "  10) Build local workspace"
        echo "  11) Install ARK-OS"
        echo "  12) Install ROS2"
        echo "  13) ZeroTier join/switch"
        echo "  14) Uninstall"
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
            5) bash "${WORKSPACES}/scripts/wifi.sh"              || true ;;
            6) calibrate_cameras                                 || true ;;
            7) camera_focus                                      || true ;;
            8) build_isaac_step                                  || true ;;
            9) colcon_isaac_step                                 || true ;;
            10) setup_ros_workspace                              || true ;;
            11) menu_install_ark                                 || true ;;
            12) menu_install_ros2                                || true ;;
            13) setup_zerotier                                   || true ;;
            14) setup_uninstall                                  || true ;;
            q|Q) echo "  Quit."; cleanup_user_exit; break ;;
            *) warn "invalid selection: '${choice}'" ;;
        esac
    done
}

# Resume after a reboot. Two stage markers disambiguate what to do:
#   .arid_setup_continue -> re-enter the tail (run_step skips finished sections), armed by the
#                           install reboot at the end of Phase A and by --continue after an abort.
#   .arid_run_smoke      -> the pre-smoke reboot: recover the boot units + run the smoke test.
# Neither marker -> a manual './setup.sh --resume': camera verification, queued builds,
# recovery, smoke test.
run_resume() {
    # flock serialises racing resume terminals; the lock file is deliberately never removed
    # (flock binds the inode).
    local lock="${HOME_DIR}/.arid_resume.lock"
    if ! exec 9>"${lock}" 2>/dev/null; then
        warn "could not open resume lock at ${lock} - continuing without serialisation"
    elif ! flock -n 9; then
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

    # Clear the continue marker BEFORE running so that if the pre-smoke reboot fires inside the
    # tail we resume via .arid_run_smoke, not here.
    if [[ -f "${HOME_DIR}/.arid_setup_continue" ]] || (( FORCE_CONTINUE )); then
        step "Continuing setup where it left off"
        PRE=1
        rm -f "${HOME_DIR}/.arid_setup_continue"
        _run_setup_tail
        return 0
    fi

    local post_reboot_smoke=0
    if [[ -f "${HOME_DIR}/.arid_run_smoke" ]]; then
        step "Continuing setup after the reboot - running the smoke test"
        PRE=1
        post_reboot_smoke=1
        rm -f "${HOME_DIR}/.arid_run_smoke"
    fi

    if (( post_reboot_smoke )); then
        : # camera verification, focus and the builds already ran before the reboot
    elif (( ${PRE:-0} )) && ! is_yes "${PRE_VERIFY:-}"; then
        step "Continuing setup - camera verification skipped (per questionnaire)"
    else
        step "Continuing setup - camera verification"
        if bash "${WORKSPACES}/scripts/verify_cv_cams.sh" both; then
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
