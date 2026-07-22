#!/bin/bash
# ARID workspace setup.
# Usage: ./setup.sh [--full|--resume|--help]
# Every step auto-detects its current state and installs / configures only what is
# missing. Safe to re-run any time.
set -euo pipefail

# Output helpers
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[1;34m'; BOLD='\033[1m'; NC='\033[0m'

CURRENT_STEP="(not started)"
step() { CURRENT_STEP="$*"; echo -e "\n${BLUE}${BOLD}==> $*${NC}"; }
ok()   { echo -e "  ${GREEN}[OK]${NC}   $*"; }
warn() { echo -e "  ${YELLOW}[WARN]${NC} $*"; }
skip() { echo -e "  [SKIP]  $*"; }
err()  { echo -e "  ${RED}[ERROR]${NC} $*" >&2; }

# Accept any case of y / yes (is_yes) or n / no (is_no) at prompts. Strip non-letter
# characters first so a trailing carriage return some terminals append does not
# defeat the match.
is_yes() { local a="${1//[^A-Za-z]/}"; case "${a,,}" in y|yes) return 0 ;; *) return 1 ;; esac; }
is_no()  { local a="${1//[^A-Za-z]/}"; case "${a,,}" in n|no)  return 0 ;; *) return 1 ;; esac; }

STEPS_RUN=()
STEPS_SKIPPED=()
RUN_FULL=0          # --full: skip the menu and run the whole setup
RUN_RESUME=0        # --resume: continue setup after a reboot (smoke test + camera verification)

# Single source of truth for the Foxglove bridge launch (the 'foxglove_bridge' alias and
# the camera-focus tool both use this, so the port stays in one place).
FOXGLOVE_LAUNCH="ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"

# Pre-collected answers (full setup only). PRE=1 means "use these, do not prompt mid-run".
PRE=0
PRE_HOST=""; PRE_PASS=""
PRE_WIFI=""; PRE_WIFI_SSID=""; PRE_WIFI_PASS=""
PRE_NOMACHINE=""; PRE_PX4=""
PRE_REALSENSE=""; PRE_VERIFY=""; PRE_SMOKE=""; PRE_BUILD_ISAAC=""; PRE_REBOOT=""; PRE_ZTNET=""

# Error trap
failure() {
    err "======================================================"
    err "SETUP FAILED in step: ${CURRENT_STEP}"
    err "Line $1: '${BASH_COMMAND}'"
    err "Log: ${LOG_FILE:-<not yet set>}"
    err "======================================================"
    exit 1
}
trap 'failure ${LINENO}' ERR

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
REPO_ROOT="$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"


# Logging - one continuous log per full-setup session: a post-reboot resume reuses the log
# path that run_full_setup pinned in ~/.arid_setup_log, so all of a session's reboots append
# to the same file.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${SCRIPT_DIR}/log"
mkdir -p "${LOG_DIR}"

# Keep only the newest $2 logs of prefix $1 in LOG_DIR (e.g. _prune_logs setup_log 10).
_prune_logs() {
    local files
    mapfile -t files < <(ls -1t "${LOG_DIR}/${1}_"*.log 2>/dev/null)
    (( ${#files[@]} > $2 )) && rm -f -- "${files[@]:$2}"
    return 0
}

if [[ " $* " == *" --resume "* ]] && [[ -s "${HOME_DIR}/.arid_setup_log" ]]; then
    LOG_FILE="$(cat "${HOME_DIR}/.arid_setup_log" 2>/dev/null)"
fi
LOG_FILE="${LOG_FILE:-${LOG_DIR}/setup_log_$(date +%Y%m%d_%H%M%S).log}"
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
            --help|-h)
                cat <<EOH
Usage: $0 [--full|--resume|--help]
  (no args)  Interactive menu: full setup or individual tools.
  --full     Run the questionnaire, then the full setup unattended.
  --resume   Continue setup after a reboot (smoke test + camera verification).
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

    if git -C "$REPO_ROOT" submodule status 2>/dev/null | grep -q "^-"; then
        err "Uninitialized submodules detected."
        err "Run: git submodule update --init --recursive"
        exit 1
    fi

    ok "Sanity checks passed"
}

# Power mode and package holds
setup_power() {
    step "Power mode & package holds"

    # nvpmodel -m 0 (MAXN) can require a reboot on some units and then prompts interactively
    # ("reboot now? YES/yes"), which hangs an unattended run and swallows stray keystrokes.
    # Skip if already in mode 0; otherwise request it non-interactively (answer 'no' to the
    # reboot prompt - we never reboot mid-setup) and keep it non-fatal.
    local cur
    cur=$(sudo /usr/sbin/nvpmodel -q 2>/dev/null | grep -oE '^[0-9]+$' | head -1)
    if [[ "${cur}" == "0" ]]; then
        skip "already in max power mode (nvpmodel 0)"
    else
        printf 'no\n' | sudo /usr/sbin/nvpmodel -m 0 >/dev/null 2>&1 \
            || warn "nvpmodel -m 0 failed (non-fatal; jetson-clocks.service still applies max clocks)"
    fi

    sudo apt-mark hold \
        nvidia-l4t-core \
        linux-firmware \
        nvidia-l4t-kernel \
        nvidia-l4t-kernel-dtbs \
        nvidia-l4t-firmware \
        nvidia-l4t-kernel-headers \
        nvidia-l4t-kernel-oot-headers \
        wireless-regdb

    STEPS_RUN+=("power")
    ok "Max power mode set, critical packages held"
}

# APT repositories
setup_repos() {
    step "APT repositories"

    # ROS keyring
    if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
        sudo curl -sSL \
            https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            -o /usr/share/keyrings/ros-archive-keyring.gpg
        ok "ROS keyring added"
    else
        skip "ROS keyring already present"
    fi

    # Nvidia CDI (safe to regenerate every run)
    sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml
    ok "CDI config regenerated"

    # Nvidia Jetson APT repo
    if ! grep -q "repo.download.nvidia.com/jetson/common" \
            /etc/apt/sources.list.d/nvidia-l4t-apt-source.list 2>/dev/null; then
        sudo apt-key adv --fetch-key \
            https://repo.download.nvidia.com/jetson/jetson-ota-public.asc
        echo "deb https://repo.download.nvidia.com/jetson/common r36.4 main" \
            | sudo tee /etc/apt/sources.list.d/nvidia-l4t-apt-source.list >/dev/null
        echo "deb https://repo.download.nvidia.com/jetson/t234 r36.4 main" \
            | sudo tee -a /etc/apt/sources.list.d/nvidia-l4t-apt-source.list >/dev/null
        ok "Nvidia Jetson repo added"
    else
        skip "Nvidia Jetson repo already present"
    fi

    # Docker APT repo
    if [[ ! -f /etc/apt/keyrings/docker.gpg ]]; then
        sudo install -m 0755 -d /etc/apt/keyrings
        curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
            | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
        sudo chmod a+r /etc/apt/keyrings/docker.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
https://download.docker.com/linux/ubuntu \
$(. /etc/os-release && echo "$VERSION_CODENAME") stable" \
            | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null
        ok "Docker repo added"
    else
        skip "Docker repo already present"
    fi

    STEPS_RUN+=("repos")
}

# APT packages
setup_apt_packages() {
    step "APT packages"

    sudo apt-get update
    # No ros-humble-librealsense2 here (2.57.x, V4L2 backend): the only host RealSense consumer
    # is the self-contained pip pyrealsense2 wheel (config_realsense), and the container stack
    # must only ever link the RSUSB 2.55.1 build at /usr/local - keep apt librealsense off entirely.
    sudo apt-get install -y \
        software-properties-common \
        ca-certificates curl gnupg \
        libusb-1.0-0-dev pkgconf gpiod \
        iputils-arping tcpdump arp-scan \
        pva-allow-2 \
        python3-colcon-clean \
        ros-humble-camera-info-manager \
        ros-humble-compressed-image-transport \
        ros-humble-foxglove-bridge \
        ros-humble-foxglove-msgs

    STEPS_RUN+=("apt")
    ok "APT packages installed"
}

# PX4 build dependencies: ensure kconfiglib Python deps are present, then ensure arm-none-eabi-gcc.
setup_px4_deps() {
    step "PX4 build dependencies"

    if [[ ! -d "${PX4_DIR}" ]]; then
        skip "PX4-Autopilot not found at ${PX4_DIR}"
        STEPS_SKIPPED+=("px4_deps")
        return
    fi

    # Python deps (idempotent)
    local req="${PX4_DIR}/Tools/setup/requirements.txt"
    if [[ -f "${req}" ]]; then
        if python3 -c 'import kconfiglib' >/dev/null 2>&1; then
            skip "PX4 Python deps already satisfied (kconfiglib importable)"
        else
            ok "Installing PX4 Python deps from ${req}..."
            python3 -m pip install -r "${req}" ${PIP_BREAK_FLAG}
            ok "PX4 Python deps installed"
        fi
    else
        warn "${req} not found; cannot reconcile Python deps"
    fi

    # ARM toolchain (heavy; only on demand)
    if command -v arm-none-eabi-gcc >/dev/null 2>&1; then
        skip "ARM toolchain already present (arm-none-eabi-gcc)"
        STEPS_RUN+=("px4_deps")
        return
    fi

    local install_px4
    while true; do
        read -r -p "ARM toolchain (arm-none-eabi-gcc) not found. Run PX4 Tools/setup/ubuntu.sh? (y/n): " install_px4
        case "$install_px4" in [yYnN]) break ;; *) echo "Choose y or n." ;; esac
    done

    if [[ "$install_px4" =~ ^[yY]$ ]]; then
        (cd "${PX4_DIR}/Tools/setup" && bash ubuntu.sh)
        STEPS_RUN+=("px4_deps")
        ok "PX4 ARM toolchain + Tools/setup deps installed"
    else
        warn "ARM toolchain install declined; PX4 firmware builds will fail until 'cd ${PX4_DIR}/Tools/setup && bash ubuntu.sh' is run"
        STEPS_SKIPPED+=("px4_deps")
    fi
}

# Git config and submodules
setup_git() {
    step "Git config & submodules"

    git config --global credential.helper "cache --timeout=604800"

    find "${WORKSPACES}/scripts" -type f \( -name "*.bash" -o -name "*.sh" \) \
        -exec chmod +x {} \;
    find "${ISAAC_ROS_WS}/container_scripts" -type f \( -name "*.bash" -o -name "*.sh" \) \
        -exec chmod +x {} \;
    ok "Script permissions set"

    "${WORKSPACES}/scripts/update_submods.sh"
    ok "Submodules synced and verified against pinned commits"

    STEPS_RUN+=("git")
}

# Isaac ROS Docker patches
setup_docker_patches() {
    step "Isaac ROS Docker patches"

    cp -f "${ISAAC_ROS_WS}/docker_resources/patched_dockerfiles/.isaac_ros_common-config" \
        "${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/"

    cp -f "${ISAAC_ROS_WS}/docker_resources/dockerfiles/Dockerfile.arid" \
        "${ISAAC_ROS_WS}/src/isaac_ros_common/docker/"

    cp -f "${ISAAC_ROS_WS}/container_scripts/run_dev.sh" \
        "${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/"

    cp -f "${ISAAC_ROS_WS}/container_scripts/arid_env.sh" \
        "${ISAAC_ROS_WS}/src/isaac_ros_common/docker/scripts/"

    # skip-worktree hides patches from isaac_ros_common submodule git tracking.
    git -C "${ISAAC_ROS_WS}/src/isaac_ros_common" update-index --skip-worktree \
        scripts/.isaac_ros_common-config \
        docker/Dockerfile.arid \
        scripts/run_dev.sh \
        docker/scripts/arid_env.sh 2>/dev/null || true

    STEPS_RUN+=("docker_patches")
    ok "Docker patches applied and protected"
}

# Config file protection (skip-worktree). Per-deployment configs (camera serials,
# calibrations) get skip-worktree so local edits don't show in git status and can't be pushed.
setup_skip_worktree() {
    step "Protecting per-deployment config files"

    local protected=0

    while IFS= read -r dir; do
        local rel_dir
        rel_dir=$(realpath --relative-to="$REPO_ROOT" "$dir")
        local files
        files=$(git -C "$REPO_ROOT" ls-files "$rel_dir" 2>/dev/null || true)
        if [[ -n "$files" ]]; then
            echo "$files" | xargs git -C "$REPO_ROOT" update-index --skip-worktree \
                2>/dev/null || true
            ok "Protected: ${rel_dir}"
            (( protected++ )) || true
        fi
    done < <(find "${ISAAC_ROS_WS}/src" "${LOCAL_WS}/src" -type d \( -name "config" -o -name "cfg" -o -name "camera_calibrations" \) 2>/dev/null)

    STEPS_RUN+=("skip_worktree")
    ok "${protected} config directories protected"
}

# .bashrc: rewrites the ARID block on every run so alias/export changes propagate
# without leaving stale duplicates.
setup_bashrc() {
    step ".bashrc environment"

    # Precondition: the resume-prompt shim MUST exist in the repo before we wire bashrc to
    # source it. If it's missing, refuse to rewrite - degrading silently here would leave
    # the operator with a bashrc that points at a non-existent file, and the post-reboot
    # resume prompt would never fire with no diagnostic.
    local shim="${SCRIPT_DIR}/scripts/arid_resume_prompt.sh"
    if [[ ! -r "${shim}" ]]; then
        err "Resume-prompt shim missing at ${shim} - re-pull the repo before re-running setup."
        return 1
    fi

    # Atomic rewrite under a lock so concurrent setup runs cannot race. The block is rewritten
    # via a temp file + mv so a concurrent shell sourcing ~/.bashrc never sees a half-written
    # state. flock has a 30 s timeout so a stuck lock surfaces an error instead of hanging.
    local lockfile="${HOME_DIR}/.arid_bashrc.lock"
    local tmpfile="${BASHRC_FILE}.arid.new.$$"
    (
        flock -w 30 -x 9 || { err "another setup_bashrc is in progress (timed out after 30 s)"; exit 1; }

        # Copy current bashrc to the working temp file, then strip the ARID block AND any
        # stray managed lines outside it (hand-edits). After this point all edits target
        # tmpfile, leaving BASHRC_FILE intact for concurrent readers until the final atomic mv.
        cp -- "$BASHRC_FILE" "$tmpfile"
        sed -i \
            -e '/# BEGIN ARID SETUP/,/# END ARID SETUP/d' \
            -e '/^[[:space:]]*source[[:space:]].*local_ws\/install\/setup\.bash/d' \
            -e '/^[[:space:]]*export[[:space:]]\+ROS_DOMAIN_ID=/d' \
            -e '/^[[:space:]]*export[[:space:]]\+ROS_LOCALHOST_ONLY=/d' \
            -e '/^[[:space:]]*export[[:space:]]\+WORKSPACES=/d' \
            -e '/^[[:space:]]*export[[:space:]]\+LOCAL_WS=/d' \
            -e '/^[[:space:]]*export[[:space:]]\+ISAAC_ROS_WS=/d' \
            -e '/^[[:space:]]*alias[[:space:]]\+\(run_isaac\|build_isaac\|start_isaac\|stop_isaac\|isaac_bash\)=/d' \
            -e '/^[[:space:]]*alias[[:space:]]\+\(reset_usb\|colcon_local\|clean_local\|rosdep_local\|foxglove_bridge\)=/d' \
            -e '/^[[:space:]]*alias[[:space:]]\+\(cam_\(down\|front\)_\(start\|stop\|status\|alive\)\|cam_refresh\)=/d' \
            -e '/^[[:space:]]*alias[[:space:]]\+\(local_test\|config_realsense\|wifi\|ver_cv_cams\|update_submods\)=/d' \
            -e '/^[[:space:]]*alias[[:space:]]\+\(initialize\|deinitialize\)=/d' \
            -e '/^[[:space:]]*alias[[:space:]]\+\(setup\|colcon_isaac\|clean_isaac\|rosdep_isaac\|cam_calibrate\|zt_join\|status\)=/d' \
            "$tmpfile"

        # IMPORTANT: the heredoc delimiter is QUOTED ('ARIDRC') so bash performs NO expansion
        # on the body - no parameter expansion, no command substitution, no backticks, no
        # arithmetic. Everything between the open and close markers is byte-literal. Setup-
        # time substitution is done via @@TOKEN@@ markers in a sed pass after the heredoc.
        #
        # To inject a new setup-time value here:
        #   1. Add an @@TOKEN@@ marker inside the heredoc body, AND
        #   2. Add a matching `sed -i "s|@@TOKEN@@|...|g"` line below.
        # NEVER change the delimiter to unquoted EOF - the previous unquoted form was the
        # source of bashrc-mid-line corruption (heredoc with unescaped backticks command-
        # substituted setup.sh --resume at write time and wrote its stdout into bashrc).
        cat >> "$tmpfile" << 'ARIDRC'
# BEGIN ARID SETUP
if [ -d /tmp/.X11-unix ]; then
    sock=$(ls /tmp/.X11-unix/X* 2>/dev/null | head -n1)
    if [ -n "$sock" ]; then export DISPLAY=":${sock##*/X}"; fi
fi
xhost +local: >/dev/null 2>&1 || true
export ROS_DOMAIN_ID=23
export ROS_LOCALHOST_ONLY=1
export WORKSPACES=@@WORKSPACES@@
export LOCAL_WS=@@LOCAL_WS@@
export ISAAC_ROS_WS=@@ISAAC_ROS_WS@@
source @@LOCAL_WS@@/install/setup.bash
alias setup='/bin/bash @@WORKSPACES@@/setup.sh'
alias run_isaac='/bin/bash @@ISAAC_ROS_WS@@/container_scripts/run_isaac_docker.sh'
alias build_isaac='/bin/bash @@ISAAC_ROS_WS@@/container_scripts/build_isaac_docker.sh'
alias start_isaac='/bin/bash @@ISAAC_ROS_WS@@/container_scripts/start_isaac_docker.sh'
alias stop_isaac='docker stop isaac_ros_dev-aarch64-container'
alias isaac_bash='/bin/bash @@ISAAC_ROS_WS@@/container_scripts/isaac_bash.sh'
alias reset_usb='/bin/bash @@WORKSPACES@@/scripts/usb_reset.sh'
alias colcon_isaac='/bin/bash @@WORKSPACES@@/scripts/colcon_isaac.sh'
alias clean_isaac='@@WORKSPACES@@/scripts/in_isaac.sh clean_isaac'
alias rosdep_isaac='@@WORKSPACES@@/scripts/in_isaac.sh rosdep_isaac'
alias rosdep_local='rosdep install --from-paths @@LOCAL_WS@@/src/ --ignore-src -y'
alias colcon_local='/bin/bash @@WORKSPACES@@/scripts/colcon_local.sh && source @@LOCAL_WS@@/install/setup.bash'
alias clean_local='cd @@LOCAL_WS@@ && colcon clean workspace --base-select build install log'
alias foxglove_bridge='ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765'
alias cam_down_start='ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool "{data: true}"'
alias cam_down_stop='ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool "{data: false}"'
alias cam_down_status='ros2 service call /gst_camera_manager/cam_down/status std_srvs/srv/Trigger "{}"'
alias cam_down_alive='ros2 topic echo --once --qos-durability transient_local /gst_camera_manager/cam_down/alive'
alias cam_refresh='ros2 service call /gst_camera_manager/refresh std_srvs/srv/Trigger'
alias cam_front_start='ros2 service call /gst_camera_manager/cam_front std_srvs/srv/SetBool "{data: true}"'
alias cam_front_stop='ros2 service call /gst_camera_manager/cam_front std_srvs/srv/SetBool "{data: false}"'
alias cam_front_status='ros2 service call /gst_camera_manager/cam_front/status std_srvs/srv/Trigger "{}"'
alias cam_front_alive='ros2 topic echo --once --qos-durability transient_local /gst_camera_manager/cam_front/alive'
alias local_test='/bin/bash @@WORKSPACES@@/scripts/local_test.sh'
alias config_realsense='/bin/bash @@WORKSPACES@@/scripts/config_realsense.sh'
alias wifi='/bin/bash @@WORKSPACES@@/scripts/wifi.sh'
alias ver_cv_cams='/bin/bash @@WORKSPACES@@/scripts/verify_cv_cams.sh'
alias update_submods='/bin/bash @@WORKSPACES@@/scripts/update_submods.sh'
alias cam_calibrate='/bin/bash @@WORKSPACES@@/local_ws/auxiliary/camera_calibration/camera_calibration_auto/camera_calibrate.sh'
alias zt_join='/bin/bash @@WORKSPACES@@/scripts/zt_join.sh'
alias status='ros2 service call /arid_supervisor/status std_srvs/srv/Trigger "{}"'
alias initialize='/bin/bash @@ISAAC_ROS_WS@@/container_scripts/initialize.sh'
alias deinitialize='/bin/bash @@ISAAC_ROS_WS@@/container_scripts/deinitialize.sh'
help() {
    cat <<'ARIDHELP'
ARID host commands:

  System
    setup              Run the ARID setup / provisioning script
    run_isaac          Start + enter the Isaac ROS dev container
    build_isaac        Build the Isaac ROS docker image (also offered as a setup option)
    start_isaac        Start the Isaac container (detached)
    stop_isaac         Stop the Isaac container
    isaac_bash         Open a bash shell in the running container
    colcon_isaac       Build isaac_ros-dev (deinitialize VSLAM first, restart supervisor after)
    clean_isaac        Clean the in-container Isaac workspace
    rosdep_isaac       Install rosdep deps for the in-container workspace
    reset_usb          USB hub reset
    rosdep_local       Install rosdep deps for local_ws
    colcon_local       Build local_ws (stop its host services first, restart them after)
    clean_local        Clean local_ws
    foxglove_bridge    Foxglove bridge on port 8765
    config_realsense   Assign RealSense serials -> front/left/right (live-feed picker)
    ver_cv_cams        Stream the CSI feeds - front/down/both IMX219 (q to quit)
    cam_down_start     Start the downward IMX219 pipeline
    cam_down_stop      Stop the downward IMX219 pipeline
    cam_down_status    Downward pipeline status
    cam_down_alive     Downward pipeline liveness topic
    cam_front_start    Start the front IMX219 pipeline
    cam_front_stop     Stop the front IMX219 pipeline
    cam_front_status   Front pipeline status
    cam_front_alive    Front pipeline liveness topic
    cam_refresh        Re-read gst_camera_manager pipelines.yaml (stops running pipelines first)
    cam_calibrate      Calibrate a CSI camera (arg: front | down)
    local_test         local_ws smoke test
    wifi               Interactive Wi-Fi picker (scan + select + password)
    update_submods     Sync submodules against pinned commits
    zt_join            Join/switch ZeroTier network (single-network model; lists joined, or new id)

  VSLAM  (supervisor must be running; container aliases via docker exec)
    initialize         Enable VSLAM via /arid_supervisor/vslam_enable
    deinitialize       Disable VSLAM (refused while airborne - landed gate)
    status             Supervisor status - vslam running (true/false) + land state
ARIDHELP
}
# Resume-after-reboot hook. The actual prompt logic lives in a dedicated script so this
# bashrc block stays free of command substitution, $(...), backticks, and runtime $VAR.
# A missing shim degrades silently (the [ -r ... ] guard); setup_bashrc fails loudly above
# if the shim is missing at setup time.
[ -r @@REPO_ROOT@@/scripts/arid_resume_prompt.sh ] && . @@REPO_ROOT@@/scripts/arid_resume_prompt.sh
# END ARID SETUP
ARIDRC

        # Token substitution pass. Each sed RHS is passed through _sed_rhs_escape() which
        # neutralises sed's metacharacters (&, \, and the chosen | delimiter) so a future
        # path that contains those characters cannot corrupt the rewrite.
        _sed_rhs_escape() { printf '%s' "$1" | sed -e 's/[\\&|]/\\&/g'; }
        sed -i "s|@@WORKSPACES@@|$(_sed_rhs_escape "${WORKSPACES}")|g"     "$tmpfile"
        sed -i "s|@@LOCAL_WS@@|$(_sed_rhs_escape "${LOCAL_WS}")|g"         "$tmpfile"
        sed -i "s|@@ISAAC_ROS_WS@@|$(_sed_rhs_escape "${ISAAC_ROS_WS}")|g" "$tmpfile"
        sed -i "s|@@REPO_ROOT@@|$(_sed_rhs_escape "${SCRIPT_DIR}")|g"      "$tmpfile"

        # Fail loud if any @@TOKEN@@ slipped through.
        if grep -Eq '@@(WORKSPACES|LOCAL_WS|ISAAC_ROS_WS|REPO_ROOT)@@' "$tmpfile"; then
            err "token substitution incomplete - stray @@TOKEN@@ in rewritten bashrc; aborting"
            rm -f "$tmpfile"
            exit 1
        fi

        # Atomic swap.
        mv -- "$tmpfile" "$BASHRC_FILE"
    ) 9>"${lockfile}" || { rm -f "$tmpfile"; return 1; }

    STEPS_RUN+=("bashrc")
    ok ".bashrc updated"
}

# Sudoers, udev, polkit, groups
setup_permissions() {
    step "Sudoers, udev, polkit, groups"

    sudo tee "$SUDOERS_FILE" > /dev/null << EOF
${USERNAME} ALL=(ALL) NOPASSWD: /usr/sbin/uhubctl, /usr/bin/gpioset, /bin/systemctl start *, /bin/systemctl stop *, /bin/systemctl restart *, /bin/systemctl kill *, /bin/systemctl reset-failed *, /bin/systemctl enable *, /bin/systemctl disable *, ${WORKSPACES}/scripts/usb_reset.sh, /usr/sbin/zerotier-cli
EOF
    sudo chmod 440 "$SUDOERS_FILE"
    ok "Sudoers rule written"

    # udev rules
    sudo tee /etc/udev/rules.d/52-usb.rules > /dev/null << 'EOL'
SUBSYSTEM=="usb", DRIVER=="usb", MODE="0664", GROUP="dialout", ATTR{idVendor}=="2109"
SUBSYSTEM=="usb", DRIVER=="usb", MODE="0664", GROUP="dialout", ATTR{idVendor}=="1d6b"
SUBSYSTEM=="usb", DRIVER=="usb", \
  RUN+="/bin/sh -c \"chown -f root:dialout $sys$devpath/*port*/disable || true\"", \
  RUN+="/bin/sh -c \"chmod -f 660 $sys$devpath/*port*/disable || true\""
EOL

    sudo tee /etc/udev/rules.d/99-gpio.rules > /dev/null << 'EOL'
SUBSYSTEM=="gpio", GROUP=="gpio", MODE=="0660"
EOL

    sudo udevadm control --reload-rules
    ok "udev rules written and reloaded"

    # Groups
    sudo usermod -aG dialout,gpio "${USERNAME}"
    ok "Groups: dialout, gpio"

    # Polkit rule for reset_usb.service
    sudo tee "$POLKIT_RULE_FILE" > /dev/null << EOF
polkit.addRule(function(action, subject) {
    if (action.id == "org.freedesktop.systemd1.manage-units" &&
        action.lookup("unit") == "reset_usb.service" &&
        subject.user == "$USERNAME") {
        return polkit.Result.YES;
    }
});
EOF
    sudo chmod 644 "$POLKIT_RULE_FILE"
    ok "Polkit rule written"

    STEPS_RUN+=("permissions")
}

# uhubctl
setup_uhubctl() {
    step "uhubctl"

    if command -v uhubctl >/dev/null 2>&1; then
        skip "uhubctl already installed"
        STEPS_SKIPPED+=("uhubctl")
        return
    fi

    local UHUBCTL_DIR="${HOME_DIR}/uhubctl"
    if [[ ! -d "${UHUBCTL_DIR}" ]]; then
        git clone https://github.com/mvp/uhubctl "${UHUBCTL_DIR}"
    fi

    (
        cd "${UHUBCTL_DIR}"
        make
        sudo make install
    )

    STEPS_RUN+=("uhubctl")
    ok "uhubctl installed"
}

# systemd services
setup_systemd() {
    step "systemd services"

    # One-time cutover from the pre-rename supervisor unit: stop, disable, and remove the
    # stale vslam_supervisor.service so it cannot race the renamed arid_supervisor.service.
    if [[ -f /etc/systemd/system/vslam_supervisor.service ]]; then
        sudo -n systemctl stop    vslam_supervisor.service 2>/dev/null || true
        sudo -n systemctl disable vslam_supervisor.service 2>/dev/null || true
        sudo rm -f /etc/systemd/system/vslam_supervisor.service
        ok "stale vslam_supervisor.service removed (renamed to arid_supervisor.service)"
    fi

    sudo cp -f "${ISAAC_ROS_WS}/services/"*.service "/etc/systemd/system/"
    sudo cp -f "${LOCAL_WS}/services/"*.service "/etc/systemd/system/"

    # Global ROS env for ALL systemd services (redundant catch-all alongside each unit's own
    # Environment= lines). ROS_LOCALHOST_ONLY=1 confines DDS to loopback+shm so discovery never
    # leaks onto WiFi/ZeroTier/other interfaces; non-ROS services simply ignore these vars.
    # daemon-reexec (not just daemon-reload) is required for manager DefaultEnvironment to apply.
    #
    # PAIRING (critical): ROS_LOCALHOST_ONLY=1 only works end-to-end if the FMU's uXRCE-DDS client
    # ALSO confines its participant to loopback - PX4 param UXRCE_DDS_PTCFG=1 (reboot_required).
    # If PTCFG stays 0 the FMU participant advertises off-box interfaces and the localhost-only
    # nodes reject it -> ALL /fmu/out telemetry + inbound /fmu/in VIO silently vanish.
    sudo install -d /etc/systemd/system.conf.d
    sudo tee /etc/systemd/system.conf.d/10-arid-ros-env.conf >/dev/null << 'EOF'
[Manager]
DefaultEnvironment=ROS_DOMAIN_ID=23 ROS_LOCALHOST_ONLY=1
EOF
    sudo systemctl daemon-reexec

    sudo systemctl enable usbfs-memory.service
    sudo systemctl enable usb_ros_reset.service
    sudo systemctl enable start_isaac_docker.service
    sudo systemctl enable arid_supervisor.service
    sudo systemctl enable jetson-clocks.service
    sudo systemctl enable gst_camera_manager.service
    sudo systemctl enable arid_description.service
    sudo systemctl daemon-reload

    STEPS_RUN+=("systemd")
    ok "Services enabled and daemon reloaded"
}

# Python packages
# --break-system-packages: required on PEP 668 systems, unknown to pip < 23.0.1.
PIP_BREAK_FLAG=""
if python3 -m pip install --help 2>/dev/null | grep -q -- '--break-system-packages'; then
    PIP_BREAK_FLAG="--break-system-packages"
fi

ensure_pip_pkg() {
    local pkg="$1"
    local name="$2"
    local want_version="$3"

    local have
    have=$(python3 -m pip show "$name" 2>/dev/null | awk '/^Version: / {print $2}' || true)

    if [[ -z "$have" ]]; then
        ok "Installing ${pkg}..."
        python3 -m pip install "$pkg" --no-deps ${PIP_BREAK_FLAG}
    elif [[ -n "$want_version" && "$have" != "$want_version" ]]; then
        ok "Upgrading ${name} from ${have} to ${want_version}..."
        python3 -m pip install "$pkg" --no-deps ${PIP_BREAK_FLAG}
    else
        skip "${name}==${have} already satisfies ${pkg}"
    fi
}

# ROS2 local workspace
setup_ros_workspace() {
    step "ROS2 local workspace"

    ensure_pip_pkg "pyudev==0.24.3"     "pyudev"     "0.24.3"
    ensure_pip_pkg "pyserial==3.5"      "pyserial"   "3.5"
    ensure_pip_pkg "empy<4"             "empy"       ""

    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
        sudo rosdep init
    else
        skip "rosdep already initialized"
    fi

    rosdep update
    rosdep install --from-paths "${LOCAL_WS}/src/" --ignore-src -y

    (
        cd "${LOCAL_WS}"
        colcon build \
            --symlink-install \
            --base-paths src \
            --event-handlers console_direct+ \
            --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON
    )

    STEPS_RUN+=("ros_workspace")
    ok "ROS2 local workspace built"
}

# Docker (each sub-step is state-detected and idempotent)
setup_docker() {
    step "Docker"

    if ! command -v docker >/dev/null 2>&1; then
        curl https://get.docker.com | sh -s -- --version 29.2.1
        ok "Docker engine installed"
    else
        skip "Docker binary already present"
    fi

    if ! systemctl is-enabled --quiet docker.service 2>/dev/null \
       || ! systemctl is-active --quiet docker.service 2>/dev/null; then
        sudo systemctl --now enable docker
        ok "Docker service enabled + started"
    else
        skip "Docker service already enabled and active"
    fi

    if ! docker info 2>/dev/null | grep -q 'nvidia'; then
        sudo nvidia-ctk runtime configure --runtime=docker
        sudo systemctl restart docker
        ok "NVIDIA container runtime configured"
    else
        skip "NVIDIA container runtime already configured"
    fi

    if id -nG "${USERNAME}" 2>/dev/null | grep -qw docker; then
        skip "${USERNAME} already in docker group"
    else
        sudo usermod -aG docker "${USERNAME}"
        ok "${USERNAME} added to docker group (log out + back in to take effect)"
    fi

    if dpkg -s docker-buildx-plugin >/dev/null 2>&1; then
        skip "docker-buildx-plugin already installed"
    else
        sudo apt-get install -y docker-buildx-plugin
        ok "docker-buildx-plugin installed"
    fi

    STEPS_RUN+=("docker")
}

# RealSense serial → mount: confirm 3 on USB, then config_realsense.sh assigns each to a mount.
setup_realsense() {
    step "RealSense serial mapping (front/left/right)"

    # Host RealSense python bindings - needed by config_realsense.sh to open the live
    # feed and stream each camera by its serial (aarch64 wheel on PyPI).
    ensure_pip_pkg "pyrealsense2" "pyrealsense2" ""
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

    # In full setup the assign/skip choice was made up front; pass it through so
    # config_realsense.sh doesn't prompt again (the live per-camera step still runs if yes).
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

# Hostname / password
first_boot() {
    step "First-boot hostname / password"

    local sentinel="${HOME_DIR}/.arid_provisioned"
    if [[ -f "${sentinel}" ]]; then
        skip "already provisioned"
        STEPS_SKIPPED+=("first_boot")
        return 0
    fi

    local new_host="${PRE_HOST:-arid}"
    if (( ! PRE )); then
        read -r -p "Set hostname? (y/n, Enter = ${new_host}): " a || a=""
        if is_yes "$a"; then read -r -p "  Hostname: " new_host || new_host=""; fi
    fi
    if [[ -n "${new_host}" ]] && [[ "${new_host}" != "$(hostname)" ]]; then
        sudo hostnamectl set-hostname "${new_host}"
        sudo sed -i "s/127\.0\.1\.1.*/127.0.1.1\t${new_host}/" /etc/hosts || true
        ok "hostname set to ${new_host}"
    fi

    if [[ -n "${PRE_PASS}" ]]; then
        echo "${USERNAME}:${PRE_PASS}" | sudo chpasswd
        ok "password updated"
    elif (( ! PRE )); then
        read -r -p "Set ${USERNAME} password? (y/n, Enter = leave unchanged): " a || a=""
        if is_yes "$a"; then sudo passwd "${USERNAME}" || warn "password change cancelled"; fi
    fi

    touch "${sentinel}"
    STEPS_RUN+=("first_boot")
}

# This unit is headless (no monitor/seat), so the user never gets an interactive
# login at boot and systemd-logind never creates /run/user/<uid>. A root process
# (the setup resume, a boot service) then creates it root-owned, and every user
# session that follows - including NoMachine's desktop session - cannot write its
# runtime sockets there (dbus/pulse: "Permission denied"), so the session exits and
# NoMachine shows a black screen then disconnects. enable-linger makes systemd start
# user@<uid>.service at boot and own /run/user/<uid> (0700, user-owned) first.
enable_user_linger() {
    step "User linger (headless runtime dir)"
    local uid; uid="$(id -u "${USERNAME}")"
    sudo loginctl enable-linger "${USERNAME}" || warn "enable-linger failed"
    # Correct a runtime dir already created root-owned earlier this boot.
    if [[ -d "/run/user/${uid}" && "$(stat -c '%U' "/run/user/${uid}" 2>/dev/null)" != "${USERNAME}" ]]; then
        sudo chown -R "${USERNAME}:${USERNAME}" "/run/user/${uid}"
        sudo chmod 700 "/run/user/${uid}"
        ok "Fixed root-owned /run/user/${uid}"
    fi
    STEPS_RUN+=("enable_user_linger")
    ok "Lingering enabled for ${USERNAME} - runtime dir persists across boots"
}

# Desktop cleanup - removes NVIDIA's default first-boot icons + the L4T-README auto-mount.
clean_nvidia_desktop() {
    step "Desktop cleanup (NVIDIA first-boot icons)"

    local desk="${HOME_DIR}/Desktop" removed=0 f
    for f in nv_jetson_zoo nv_devzone nv_jetson_projects nv_l4t_readme nv_forums; do
        if [[ -e "${desk}/${f}.desktop" ]]; then rm -f "${desk}/${f}.desktop"; removed=$((removed + 1)); fi
    done
    (( removed )) && ok "removed ${removed} NVIDIA desktop shortcut(s)" || skip "no NVIDIA desktop shortcuts present"

    # The "L4T-README folder" is really the L4T-README partition auto-mounted at login.
    local mp="/media/${USERNAME}/L4T-README"
    if mount 2>/dev/null | grep -qF "${mp}"; then
        udisksctl unmount -b /dev/disk/by-label/L4T-README >/dev/null 2>&1 \
            || sudo umount "${mp}" 2>/dev/null || true
    fi
    [[ -d "${mp}" ]] && rmdir "${mp}" 2>/dev/null || true

    if [[ -e /etc/xdg/autostart/nvl4t-readme.sh ]]; then
        sudo rm -f /etc/xdg/autostart/nvl4t-readme.sh && ok "removed L4T-README auto-mount autostart"
    else
        skip "L4T-README autostart already absent"
    fi

    STEPS_RUN+=("desktop_cleanup")
}

# Disable unattended-upgrades on a deployed drone (predictable boot, no surprise updates).
disable_updates() {
    step "Disable unattended apt upgrades"
    if [[ -f /etc/apt/apt.conf.d/20auto-upgrades ]]; then
        sudo sed -i 's|^APT::Periodic::Unattended-Upgrade.*|APT::Periodic::Unattended-Upgrade "0";|' \
            /etc/apt/apt.conf.d/20auto-upgrades 2>/dev/null || true
        sudo sed -i 's|^APT::Periodic::Update-Package-Lists.*|APT::Periodic::Update-Package-Lists "0";|' \
            /etc/apt/apt.conf.d/20auto-upgrades 2>/dev/null || true
    fi
    sudo systemctl disable --now unattended-upgrades.service 2>/dev/null || true
    sudo systemctl disable --now apt-daily.timer apt-daily-upgrade.timer 2>/dev/null || true
    STEPS_RUN+=("disable_updates")
    ok "unattended upgrades disabled"
}

# Enable network time sync so PX4 timestamp alignment stays correct after a power cycle.
enable_clock_sync() {
    step "Enable network time sync"
    sudo systemctl enable --now systemd-timesyncd.service 2>/dev/null || true
    STEPS_RUN+=("clock_sync")
    ok "systemd-timesyncd active"
}

# Interactive Wi-Fi connect (calls scripts/wifi.sh; honours pre-collected SSID / password).
ensure_wifi() {
    step "Wi-Fi"
    if nmcli -t -f TYPE,STATE device status 2>/dev/null | grep -q '^wifi:connected'; then
        local cur
        cur=$(nmcli -t -f NAME,TYPE connection show --active 2>/dev/null \
            | awk -F: '/:802-11-wireless$/{print $1; exit}')
        ok "already connected${cur:+ to '${cur}'}"
    fi
    if [[ "${PRE_WIFI:-}" == "skip" ]]; then
        skip "Wi-Fi (declined)"
        STEPS_SKIPPED+=("wifi")
        return 0
    fi
    if [[ "${PRE_WIFI:-}" == "yes" && -n "${PRE_WIFI_SSID:-}" ]]; then
        ARID_WIFI_SSID="${PRE_WIFI_SSID}" ARID_WIFI_PASS="${PRE_WIFI_PASS:-}" \
            bash "${WORKSPACES}/scripts/wifi.sh" || warn "wifi.sh returned non-zero"
        STEPS_RUN+=("wifi")
        return 0
    fi
    if (( PRE )); then return 0; fi
    local a=""
    read -r -p "Connect to a Wi-Fi network now? (y/n, Enter = skip): " a || a=""
    if is_yes "$a"; then
        bash "${WORKSPACES}/scripts/wifi.sh" || warn "wifi.sh returned non-zero"
        STEPS_RUN+=("wifi")
    else
        skip "Wi-Fi (declined)"
        STEPS_SKIPPED+=("wifi")
    fi
}

# NoMachine install (idempotent; deferred to the user if a fresh install is needed).
nomachine() {
    step "NoMachine"
    if dpkg -s nomachine >/dev/null 2>&1 || [[ -x /usr/NX/bin/nxserver ]]; then
        if [[ "${PRE_NOMACHINE:-}" == "skip" ]]; then
            skip "NoMachine already installed (upgrade declined)"
            STEPS_SKIPPED+=("nomachine")
            return 0
        fi
        if (( PRE )); then
            ok "NoMachine already installed"
            STEPS_SKIPPED+=("nomachine")
            return 0
        fi
        local a=""
        read -r -p "NoMachine already installed. Reinstall? (y/n, Enter = skip): " a || a=""
        if ! is_yes "$a"; then
            ok "NoMachine already installed"
            STEPS_SKIPPED+=("nomachine")
            return 0
        fi
    fi
    warn "NoMachine arm64 .deb must be downloaded from https://www.nomachine.com manually."
    warn "After downloading: sudo dpkg -i nomachine_*_arm64.deb && sudo /usr/NX/bin/nxserver --restart"
    STEPS_SKIPPED+=("nomachine (manual)")
}

# ZeroTier. Install the daemon (official installer, adds the ZT apt repo) and optionally join
# a network. Joining goes through scripts/zt_join.sh (single-network model: leaves any other
# joined network first). A join that lands ACCESS_DENIED is NOT a provisioning failure: the
# node is joined and starts working the moment it is authorized in ZeroTier Central.
setup_zerotier() {
    step "ZeroTier"

    if ! command -v zerotier-cli >/dev/null 2>&1; then
        curl -s https://install.zerotier.com | sudo bash || true
        if ! command -v zerotier-cli >/dev/null 2>&1; then
            err "ZeroTier install failed (installer needs internet)"
            return 1
        fi
        ok "ZeroTier installed"
    fi
    sudo systemctl enable --now zerotier-one >/dev/null 2>&1 || true

    # Already a member of a network: leave it alone (switching is zt_join's job).
    local joined
    joined=$(sudo zerotier-cli -j listnetworks 2>/dev/null \
        | python3 -c "import json,sys;print(' '.join(n['nwid'] for n in json.load(sys.stdin)))" 2>/dev/null || true)
    if [[ -n "${joined// /}" ]]; then
        skip "already on ZeroTier network(s): ${joined} - use 'zt_join' to switch"
        STEPS_RUN+=("zerotier")
        return
    fi

    local net=""
    if (( PRE )); then
        net="${PRE_ZTNET:-}"
    else
        read -r -p "  ZeroTier network id to join (16 hex chars, Enter = skip): " net || net=""
    fi
    net="${net// /}"
    if [[ -z "${net}" || "${net}" == "skip" ]]; then
        skip "no ZeroTier network joined - run 'zt_join' later if needed"
        STEPS_RUN+=("zerotier")
        return
    fi
    if bash "${WORKSPACES}/scripts/zt_join.sh" --setup "${net}"; then
        ok "ZeroTier network ${net} joined"
    else
        warn "ZeroTier join incomplete (authorize this node in ZeroTier Central, or run 'zt_join' later)"
    fi
    STEPS_RUN+=("zerotier")
}

# Foxglove + live cam_front/cam_down feed (operator-selected); operator tunes the lens focus.
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

    # Which CSI camera to focus: front (sensor-id 0) or down (sensor-id 1).
    local cam="" sid=""
    while [[ "${cam}" != "cam_front" && "${cam}" != "cam_down" ]]; do
        read -r -p "  Focus which camera? (front/down, Enter = down): " cam || cam=""
        case "${cam,,}" in
            ""|d|down|cam_down)   cam="cam_down";  sid=1 ;;
            f|front|cam_front)    cam="cam_front"; sid=0 ;;
            *) warn "choose front or down"; cam="" ;;
        esac
    done

    echo "  Starting ${cam}..."
    ros2 service call /gst_camera_manager/${cam} std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1 || true
    sleep 2
    ros2 service call /gst_camera_manager/${cam} std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1 \
        && ok "${cam} streaming" \
        || warn "${cam} failed to start (is another consumer holding sensor-id=${sid}?)"

    echo "  Starting Foxglove bridge..."
    # shellcheck disable=SC2086
    setsid ${FOXGLOVE_LAUNCH} >/tmp/arid_focus_foxglove.log 2>&1 &
    local fox_pgid=$!
    sleep 3

    local fox_port; fox_port=$(grep -oE 'port:=[0-9]+' <<<"${FOXGLOVE_LAUNCH}" | grep -oE '[0-9]+')
    fox_port="${fox_port:-8765}"
    echo ""
    echo -e "  ${BOLD}Open Foxglove Studio -> Open connection -> Foxglove WebSocket. Connect to:${NC}"
    local ip found=0
    for ip in $(ip -4 -o addr show scope global 2>/dev/null \
            | awk '$2 !~ /^(docker|br-|veth|l4tbr|usb|virbr)/ {print $4}' | cut -d/ -f1); do
        echo -e "    ${GREEN}ws://${ip}:${fox_port}${NC}"; found=1
    done
    (( found )) || warn "no routable IP detected - check 'ip addr' (port ${fox_port})"
    echo "  Add an Image panel for: /${cam}/image_raw/compressed"
    echo ""
    echo "  Adjust focus in Foxglove, then press q to stop and return to the menu."
    while read -rsn1 -t 0.1 _ 2>/dev/null; do :; done
    local key=""
    while [[ "$key" != "q" && "$key" != "Q" ]]; do read -rn1 key || break; done

    echo "  Stopping Foxglove bridge and ${cam}..."
    kill -INT  -- -"${fox_pgid}" 2>/dev/null || true
    sleep 1
    kill -KILL -- -"${fox_pgid}" 2>/dev/null || true
    ros2 service call /gst_camera_manager/${cam} std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1 || true
    ok "stopped"
}

# Wrappers for the menu / questionnaire-driven flows.
verify_cameras() {
    if [[ "${PRE_VERIFY:-}" == "skip" ]]; then
        skip "camera verification (declined)"
        STEPS_SKIPPED+=("verify_cameras")
        return 0
    fi
    if (( PRE )) && [[ "${PRE_VERIFY:-}" != "yes" ]]; then return 0; fi
    if (( ! PRE )); then
        local a=""
        read -r -p "Verify the camera feed now? (y/n, Enter = skip): " a || a=""
        is_yes "$a" || { STEPS_SKIPPED+=("verify_cameras"); return 0; }
    fi
    bash "${WORKSPACES}/scripts/verify_cv_cams.sh" || warn "camera verification reported failure"
    STEPS_RUN+=("verify_cameras")
}

verify_cameras_menu() {
    bash "${WORKSPACES}/scripts/verify_cv_cams.sh"
}

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

# Build the Isaac container with a confirmation prompt. Image present -> "Rebuild?"
# default skip. Image absent -> "Continue with building?" default skip; strict yes on both.
build_isaac_step() {
    step "Build Isaac container"
    local sentinel="${HOME_DIR}/.arid_pending_build_isaac"
    local exists=0 ans build_rc=0
    if docker image inspect isaac_ros_dev-aarch64-container >/dev/null 2>&1; then exists=1; fi
    if (( PRE )); then
        ans="${PRE_BUILD_ISAAC:-skip}"   # already chosen in the questionnaire - don't re-prompt
    elif (( exists )); then
        read -r -p "Isaac container image already exists. Rebuild it now? (y/n, Enter = no): " ans || ans=""
    else
        read -r -p "Continue with building the Isaac container? (y/n, Enter = no): " ans || ans=""
    fi
    # Strict yes on both prompts: garbage input is not a green light for an expensive operation.
    if ! is_yes "${ans}"; then
        skip "Isaac container build deferred (rebuild any time with build_isaac)"
        STEPS_SKIPPED+=("build_isaac")
        rm -f "${sentinel}"
        return 0
    fi
    # </dev/null: run_dev.sh must never attach interactively during an unattended setup run.
    if /bin/bash "${ISAAC_ROS_WS}/container_scripts/build_isaac_docker.sh" </dev/null; then
        STEPS_RUN+=("build_isaac")
        rm -f "${sentinel}"
        ok "Isaac container build complete"
        return 0
    else
        build_rc=$?
        err "build_isaac_docker.sh returned ${build_rc} - container image not built"
        STEPS_RUN+=("build_isaac (failed)")
        # Keep the sentinel on FAILURE so a post-reboot resume retries the queued build.
        return "${build_rc}"
    fi
}

_run_build_isaac_if_queued() {
    local sentinel="${HOME_DIR}/.arid_pending_build_isaac"
    [[ -f "${sentinel}" ]] || return 0
    build_isaac_step
}

# Colcon-build the in-container workspace and bring the supervisor up.
# Prerequisite: the Isaac container image must exist (built by build_isaac_step). The
# supervisor service launches `ros2 launch arid_supervisor arid_supervisor.launch.py`
# inside the container; without an in-container install/setup.bash carrying that package
# the unit fails the StartLimitBurst at boot.
colcon_isaac_step() {
    step "Colcon-build in-container workspace"
    local container="isaac_ros_dev-aarch64-container"
    if ! docker image inspect "${container}" >/dev/null 2>&1; then
        skip "Isaac container image not built; run build_isaac first"
        STEPS_SKIPPED+=("colcon_isaac")
        return 0
    fi
    local ans
    # When `--full` queued the container build, run colcon without re-prompting so the
    # supervisor comes up at boot. Otherwise prompt, defaulting to yes so an Enter-press
    # proceeds with the build. The operator can still opt out with an explicit n / no.
    if (( PRE )) && [[ "${PRE_BUILD_ISAAC:-}" == "yes" ]]; then
        ans="y"
    else
        read -r -p "Colcon-build the workspace inside the container now? (y/n, Enter = yes): " ans || ans=""
    fi
    if is_no "${ans}"; then
        skip "colcon_isaac deferred (run colcon_isaac inside the container any time)"
        STEPS_SKIPPED+=("colcon_isaac")
        return 0
    fi

    # Container must be running. Refuse to colcon-build into a stopped container so the
    # operator sees the explicit dependency rather than getting a confusing build failure.
    if ! docker inspect -f '{{.State.Running}}' "${container}" 2>/dev/null | grep -q true; then
        err "Isaac container is not running."
        err "Start it first: 'start_isaac' (alias) or 'sudo systemctl start start_isaac_docker.service'"
        STEPS_SKIPPED+=("colcon_isaac (container down)")
        return 1
    fi

    # Self-heal a poisoned container: apt ros-humble-librealsense2 is the 2.57.x V4L2 build and must
    # never shadow the RSUSB 2.55.1 librealsense at /usr/local (wrapper linked against it dies on
    # bringup with set_xu/UVCIOC_CTRL_QUERY "No such device"). Remove it before building.
    if docker exec -u root "${container}" dpkg -l ros-humble-librealsense2 2>/dev/null | grep -q '^ii'; then
        warn "apt ros-humble-librealsense2 found in the container (V4L2, conflicts with the RSUSB /usr/local build) - removing"
        if docker exec -u root "${container}" apt-get remove -y ros-humble-librealsense2 >/dev/null; then
            ok "apt ros-humble-librealsense2 removed from the container"
        else
            warn "could not remove ros-humble-librealsense2 - continuing; -Drealsense2_DIR still pins the link to /usr/local"
        fi
    fi

    echo "  Building workspace (several minutes on a cold cache)..."
    # Bounded + non-interactive so a stuck build returns control to setup instead of hanging.
    # -u admin: bare exec is root (image default), which poisons build/install/log against the aliases.
    if timeout 3600 docker exec -u admin "${container}" bash -lc \
        'cd /workspaces/isaac_ros-dev && colcon build --symlink-install --base-paths src --cmake-args -DBUILD_TESTING=OFF -Drealsense2_DIR=/usr/local/lib/cmake/realsense2'; then
        ok "colcon build complete"
        STEPS_RUN+=("colcon_isaac")
        # Bring the supervisor up now that the workspace is built. Reset-failed clears
        # StartLimitBurst from the first-boot failures; restart picks up the new image.
        sudo -n systemctl reset-failed arid_supervisor.service 2>/dev/null || true
        sudo -n systemctl restart arid_supervisor.service 2>/dev/null \
            || warn "arid_supervisor.service restart failed - check 'sudo systemctl status arid_supervisor.service'"
        return 0
    fi
    local rc=$?
    err "colcon build returned ${rc}"
    STEPS_RUN+=("colcon_isaac (failed)")
    return "${rc}"
}

# Post-install validation: services up, ROS graph contract, no orphans.
# Uninstall: undo the host-side state that setup.sh creates. Strict explicit confirmation;
# never called from run_full_setup or run_resume. Leaves the repo clone, hostname, password,
# group memberships, apt holds, ROS / JetPack / Docker engine in place. Removes the Isaac
# container image so a re-install starts from a clean slate.
setup_uninstall() {
    echo ""
    echo -e "${RED}${BOLD}============== ARID uninstall ==============${NC}"
    echo "  This will stop and remove every systemd unit installed by setup.sh,"
    echo "  delete /etc/sudoers.d/jetson_systemctl, the polkit rule, the ARID block"
    echo "  in ~/.bashrc, the docker patches inside isaac_ros_common, every setup.sh"
    echo "  sentinel file, and the Isaac container + image."
    echo ""
    echo -e "${BOLD}Left in place:${NC} the repo clone, hostname, password, group memberships,"
    echo "  apt-mark holds, ROS 2 Humble, JetPack, the Docker engine itself, and any other"
    echo "  state setup.sh did not write."
    echo -e "${RED}${BOLD}============================================${NC}"
    local ans
    read -r -p "Type 'yes' to proceed: " ans || ans=""
    if [[ "${ans}" != "yes" ]]; then
        echo "  Aborted."
        return 0
    fi

    step "Stop + disable + remove systemd units"
    local units=(
        usbfs-memory.service
        jetson-clocks.service
        start_isaac_docker.service
        arid_supervisor.service
        gst_camera_manager.service
        arid_description.service
        usb_ros_reset.service
        reset_usb.service
    )
    for u in "${units[@]}"; do
        sudo -n systemctl stop    "$u" 2>/dev/null || true
        sudo -n systemctl disable "$u" 2>/dev/null || true
        sudo rm -f "/etc/systemd/system/$u" 2>/dev/null || true
    done
    sudo systemctl daemon-reload 2>/dev/null || true
    sudo systemctl reset-failed 2>/dev/null || true
    ok "systemd units removed"

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
    while IFS= read -r dir; do
        local rel_dir
        rel_dir=$(realpath --relative-to="$REPO_ROOT" "$dir")
        local files
        files=$(git -C "$REPO_ROOT" ls-files "$rel_dir" 2>/dev/null || true)
        if [[ -n "$files" ]]; then
            echo "$files" | xargs git -C "$REPO_ROOT" update-index --no-skip-worktree 2>/dev/null || true
        fi
    done < <(find "${ISAAC_ROS_WS}/src" "${LOCAL_WS}/src" -type d \( -name "config" -o -name "cfg" -o -name "camera_calibrations" \) 2>/dev/null)
    if [[ -d "${ISAAC_ROS_WS}/src/isaac_ros_common/.git" ]] || [[ -f "${ISAAC_ROS_WS}/src/isaac_ros_common/.git" ]]; then
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
        "${HOME_DIR}/.arid_resume.lock" \
        "${HOME_DIR}/.arid_setup_log" \
        "${HOME_DIR}/.arid_questionnaire" 2>/dev/null || true
    ok "sentinel files removed"

    step "Remove Isaac container + image"
    docker rm -f isaac_ros_dev-aarch64-container 2>/dev/null || true
    docker rmi -f isaac_ros_dev-aarch64-container 2>/dev/null || true
    docker images --format '{{.Repository}}:{{.Tag}}' 2>/dev/null \
        | grep -E '^isaac_ros_dev-aarch64-container' \
        | xargs -r docker rmi -f 2>/dev/null || true
    ok "container + image removed"

    echo ""
    echo -e "${GREEN}${BOLD}Uninstall complete.${NC}"
    echo "  Open a new shell to drop the ARID environment from your current bashrc."
    echo "  The repo clone, hostname, and password are unchanged."
}

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

# Reboot prompt. Arms the bashrc resume hook so the smoke test fires on the next
# interactive terminal post-reboot. If `sudo reboot` itself fails, the flag is rolled
# back so the next login does not prompt for a resume that never actually happened.
prompt_reboot() {
    echo ""
    local do_reboot="${PRE_REBOOT:-y}"
    if (( ! PRE )); then read -r -p "Reboot now? (y/n): " do_reboot || do_reboot="n"; fi
    if is_yes "$do_reboot"; then
        echo -e "${YELLOW}${BOLD}Rebooting now.${NC}"
        touch "${HOME_DIR}/.arid_resume_setup"
        if ! sudo reboot; then
            warn "sudo reboot failed - rolling back the resume flag"
            rm -f "${HOME_DIR}/.arid_resume_setup"
            return 1
        fi
    else
        echo -e "${YELLOW}${BOLD}A reboot is required for all changes to take effect.${NC}"
        echo "Remember to reboot before using this system - the smoke test runs on the next post-reboot terminal."
    fi
}

# Front-load every decision so the rest of the setup can run unattended. Each step
# consults its PRE_* answer instead of prompting. Live-camera steps still show their
# feeds when reached.
collect_answers() {
    step "Setup questionnaire - answer once; setup then runs without further prompts"
    PRE=1
    local a

    if [[ -f "${HOME_DIR}/.arid_provisioned" ]]; then
        echo "  Already provisioned - hostname / password unchanged."
    else
        read -r -p "  Set hostname? (y/n, Enter = arid): " a || a=""
        is_yes "$a" && { read -r -p "    Hostname: " PRE_HOST || PRE_HOST=""; }
        read -r -p "  Set password? (y/n, Enter = leave unchanged): " a || a=""
        if is_yes "$a"; then read -r -s -p "    Password: " PRE_PASS || PRE_PASS=""; echo ""; fi
    fi

    if nmcli -t -f TYPE,STATE device status 2>/dev/null | grep -q '^wifi:connected'; then
        local cur
        cur=$(nmcli -t -f NAME,TYPE connection show --active 2>/dev/null \
            | awk -F: '/:802-11-wireless$/{print $1; exit}')
        ok "Wi-Fi: connected${cur:+ to '${cur}'}"
    else
        warn "Wi-Fi: not connected"
    fi
    read -r -p "  Connect to Wi-Fi? (y/n, Enter = skip): " a || a=""
    if is_yes "$a"; then
        read -r -p "    SSID: " PRE_WIFI_SSID || PRE_WIFI_SSID=""
        read -r -s -p "    Password (empty = open): " PRE_WIFI_PASS || PRE_WIFI_PASS=""; echo ""
        PRE_WIFI=yes
    else
        PRE_WIFI=skip
    fi

    if dpkg -s nomachine >/dev/null 2>&1 || [[ -x /usr/NX/bin/nxserver ]]; then
        read -r -p "  Reinstall NoMachine? (y/n, Enter = skip): " a || a=""
        is_yes "$a" && PRE_NOMACHINE=yes || PRE_NOMACHINE=skip
    else
        PRE_NOMACHINE=install
    fi

    # ZeroTier network to join (blank = install the daemon only). Validated here so a typo
    # is caught at questionnaire time, not mid-provision. Already-a-member drones are not
    # asked at all: setup never switches networks (that is zt_join's job).
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
        read -r -p "  Install PX4 toolchain? (y/n, Enter = no): " a || a=""
        is_yes "$a" && PRE_PX4=yes || PRE_PX4=no
    fi

    read -r -p "  Assign the three RealSense cameras (front/left/right) into vslam_config.yaml? (y/n, Enter = skip): " a || a=""
    is_yes "$a" && PRE_REALSENSE=yes || PRE_REALSENSE=skip

    read -r -p "  Verify the camera feed at the end? (y/n, Enter = skip): " a || a=""
    is_yes "$a" && PRE_VERIFY=yes || PRE_VERIFY=skip

    if docker image inspect isaac_ros_dev-aarch64-container >/dev/null 2>&1; then
        read -r -p "  Rebuild the Isaac container? (y/n, Enter = skip): " a || a=""
    else
        read -r -p "  Build the Isaac container? (y/n, Enter = skip): " a || a=""
    fi
    if is_yes "$a"; then
        PRE_BUILD_ISAAC=yes
        touch "${HOME_DIR}/.arid_pending_build_isaac"
    else
        PRE_BUILD_ISAAC=skip
    fi

    # Run the local smoke test on the post-reboot resume? Default yes - it's the final
    # validation that boot-time services + the ROS graph contract are healthy.
    read -r -p "  Run smoke test on post-reboot resume? (y/n, Enter = y): " a || a=""
    is_no "$a" && PRE_SMOKE=skip || PRE_SMOKE=yes

    read -r -p "  Reboot when done? (y/n, Enter = y): " a || a=""
    is_no "$a" && PRE_REBOOT=no || PRE_REBOOT=yes

    ok "Answers recorded - running unattended."

    # Persist every questionnaire answer to disk so the post-reboot resume path can honour
    # them just like the pre-reboot pipeline does. Without this, any PRE_* answered before
    # the reboot would silently revert to the script's default in run_resume.
    _persist_questionnaire
}

# Write every PRE_* answer to ${HOME_DIR}/.arid_questionnaire as KEY=value lines. Sourced
# by run_resume() so the post-reboot half of the pipeline sees the same questionnaire state
# the pre-reboot half had. Secrets (PRE_PASS, PRE_WIFI_PASS) are deliberately NOT persisted -
# those gate first_boot / wifi steps which only ever run before the reboot. File mode 600.
_persist_questionnaire() {
    local f="${HOME_DIR}/.arid_questionnaire"
    umask 077
    {
        echo "# Generated by setup.sh collect_answers - sourced by run_resume."
        echo "PRE=1"
        for var in PRE_HOST PRE_WIFI PRE_WIFI_SSID PRE_NOMACHINE PRE_PX4 \
                   PRE_REALSENSE PRE_VERIFY PRE_SMOKE PRE_BUILD_ISAAC PRE_REBOOT PRE_ZTNET; do
            printf '%s=%q\n' "$var" "${!var-}"
        done
    } > "$f"
}

run_full_setup() {
    preflight
    printf '%s\n' "${LOG_FILE}" > "${HOME_DIR}/.arid_setup_log"   # pin this session's log across its reboots
    collect_answers
    first_boot
    setup_power
    disable_updates
    enable_clock_sync
    enable_user_linger
    clean_nvidia_desktop
    ensure_wifi
    nomachine
    setup_repos
    setup_apt_packages
    # After apt: the ZeroTier installer needs curl + working apt sources. A ZT failure is
    # NOT a provisioning failure (node joins later via zt_join), so never abort the run.
    setup_zerotier || true
    setup_px4_deps
    setup_git
    setup_docker_patches
    setup_skip_worktree
    setup_bashrc
    setup_permissions
    setup_uhubctl
    setup_ros_workspace
    # Docker engine + NVIDIA runtime must be in place before systemd enables
    # `start_isaac_docker.service`, otherwise a mid-run abort + reboot leaves the boot
    # service trying to start an absent daemon.
    setup_docker
    setup_systemd
    setup_realsense
    verify_cameras
    _run_build_isaac_if_queued
    colcon_isaac_step
    print_summary
    prompt_reboot
}

# Continue setup where it left off after a reboot. The smoke test runs after boot-enabled
# units have come up naturally and the ROS graph has settled. local_test.sh bootstraps
# any still-inactive required service, but the post-reboot run is the meaningful one.
run_resume() {
    # Prevent two terminals (both prompted by the bashrc resume hook) from racing on
    # cameras, port 8765, gst pipelines, etc. A second terminal exits if the lock is
    # already held. We DELIBERATELY do not rm the lock file on exit: flock is bound to
    # the inode, so leaving a zero-byte file on disk ensures all subsequent opens land
    # on the same inode and inherit the exclusion guarantee.
    local lock="${HOME_DIR}/.arid_resume.lock"
    if ! exec 9>"${lock}" 2>/dev/null; then
        warn "could not open resume lock at ${lock} - continuing without serialisation"
    elif ! flock -n 9; then
        echo "Another resume is already in progress in another terminal - exiting."
        return 1
    fi

    # Restore every PRE_* answer from the questionnaire file written by collect_answers.
    # Without this, PRE_VERIFY (and any other post-reboot-relevant answer) silently reverts
    # to the script's default. Missing file = pre-reboot questionnaire was not run (e.g. the
    # resume was triggered manually) - leave PRE unset and run the default flow.
    if [[ -r "${HOME_DIR}/.arid_questionnaire" ]]; then
        # shellcheck disable=SC1090
        set +u
        source "${HOME_DIR}/.arid_questionnaire" || true
        set -u
    fi

    if (( ${PRE:-0} )) && ! is_yes "${PRE_VERIFY:-}"; then
        step "Continuing setup where it left off - camera verification skipped (per questionnaire)"
    else
        step "Continuing setup where it left off - camera verification"
        if bash "${WORKSPACES}/scripts/verify_cv_cams.sh"; then
            ok "camera verification complete"
        else
            warn "camera verification incomplete - run 'ver_cv_cams' to retry"
        fi
    fi

    if [[ -f "${HOME_DIR}/.arid_pending_build_isaac" ]]; then
        _run_build_isaac_if_queued || true
        # Chain the workspace build so the supervisor unit can actually launch on the
        # advertise-wait that follows. The operator already opted in to building during the
        # questionnaire; PRE=1 + PRE_BUILD_ISAAC=yes short-circuits the second prompt.
        PRE=1
        PRE_BUILD_ISAAC=yes
        colcon_isaac_step || true
    fi

    # After a normal end-of-setup reboot, systemd starts start_isaac_docker +
    # arid_supervisor at boot, but the container + ROS launch take 30-90 s cold.
    # Without this wait, smoke-test Section 4 races the supervisor and reports false
    # negatives. The block is a no-op if the image does not exist or the supervisor is
    # already up.
    local have_image=0
    if docker image inspect isaac_ros_dev-aarch64-container >/dev/null 2>&1; then
        have_image=1
    fi
    if (( have_image == 0 )); then
        warn "Isaac container image not available - skipping supervisor wait."
        warn "Re-run 'build_isaac' once the underlying issue is resolved, then:"
        warn "sudo systemctl restart start_isaac_docker.service arid_supervisor.service"
    else
        step "Recovering Docker + supervisor units (if not yet running)"
        sudo -n systemctl reset-failed start_isaac_docker.service arid_supervisor.service 2>/dev/null || true
        sudo -n systemctl stop  start_isaac_docker.service arid_supervisor.service 2>/dev/null || true
        sudo -n systemctl start start_isaac_docker.service 2>/dev/null || warn "start_isaac_docker.service start failed"
        sudo -n systemctl start arid_supervisor.service    2>/dev/null || warn "arid_supervisor.service start failed"
        step "Waiting for arid_supervisor to advertise (up to 90 s)"
        set +u
        [[ -f /opt/ros/humble/setup.bash ]] && source /opt/ros/humble/setup.bash
        export ROS_DOMAIN_ID=23
        set -u
        local i ok_sup=0
        for i in $(seq 1 18); do
            if timeout 5 ros2 service list 2>/dev/null \
                | grep -qx /arid_supervisor/vslam_enable; then
                ok_sup=1; break
            fi
            (( i % 3 == 0 )) && echo "    waiting... (${i}/18, $((i * 5)) s elapsed)"
            sleep 5
        done
        if (( ok_sup )); then
            ok "supervisor service on the graph"
        else
            warn "supervisor did not advertise within 90 s - smoke test Section 4 may report failures"
        fi
    fi

    # Honour the questionnaire's smoke-test answer. Default (PRE_SMOKE unset or anything
    # other than "skip", e.g. a manually-triggered --resume with no questionnaire) is to run
    # the smoke test - the legacy behaviour.
    local smoke_rc=0
    if [[ "${PRE_SMOKE:-}" == "skip" ]]; then
        step "Smoke test skipped (per questionnaire)"
    else
        run_smoke_test || smoke_rc=$?
    fi

    # Clear the persisted questionnaire only on a clean run. A non-zero smoke_rc leaves the
    # file in place so a manually-retried resume sees the same answers without re-running the
    # whole questionnaire.
    if (( smoke_rc == 0 )); then
        rm -f "${HOME_DIR}/.arid_questionnaire"
    fi

    echo ""
    echo -e "${BOLD}Setup complete.${NC}"
    return "${smoke_rc}"
}

# Interactive menu: full setup, or any one tool. Individual tools are the same scripts
# the host aliases call; a non-zero exit returns to the menu rather than aborting.
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
        echo "  9) Colcon-build container workspace"
        echo "  10) ZeroTier join/switch"
        echo "  11) Uninstall"
        echo "  q) Quit"
        echo -e "${BOLD}========================================${NC}"
        local choice
        read -r -p "  Select: " choice || choice="q"
        case "${choice}" in
            1) run_full_setup; break ;;
            2) bash "${WORKSPACES}/scripts/local_test.sh"        || true ;;
            3) bash "${WORKSPACES}/scripts/config_realsense.sh"  || true ;;
            4) verify_cameras_menu                               || true ;;
            5) bash "${WORKSPACES}/scripts/wifi.sh"              || true ;;
            6) calibrate_cameras                                 || true ;;
            7) camera_focus                                      || true ;;
            8) build_isaac_step                                  || true ;;
            9) colcon_isaac_step                                 || true ;;
            10) setup_zerotier                                   || true ;;
            11) setup_uninstall                                  || true ;;
            q|Q) echo "  Quit."; break ;;
            *) warn "invalid selection: '${choice}'" ;;
        esac
    done
}

main() {
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
