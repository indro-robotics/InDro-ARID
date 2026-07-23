#!/bin/bash
# ARID workspace setup.
# Usage: ./setup.sh [--full|--resume|--help]
# Every step is idempotent; safe to re-run any time.
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

# Case-insensitive y/yes n/no; strip non-letters first (a stray CR breaks the match).
is_yes() { local a="${1//[^A-Za-z]/}"; case "${a,,}" in y|yes) return 0 ;; *) return 1 ;; esac; }
is_no()  { local a="${1//[^A-Za-z]/}"; case "${a,,}" in n|no)  return 0 ;; *) return 1 ;; esac; }

STEPS_RUN=()
STEPS_SKIPPED=()
RUN_FULL=0          # --full: skip the menu and run the whole setup
RUN_RESUME=0        # --resume: continue setup after a reboot (smoke test + camera verification)

# Single source of truth for the Foxglove launch (alias + camera-focus both use it).
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
export WORKSPACES LOCAL_WS ISAAC_ROS_WS

# RSAIRY fallback values; scripts/config_lidar.sh overwrites these from a
# live sniff if a LiDAR is reachable at setup time.
RSLIDAR_NIC="enP8p1s0"
RSLIDAR_HOST_IP="192.168.1.102"
RSLIDAR_LIDAR_IP="192.168.1.200"
RSLIDAR_DISPATCHER="/etc/NetworkManager/dispatcher.d/90-rslidar"
RSLIDAR_SYSCTL="/etc/sysctl.d/99-rslidar.conf"

# One continuous log per session: --resume reuses the path pinned in ~/.arid_setup_log.
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

    # nvpmodel -m 0 can prompt "reboot now?" and hang an unattended run: feed it 'no', non-fatal.
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
    # Never add ros-humble-librealsense2: the stack must only link the RSUSB build at /usr/local.
    sudo apt-get install -y \
        software-properties-common \
        ca-certificates curl gnupg git-lfs \
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

# PX4 build dependencies
setup_px4_deps() {
    step "PX4 build dependencies"

    if [[ ! -d "${PX4_DIR}" ]]; then
        skip "PX4-Autopilot not found at ${PX4_DIR}"
        STEPS_SKIPPED+=("px4_deps")
        return
    fi

    # Python deps
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
    if (( PRE )); then
        case "${PRE_PX4:-no}" in yes) install_px4=y ;; *) install_px4=n ;; esac
    else
        while true; do
            read -r -p "ARM toolchain (arm-none-eabi-gcc) not found. Run PX4 Tools/setup/ubuntu.sh? (y/n): " install_px4 || { install_px4=n; break; }
            case "$install_px4" in [yYnN]) break ;; *) echo "Choose y or n." ;; esac
        done
    fi

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

    # Symlink (relative target), not a dir: the container mounts only isaac_ros-dev,
    # so a physical root dir would be invisible in-container.
    ln -sfn isaac_ros-dev/run_logs "${WORKSPACES}/run_logs"
    ok "run_logs symlink at the workspaces root"

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

# skip-worktree per-deployment configs (serials, calibrations) so local edits cannot be pushed.
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

    # Refuse to rewrite if the resume shim is missing: bashrc would point at a missing file
    # and the post-reboot resume prompt would silently never fire.
    local shim="${SCRIPT_DIR}/scripts/arid_resume_prompt.sh"
    if [[ ! -r "${shim}" ]]; then
        err "Resume-prompt shim missing at ${shim} - re-pull the repo before re-running setup."
        return 1
    fi

    # Temp file + atomic mv under flock (30 s timeout): a shell sourcing ~/.bashrc must
    # never see a half-written state.
    local lockfile="${HOME_DIR}/.arid_bashrc.lock"
    local tmpfile="${BASHRC_FILE}.arid.new.$$"
    (
        flock -w 30 -x 9 || { err "another setup_bashrc is in progress (timed out after 30 s)"; exit 1; }

        # Strip the ARID block AND any stray managed lines outside it (hand-edits).
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
            -e '/^[[:space:]]*alias[[:space:]]\+\(cam_down_\(start\|stop\|status\|alive\)\|cam_refresh\)=/d' \
            -e '/^[[:space:]]*alias[[:space:]]\+rslidar_\(start\|stop\|status\|alive\|restart\)=/d' \
            -e '/^[[:space:]]*alias[[:space:]]\+\(lidar_diag\|local_test\|config_lidar\|config_realsense\|wifi\|ver_cv_cams\|update_submods\)=/d' \
            -e '/^[[:space:]]*alias[[:space:]]\+\(initialize\|deinitialize\)=/d' \
            -e '/^[[:space:]]*alias[[:space:]]\+\(setup\|colcon_isaac\|clean_isaac\|rosdep_isaac\|cam_calibrate\|zt_join\|status\)=/d' \
            "$tmpfile"

        # Delimiter is QUOTED ('ARIDRC'): the body is byte-literal, setup-time values go in
        # via @@TOKEN@@ markers + a matching sed line below. NEVER unquote the delimiter:
        # an unquoted heredoc once command-substituted backticks straight into bashrc.
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
[ -f @@LOCAL_WS@@/install/setup.bash ] && source @@LOCAL_WS@@/install/setup.bash
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
alias rslidar_start='ros2 service call /rslidar_coordinator/enable std_srvs/srv/SetBool "{data: true}"'
alias rslidar_stop='ros2 service call /rslidar_coordinator/enable std_srvs/srv/SetBool "{data: false}"'
alias rslidar_status='ros2 service call /rslidar_coordinator/status std_srvs/srv/Trigger "{}"'
alias rslidar_alive='ros2 topic echo --once --qos-durability transient_local /rslidar_coordinator/alive'
alias rslidar_restart='ros2 service call /rslidar_coordinator/restart std_srvs/srv/Trigger "{}"'
alias lidar_diag='/bin/bash @@WORKSPACES@@/scripts/lidar_diag.sh'
alias local_test='/bin/bash @@WORKSPACES@@/scripts/local_test.sh'
alias config_lidar='sudo /bin/bash @@WORKSPACES@@/scripts/config_lidar.sh'
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
    config_realsense   Auto-detect the front RealSense serial -> vslam_config.yaml
    config_lidar       Auto-detect the RSAIRY LiDAR IPs -> network profiles
    lidar_diag         RSAIRY LiDAR network diagnostic
    ver_cv_cams        Stream the downward IMX219 feed (q to quit)
    cam_down_start     Start the downward IMX219 pipeline
    cam_down_stop      Stop the downward IMX219 pipeline
    cam_down_status    Downward pipeline status
    cam_down_alive     Downward pipeline liveness topic
    cam_refresh        Re-read gst_camera_manager pipelines.yaml (stops running pipelines first)
    cam_calibrate      Calibrate the downward camera (setup.sh-integrated flow)
    rslidar_start      Enable the RSAIRY LiDAR driver
    rslidar_stop       Disable the RSAIRY LiDAR driver
    rslidar_status     LiDAR coordinator status
    rslidar_alive      LiDAR coordinator liveness topic
    rslidar_restart    Restart the LiDAR driver
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
# Resume-after-reboot hook; the shim keeps this block free of command substitution.
[ -r @@REPO_ROOT@@/scripts/arid_resume_prompt.sh ] && . @@REPO_ROOT@@/scripts/arid_resume_prompt.sh
# END ARID SETUP
ARIDRC

        # _sed_rhs_escape neutralises &, \, | so a path containing them cannot corrupt the rewrite.
        _sed_rhs_escape() { printf '%s' "$1" | sed -e 's/[\\&|]/\\&/g'; }
        sed -i "s|@@WORKSPACES@@|$(_sed_rhs_escape "${WORKSPACES}")|g"     "$tmpfile"
        sed -i "s|@@LOCAL_WS@@|$(_sed_rhs_escape "${LOCAL_WS}")|g"         "$tmpfile"
        sed -i "s|@@ISAAC_ROS_WS@@|$(_sed_rhs_escape "${ISAAC_ROS_WS}")|g" "$tmpfile"
        sed -i "s|@@REPO_ROOT@@|$(_sed_rhs_escape "${SCRIPT_DIR}")|g"      "$tmpfile"

        # Abort if any @@TOKEN@@ remains unsubstituted.
        if grep -Eq '@@(WORKSPACES|LOCAL_WS|ISAAC_ROS_WS|REPO_ROOT)@@' "$tmpfile"; then
            err "token substitution incomplete - stray @@TOKEN@@ in rewritten bashrc; aborting"
            rm -f "$tmpfile"
            exit 1
        fi

        mv -- "$tmpfile" "$BASHRC_FILE"
    ) 9>"${lockfile}" || { rm -f "$tmpfile"; return 1; }

    STEPS_RUN+=("bashrc")
    ok ".bashrc updated"
}

# Sudoers, udev, polkit, groups
setup_permissions() {
    step "Sudoers, udev, polkit, groups"

    local _sudtmp; _sudtmp=$(mktemp)
    tee "${_sudtmp}" > /dev/null << EOF
${USERNAME} ALL=(ALL) NOPASSWD: /usr/sbin/uhubctl, /usr/bin/gpioset, /bin/systemctl start *, /bin/systemctl stop *, /bin/systemctl restart *, /bin/systemctl kill *, /bin/systemctl reset-failed *, /bin/systemctl enable *, /bin/systemctl disable *, ${WORKSPACES}/scripts/usb_reset.sh, /usr/sbin/zerotier-cli, /usr/sbin/reboot, /sbin/reboot
EOF
    visudo -cf "${_sudtmp}" >/dev/null
    sudo install -m 440 -o root -g root "${_sudtmp}" "$SUDOERS_FILE"
    rm -f "${_sudtmp}"
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

# RSAIRY LiDAR sysctl (UDP receive buffer)
setup_lidar_sysctl() {
    step "RSAIRY LiDAR sysctl (UDP rmem)"

    sudo tee "${RSLIDAR_SYSCTL}" > /dev/null << 'EOF'
# UDP rmem ceiling for RSAIRY burst (~32 MB/s). Default ~200KB drops bursts.
net.core.rmem_max=26214400
net.core.rmem_default=26214400
EOF
    # `-p file` not `--system`: avoids reapplying unrelated L4T-incompatible drop-ins.
    sudo sysctl -p "${RSLIDAR_SYSCTL}" >/dev/null

    STEPS_RUN+=("lidar_sysctl")
    ok "sysctl: net.core.rmem_max=net.core.rmem_default=25 MiB"
}

# RSAIRY LiDAR network (NetworkManager + dispatcher)
setup_lidar_network() {
    step "RSAIRY LiDAR network (NetworkManager)"

    for con_name in rslidar dev; do
        if nmcli -t -f NAME connection show 2>/dev/null | grep -qxF "$con_name"; then
            nmcli connection delete "$con_name" >/dev/null
        fi
    done

    # Sweep stray auto-profiles bound to the NIC (e.g. "Wired connection 1").
    while IFS=: read -r name dev; do
        [[ "$dev" == "${RSLIDAR_NIC}" ]] || continue
        case "$name" in
            rslidar|dev) ;;
            *) nmcli connection delete "$name" >/dev/null 2>&1 || true ;;
        esac
    done < <(nmcli -t -f NAME,DEVICE connection show)

    # priority 10 ⇒ NM activates this before DHCP fallback on link-up.
    # ipv4.routes is required: NM sets IFA_F_NOPREFIXROUTE on manual addresses,
    # so the kernel's connected-route is suppressed and LiDAR traffic would
    # otherwise route out wifi via the default gateway.
    nmcli connection add type ethernet con-name rslidar ifname "${RSLIDAR_NIC}" \
        ipv4.method manual \
        ipv4.addresses "${RSLIDAR_HOST_IP}/24" \
        ipv4.routes "192.168.1.0/24 0.0.0.0" \
        autoconnect yes \
        connection.autoconnect-priority 10 >/dev/null

    # DHCP fallback for when the dispatcher decides no LiDAR is present.
    nmcli connection add type ethernet con-name dev ifname "${RSLIDAR_NIC}" \
        ipv4.method auto \
        ipv4.dhcp-timeout 8 \
        autoconnect yes \
        connection.autoconnect-priority 0 >/dev/null

    # Dispatcher: ARP-probe the LiDAR on rslidar-up; fall back to DHCP after 8s.
    sudo tee "${RSLIDAR_DISPATCHER}" > /dev/null << EOL
#!/bin/bash
# Installed by setup.sh. Switches 'rslidar' static -> 'dev' DHCP on no-response.
IFACE="\$1"
ACTION="\$2"

[[ "\$IFACE" != "${RSLIDAR_NIC}" ]] && exit 0
[[ "\$ACTION" != "up" ]] && exit 0

ACTIVE=\$(nmcli -t -f NAME connection show --active 2>/dev/null | grep -xE 'rslidar|dev' | head -1)
[[ "\$ACTIVE" != "rslidar" ]] && exit 0

# 8s covers cold-boot LiDAR; -s pins ARP source IP (kernel otherwise picks wifi).
for _ in 1 2 3 4 5 6 7 8; do
    if arping -c 1 -w 1 -s ${RSLIDAR_HOST_IP} -I "\$IFACE" ${RSLIDAR_LIDAR_IP} >/dev/null 2>&1; then
        logger -t rslidar-net "LiDAR detected at ${RSLIDAR_LIDAR_IP}; staying on static."
        exit 0
    fi
done

logger -t rslidar-net "LiDAR not detected after 8 s; switching to DHCP."
nmcli connection down rslidar >/dev/null 2>&1 || true
nmcli connection up dev >/dev/null 2>&1 || true
EOL
    sudo chmod 755 "${RSLIDAR_DISPATCHER}"
    sudo chown root:root "${RSLIDAR_DISPATCHER}"

    STEPS_RUN+=("lidar_network")
    ok "NM: 'rslidar' (static ${RSLIDAR_HOST_IP}/24) + 'dev' (DHCP) + dispatcher installed (fallback values)"

    # Live sniff overwrites fallback values when a LiDAR is reachable.
    if ip link show "${RSLIDAR_NIC}" 2>/dev/null | grep -qE 'LOWER_UP'; then
        ok "carrier on ${RSLIDAR_NIC} is UP - running config_lidar to auto-detect actual LiDAR IPs"
        if sudo bash "${WORKSPACES}/scripts/config_lidar.sh"; then
            ok "config_lidar succeeded: rslidar configured against discovered LiDAR"
        else
            warn "config_lidar didn't detect a LiDAR. Fallback values remain active."
            warn "Once the LiDAR is plugged in and powered, run 'config_lidar' manually."
        fi
    else
        warn "no carrier on ${RSLIDAR_NIC} - skipping auto-detect."
        warn "After the LiDAR is plugged in and powered, run 'config_lidar' to auto-configure."
    fi
}

# systemd services
setup_systemd() {
    step "systemd services"

    # Remove the stale pre-rename vslam_supervisor.service so it cannot race arid_supervisor.service.
    if [[ -f /etc/systemd/system/vslam_supervisor.service ]]; then
        sudo -n systemctl stop    vslam_supervisor.service 2>/dev/null || true
        sudo -n systemctl disable vslam_supervisor.service 2>/dev/null || true
        sudo rm -f /etc/systemd/system/vslam_supervisor.service
        ok "stale vslam_supervisor.service removed (renamed to arid_supervisor.service)"
    fi

    sudo cp -f "${ISAAC_ROS_WS}/services/"*.service "/etc/systemd/system/"
    sudo cp -f "${LOCAL_WS}/services/"*.service "/etc/systemd/system/"

    # Global ROS env for ALL systemd services; daemon-reexec (not daemon-reload) is required
    # for DefaultEnvironment to apply. PAIRING: PX4 UXRCE_DDS_PTCFG=1 must also be set or
    # no /fmu topics are published.
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
    sudo systemctl enable rslidar_coordinator.service
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

# Docker
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

# Front RealSense serial (best-effort; depends only on the camera being plugged in)
setup_realsense() {
    step "Front RealSense serial → vslam_config.yaml"

    # config_realsense.sh needs pyrealsense2 to query the serial by device.
    ensure_pip_pkg "pyrealsense2" "pyrealsense2" "" || warn "pyrealsense2 install failed - config_realsense degrades to rs-enumerate"
    # pip-satisfied is not the same as importable: an aarch64 wheel can install yet fail to load.
    python3 -c 'import pyrealsense2' >/dev/null 2>&1 \
        || warn "pyrealsense2 installed but not importable on the host - config_realsense will use rs-enumerate"

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

# Headless: /run/user/<uid> otherwise gets created root-owned by a boot process, and
# NoMachine sessions fail with a black screen. Linger makes systemd own it user-owned at boot.
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

# Remove NVIDIA first-boot icons + the L4T-README auto-mount.
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

# Predictable boot: no surprise updates on a deployed drone.
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

# PX4 timestamp alignment needs correct wall time after a power cycle.
enable_clock_sync() {
    step "Enable network time sync"
    sudo systemctl enable --now systemd-timesyncd.service 2>/dev/null || true
    STEPS_RUN+=("clock_sync")
    ok "systemd-timesyncd active"
}

# Wi-Fi connect via scripts/wifi.sh; honours pre-collected SSID / password.
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

# NoMachine: detect install; a fresh install is manual (arm64 .deb).
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

# ZeroTier daemon + optional join (zt_join.sh, single-network model). ACCESS_DENIED is not
# a failure: the node works the moment it is authorized in ZeroTier Central.
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

    # Already a member of a network: leave it unchanged (switching is zt_join's job).
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

# Foxglove + live cam_down feed; operator tunes the lens focus by watching it.
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

    echo "  Starting cam_down..."
    ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1 || true
    sleep 2
    ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool "{data: true}" >/dev/null 2>&1 \
        && ok "cam_down streaming" \
        || warn "cam_down failed to start (is the Isaac container holding sensor-id=0?)"

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
    echo "  Add an Image panel for: /cam_down/image_raw/compressed"
    echo ""
    echo "  Adjust focus in Foxglove, then press q to stop and return to the menu."
    while read -rsn1 -t 0.1 _ 2>/dev/null; do :; done
    local key=""
    while [[ "$key" != "q" && "$key" != "Q" ]]; do read -rn1 key || break; done

    echo "  Stopping Foxglove bridge and cam_down..."
    kill -INT  -- -"${fox_pgid}" 2>/dev/null || true
    sleep 1
    kill -KILL -- -"${fox_pgid}" 2>/dev/null || true
    ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1 || true
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
    bash "${WORKSPACES}/local_ws/auxiliary/camera_calibration/camera_calibration_auto/camera_calibrate.sh"
}

# Build the Isaac container (confirmation prompt, default skip).
build_isaac_step() {
    step "Build Isaac container"
    local sentinel="${HOME_DIR}/.arid_pending_build_isaac"
    local exists=0 ans build_rc=0
    if docker image inspect isaac_ros_dev-aarch64 >/dev/null 2>&1; then exists=1; fi
    if (( PRE )); then
        ans="${PRE_BUILD_ISAAC:-skip}"   # chosen in the questionnaire, no re-prompt
    elif (( exists )); then
        read -r -p "Isaac container image already exists. Rebuild it now? (y/n, Enter = no): " ans || ans=""
    else
        read -r -p "Continue with building the Isaac container? (y/n, Enter = no): " ans || ans=""
    fi
    # Strict yes: invalid input must not trigger an expensive build.
    if ! is_yes "${ans}"; then
        skip "Isaac container build deferred (rebuild any time with build_isaac)"
        STEPS_SKIPPED+=("build_isaac")
        rm -f "${sentinel}"
        return 0
    fi
    # A rebuild needs the old container gone; the boot service is restarted after the build.
    sudo -n systemctl stop arid_supervisor.service start_isaac_docker.service 2>/dev/null || true
    docker rm -f isaac_ros_dev-aarch64-container 2>/dev/null || true
    # sg docker: on the first provisioning run the docker group is not yet active in-session.
    local _build_rc=0
    # </dev/null: run_dev.sh must never attach interactively during an unattended setup run.
    if ! id -nG | grep -qw docker && grep -qw docker /etc/group; then
        sg docker -c "/bin/bash '${ISAAC_ROS_WS}/container_scripts/build_isaac_docker.sh'" </dev/null || _build_rc=$?
    else
        /bin/bash "${ISAAC_ROS_WS}/container_scripts/build_isaac_docker.sh" </dev/null || _build_rc=$?
    fi
    if (( _build_rc == 0 )); then
        sudo -n systemctl start start_isaac_docker.service 2>/dev/null || true
        STEPS_RUN+=("build_isaac")
        rm -f "${sentinel}"
        ok "Isaac container build complete"
        return 0
    else
        build_rc=${_build_rc}
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

# Colcon-build the in-container workspace and bring the supervisor up. Without an
# in-container install carrying arid_supervisor, the unit fails StartLimitBurst at boot.
colcon_isaac_step() {
    step "Colcon-build in-container workspace"
    local container="isaac_ros_dev-aarch64-container"
    if ! docker image inspect isaac_ros_dev-aarch64 >/dev/null 2>&1; then
        skip "Isaac container image not built; run build_isaac first"
        STEPS_SKIPPED+=("colcon_isaac")
        return 0
    fi
    local ans
    # --full already answered; otherwise prompt (Enter = yes).
    if (( PRE )); then
        [[ "${PRE_BUILD_ISAAC:-}" == "yes" ]] && ans="y" || ans="n"
    else
        read -r -p "Colcon-build the workspace inside the container now? (y/n, Enter = yes): " ans || ans=""
    fi
    if is_no "${ans}"; then
        skip "colcon_isaac deferred (run colcon_isaac inside the container any time)"
        STEPS_SKIPPED+=("colcon_isaac")
        return 0
    fi

    # Refuse to build into a stopped container: surface the dependency, not a confusing build failure.
    if ! docker inspect -f '{{.State.Running}}' "${container}" 2>/dev/null | grep -q true; then
        err "Isaac container is not running."
        err "Start it first: 'start_isaac' (alias) or 'sudo systemctl start start_isaac_docker.service'"
        STEPS_SKIPPED+=("colcon_isaac (container down)")
        return 1
    fi

    # Self-heal: apt ros-humble-librealsense2 must never shadow the RSUSB librealsense at
    # /usr/local (a wrapper linked against it crashes on bringup). Remove it before building.
    if docker exec -u root "${container}" dpkg -l ros-humble-librealsense2 2>/dev/null | grep -q '^ii'; then
        warn "apt ros-humble-librealsense2 found in the container (V4L2, conflicts with the RSUSB /usr/local build) - removing"
        if docker exec -u root "${container}" apt-get remove -y ros-humble-librealsense2 >/dev/null; then
            ok "apt ros-humble-librealsense2 removed from the container"
        else
            warn "could not remove ros-humble-librealsense2 - continuing; -Drealsense2_DIR still pins the link to /usr/local"
        fi
    fi

    echo "  Building workspace (several minutes on a cold cache)..."
    # Bounded so a stuck build returns control instead of hanging.
    # -u admin: bare exec is root, which leaves build/install/log root-owned and breaks the aliases.
    if timeout 3600 docker exec -u admin "${container}" bash -lc \
        'cd /workspaces/isaac_ros-dev && colcon build --symlink-install --base-paths src --cmake-args -DBUILD_TESTING=OFF -Drealsense2_DIR=/usr/local/lib/cmake/realsense2'; then
        ok "colcon build complete"
        STEPS_RUN+=("colcon_isaac")
        # reset-failed clears StartLimitBurst from the first-boot failures.
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
        rslidar_coordinator.service
        usb_ros_reset.service
        reset_usb.service
    )
    for u in "${units[@]}"; do
        sudo -n systemctl stop    "$u" 2>/dev/null || true
        sudo -n systemctl disable "$u" 2>/dev/null || true
        sudo rm -f "/etc/systemd/system/$u" 2>/dev/null || true
    done
    sudo rm -f /etc/systemd/system.conf.d/10-arid-ros-env.conf 2>/dev/null || true
    sudo systemctl daemon-reexec 2>/dev/null || true
    sudo systemctl daemon-reload 2>/dev/null || true
    for u in "${units[@]}"; do sudo -n systemctl reset-failed "$u" 2>/dev/null || true; done
    ok "systemd units removed"

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

# Arms the bashrc resume hook; rolls the flag back if the reboot itself fails.
prompt_reboot() {
    echo ""
    local do_reboot="${PRE_REBOOT:-y}"
    if (( ! PRE )); then read -r -p "Reboot now? (y/n): " do_reboot || do_reboot="n"; fi
    if is_yes "$do_reboot"; then
        echo -e "${YELLOW}${BOLD}Rebooting now.${NC}"
        touch "${HOME_DIR}/.arid_resume_setup"
        if ! sudo -n reboot 2>/dev/null && ! sudo reboot; then
            warn "sudo reboot failed - rolling back the resume flag"
            rm -f "${HOME_DIR}/.arid_resume_setup"
            return 1
        fi
    else
        echo -e "${YELLOW}${BOLD}A reboot is required for all changes to take effect.${NC}"
        echo "Remember to reboot before using this system - the smoke test runs on the next post-reboot terminal."
    fi
}

# Front-load every decision so the run is unattended; steps consult PRE_* instead of prompting.
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
        read -r -p "  Install PX4 toolchain? (y/n, Enter = no): " a || a=""
        is_yes "$a" && PRE_PX4=yes || PRE_PX4=no
    fi

    read -r -p "  Assign RealSense serial into vslam_config.yaml? (y/n, Enter = skip): " a || a=""
    is_yes "$a" && PRE_REALSENSE=yes || PRE_REALSENSE=skip

    read -r -p "  Verify the camera feed at the end? (y/n, Enter = skip): " a || a=""
    is_yes "$a" && PRE_VERIFY=yes || PRE_VERIFY=skip

    if docker image inspect isaac_ros_dev-aarch64 >/dev/null 2>&1; then
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

    read -r -p "  Run smoke test on post-reboot resume? (y/n, Enter = y): " a || a=""
    is_no "$a" && PRE_SMOKE=skip || PRE_SMOKE=yes

    read -r -p "  Reboot when done? (y/n, Enter = y): " a || a=""
    is_no "$a" && PRE_REBOOT=no || PRE_REBOOT=yes

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
    # A ZT failure is not a provisioning failure (join later via zt_join); never abort.
    setup_zerotier || true
    setup_px4_deps
    setup_git
    setup_docker_patches
    setup_skip_worktree
    setup_bashrc
    setup_permissions
    setup_uhubctl
    setup_lidar_sysctl
    setup_lidar_network
    setup_ros_workspace
    # Docker must precede setup_systemd: a mid-run abort would leave the boot unit
    # pointing at an absent daemon.
    setup_docker
    setup_systemd
    setup_realsense
    verify_cameras
    _run_build_isaac_if_queued || true
    colcon_isaac_step || true
    print_summary
    prompt_reboot
}

# Continue setup after the reboot: camera verify, queued builds, supervisor wait, smoke test.
run_resume() {
    # flock serialises racing resume terminals; the lock file is deliberately never
    # removed (flock binds the inode).
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
        # Chain the workspace build; the operator already opted in, short-circuit the prompt.
        PRE=1
        PRE_BUILD_ISAAC=yes
        colcon_isaac_step || true
    fi

    # Container + ROS launch take 30-90 s cold; without this wait the smoke test races
    # the supervisor and reports false negatives.
    local have_image=0
    if docker image inspect isaac_ros_dev-aarch64 >/dev/null 2>&1; then
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

    # Default (no questionnaire) = run the smoke test.
    local smoke_rc=0
    if [[ "${PRE_SMOKE:-}" == "skip" ]]; then
        step "Smoke test skipped (per questionnaire)"
    else
        run_smoke_test || smoke_rc=$?
    fi

    # Keep the questionnaire on failure so a retried resume sees the same answers.
    if (( smoke_rc == 0 )); then
        rm -f "${HOME_DIR}/.arid_questionnaire"
    fi

    echo ""
    echo -e "${BOLD}Setup complete.${NC}"
    return "${smoke_rc}"
}

# Interactive menu; a non-zero tool exit returns to the menu rather than aborting.
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
        echo "  11) Colcon-build container workspace"
        echo "  12) ZeroTier join/switch"
        echo "  13) Uninstall"
        echo "  q) Quit"
        echo -e "${BOLD}========================================${NC}"
        local choice
        read -r -p "  Select: " choice || choice="q"
        case "${choice}" in
            1) run_full_setup; break ;;
            2) bash "${WORKSPACES}/scripts/local_test.sh"        || true ;;
            3) bash "${WORKSPACES}/scripts/config_realsense.sh"  || true ;;
            4) verify_cameras_menu                               || true ;;
            5) bash "${WORKSPACES}/scripts/lidar_diag.sh"        || true ;;
            6) sudo bash "${WORKSPACES}/scripts/config_lidar.sh" || true ;;
            7) bash "${WORKSPACES}/scripts/wifi.sh"              || true ;;
            8) calibrate_cameras                                 || true ;;
            9) camera_focus                                      || true ;;
            10) build_isaac_step                                 || true ;;
            11) colcon_isaac_step                                || true ;;
            12) setup_zerotier                                   || true ;;
            13) setup_uninstall                                  || true ;;
            q|Q) echo "  Quit."; break ;;
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
