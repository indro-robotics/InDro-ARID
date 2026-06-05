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
PRE_REALSENSE=""; PRE_VERIFY=""; PRE_SMOKE=""; PRE_BUILD_ISAAC=""; PRE_REBOOT=""

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

# RSAIRY fallback values; scripts/config_lidar.sh overwrites these from a
# live sniff if a LiDAR is reachable at setup time.
RSLIDAR_NIC="enP8p1s0"
RSLIDAR_HOST_IP="192.168.1.102"
RSLIDAR_LIDAR_IP="192.168.1.200"
RSLIDAR_DISPATCHER="/etc/NetworkManager/dispatcher.d/90-rslidar"
RSLIDAR_SYSCTL="/etc/sysctl.d/99-rslidar.conf"

# Logging
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${SCRIPT_DIR}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/setup_log_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee -a "${LOG_FILE}") 2>&1
echo "Logging to ${LOG_FILE}"

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

    sudo /usr/sbin/nvpmodel -m 0

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
        ros-humble-foxglove-msgs \
        ros-humble-librealsense2

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
            -e '/^[[:space:]]*export[[:space:]]\+WORKSPACES=/d' \
            -e '/^[[:space:]]*export[[:space:]]\+LOCAL_WS=/d' \
            -e '/^[[:space:]]*export[[:space:]]\+ISAAC_ROS_WS=/d' \
            -e '/^[[:space:]]*alias[[:space:]]\+\(run_isaac\|build_isaac\|start_isaac\|stop_isaac\|isaac_bash\)=/d' \
            -e '/^[[:space:]]*alias[[:space:]]\+\(reset_usb\|colcon_local\|clean_local\|rosdep_local\|foxglove_bridge\)=/d' \
            -e '/^[[:space:]]*alias[[:space:]]\+cam_down_\(start\|stop\|status\|alive\)=/d' \
            -e '/^[[:space:]]*alias[[:space:]]\+rslidar_\(start\|stop\|status\|alive\|restart\)=/d' \
            -e '/^[[:space:]]*alias[[:space:]]\+\(lidar_diag\|local_test\|config_lidar\|config_realsense\|wifi\|ver_cv_cams\|update_submods\)=/d' \
            -e '/^[[:space:]]*alias[[:space:]]\+\(initialize\|deinitialize\)=/d' \
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
export WORKSPACES=@@WORKSPACES@@
export LOCAL_WS=@@LOCAL_WS@@
export ISAAC_ROS_WS=@@ISAAC_ROS_WS@@
source @@LOCAL_WS@@/install/setup.bash
alias run_isaac='/bin/bash @@ISAAC_ROS_WS@@/container_scripts/run_isaac_docker.sh'
alias build_isaac='/bin/bash @@ISAAC_ROS_WS@@/container_scripts/build_isaac_docker.sh'
alias start_isaac='/bin/bash @@ISAAC_ROS_WS@@/container_scripts/start_isaac_docker.sh'
alias stop_isaac='docker stop isaac_ros_dev-aarch64-container'
alias isaac_bash='/bin/bash @@ISAAC_ROS_WS@@/container_scripts/isaac_bash.sh'
alias reset_usb='/bin/bash @@WORKSPACES@@/scripts/usb_reset.sh'
alias rosdep_local='rosdep install --from-paths @@LOCAL_WS@@/src/ --ignore-src -y'
alias colcon_local='cd @@LOCAL_WS@@ && colcon build --symlink-install --base-paths src && source ./install/setup.bash'
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
alias initialize='/bin/bash @@ISAAC_ROS_WS@@/container_scripts/initialize.sh'
alias deinitialize='/bin/bash @@ISAAC_ROS_WS@@/container_scripts/deinitialize.sh'
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
${USERNAME} ALL=(ALL) NOPASSWD: /usr/sbin/uhubctl, /usr/bin/gpioset, /bin/systemctl start *, /bin/systemctl stop *, /bin/systemctl restart *, /bin/systemctl kill *, /bin/systemctl reset-failed *, ${WORKSPACES}/scripts/usb_reset.sh
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

    sudo cp -f "${ISAAC_ROS_WS}/services/"*.service "/etc/systemd/system/"
    sudo cp -f "${LOCAL_WS}/services/"*.service "/etc/systemd/system/"

    sudo systemctl enable usbfs-memory.service
    sudo systemctl enable usb_ros_reset.service
    sudo systemctl enable start_isaac_docker.service
    sudo systemctl enable vslam_supervisor.service
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

# Front RealSense serial (best-effort; depends only on the camera being plugged in)
setup_realsense() {
    step "Front RealSense serial → vslam_config.yaml"

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

# Build the Isaac container with a confirmation prompt. Image present -> "Rebuild?"
# default skip. Image absent -> "Continue with building?" default skip; strict yes on both.
build_isaac_step() {
    step "Build Isaac container"
    local sentinel="${HOME_DIR}/.arid_pending_build_isaac"
    local exists=0 ans build_rc=0
    if docker image inspect isaac_ros_dev-aarch64-container >/dev/null 2>&1; then exists=1; fi
    if (( exists )); then
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
    if /bin/bash "${ISAAC_ROS_WS}/container_scripts/build_isaac_docker.sh"; then
        STEPS_RUN+=("build_isaac")
        rm -f "${sentinel}"
        ok "Isaac container build complete"
        return 0
    else
        build_rc=$?
        err "build_isaac_docker.sh returned ${build_rc} - container image not built"
        STEPS_RUN+=("build_isaac (failed)")
        rm -f "${sentinel}"
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
# supervisor service launches `ros2 launch vslam_supervisor vslam_supervisor.launch.py`
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

    echo "  Building workspace (several minutes on a cold cache)..."
    if docker exec "${container}" bash -lc \
        'cd /workspaces/isaac_ros-dev && colcon build --symlink-install --base-paths src --cmake-args -DBUILD_TESTING=OFF'; then
        ok "colcon build complete"
        STEPS_RUN+=("colcon_isaac")
        # Bring the supervisor up now that the workspace is built. Reset-failed clears
        # StartLimitBurst from the first-boot failures; restart picks up the new image.
        sudo -n systemctl reset-failed vslam_supervisor.service 2>/dev/null || true
        sudo -n systemctl restart vslam_supervisor.service 2>/dev/null \
            || warn "vslam_supervisor.service restart failed - check 'sudo systemctl status vslam_supervisor.service'"
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
        vslam_supervisor.service
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
    sudo systemctl daemon-reload 2>/dev/null || true
    sudo systemctl reset-failed 2>/dev/null || true
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
        "${HOME_DIR}/.arid_resume.lock" 2>/dev/null || true
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
    bash "${WORKSPACES}/scripts/local_test.sh" || rc=$?
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
                   PRE_REALSENSE PRE_VERIFY PRE_SMOKE PRE_BUILD_ISAAC PRE_REBOOT; do
            printf '%s=%q\n' "$var" "${!var-}"
        done
    } > "$f"
}

run_full_setup() {
    preflight
    collect_answers
    first_boot
    setup_power
    disable_updates
    enable_clock_sync
    ensure_wifi
    nomachine
    setup_repos
    setup_apt_packages
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
    # vslam_supervisor at boot, but the container + ROS launch take 30-90 s cold.
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
        warn "sudo systemctl restart start_isaac_docker.service vslam_supervisor.service"
    else
        step "Recovering Docker + supervisor units (if not yet running)"
        sudo -n systemctl reset-failed start_isaac_docker.service vslam_supervisor.service 2>/dev/null || true
        sudo -n systemctl stop  start_isaac_docker.service vslam_supervisor.service 2>/dev/null || true
        sudo -n systemctl start start_isaac_docker.service 2>/dev/null || warn "start_isaac_docker.service start failed"
        sudo -n systemctl start vslam_supervisor.service    2>/dev/null || warn "vslam_supervisor.service start failed"
        step "Waiting for vslam_supervisor to advertise (up to 90 s)"
        set +u
        [[ -f /opt/ros/humble/setup.bash ]] && source /opt/ros/humble/setup.bash
        export ROS_DOMAIN_ID=23
        set -u
        local i ok_sup=0
        for i in $(seq 1 18); do
            if timeout 5 ros2 service list 2>/dev/null \
                | grep -qx /vslam_supervisor/vslam_enable; then
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
        echo "  5) LiDAR diagnostic"
        echo "  6) LiDAR auto-detect"
        echo "  7) Wi-Fi connect"
        echo "  8) Camera calibration"
        echo "  9) Camera focus"
        echo "  10) Build Isaac container"
        echo "  11) Colcon-build container workspace"
        echo "  12) Uninstall"
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
            12) setup_uninstall                                  || true ;;
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
