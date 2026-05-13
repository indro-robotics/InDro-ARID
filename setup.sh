#!/bin/bash
# ARID Drone Workspace Setup
# Usage: ./setup.sh [--fresh | --patch]
#   --fresh   First-time installation on a new system
#   --patch   Re-run after a git pull (auto-selected if sentinel exists)
set -euo pipefail

###############################################################################
# COLOURS & OUTPUT HELPERS
###############################################################################
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[1;34m'; BOLD='\033[1m'; NC='\033[0m'

CURRENT_STEP="(not started)"
step() { CURRENT_STEP="$*"; echo -e "\n${BLUE}${BOLD}==> $*${NC}"; }
ok()   { echo -e "  ${GREEN}[OK]${NC}   $*"; }
warn() { echo -e "  ${YELLOW}[WARN]${NC} $*"; }
skip() { echo -e "  [SKIP]  $*"; }
err()  { echo -e "  ${RED}[ERROR]${NC} $*" >&2; }

STEPS_RUN=()
STEPS_SKIPPED=()

###############################################################################
# ERROR TRAP
###############################################################################
failure() {
    err "======================================================"
    err "SETUP FAILED in step: ${CURRENT_STEP}"
    err "Line $1: '${BASH_COMMAND}'"
    err "Log: ${LOG_FILE:-<not yet set>}"
    err "======================================================"
    exit 1
}
trap 'failure ${LINENO}' ERR

###############################################################################
# CONSTANTS
###############################################################################
USERNAME="jetson"
HOME_DIR="/home/${USERNAME}"
BASHRC_FILE="${HOME_DIR}/.bashrc"
WORKSPACES="${HOME_DIR}/workspaces"
LOCAL_WS="${WORKSPACES}/local_ws"
ISAAC_ROS_WS="${WORKSPACES}/isaac_ros-dev"
PX4_DIR="${LOCAL_WS}/auxiliary/PX4-Autopilot"
POLKIT_RULE_FILE="/etc/polkit-1/rules.d/10-reset-usb.rules"
SUDOERS_FILE="/etc/sudoers.d/${USERNAME}_systemctl"
SENTINEL="/etc/arid_first_setup_done"
REPO_ROOT="$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"

# RSAIRY LiDAR network
RSLIDAR_NIC="enP8p1s0"
RSLIDAR_HOST_IP="192.168.1.102"
RSLIDAR_LIDAR_IP="192.168.1.200"
RSLIDAR_DISPATCHER="/etc/NetworkManager/dispatcher.d/90-rslidar"
RSLIDAR_SYSCTL="/etc/sysctl.d/99-rslidar.conf"

###############################################################################
# LOGGING
###############################################################################
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${SCRIPT_DIR}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/setup_log_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee -a "${LOG_FILE}") 2>&1
echo "Logging to ${LOG_FILE}"

###############################################################################
# ARGUMENT PARSING
###############################################################################
SETUP_MODE=""

parse_args() {
    for arg in "$@"; do
        case "$arg" in
            --fresh) SETUP_MODE="fresh" ;;
            --patch) SETUP_MODE="patch" ;;
            --help|-h)
                echo "Usage: $0 [--fresh | --patch]"
                echo "  --fresh  First-time installation"
                echo "  --patch  Re-run after a git pull"
                exit 0 ;;
            *) err "Unknown argument: $arg"; exit 1 ;;
        esac
    done

    if [[ -z "$SETUP_MODE" ]]; then
        if [[ -f "$SENTINEL" ]]; then
            SETUP_MODE="patch"
            echo "Sentinel found — running as patch."
        else
            echo "Select setup type:"
            select SETUP_MODE in "fresh" "patch"; do
                [[ -n "$SETUP_MODE" ]] && break
                echo "Choose 1 or 2."
            done
        fi
    fi

    echo -e "Mode: ${BOLD}${SETUP_MODE}${NC}\n"
}

###############################################################################
# SANITY CHECKS
###############################################################################
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

###############################################################################
# POWER & PACKAGE HOLDS
###############################################################################
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

###############################################################################
# APT REPOSITORIES
###############################################################################
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

###############################################################################
# APT PACKAGES
###############################################################################
setup_apt_packages() {
    step "APT packages"

    sudo apt-get update
    sudo apt-get install -y \
        software-properties-common \
        ca-certificates curl gnupg \
        libusb-1.0-0-dev pkgconf gpiod \
        iputils-arping \
        pva-allow-2 \
        python3-colcon-clean \
        ros-humble-camera-info-manager \
        ros-humble-compressed-image-transport

    STEPS_RUN+=("apt")
    ok "APT packages installed"
}

###############################################################################
# PX4 BUILD DEPENDENCIES  (fresh only)
###############################################################################
setup_px4_deps() {
    if [[ "$SETUP_MODE" != "fresh" ]]; then
        skip "PX4 deps (patch mode — skipped)"
        return
    fi

    step "PX4 build dependencies"

    if [[ ! -d "${PX4_DIR}" ]]; then
        skip "PX4-Autopilot not found at ${PX4_DIR}"
        STEPS_SKIPPED+=("px4_deps")
        return
    fi

    local install_px4
    while true; do
        read -r -p "Install PX4 build dependencies? (y/n): " install_px4
        case "$install_px4" in [yYnN]) break ;; *) echo "Choose y or n." ;; esac
    done

    if [[ "$install_px4" =~ ^[yY]$ ]]; then
        (cd "${PX4_DIR}/Tools/setup" && bash ubuntu.sh)
        STEPS_RUN+=("px4_deps")
        ok "PX4 dependencies installed"
    else
        skip "PX4 dependencies"
        STEPS_SKIPPED+=("px4_deps")
    fi
}

###############################################################################
# GIT CONFIG & SUBMODULES
###############################################################################
setup_git() {
    step "Git config & submodules"

    git config --global credential.helper "cache --timeout=604800"

    find "${WORKSPACES}/scripts" -type f \( -name "*.bash" -o -name "*.sh" \) \
        -exec chmod +x {} \;
    find "${ISAAC_ROS_WS}/container_scripts" -type f \( -name "*.bash" -o -name "*.sh" \) \
        -exec chmod +x {} \;
    ok "Script permissions set"

    git -C "${REPO_ROOT}" submodule update --init --recursive
    ok "Submodules updated"

    STEPS_RUN+=("git")
}

###############################################################################
# ISAAC ROS DOCKER PATCHES
###############################################################################
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

    # Protect patched files in isaac_ros_common submodule from git modification
    git -C "${ISAAC_ROS_WS}/src/isaac_ros_common" update-index --skip-worktree \
        scripts/.isaac_ros_common-config \
        docker/Dockerfile.arid \
        scripts/run_dev.sh \
        docker/scripts/arid_env.sh 2>/dev/null || true

    STEPS_RUN+=("docker_patches")
    ok "Docker patches applied and protected"
}

###############################################################################
# CONFIG FILE PROTECTION (skip-worktree)
# Marks tracked files inside per-deployment config directories as skip-worktree
# so local edits (camera serials, calibrations, pipeline tuning) don't show up
# as `git status` modifications and can't be accidentally pushed.
###############################################################################
setup_skip_worktree() {
    step "Protecting per-deployment config files"

    # Repo-relative paths. Add any new per-deployment config dirs here.
    local protected_dirs=(
        "local_ws/src/ros_gst_cameras/gst_camera_manager/config"
        "isaac_ros-dev/src/px4_vslam/config"
    )

    local protected=0
    for rel_dir in "${protected_dirs[@]}"; do
        local files
        files=$(git -C "$REPO_ROOT" ls-files "$rel_dir" 2>/dev/null || true)
        if [[ -n "$files" ]]; then
            echo "$files" | xargs git -C "$REPO_ROOT" update-index --skip-worktree \
                2>/dev/null || true
            ok "Protected: ${rel_dir}"
            (( protected++ )) || true
        else
            warn "No tracked files in ${rel_dir} — skipped"
        fi
    done

    STEPS_RUN+=("skip_worktree")
    ok "${protected} config directories protected"
}

###############################################################################
# .BASHRC
# Always removes and rewrites the ARID block so patch runs pick up
# any alias or export changes without leaving stale duplicates.
###############################################################################
setup_bashrc() {
    step ".bashrc environment"

    # Remove existing block (idempotent)
    sed -i '/# BEGIN ARID SETUP/,/# END ARID SETUP/d' "$BASHRC_FILE"

    cat >> "$BASHRC_FILE" << EOF
# BEGIN ARID SETUP
if [ -d /tmp/.X11-unix ]; then
    sock=\$(ls /tmp/.X11-unix/X* 2>/dev/null | head -n1)
    if [ -n "\$sock" ]; then export DISPLAY=":\${sock##*/X}"; fi
fi
xhost +local: >/dev/null 2>&1 || true
export ROS_DOMAIN_ID=23
export GST_PLUGIN_PATH=/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0\${GST_PLUGIN_PATH:+:\$GST_PLUGIN_PATH}
export WORKSPACES=${WORKSPACES}
export LOCAL_WS=${LOCAL_WS}
export ISAAC_ROS_WS=${ISAAC_ROS_WS}
source ${LOCAL_WS}/install/setup.bash
alias run_isaac='/bin/bash ${ISAAC_ROS_WS}/container_scripts/run_isaac_docker.sh'
alias build_isaac='/bin/bash ${ISAAC_ROS_WS}/container_scripts/build_isaac_docker.sh'
alias start_isaac='/bin/bash ${ISAAC_ROS_WS}/container_scripts/start_isaac_docker.sh'
alias stop_isaac='docker stop isaac_ros_dev-aarch64-container'
alias isaac_bash='/bin/bash ${ISAAC_ROS_WS}/container_scripts/isaac_bash.sh'
alias reset_usb='/bin/bash ${WORKSPACES}/scripts/usb_reset.sh'
alias rosdep_local='rosdep install --from-paths ${LOCAL_WS}/src/ --ignore-src -y'
alias colcon_local='cd ${LOCAL_WS} && colcon build --symlink-install --base-paths src && source ./install/setup.bash'
alias clean_local='cd ${LOCAL_WS} && colcon clean workspace --base-select build install log'
alias lidar_start='ros2 service call /rslidar_coordinator/enable std_srvs/srv/SetBool "{data: true}"'
alias lidar_stop='ros2 service call /rslidar_coordinator/enable std_srvs/srv/SetBool "{data: false}"'
alias lidar_status='ros2 service call /rslidar_coordinator/status std_srvs/srv/Trigger "{}"'
alias lidar_restart='ros2 service call /rslidar_coordinator/restart std_srvs/srv/Trigger "{}"'
# END ARID SETUP
EOF

    STEPS_RUN+=("bashrc")
    ok ".bashrc updated"
}

###############################################################################
# SUDOERS / UDEV / POLKIT / GROUPS
###############################################################################
setup_permissions() {
    step "Sudoers, udev, polkit, groups"

    # Sudoers — always write (idempotent, fixed content)
    sudo tee "$SUDOERS_FILE" > /dev/null << EOF
${USERNAME} ALL=(ALL) NOPASSWD: /usr/sbin/uhubctl, /usr/bin/gpioset, /bin/systemctl start *, /bin/systemctl stop *, /bin/systemctl restart *, /bin/systemctl kill *, ${WORKSPACES}/scripts/usb_reset.sh
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

###############################################################################
# UHUBCTL
###############################################################################
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

###############################################################################
# RSAIRY LIDAR SYSCTL  (UDP receive buffer)
###############################################################################
setup_lidar_sysctl() {
    step "RSAIRY LiDAR sysctl (UDP rmem)"

    sudo tee "${RSLIDAR_SYSCTL}" > /dev/null << 'EOF'
# Raise UDP receive-buffer ceilings so the RSAIRY firehose (~32 MB/s burst)
# doesn't overflow the kernel queue during multi-packet arrivals.
net.core.rmem_max=26214400
net.core.rmem_default=26214400
EOF
    sudo sysctl --system >/dev/null

    STEPS_RUN+=("lidar_sysctl")
    ok "sysctl: net.core.rmem_max=net.core.rmem_default=25 MiB"
}

###############################################################################
# RSAIRY LIDAR NETWORK  (NetworkManager + dispatcher)
###############################################################################
setup_lidar_network() {
    step "RSAIRY LiDAR network (NetworkManager)"

    # Idempotent: delete our own connections first
    for con_name in rslidar dev; do
        if nmcli -t -f NAME connection show 2>/dev/null | grep -qxF "$con_name"; then
            nmcli connection delete "$con_name" >/dev/null
        fi
    done

    # Sweep any leftover auto-created profile bound to our NIC (e.g. "Wired connection 1")
    while IFS=: read -r name dev; do
        [[ "$dev" == "${RSLIDAR_NIC}" ]] || continue
        case "$name" in
            rslidar|dev) ;;
            *) nmcli connection delete "$name" >/dev/null 2>&1 || true ;;
        esac
    done < <(nmcli -t -f NAME,DEVICE connection show)

    # Static profile — autoconnect-priority 10 (NM activates this first on link-up;
    # static IPs activate instantly so the LiDAR path is sub-second).
    nmcli connection add type ethernet con-name rslidar ifname "${RSLIDAR_NIC}" \
        ipv4.method manual \
        ipv4.addresses "${RSLIDAR_HOST_IP}/24" \
        autoconnect yes \
        connection.autoconnect-priority 10 >/dev/null

    # DHCP fallback — used when the dispatcher confirms no LiDAR is present.
    nmcli connection add type ethernet con-name dev ifname "${RSLIDAR_NIC}" \
        ipv4.method auto \
        ipv4.dhcp-timeout 8 \
        autoconnect yes \
        connection.autoconnect-priority 0 >/dev/null

    # Dispatcher: ARP-probe the LiDAR after 'rslidar' activates; if no response
    # within ~8 s, fall back to DHCP. Runs as root (dispatcher always does).
    sudo tee "${RSLIDAR_DISPATCHER}" > /dev/null << EOL
#!/bin/bash
# Installed by setup.sh — switches 'rslidar' static -> 'dev' DHCP if no LiDAR responds.
IFACE="\$1"
ACTION="\$2"

[[ "\$IFACE" != "${RSLIDAR_NIC}" ]] && exit 0
[[ "\$ACTION" != "up" ]] && exit 0

ACTIVE=\$(nmcli -t -f NAME connection show --active 2>/dev/null | grep -xE 'rslidar|dev' | head -1)
[[ "\$ACTIVE" != "rslidar" ]] && exit 0

# ~8 s of probing — gives the LiDAR time to boot before we give up.
for _ in 1 2 3 4 5 6 7 8; do
    if arping -c 1 -w 1 -I "\$IFACE" ${RSLIDAR_LIDAR_IP} >/dev/null 2>&1; then
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
    ok "NM: 'rslidar' (static ${RSLIDAR_HOST_IP}/24) + 'dev' (DHCP) + dispatcher installed"
}

###############################################################################
# SYSTEMD SERVICES
###############################################################################
setup_systemd() {
    step "systemd services"

    sudo cp -f "${ISAAC_ROS_WS}/services/"*.service "/etc/systemd/system/"
    sudo cp -f "${LOCAL_WS}/services/"*.service "/etc/systemd/system/"

    sudo systemctl enable usbfs-memory.service
    sudo systemctl enable usb_ros_reset.service
    sudo systemctl enable start_isaac_docker.service
    sudo systemctl enable jetson-clocks.service
    sudo systemctl enable gst_camera_manager.service
    sudo systemctl enable arid_description.service
    sudo systemctl enable rslidar_coordinator.service
    sudo systemctl daemon-reload

    STEPS_RUN+=("systemd")
    ok "Services enabled and daemon reloaded"
}

###############################################################################
# PYTHON PACKAGES
###############################################################################
ensure_pip_pkg() {
    local pkg="$1"
    local name="$2"
    local want_version="$3"

    local have
    have=$(python3 -m pip show "$name" 2>/dev/null | awk '/^Version: / {print $2}' || true)

    if [[ -z "$have" ]]; then
        ok "Installing ${pkg}..."
        python3 -m pip install "$pkg" --no-deps --break-system-packages
    elif [[ -n "$want_version" && "$have" != "$want_version" ]]; then
        ok "Upgrading ${name} from ${have} to ${want_version}..."
        python3 -m pip install "$pkg" --no-deps --break-system-packages
    else
        skip "${name}==${have} already satisfies ${pkg}"
    fi
}

###############################################################################
# ROS2 LOCAL WORKSPACE
###############################################################################
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

###############################################################################
# DOCKER  (install only on fresh; ensure running on patch)
###############################################################################
setup_docker() {
    step "Docker"

    if [[ "$SETUP_MODE" == "patch" ]]; then
        if command -v docker >/dev/null 2>&1; then
            sudo systemctl --now enable docker
            skip "Docker already installed (patch mode)"
            STEPS_SKIPPED+=("docker_install")
        else
            warn "Docker not installed — re-run with --fresh"
        fi
        return
    fi

    # Fresh install
    if ! command -v docker >/dev/null 2>&1; then
        curl https://get.docker.com | sh -s -- --version 29.2.1
        ok "Docker engine installed"
    else
        skip "Docker binary already present"
    fi

    sudo systemctl --now enable docker
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
    sudo usermod -aG docker "${USERNAME}"
    sudo apt-get install -y docker-buildx-plugin

    STEPS_RUN+=("docker")
    ok "Docker configured with NVIDIA runtime"
}

###############################################################################
# SENTINEL
###############################################################################
write_sentinel() {
    [[ "$SETUP_MODE" != "fresh" ]] && return
    sudo touch "$SENTINEL"
    ok "Sentinel written to ${SENTINEL} (future runs will default to patch)"
}

###############################################################################
# SUMMARY
###############################################################################
print_summary() {
    echo ""
    echo -e "${BOLD}======================================${NC}"
    echo -e "${BOLD}  Setup Complete${NC}"
    echo -e "${BOLD}======================================${NC}"
    echo -e "  Mode:  ${BOLD}${SETUP_MODE}${NC}"
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

###############################################################################
# REBOOT PROMPT
###############################################################################
prompt_reboot() {
    echo ""
    echo -e "${YELLOW}${BOLD}A reboot is required for all changes to take effect.${NC}"
    read -r -p "Reboot now? (y/n): " do_reboot
    if [[ "$do_reboot" =~ ^[yY]$ ]]; then
        sudo reboot
    else
        echo "Remember to reboot before using this system."
    fi
}

###############################################################################
# MAIN
###############################################################################
main() {
    parse_args "$@"
    preflight
    setup_power
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
    setup_systemd
    setup_ros_workspace
    setup_docker
    write_sentinel
    print_summary
    prompt_reboot
}

main "$@"
