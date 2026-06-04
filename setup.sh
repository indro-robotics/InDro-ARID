#!/bin/bash
# ARID drone workspace setup. Every step is idempotent. Safe to re-run.
# Usage: ./setup.sh [--help]
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

STEPS_RUN=()
STEPS_SKIPPED=()

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
            --help|-h)
                echo "Usage: $0 [--help]"
                echo "All steps auto-detect their current state and install/configure"
                echo "only what's missing. Safe to re-run any time."
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

    git -C "${REPO_ROOT}" submodule update --init --recursive
    ok "Submodules updated"

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
            warn "No tracked files in ${rel_dir} - skipped"
        fi
    done

    STEPS_RUN+=("skip_worktree")
    ok "${protected} config directories protected"
}

# .bashrc: rewrites the ARID block on every run so alias/export changes propagate
# without leaving stale duplicates.
setup_bashrc() {
    step ".bashrc environment"

    # Strip the marker block AND any stray managed lines outside it (hand-edits).
    # Patterns below must stay in sync with the heredoc.
    sed -i \
        -e '/# BEGIN ARID SETUP/,/# END ARID SETUP/d' \
        -e '/^[[:space:]]*source[[:space:]].*local_ws\/install\/setup\.bash/d' \
        -e '/^[[:space:]]*export[[:space:]]\+ROS_DOMAIN_ID=/d' \
        -e '/^[[:space:]]*export[[:space:]]\+GST_PLUGIN_PATH=/d' \
        -e '/^[[:space:]]*export[[:space:]]\+WORKSPACES=/d' \
        -e '/^[[:space:]]*export[[:space:]]\+LOCAL_WS=/d' \
        -e '/^[[:space:]]*export[[:space:]]\+ISAAC_ROS_WS=/d' \
        -e '/^[[:space:]]*alias[[:space:]]\+\(run_isaac\|build_isaac\|start_isaac\|stop_isaac\|isaac_bash\)=/d' \
        -e '/^[[:space:]]*alias[[:space:]]\+\(reset_usb\|colcon_local\|clean_local\|rosdep_local\|foxglove_bridge\)=/d' \
        -e '/^[[:space:]]*alias[[:space:]]\+cam_down_\(start\|stop\|status\|alive\)=/d' \
        -e '/^[[:space:]]*alias[[:space:]]\+rslidar_\(start\|stop\|status\|alive\|restart\)=/d' \
        -e '/^[[:space:]]*alias[[:space:]]\+\(lidar_diag\|local_test\|config_lidar\|config_realsense\)=/d' \
        "$BASHRC_FILE"

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
alias foxglove_bridge='ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765'
alias cam_down_start='ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool "{data: true}"'
alias cam_down_stop='ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool "{data: false}"'
alias cam_down_status='ros2 service call /gst_camera_manager/cam_down/status std_srvs/srv/Trigger "{}"'
alias cam_down_alive='ros2 topic echo --once --qos-durability transient_local /gst_camera_manager/cam_down/alive'
alias rslidar_start='ros2 service call /rslidar_coordinator/enable std_srvs/srv/SetBool "{data: true}"'
alias rslidar_stop='ros2 service call /rslidar_coordinator/enable std_srvs/srv/SetBool "{data: false}"'
alias rslidar_status='ros2 service call /rslidar_coordinator/status std_srvs/srv/Trigger "{}"'
alias rslidar_alive='ros2 topic echo --once --qos-durability transient_local /rslidar_coordinator/alive'
alias rslidar_restart='ros2 service call /rslidar_coordinator/restart std_srvs/srv/Trigger "{}"'
alias lidar_diag='/bin/bash ${WORKSPACES}/scripts/lidar_diag.sh'
alias local_test='/bin/bash ${WORKSPACES}/scripts/local_test.sh'
alias config_lidar='sudo /bin/bash ${WORKSPACES}/scripts/config_lidar.sh'
alias config_realsense='/bin/bash ${WORKSPACES}/scripts/config_realsense.sh'
# END ARID SETUP
EOF

    STEPS_RUN+=("bashrc")
    ok ".bashrc updated"
}

# Sudoers, udev, polkit, groups
setup_permissions() {
    step "Sudoers, udev, polkit, groups"

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

# Reboot prompt
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

# Main
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
    setup_realsense
    print_summary
    prompt_reboot
}

main "$@"
