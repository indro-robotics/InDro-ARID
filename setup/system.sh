# system.sh: host bring-up - power, first-boot, wifi, nomachine, desktop, updates, clock,
# repos, apt, px4 deps, git, bashrc, permissions, uhubctl, systemd, python, local workspace.

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
        if ask_yn "Set hostname? (y/n, Enter = ${new_host}): " n; then
            read -r -p "  Hostname: " new_host || new_host=""
        fi
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
        if ask_yn "Set ${USERNAME} password? (y/n, Enter = leave unchanged): " n; then
            sudo passwd "${USERNAME}" || warn "password change cancelled"
        fi
    fi

    touch "${sentinel}"
    STEPS_RUN+=("first_boot")
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
    if ask_yn "Connect to a Wi-Fi network now? (y/n, Enter = skip): " n; then
        bash "${WORKSPACES}/scripts/wifi.sh" || warn "wifi.sh returned non-zero"
        STEPS_RUN+=("wifi")
    else
        skip "Wi-Fi (declined)"
        STEPS_SKIPPED+=("wifi")
    fi
}

# NoMachine: detect install; a fresh install is manual (arm64 .deb).
nomachine() {
    step "NoMachine remote desktop"

    # A reinstall drops the session running setup, and this runs with no resume armed yet.
    if is_inside_nomachine; then
        skip "running inside a NoMachine session - left untouched (upgrade over SSH if needed)"
        STEPS_SKIPPED+=("nomachine")
        return 0
    fi

    local installed=0 ver=""
    if dpkg -s nomachine >/dev/null 2>&1 || [[ -x /usr/NX/bin/nxserver ]]; then
        installed=1
        ver=$(dpkg-query -W -f='${Version}' nomachine 2>/dev/null || true)
        ok "NoMachine is installed${ver:+ (version ${ver})}"
        local up
        if (( PRE )); then up="${PRE_NOMACHINE}"; else ask_yn "  Reinstall NoMachine? (y/n, Enter = skip): " n && up=yes || up=skip; fi
        if ! is_yes "${up}"; then
            skip "NoMachine left as-is"
            STEPS_SKIPPED+=("nomachine")
            return 0
        fi
    else
        warn "NoMachine is not installed - installing"
    fi

    local deb="/tmp/nomachine_arm64.deb"
    echo "  Downloading the latest NoMachine arm64 .deb..."
    if ! wget -q -O "${deb}" "https://www.nomachine.com/free/arm/v8/deb"; then
        warn "NoMachine download failed (no internet?); skipping"
        rm -f "${deb}"
        STEPS_SKIPPED+=("nomachine")
        return 0
    fi
    if (( installed )); then
        warn "removing the existing NoMachine before reinstall (drops any active NoMachine session)"
        sudo dpkg -r nomachine >/dev/null 2>&1 || sudo apt-get remove -y nomachine >/dev/null 2>&1 || true
    fi
    echo "  Installing: ${deb} (log: /tmp/nomachine-install.log)"
    # nxserver daemons inherit our stdio and would hang dpkg; redirect so it returns.
    sudo DEBIAN_FRONTEND=noninteractive dpkg -i --force-confnew "${deb}" \
        </dev/null >/tmp/nomachine-install.log 2>&1 \
        || { warn "dpkg -i nomachine failed (see /tmp/nomachine-install.log); skipping"; rm -f "${deb}"; STEPS_SKIPPED+=("nomachine"); return 0; }
    rm -f "${deb}"
    sudo systemctl disable gdm3 --now 2>/dev/null || true
    sudo rm -f "${HOME_DIR}/.Xauthority"
    sudo touch "${HOME_DIR}/.Xauthority"
    sudo chown "${USERNAME}:${USERNAME}" "${HOME_DIR}/.Xauthority"
    chmod 600 "${HOME_DIR}/.Xauthority"
    sudo /usr/NX/bin/nxserver --restart </dev/null >/tmp/nxserver-restart.log 2>&1 \
        || warn "nxserver --restart returned non-zero (see /tmp/nxserver-restart.log)"

    STEPS_RUN+=("nomachine")
    (( installed )) && ok "NoMachine upgraded; nxserver restarting" || ok "NoMachine installed; nxserver restarting"
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

# systemd-time-wait-sync blocks until /run/systemd/timesync/synchronized appears, and only
# systemd-timesyncd ever creates it - which enable_clock_sync disables in favour of chrony. The
# unit then sits in 'activating' forever and jams the systemd job queue: every later
# deb-systemd-invoke (snapd's postinst is the usual victim) enqueues into that stuck transaction
# and apt hangs with no output. Mask it before any apt work runs. Masking survives reboots, so
# this is a no-op on every pass after the first; chrony owns the clock, so nothing is lost.
guard_time_wait_sync() {
    step "systemd-time-wait-sync (apt deadlock guard)"

    if [[ "$(systemctl is-enabled systemd-time-wait-sync.service 2>/dev/null)" == "masked" ]]; then
        skip "systemd-time-wait-sync already masked"
        return 0
    fi

    # --no-block is required, not cosmetic: a plain stop waits on the very queue that is jammed.
    sudo systemctl stop --no-block systemd-time-wait-sync.service 2>/dev/null || true
    if sudo systemctl mask systemd-time-wait-sync.service >/dev/null 2>&1; then
        ok "systemd-time-wait-sync masked (chrony disciplines the clock)"
    else
        warn "could not mask systemd-time-wait-sync - apt may stall on a blocked systemd job"
    fi
}

# PX4 timestamp alignment needs correct wall time after a power cycle. chrony (installed in
# setup_apt_packages) steps only at boot and slews afterward; systemd-timesyncd is disabled so
# the two cannot both discipline the clock and step it mid-mission.
enable_clock_sync() {
    step "Network time sync (chrony)"

    if dpkg -s chrony >/dev/null 2>&1; then
        sudo systemctl disable --now systemd-timesyncd.service >/dev/null 2>&1 || true
        sudo systemctl enable --now chrony.service 2>/dev/null \
            || sudo systemctl enable --now chronyd.service 2>/dev/null \
            || warn "chrony service could not be started"
        ok "chrony active (timesyncd disabled)"
    else
        # apt step has not run yet on this pass; fall back so the clock is still disciplined.
        sudo systemctl enable --now systemd-timesyncd.service 2>/dev/null || true
        warn "chrony not installed yet - systemd-timesyncd left active for now"
    fi

    sudo timedatectl set-ntp true >/dev/null 2>&1 || true
    STEPS_RUN+=("clock_sync")
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

    # ROS 2 apt source. The keyring alone resolves nothing: without this list every
    # ros-humble-* package below fails with "Unable to locate package". Match on the repo
    # URL, not a fixed filename, so an image that already ships the ROS repo (e.g. ARK-OS)
    # is left alone instead of adding a duplicate source.
    if ! grep -rqs "packages.ros.org/ros2" /etc/apt/sources.list /etc/apt/sources.list.d/; then
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu \
$(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
            | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
        ok "ROS 2 apt source added"
    else
        skip "ROS 2 apt source already present"
    fi

    # Nvidia CDI (regenerate each run). Jetson needs --mode=csv; auto-detect picks nvml
    # and errors out on the iGPU. Non-fatal: a CDI failure must not abort provisioning.
    if sudo nvidia-ctk cdi generate --mode=csv --output=/etc/cdi/nvidia.yaml; then
        ok "CDI config regenerated"
    else
        warn "nvidia-ctk cdi generate failed - continuing (container GPU access may be degraded)"
    fi

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
    # chrony replaces systemd-timesyncd: it steps only at boot and slews after, so no NTP step
    # can land mid-mission and tear a hole in the VO timestamp stream.
    sudo apt-get install -y \
        software-properties-common \
        ca-certificates curl gnupg git-lfs \
        chrony \
        libusb-1.0-0-dev pkgconf gpiod \
        iputils-arping tcpdump arp-scan \
        pva-allow-2 \
        python3-colcon-clean \
        python3-opencv \
        ros-humble-camera-calibration \
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

    local req="${PX4_DIR}/Tools/setup/requirements.txt"
    if [[ -f "${req}" ]]; then
        if python3 -c 'import kconfiglib' >/dev/null 2>&1; then
            skip "PX4 Python deps already satisfied (kconfiglib importable)"
        else
            # Uncapped, PX4's floor-only deps pull numpy 2.x and break the host's
            # numpy-1.x-ABI cv2 / cv_bridge (camera verify + calibration).
            ok "Installing PX4 Python deps from ${req}..."
            pip_install -r "${req}" 'numpy<2'
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
        install_px4="${PRE_PX4:-no}"
    else
        ask_yn "ARM toolchain (arm-none-eabi-gcc) not found. Run PX4 Tools/setup/ubuntu.sh? (y/n, Enter = no): " n \
            && install_px4=yes || install_px4=no
    fi

    if is_yes "${install_px4}"; then
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
    find "${SETUP_DIR}" -type f -name "*.sh" -exec chmod +x {} \;
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

# Wire the resume hook into .bashrc before arming any reboot flag. Matching the full shim
# path (not just the filename) re-heals a bashrc left pointing at an old location.
ensure_resume_hook() {
    grep -qF "${SETUP_DIR}/arid_resume_prompt.sh" "${BASHRC_FILE}" 2>/dev/null || setup_bashrc
}

# .bashrc: rewrites the ARID block on every run so alias/export changes propagate
# without leaving stale duplicates.
setup_bashrc() {
    step ".bashrc environment"

    # Refuse to rewrite if the resume shim is missing: bashrc would point at a missing file
    # and the post-reboot resume prompt would silently never fire.
    local shim="${SETUP_DIR}/arid_resume_prompt.sh"
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
        cp -- "$BASHRC_FILE" "$tmpfile" 2>/dev/null || touch "$tmpfile"
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
            -e '/^[[:space:]]*alias[[:space:]]\+\(setup\|colcon_isaac\|clean_isaac\|rosdep_isaac\|cam_calibrate\|zt_join\|status\|sentry\)=/d' \
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
alias foxglove_bridge='@@FOXGLOVE_LAUNCH@@'
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
    ver_cv_cams        Stream the downward IMX477 feed (q to quit)
    cam_down_start     Start the downward IMX477 pipeline
    cam_down_stop      Stop the downward IMX477 pipeline
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
[ -r @@SETUP_DIR@@/arid_resume_prompt.sh ] && . @@SETUP_DIR@@/arid_resume_prompt.sh
# END ARID SETUP
ARIDRC

        # _sed_rhs_escape neutralises &, \, | so a path containing them cannot corrupt the rewrite.
        _sed_rhs_escape() { printf '%s' "$1" | sed -e 's/[\\&|]/\\&/g'; }
        sed -i "s|@@WORKSPACES@@|$(_sed_rhs_escape "${WORKSPACES}")|g"           "$tmpfile"
        sed -i "s|@@LOCAL_WS@@|$(_sed_rhs_escape "${LOCAL_WS}")|g"               "$tmpfile"
        sed -i "s|@@ISAAC_ROS_WS@@|$(_sed_rhs_escape "${ISAAC_ROS_WS}")|g"       "$tmpfile"
        sed -i "s|@@SETUP_DIR@@|$(_sed_rhs_escape "${SETUP_DIR}")|g"             "$tmpfile"
        sed -i "s|@@FOXGLOVE_LAUNCH@@|$(_sed_rhs_escape "${FOXGLOVE_LAUNCH}")|g" "$tmpfile"

        # Abort if any @@TOKEN@@ remains unsubstituted.
        if grep -Eq '@@(WORKSPACES|LOCAL_WS|ISAAC_ROS_WS|SETUP_DIR|FOXGLOVE_LAUNCH)@@' "$tmpfile"; then
            err "token substitution incomplete - stray @@TOKEN@@ in rewritten bashrc; aborting"
            rm -f "$tmpfile"
            exit 1
        fi

        mv -- "$tmpfile" "$BASHRC_FILE"
    ) 9>"${lockfile}" || { rm -f "$tmpfile"; return 1; }

    # A login shell that does not source ~/.bashrc never sees the aliases or the resume hook.
    local login_rc="${HOME_DIR}/.profile"
    [[ -f "${HOME_DIR}/.bash_profile" ]] && login_rc="${HOME_DIR}/.bash_profile"
    if ! grep -q 'ARID-login-bashrc' "$login_rc" 2>/dev/null \
       && ! grep -qE '(\.|source)[[:space:]].*\.bashrc' "$login_rc" 2>/dev/null; then
        cat >> "$login_rc" <<'PROFEOF'

# ARID-login-bashrc: ensure login shells source ~/.bashrc (aliases + resume hook)
if [ -n "$BASH_VERSION" ] && [ -f "$HOME/.bashrc" ]; then . "$HOME/.bashrc"; fi
PROFEOF
        ok "login shells set to source ~/.bashrc ($(basename "$login_rc"))"
    fi

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
    if ! visudo -cf "${_sudtmp}" >/dev/null; then
        err "Sudoers rule failed visudo validation - not installed"
        rm -f "${_sudtmp}"
        return 1
    fi
    sudo install -m 440 -o root -g root "${_sudtmp}" "$SUDOERS_FILE"
    rm -f "${_sudtmp}"
    ok "Sudoers rule written and validated"

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
    sudo udevadm trigger
    ok "udev rules written, reloaded, and triggered against current devices"

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
    if [[ ! -d "${UHUBCTL_DIR}/.git" ]]; then
        rm -rf "${UHUBCTL_DIR}" 2>/dev/null || true   # drop a half-cloned dir so the clone can retry
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

# Every unit shipped by the repo, in install order. arid_units_shipped echoes the unit
# file names so the install and uninstall lists cannot drift apart.
arid_units_shipped() {
    local f
    for f in "${ISAAC_ROS_WS}/services/"*.service "${LOCAL_WS}/services/"*.service; do
        [[ -e "$f" ]] && basename "$f"
    done
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

    # Copy through arid_units_shipped, not a raw glob: install and removal then share one
    # list, and an empty services dir cannot feed cp an unexpanded glob under set -e.
    local unit src
    while IFS= read -r unit; do
        src="${ISAAC_ROS_WS}/services/${unit}"
        [[ -e "$src" ]] || src="${LOCAL_WS}/services/${unit}"
        sudo cp -f "$src" "/etc/systemd/system/"
    done < <(arid_units_shipped)
    ok "installed: $(arid_units_shipped | tr '\n' ' ')"

    # Global ROS env for ALL systemd services; daemon-reexec (not daemon-reload) is required
    # for DefaultEnvironment to apply. PAIRING: PX4 UXRCE_DDS_PTCFG=1 must also be set or
    # no /fmu topics are published.
    sudo install -d /etc/systemd/system.conf.d
    sudo tee /etc/systemd/system.conf.d/10-arid-ros-env.conf >/dev/null << 'EOF'
[Manager]
DefaultEnvironment=ROS_DOMAIN_ID=23 ROS_LOCALHOST_ONLY=1
EOF
    sudo systemctl daemon-reexec

    # reset_usb.service is deliberately not enabled: it is a oneshot started on demand
    # (polkit rule above), never at boot.
    local u
    for u in $(arid_units_shipped); do
        [[ "$u" == "reset_usb.service" ]] && continue
        sudo systemctl enable "$u"
    done
    sudo systemctl daemon-reload

    STEPS_RUN+=("systemd")
    ok "Services enabled and daemon reloaded"
}

# pip >= 23 enforces PEP 668 and needs --break-system-packages; older pip rejects the flag.
# Detect support once, then install with or without it.
pip_install() {
    if [[ -z "${_PIP_BSP+set}" ]]; then
        if python3 -m pip install --help 2>/dev/null | grep -q -- '--break-system-packages'; then
            _PIP_BSP="--break-system-packages"
        else
            _PIP_BSP=""
        fi
    fi
    python3 -m pip install "$@" ${_PIP_BSP}
}

ensure_pip_pkg() {
    local pkg="$1"
    local name="$2"
    local want_version="$3"

    local have
    have=$(python3 -m pip show "$name" 2>/dev/null | awk '/^Version: / {print $2}' || true)

    if [[ -z "$have" ]]; then
        ok "Installing ${pkg}..."
        pip_install "$pkg" --no-deps
    elif [[ -n "$want_version" && "$have" != "$want_version" ]]; then
        ok "Upgrading ${name} from ${have} to ${want_version}..."
        pip_install "$pkg" --no-deps
    else
        skip "${name}==${have} already satisfies ${pkg}"
    fi
}

# ROS2 local workspace
setup_ros_workspace() {
    step "ROS2 local workspace"

    if [[ "${PRE_LOCAL_WS:-}" == "skip" ]]; then
        skip "local workspace build skipped"
        STEPS_SKIPPED+=("ros_workspace")
        return 0
    fi
    if [[ ! -f /opt/ros/humble/setup.bash ]]; then
        warn "ROS2 not installed - skipping local workspace build"
        STEPS_SKIPPED+=("ros_workspace")
        return 0
    fi

    # ark_os installs ROS in this same run, so this shell has never sourced it and colcon
    # would build with no underlay (ament setup files need set -u disabled).
    if [[ -z "${ROS_DISTRO:-}" ]]; then
        set +u
        source /opt/ros/humble/setup.bash
        set -u
    fi

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

    if (
        cd "${LOCAL_WS}"
        colcon build \
            --symlink-install \
            --base-paths src \
            --event-handlers console_direct+ \
            --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON
    ); then
        STEPS_RUN+=("ros_workspace")
        DID_BUILD=1; touch "${HOME_DIR}/.arid_did_build"   # a build ran - gates the pre-smoke reboot
        ok "ROS2 local workspace built"
    else
        prompt_failure_action "local_ws colcon build failed" "Fix the errors above, then re-run 'colcon_local' or setup."
        STEPS_SKIPPED+=("ros_workspace")
    fi
}
