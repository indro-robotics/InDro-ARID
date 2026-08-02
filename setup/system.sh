# system.sh: host bring-up - power, first boot, wifi, nomachine, repos, apt, px4 deps,
# git, bashrc, permissions, desktop cleanup, uhubctl, systemd, pip, local workspace.

# Power mode and package holds
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

# Hostname / password (one-time, sentinel-gated)
first_boot() {
    step "First-boot hostname / password"

    local sentinel="${HOME_DIR}/.arid_provisioned"
    if [[ -f "${sentinel}" ]]; then
        skip "already provisioned"
        STEPS_SKIPPED+=("first_boot")
        return 0
    fi

    local a new_host="${PRE_HOST:-arid}"
    if (( ! PRE )); then
        if ask_yn "Set hostname? (y/n, Enter = ${new_host}): " n; then
            read -r -p "  Hostname: " a || a=""
            [[ -n "$a" ]] && new_host="$a"
        fi
    fi
    if [[ -n "${new_host}" && "${new_host}" != "$(hostname)" ]]; then
        sudo hostnamectl set-hostname "${new_host}"
        # Keep /etc/hosts in sync so sudo can always resolve the hostname.
        if grep -qE '^[[:space:]]*127\.0\.1\.1[[:space:]]' /etc/hosts; then
            sudo sed -i -E "s/^([[:space:]]*127\.0\.1\.1[[:space:]]+).*/\1${new_host}/" /etc/hosts
        else
            echo "127.0.1.1 ${new_host}" | sudo tee -a /etc/hosts >/dev/null
        fi
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

# NoMachine: fetch + install the current arm64 .deb.
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
        if (( PRE )); then up="${PRE_NOMACHINE}"; else ask_yn "  Upgrade NoMachine? (y/n, Enter = skip): " n && up=yes || up=skip; fi
        if ! is_yes "${up}"; then
            skip "NoMachine left as-is"
            STEPS_SKIPPED+=("nomachine")
            return 0
        fi
    else
        warn "NoMachine is not installed - installing"
    fi

    # The .deb ships with the repo rather than being downloaded. NoMachine's ARM page now serves
    # only nomachine-personal-edition, which installs happily and then refuses every connection
    # with "the subscription license on this server has expired" - the free 9.x line is no longer
    # published there, and the old free URL 404s. Vendoring also means a drone with no internet
    # still provisions. Resolved here, not at file scope: system.sh is sourced before setup.sh
    # defines LOCAL_WS, and a top-level expansion would trip `set -u` before anything prints.
    # Newest .deb in the directory wins, so dropping in a newer build is all an upgrade takes.
    local nm_dir="${LOCAL_WS}/auxiliary/nomachine" deb
    deb=$(ls -1t "${nm_dir}"/*.deb 2>/dev/null | head -1)
    if [[ -z "${deb}" ]]; then
        warn "no .deb found in ${nm_dir}"
        warn "drop the NoMachine arm64 .deb there and re-run setup"
        STEPS_SKIPPED+=("nomachine (no package)")
        return 0
    fi
    if ! dpkg-deb --info "${deb}" >/dev/null 2>&1; then
        warn "not a Debian package: ${deb} ($(file -b "${deb}" 2>/dev/null | head -c 50))"
        STEPS_SKIPPED+=("nomachine (bad package)")
        return 0
    fi
    # Guard against the paid edition being dropped in by mistake: it installs, then refuses to
    # serve. Cheaper to catch here than to debug a licence dialog on a headless drone.
    local pkg; pkg=$(dpkg-deb -f "${deb}" Package 2>/dev/null)
    if [[ "${pkg}" == *personal-edition* ]]; then
        warn "${deb} is ${pkg} - the subscription edition, which will refuse connections."
        warn "Use the free 'nomachine' package instead."
        STEPS_SKIPPED+=("nomachine (paid edition)")
        return 0
    fi

    if (( installed )); then
        warn "removing the existing NoMachine before reinstall (drops any active NoMachine session)"
        sudo dpkg -r nomachine nomachine-personal-edition >/dev/null 2>&1 \
            || sudo apt-get remove -y nomachine nomachine-personal-edition >/dev/null 2>&1 || true
    fi
    echo "  Installing: ${deb} ($(dpkg-deb -f "${deb}" Version 2>/dev/null)) - log: /tmp/nomachine-install.log"
    # nxserver daemons inherit our stdio and would hang dpkg; redirect so it returns.
    # NOTE: no rm afterwards - the .deb is a tracked repo file, not a temp download.
    sudo DEBIAN_FRONTEND=noninteractive dpkg -i --force-confnew "${deb}" \
        </dev/null >/tmp/nomachine-install.log 2>&1 \
        || { warn "dpkg -i nomachine failed (see /tmp/nomachine-install.log); skipping"; STEPS_SKIPPED+=("nomachine"); return 0; }
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
    sudo tee /etc/apt/apt.conf.d/99-arid-disable-auto-updates >/dev/null << 'EOF'
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Download-Upgradeable-Packages "0";
APT::Periodic::AutocleanInterval "0";
APT::Periodic::Unattended-Upgrade "0";
EOF
    sudo systemctl disable --now unattended-upgrades.service 2>/dev/null || true
    sudo systemctl disable --now apt-daily.timer apt-daily-upgrade.timer 2>/dev/null || true

    if [[ -f /etc/xdg/autostart/update-notifier.desktop ]]; then
        sudo sed -i '/^Hidden=/d; /^X-GNOME-Autostart-enabled=/d' \
            /etc/xdg/autostart/update-notifier.desktop
        echo "Hidden=true"                     | sudo tee -a /etc/xdg/autostart/update-notifier.desktop >/dev/null
        echo "X-GNOME-Autostart-enabled=false" | sudo tee -a /etc/xdg/autostart/update-notifier.desktop >/dev/null
    fi

    STEPS_RUN+=("disable_updates")
    ok "unattended upgrades disabled"
}

# Clock sync via chrony: it steps only at boot and slews afterwards. A mid-mission NTP
# step (systemd-timesyncd's behaviour) shows up as a phantom VO stamp gap.
enable_clock_sync() {
    step "System clock (chrony)"

    if dpkg -s chrony >/dev/null 2>&1; then
        sudo systemctl disable --now systemd-timesyncd.service 2>/dev/null || true
        sudo systemctl enable  --now chrony.service 2>/dev/null \
            || warn "chrony.service did not start - check 'systemctl status chrony'"
        ok "chrony active (timesyncd disabled)"
    else
        warn "chrony not installed yet - falling back to systemd-timesyncd for now"
        sudo systemctl enable --now systemd-timesyncd.service 2>/dev/null || true
    fi
    sudo timedatectl set-ntp true >/dev/null 2>&1 || true
    gsettings set org.gnome.desktop.datetime automatic-timezone true >/dev/null 2>&1 || true

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

    # Nvidia Jetson APT repo (both lines, or the t234 half can silently go missing)
    if ! grep -q "repo.download.nvidia.com/jetson/common" /etc/apt/sources.list.d/nvidia-l4t-apt-source.list 2>/dev/null \
       || ! grep -q "repo.download.nvidia.com/jetson/t234" /etc/apt/sources.list.d/nvidia-l4t-apt-source.list 2>/dev/null; then
        sudo apt-key adv --fetch-key \
            https://repo.download.nvidia.com/jetson/jetson-ota-public.asc
        sudo tee /etc/apt/sources.list.d/nvidia-l4t-apt-source.list >/dev/null <<'EOF'
deb https://repo.download.nvidia.com/jetson/common r36.4 main
deb https://repo.download.nvidia.com/jetson/t234 r36.4 main
EOF
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
    # chrony replaces systemd-timesyncd (see enable_clock_sync).
    sudo apt-get install -y \
        software-properties-common \
        ca-certificates curl gnupg git-lfs \
        chrony \
        libusb-1.0-0-dev pkgconf gpiod \
        iputils-arping tcpdump arp-scan \
        v4l-utils \
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
            # Uncapped, PX4's floor-only deps pull numpy 2.x and break the host's numpy-1.x-ABI cv2.
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
    find "${WORKSPACES}/setup" -type f -name "*.sh" -exec chmod +x {} \;
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

# Ensure the resume hook is in .bashrc before arming a reboot flag; call before every reboot.
ensure_resume_hook() {
    grep -qF "${SCRIPT_DIR}/setup/arid_resume_prompt.sh" "${BASHRC_FILE}" 2>/dev/null || setup_bashrc
}

# .bashrc: rewrites the ARID block on every run so alias/export changes propagate
# without leaving stale duplicates.
setup_bashrc() {
    step ".bashrc environment"

    # Refuse to rewrite if the resume shim is missing: bashrc would point at a missing file
    # and the post-reboot resume prompt would silently never fire.
    local shim="${SCRIPT_DIR}/setup/arid_resume_prompt.sh"
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
            -e '/^[[:space:]]*alias[[:space:]]\+\(cam_\(down\|front\)_\(start\|stop\|status\|alive\)\|cam_refresh\|cam_stop\)=/d' \
            -e '/^[[:space:]]*alias[[:space:]]\+\(local_test\|config_realsense\|wifi\|ver_cv_cams\|update_submods\)=/d' \
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
alias cam_front_start='ros2 service call /gst_camera_manager/cam_front std_srvs/srv/SetBool "{data: true}"'
alias cam_front_stop='ros2 service call /gst_camera_manager/cam_front std_srvs/srv/SetBool "{data: false}"'
alias cam_front_status='ros2 service call /gst_camera_manager/cam_front/status std_srvs/srv/Trigger "{}"'
alias cam_front_alive='ros2 topic echo --once --qos-durability transient_local /gst_camera_manager/cam_front/alive'
alias cam_stop='ros2 service call /gst_camera_manager/cam_front std_srvs/srv/SetBool "{data: false}"; ros2 service call /gst_camera_manager/cam_down std_srvs/srv/SetBool "{data: false}"'
alias cam_refresh='ros2 service call /gst_camera_manager/refresh std_srvs/srv/Trigger'
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
    cam_front_start    Start the front IMX219 pipeline
    cam_front_stop     Stop the front IMX219 pipeline
    cam_front_status   Front pipeline status
    cam_front_alive    Front pipeline liveness topic
    cam_down_start     Start the downward IMX219 pipeline
    cam_down_stop      Stop the downward IMX219 pipeline
    cam_down_status    Downward pipeline status
    cam_down_alive     Downward pipeline liveness topic
    cam_stop           Stop both CSI pipelines
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
# Resume-after-reboot hook; the shim keeps this block free of command substitution.
[ -r @@REPO_ROOT@@/setup/arid_resume_prompt.sh ] && . @@REPO_ROOT@@/setup/arid_resume_prompt.sh
# END ARID SETUP
ARIDRC

        # _sed_rhs_escape neutralises &, \, | so a path containing them cannot corrupt the rewrite.
        _sed_rhs_escape() { printf '%s' "$1" | sed -e 's/[\\&|]/\\&/g'; }
        sed -i "s|@@WORKSPACES@@|$(_sed_rhs_escape "${WORKSPACES}")|g"           "$tmpfile"
        sed -i "s|@@LOCAL_WS@@|$(_sed_rhs_escape "${LOCAL_WS}")|g"               "$tmpfile"
        sed -i "s|@@ISAAC_ROS_WS@@|$(_sed_rhs_escape "${ISAAC_ROS_WS}")|g"       "$tmpfile"
        sed -i "s|@@FOXGLOVE_LAUNCH@@|$(_sed_rhs_escape "${FOXGLOVE_LAUNCH}")|g" "$tmpfile"
        sed -i "s|@@REPO_ROOT@@|$(_sed_rhs_escape "${SCRIPT_DIR}")|g"            "$tmpfile"

        # Abort if any @@TOKEN@@ remains unsubstituted.
        if grep -Eq '@@(WORKSPACES|LOCAL_WS|ISAAC_ROS_WS|FOXGLOVE_LAUNCH|REPO_ROOT)@@' "$tmpfile"; then
            err "token substitution incomplete - stray @@TOKEN@@ in rewritten bashrc; aborting"
            rm -f "$tmpfile"
            exit 1
        fi

        mv -- "$tmpfile" "$BASHRC_FILE"
    ) 9>"${lockfile}" || { rm -f "$tmpfile"; return 1; }

    # Login shells (SSH, console) must reach ~/.bashrc too - that is where the alias set and
    # the resume hook live. Ubuntu's ~/.profile already sources it; only add a guard if not.
    local login_rc="${HOME_DIR}/.profile"
    if [ -f "${HOME_DIR}/.bash_profile" ]; then login_rc="${HOME_DIR}/.bash_profile"; fi
    if ! grep -q 'ARID-login-bashrc' "$login_rc" 2>/dev/null \
       && ! grep -qE '(\.|source)[[:space:]].*\.bashrc' "$login_rc" 2>/dev/null; then
        cat >> "$login_rc" <<'PROFEOF'

# ARID-login-bashrc: ensure login shells source ~/.bashrc (aliases + setup resume hook)
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

    # Write to a temp file and validate with visudo before installing: a typo here would
    # otherwise break sudo for the rest of the run (and the rest of the system).
    local _sudtmp; _sudtmp=$(mktemp)
    cat > "${_sudtmp}" << EOF
${USERNAME} ALL=(ALL) NOPASSWD: /usr/sbin/uhubctl, /usr/bin/gpioset, /bin/systemctl start *, /bin/systemctl stop *, /bin/systemctl restart *, /bin/systemctl kill *, /bin/systemctl reset-failed *, /bin/systemctl enable *, /bin/systemctl disable *, /usr/bin/systemctl start *, /usr/bin/systemctl stop *, /usr/bin/systemctl restart *, /usr/bin/systemctl kill *, /usr/bin/systemctl reset-failed *, /usr/bin/systemctl enable *, /usr/bin/systemctl disable *, /usr/bin/nmcli, ${WORKSPACES}/scripts/usb_reset.sh, /usr/sbin/zerotier-cli, /usr/sbin/reboot, /sbin/reboot
EOF
    if sudo visudo -c -f "${_sudtmp}" >/dev/null 2>&1; then
        sudo install -m 0440 -o root -g root "${_sudtmp}" "$SUDOERS_FILE"
        ok "Sudoers rule written and validated"
    else
        err "Sudoers rule failed visudo validation - not installed"
        rm -f "${_sudtmp}"
        return 1
    fi
    rm -f "${_sudtmp}"

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

    # RealSense host-side libusb access. setup installs pyrealsense2 from a pip wheel, which
    # ships bindings and no udev rules; librealsense's own source install is what normally drops
    # them, and that is not part of provisioning. Without this the node stays 0664 root:root,
    # pyrealsense2 enumerates 0 devices as ${USERNAME}, and config_realsense writes a blank
    # serial_no. The Isaac container covers itself separately (image rules + plugdev in the
    # entrypoint + arid_supervisor._repair_camera_nodes); none of that applies on the host.
    # PIDs match the set arid_supervisor gates on.
    sudo tee /etc/udev/rules.d/99-realsense-libusb.rules > /dev/null << 'EOL'
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", MODE:="0666", GROUP:="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3a", MODE:="0666", GROUP:="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3d", MODE:="0666", GROUP:="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5c", MODE:="0666", GROUP:="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b64", MODE:="0666", GROUP:="plugdev"
KERNEL=="iio*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3a", MODE:="0777", GROUP:="plugdev"
KERNEL=="iio*", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b5c", MODE:="0777", GROUP:="plugdev"
EOL

    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ok "udev rules written, reloaded, and triggered against current devices"

    # Groups. plugdev: the group the RealSense udev rules above assign.
    sudo usermod -aG dialout,gpio,plugdev "${USERNAME}"
    ok "Groups: dialout, gpio, plugdev"

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

# Every unit this repo ships, from both service directories. Single source for the
# install (setup_systemd) and removal (setup_uninstall) lists so they cannot drift.
shipped_units() {
    local d f
    for d in "${ISAAC_ROS_WS}/services" "${LOCAL_WS}/services"; do
        for f in "${d}"/*.service; do [[ -e "$f" ]] && basename "$f"; done
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

    # Copy through shipped_units, not a raw glob: install and removal then share one list,
    # and an empty services dir cannot feed cp an unexpanded glob under set -e.
    local unit src
    while IFS= read -r unit; do
        src="${ISAAC_ROS_WS}/services/${unit}"
        [[ -e "$src" ]] || src="${LOCAL_WS}/services/${unit}"
        sudo cp -f "$src" "/etc/systemd/system/"
    done < <(shipped_units)

    # Global ROS env for ALL systemd services; daemon-reexec (not daemon-reload) is required
    # for DefaultEnvironment to apply. PAIRING: PX4 UXRCE_DDS_PTCFG=1 must also be set or
    # no /fmu topics are published.
    sudo install -d /etc/systemd/system.conf.d
    sudo tee /etc/systemd/system.conf.d/10-arid-ros-env.conf >/dev/null << 'EOF'
[Manager]
DefaultEnvironment=ROS_DOMAIN_ID=23 ROS_LOCALHOST_ONLY=1
EOF
    sudo systemctl daemon-reexec

    # reset_usb.service is triggered on demand through polkit, never enabled at boot.
    local u enabled=0
    while IFS= read -r u; do
        [[ "$u" == "reset_usb.service" ]] && continue
        sudo systemctl enable "$u" </dev/null   # keep sudo off the loop's stdin
        enabled=$((enabled + 1))
    done < <(shipped_units)
    sudo systemctl daemon-reload

    STEPS_RUN+=("systemd")
    ok "${enabled} service(s) installed + enabled, daemon reloaded"
}

# pip >= 23 enforces PEP 668 and needs --break-system-packages; older pip rejects the flag.
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
