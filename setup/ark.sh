# ark.sh: ARK-OS clone + non-interactive install via a generated user.env.

# Upstream pin. The branch carries L4T 36.4.4, which the r36.4 apt sources in system.sh expect.
ARK_REPO="https://github.com/indro-robotics/ARK-OS.git"
ARK_BRANCH="ARID_L4T_36.4.4"

# JetPack version string if the nvidia-jetpack package is installed, else empty (= not installed).
detect_jetpack_version() {
    local jp l4t
    jp=$(dpkg-query --showformat='${Version}' --show nvidia-jetpack 2>/dev/null)
    [[ -z "$jp" ]] && return 0
    l4t=$(sed -n 's/^# R\([0-9]*\).*REVISION: \([0-9.]*\).*/R\1.\2/p' /etc/nv_tegra_release 2>/dev/null)
    printf 'JetPack %s%s' "$jp" "${l4t:+ (L4T $l4t)}"
}

# Write ARK-OS user.env (install.sh then skips prompts). $1 = ark dir, $2 = INSTALL_JETPACK (y/n).
ark_write_user_env() {
    cat > "$1/user.env" <<ENV
export INSTALL_DDS_AGENT="y"
export INSTALL_RTSP_SERVER="n"
export INSTALL_RID_TRANSMITTER="n"
export MANUFACTURER_CODE="ARK1"
export SERIAL_NUMBER="C0FFEE123"
export INSTALL_LOGLOADER="n"
export USER_EMAIL=""
export UPLOAD_TO_FLIGHT_REVIEW="n"
export PUBLIC_LOGS="n"
export INSTALL_POLARIS="n"
export POLARIS_API_KEY=""
export INSTALL_JETPACK="${2:-n}"
ENV
}

# Re-assert JetPack/L4T holds, then run ARK ./install.sh. $1 = ark dir, $2 = INSTALL_JETPACK (y/n).
ark_run_install() {
    sudo apt-mark hold \
        nvidia-l4t-core linux-firmware nvidia-l4t-kernel nvidia-l4t-kernel-dtbs \
        nvidia-l4t-firmware nvidia-l4t-kernel-headers nvidia-l4t-kernel-oot-headers \
        wireless-regdb >/dev/null
    ark_write_user_env "$1" "$2"
    if ( cd "$1" && ./install.sh ); then ok "ARK install.sh complete"
    else prompt_failure_action "ARK install.sh returned non-zero" "Review the output above, then re-run setup."; fi
}

# Clone or force-refresh the ARK-OS repo at ${ARK_BRANCH}.
ark_ensure_repo() {
    local ark_dir="${HOME_DIR}/ARK-OS"
    if [[ -d "${ark_dir}/.git" ]]; then
        ok "ARK-OS present - refreshing to ${ARK_BRANCH}"
        ( cd "${ark_dir}" \
            && git fetch --recurse-submodules origin "${ARK_BRANCH}" \
            && git checkout -f -B "${ARK_BRANCH}" FETCH_HEAD \
            && git submodule update --init --recursive --force ) \
            || warn "ARK-OS repo refresh had issues - continuing with what's on disk"
    else
        rm -rf "${ark_dir}" 2>/dev/null || true
        git clone --recurse-submodules -b "${ARK_BRANCH}" "${ARK_REPO}" "${ark_dir}"
        ok "ARK-OS cloned"
    fi
}

# Menu: install/reinstall ARK-OS base (repo + ./install.sh; ROS2 is a separate option).
menu_install_ark() {
    step "Install ARK-OS"
    ark_ensure_repo
    local apt_yes="/etc/apt/apt.conf.d/99-arid-assume-yes"
    printf 'APT::Get::Assume-Yes "true";\nDpkg::Options { "--force-confold"; "--force-confdef"; };\n' | sudo tee "${apt_yes}" >/dev/null
    local jetpack="n"; is_yes "${PRE_JETPACK:-}" && jetpack="y"
    ark_run_install "${HOME_DIR}/ARK-OS" "${jetpack}"
    sudo rm -f "${apt_yes}"
    touch "${HOME_DIR}/.arid_ark_os_installed"
    ok "ARK-OS install complete"
}

# Menu: install/reinstall ROS2 (ARK install_ros2.sh); requires ARK-OS present.
menu_install_ros2() {
    step "Install ROS2"
    local ark_dir="${HOME_DIR}/ARK-OS"
    if [[ ! -d "${ark_dir}" ]]; then
        err "ARK-OS not present - install ARK-OS first."
        read -r -p "  Press Enter to return to the menu: " _ || true
        return 1
    fi
    set +o pipefail
    yes '' | bash "${ark_dir}/tools/install_ros2.sh" && ok "ROS2 install complete" || warn "install_ros2.sh returned non-zero"
    set -o pipefail
}

# ARK-OS installed: the sentinel, or the repo + ROS Humble both present.
ark_installed() {
    [[ -f "${HOME_DIR}/.arid_ark_os_installed" ]] && return 0
    [[ -d "${HOME_DIR}/ARK-OS" && -f /opt/ros/humble/setup.bash ]]
}

ark_os() {
    step "ARK-OS install"

    local ark_dir="${HOME_DIR}/ARK-OS"

    # Independent gates: ARK base (PRE_ARK) and ROS2 (PRE_ARK_ROS2). Skip entirely if neither runs.
    local do_ark=1 do_ros2=1
    [[ "${PRE_ARK:-}" == "skip" ]] && do_ark=0
    [[ "${PRE_ARK_ROS2:-}" == "skip" ]] && do_ros2=0
    if (( ! do_ark && ! do_ros2 )); then
        skip "ARK-OS + ROS2 left as-is (already installed)"
        STEPS_SKIPPED+=("ark_os")
        return 0
    fi

    ark_ensure_repo

    # ARK's install_ros2.sh omits -y on some apt calls; force non-interactive apt while it runs.
    local apt_yes="/etc/apt/apt.conf.d/99-arid-assume-yes"
    printf 'APT::Get::Assume-Yes "true";\nDpkg::Options { "--force-confold"; "--force-confdef"; };\n' \
        | sudo tee "${apt_yes}" >/dev/null

    # install.sh masks its exit code (pipe to tee, no pipefail), so verify JetPack via dpkg below.
    local jetpack="n"; is_yes "${PRE_JETPACK:-}" && jetpack="y"
    if (( do_ark )); then
        while true; do
            ark_run_install "${ark_dir}" "${jetpack}"
            if [[ "${jetpack}" != "y" ]] || dpkg -s nvidia-jetpack >/dev/null 2>&1; then break; fi
            case "$(prompt_core_failure "ARK-OS JetPack install" "nvidia-jetpack not installed - see ${ark_dir}/output.txt")" in
                retry) continue ;;
                exit)  sudo rm -f "${apt_yes}"; user_exit 1 ;;
                *)     warn "proceeding without JetPack"; PRE_POST_ARK_REBOOT=no; break ;;
            esac
        done
    fi

    if (( do_ros2 )); then
        # install_ros2.sh has interactive gaps (add-apt-repository ENTER prompt); feed blank lines.
        set +o pipefail
        yes '' | bash "${ark_dir}/tools/install_ros2.sh" && ok "ARK install_ros2.sh complete" \
            || warn "install_ros2.sh returned non-zero"
        set -o pipefail
    fi

    sudo rm -f "${apt_yes}"

    # install_ros2.sh adds 'source /opt/ros/humble/setup.bash' to ~/.bashrc - collapse dupes to one.
    local rosline='source /opt/ros/humble/setup.bash' n
    n=$(grep -Fxc -- "${rosline}" "${BASHRC_FILE}" 2>/dev/null || true)
    if (( ${n:-0} > 1 )); then
        awk -v l="${rosline}" '$0==l{if(seen++) next} {print}' "${BASHRC_FILE}" > "${BASHRC_FILE}.aridtmp" \
            && mv "${BASHRC_FILE}.aridtmp" "${BASHRC_FILE}"
        ok "removed duplicate ROS source line(s) from ~/.bashrc"
    fi

    touch "${HOME_DIR}/.arid_ark_os_installed"   # mark installed so a re-run can offer skip
    STEPS_RUN+=("ark_os")
    ok "ARK-OS step complete"

    # Do NOT reboot here. The install-group reboot is deferred to the end of Phase A so the apt
    # and host-config work lands first; we reach this line only when ARK-OS and/or ROS2 actually
    # installed (the neither-case returned above), so that reboot is required either way.
    INSTALL_GROUP_REBOOT=1
}
