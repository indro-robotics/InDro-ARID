# container.sh: Docker engine, Isaac Dockerfile patches, skip-worktree, image + workspace builds.

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

    # The copies above overwrite tracked files inside the isaac_ros_common submodule;
    # skip-worktree keeps the patched copies out of that submodule's git status.
    git -C "${ISAAC_ROS_WS}/src/isaac_ros_common" update-index --skip-worktree \
        scripts/.isaac_ros_common-config \
        docker/Dockerfile.arid \
        scripts/run_dev.sh \
        docker/scripts/arid_env.sh 2>/dev/null || true

    STEPS_RUN+=("docker_patches")
    ok "Docker patches applied and protected"
}

# Camera calibrations are per-drone and must never leave the machine as a commit; everything
# else in the repo, tuning config included, stays tracked and committable.
setup_skip_worktree() {
    step "Protecting per-deployment camera calibrations"

    local specs=(
        "local_ws/src/ros_gst_cameras/gst_camera_manager/config/calibrations"
        "local_ws/auxiliary/camera_calibration/camera_calibrations"
    )

    local files
    files=$(git -C "$REPO_ROOT" ls-files -- "${specs[@]}" 2>/dev/null || true)

    # Unmark anything outside the current spec set: a mark left by an earlier, wider spec
    # silently blocks committing that file.
    local marked f
    while IFS= read -r marked; do
        [[ -z "$marked" ]] && continue
        grep -qxF "$marked" <<< "$files" && continue
        git -C "$REPO_ROOT" update-index --no-skip-worktree "$marked" 2>/dev/null || true
        ok "Unprotected (now tracked normally): ${marked}"
    done < <(git -C "$REPO_ROOT" ls-files -v | sed -n 's/^S //p')

    if [[ -z "$files" ]]; then
        warn "No matching tracked files to protect"
        STEPS_RUN+=("skip_worktree")
        return 0
    fi

    while IFS= read -r f; do
        [[ -e "${REPO_ROOT}/${f}" ]] || warn "Tracked baseline missing from disk: ${f}"
    done <<< "$files"

    echo "$files" | xargs git -C "$REPO_ROOT" update-index --skip-worktree 2>/dev/null || true
    STEPS_RUN+=("skip_worktree")
    ok "Protected (skip-worktree): $(echo "$files" | wc -l) file(s)"
}

build_isaac_step() {
    step "Build Isaac container"
    local sentinel="${HOME_DIR}/.arid_pending_build_isaac"
    local exists=0 ans build_rc=0
    if docker image inspect isaac_ros_dev-aarch64 >/dev/null 2>&1; then exists=1; fi
    if (( PRE )); then
        ans="${PRE_BUILD_ISAAC:-skip}"
    elif (( exists )); then
        ask_yn "Isaac container image already exists. Rebuild it now? (y/n, Enter = no): " n && ans=yes || ans=skip
    else
        ask_yn "Continue with building the Isaac container? (y/n, Enter = no): " n && ans=yes || ans=skip
    fi
    if ! is_yes "${ans}"; then
        skip "Isaac container build deferred (rebuild any time with build_isaac)"
        STEPS_SKIPPED+=("build_isaac")
        rm -f "${sentinel}"
        return 0
    fi
    # The boot service recreates the container, so stop it before removing the old one.
    sudo -n systemctl stop arid_supervisor.service start_isaac_docker.service 2>/dev/null || true
    docker rm -f isaac_ros_dev-aarch64-container >/dev/null 2>&1 || true
    # sg docker: on the first provisioning run the docker group is not yet active in-session.
    # </dev/null: run_dev.sh must never attach interactively during an unattended setup run.
    local _build_rc=0
    if ! id -nG | grep -qw docker && grep -qw docker /etc/group; then
        sg docker -c "/bin/bash '${ISAAC_ROS_WS}/container_scripts/build_isaac_docker.sh'" </dev/null || _build_rc=$?
    else
        /bin/bash "${ISAAC_ROS_WS}/container_scripts/build_isaac_docker.sh" </dev/null || _build_rc=$?
    fi
    if (( _build_rc == 0 )); then
        sudo -n systemctl start start_isaac_docker.service 2>/dev/null || true
        STEPS_RUN+=("build_isaac")
        DID_BUILD=1; touch "${HOME_DIR}/.arid_did_build"   # gates the pre-smoke reboot
        rm -f "${sentinel}"
        ok "Isaac container build complete"
        return 0
    fi
    build_rc=${_build_rc}
    STEPS_RUN+=("build_isaac (failed)")
    # The sentinel is kept on failure so a resume retries the queued build.
    prompt_failure_action "build_isaac_docker.sh returned ${build_rc} - container image not built" \
        "Fix the errors above, then re-run 'build_isaac'."
    return "${build_rc}"
}

# The sentinel is what carries a queued build across the reboot into the resumed run.
_run_build_isaac_if_queued() {
    local sentinel="${HOME_DIR}/.arid_pending_build_isaac"
    [[ -f "${sentinel}" ]] || return 0
    build_isaac_step
}

# Without an in-container install carrying arid_supervisor, the unit fails StartLimitBurst
# at boot.
colcon_isaac_step() {
    step "Colcon-build in-container workspace"
    local container="isaac_ros_dev-aarch64-container"
    if ! docker image inspect isaac_ros_dev-aarch64 >/dev/null 2>&1; then
        skip "Isaac container image not built; run build_isaac first"
        STEPS_SKIPPED+=("colcon_isaac")
        return 0
    fi
    local ans
    if (( PRE )); then
        ans="${PRE_COLCON:-yes}"
    else
        ask_yn "Colcon-build the workspace inside the container now? (y/n, Enter = yes): " y && ans=yes || ans=skip
    fi
    if is_no "${ans}"; then
        skip "colcon_isaac deferred (run colcon_isaac inside the container any time)"
        STEPS_SKIPPED+=("colcon_isaac")
        return 0
    fi

    if ! docker inspect -f '{{.State.Running}}' "${container}" 2>/dev/null | grep -q true; then
        err "Isaac container is not running."
        err "Start it first: 'start_isaac' (alias) or 'sudo systemctl start start_isaac_docker.service'"
        STEPS_SKIPPED+=("colcon_isaac (container down)")
        return 1
    fi

    # apt ros-humble-librealsense2 must never shadow the RSUSB librealsense at /usr/local:
    # a wrapper linked against the apt V4L2 build crashes on bringup.
    if docker exec -u root "${container}" dpkg -l ros-humble-librealsense2 2>/dev/null | grep -q '^ii'; then
        warn "apt ros-humble-librealsense2 found in the container (V4L2, conflicts with the RSUSB /usr/local build) - removing"
        if docker exec -u root "${container}" apt-get remove -y ros-humble-librealsense2 >/dev/null; then
            ok "apt ros-humble-librealsense2 removed from the container"
        else
            warn "could not remove ros-humble-librealsense2 - continuing; -Drealsense2_DIR still pins the link to /usr/local"
        fi
    fi

    echo "  Building workspace (several minutes on a cold cache)..."
    # -u admin: a bare exec runs as root, which leaves build/install/log root-owned and breaks
    # the in-container aliases.
    if timeout 3600 docker exec -u admin "${container}" bash -lc \
        'cd /workspaces/isaac_ros-dev && colcon build --symlink-install --base-paths src --cmake-args -DBUILD_TESTING=OFF -Drealsense2_DIR=/usr/local/lib/cmake/realsense2'; then
        if ! docker exec -u admin "${container}" test -f /workspaces/isaac_ros-dev/install/setup.bash; then
            STEPS_RUN+=("colcon_isaac (no install)")
            prompt_failure_action "colcon reported success but install/setup.bash is missing - workspace not built" \
                "Check the colcon output above."
            return 1
        fi
        ok "colcon build complete"
        STEPS_RUN+=("colcon_isaac")
        DID_BUILD=1; touch "${HOME_DIR}/.arid_did_build"
        # reset-failed clears the StartLimitBurst left by the pre-build boot failures.
        sudo -n systemctl reset-failed arid_supervisor.service 2>/dev/null || true
        sudo -n systemctl restart arid_supervisor.service 2>/dev/null \
            || warn "arid_supervisor.service restart failed - check 'sudo systemctl status arid_supervisor.service'"
        return 0
    fi
    local rc=$?
    STEPS_RUN+=("colcon_isaac (failed)")
    prompt_failure_action "Isaac colcon build returned ${rc}" \
        "Fix the errors above, then re-run 'colcon_isaac' in the container."
    return "${rc}"
}
