# container.sh: Docker engine, Isaac Dockerfile patches, skip-worktree, image + workspace builds.

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

# Per-drone files kept local via skip-worktree: camera calibration outputs only.
# Tuning config (pipelines.yaml, px4_vslam_reactor.yaml, rslidar.yaml) stays tracked and
# committable. vslam_config.yaml is deliberately NOT here: it is gitignored and reseeded
# from vslam_config.template.yaml, and skip-worktree on an untracked file is meaningless.
ARID_SKIP_WORKTREE_SPECS=(
    "local_ws/src/ros_gst_cameras/gst_camera_manager/config/calibrations"
    "local_ws/auxiliary/camera_calibration/camera_calibrations"
)

setup_skip_worktree() {
    step "Protecting per-drone camera calibrations"

    local live_vslam="isaac_ros-dev/src/px4_vslam/config/vslam_config.yaml"
    if git -C "$REPO_ROOT" ls-files --error-unmatch -- "${live_vslam}" >/dev/null 2>&1; then
        warn "${live_vslam} is gitignored but still in the index"
        warn "run 'git rm --cached ${live_vslam}' to finish the template/live split"
    fi

    local files
    files=$(git -C "$REPO_ROOT" ls-files -- "${ARID_SKIP_WORKTREE_SPECS[@]}" 2>/dev/null || true)

    # Clear marks left by an earlier spec list; a stale mark makes git silently swallow
    # incoming updates to a file that is now meant to be tracked normally.
    local marked f
    marked=$(git -C "$REPO_ROOT" ls-files -v 2>/dev/null | awk '/^S /{sub(/^S /, ""); print}' || true)
    while IFS= read -r f; do
        [[ -n "$f" ]] || continue
        grep -qxF "$f" <<< "$files" && continue
        git -C "$REPO_ROOT" update-index --no-skip-worktree -- "$f" 2>/dev/null || true
        ok "unprotected (no longer per-drone): ${f}"
    done <<< "$marked"

    if [[ -z "$files" ]]; then
        warn "no matching tracked files to protect"
        STEPS_RUN+=("skip_worktree")
        return 0
    fi

    # Warn (don't auto-restore) on a tracked baseline missing from disk; the operator decides.
    while IFS= read -r f; do
        [[ -e "${REPO_ROOT}/${f}" ]] || warn "tracked baseline missing from disk: ${f}"
    done <<< "$files"

    echo "$files" | xargs git -C "$REPO_ROOT" update-index --skip-worktree 2>/dev/null || true
    STEPS_RUN+=("skip_worktree")
    ok "Protected (skip-worktree): $(echo "$files" | wc -l) file(s)"
}

# Build the Isaac container (confirmation prompt, default skip).
build_isaac_step() {
    step "Build Isaac container"
    local sentinel="${HOME_DIR}/.arid_pending_build_isaac"
    local exists=0 ans
    if docker image inspect isaac_ros_dev-aarch64 >/dev/null 2>&1; then exists=1; fi
    if (( PRE )); then
        ans="${PRE_BUILD_ISAAC:-skip}"   # chosen in the questionnaire, no re-prompt
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
    # A rebuild needs the old container gone; the boot service is restarted after the build.
    sudo -n systemctl stop arid_supervisor.service start_isaac_docker.service 2>/dev/null || true
    docker rm -f isaac_ros_dev-aarch64-container 2>/dev/null || true
    # sg docker: on the first provisioning run the docker group is not yet active in-session.
    # </dev/null: run_dev.sh must never attach interactively during an unattended setup run.
    local build_rc=0
    if ! id -nG | grep -qw docker && grep -qw docker /etc/group; then
        sg docker -c "/bin/bash '${ISAAC_ROS_WS}/container_scripts/build_isaac_docker.sh'" </dev/null || build_rc=$?
    else
        /bin/bash "${ISAAC_ROS_WS}/container_scripts/build_isaac_docker.sh" </dev/null || build_rc=$?
    fi
    if (( build_rc == 0 )); then
        sudo -n systemctl start start_isaac_docker.service 2>/dev/null || true
        STEPS_RUN+=("build_isaac")
        DID_BUILD=1; touch "${HOME_DIR}/.arid_did_build"
        rm -f "${sentinel}"
        ok "Isaac container build complete"
        return 0
    fi
    STEPS_RUN+=("build_isaac (failed)")
    # Keep the sentinel on FAILURE so a post-reboot resume retries the queued build.
    prompt_failure_action "build_isaac_docker.sh returned ${build_rc} - container image not built" \
        "Fix the errors above, then re-run 'build_isaac'."
    return "${build_rc}"
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
    if (( PRE )); then
        ans="${PRE_COLCON:-yes}"
    else
        ask_yn "Colcon-build the workspace inside the container now? (y/n, Enter = yes): " y && ans=yes || ans=skip
    fi
    if ! is_yes "${ans}"; then
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
        if ! docker exec -u admin "${container}" test -f /workspaces/isaac_ros-dev/install/setup.bash; then
            STEPS_RUN+=("colcon_isaac (no install)")
            prompt_failure_action "colcon reported success but install/setup.bash is missing - workspace not built" \
                "Check the colcon output above."
            return 1
        fi
        ok "colcon build complete"
        STEPS_RUN+=("colcon_isaac")
        DID_BUILD=1; touch "${HOME_DIR}/.arid_did_build"
        # reset-failed clears StartLimitBurst from the first-boot failures.
        sudo -n systemctl reset-failed arid_supervisor.service 2>/dev/null || true
        sudo -n systemctl restart arid_supervisor.service 2>/dev/null \
            || warn "arid_supervisor.service restart failed - check 'sudo systemctl status arid_supervisor.service'"
        return 0
    fi
    local rc=$?
    STEPS_RUN+=("colcon_isaac (failed)")
    prompt_failure_action "Isaac colcon build returned ${rc}" "Fix the errors above, then re-run 'colcon_isaac'."
    return "${rc}"
}
