#!/bin/bash

set -euo pipefail

USERNAME="jetson"
BASHRC_FILE=${HOME}/.bashrc
EXPORT_DISPLAY="export DISPLAY=:1001"
LOCAL_WS="${HOME}/workspaces/local_ws"
EXPORT_ROS_DOMAIN_ID="export ROS_DOMAIN_ID=23"
ISAAC_ROS_WS="${HOME}/workspaces/isaac_ros-dev"
EXPORT_ISAAC_WS="export ISAAC_ROS_WS=${ISAAC_ROS_WS}"
ISAAC_RUN_ALIAS='alias run_isaac="/bin/bash $ISAAC_ROS_WS/scripts/run_isaac_docker.sh"'
ISAAC_BUILD_ALIAS='alias build_isaac="/bin/bash $ISAAC_ROS_WS/scripts/build_isaac_docker.sh"'
ISAAC_START_ALIAS='alias start_isaac="/bin/bash $ISAAC_ROS_WS/scripts/start_isaac_docker.sh"'
ISAAC_STOP_ALIAS='alias stop_isaac="docker stop isaac_ros_dev-aarch64-container"'
ISAAC_BASH_ALIAS='alias isaac_bash="/bin/bash $ISAAC_ROS_WS/scripts/isaac_bash.sh"'
SUDOERS_FILE="/etc/sudoers.d/${USERNAME}_systemctl"
SUDOERS_LINE="$USERNAME ALL=(ALL) NOPASSWD: \
    /usr/sbin/uhubctl, \
    /usr/bin/gpioset, \
    /bin/systemctl start *, \
    /bin/systemctl stop *, \
    /bin/systemctl restart *"
REPO_ROOT="$(git rev-parse --show-toplevel)"

cd "$REPO_ROOT" || exit

sudo /usr/sbin/nvpmodel -m 0


echo "Select setup type:"
select setup_type in "New Setup" "Patch"; do
    case $setup_type in
        "New Setup"|"Patch")
            break
            ;;
        *)
            echo "Invalid option, please choose 1 or 2."
            ;;
    esac
done

# Cache credentials in memory for 7 days
git config --global credential.helper "cache --timeout=604800"
git update-index --assume-unchanged ${ISAAC_ROS_WS}/src/px4_vslam/config/

# Script permissions
echo "Setting script permissions..."
find ${ISAAC_ROS_WS}/scripts -type f \( -name "*.bash" -o -name "*.sh" \) -exec chmod +x {} \;

echo "Update/checkout submodules..."
sudo "${ISAAC_ROS_WS}/scripts/update_submods.sh"

# Copy custom docker-config files
echo "Patching dockerfiles..."
sudo cp -f "${ISAAC_ROS_WS}/scripts/docker/patched_dockerfiles/.isaac_ros_common-config" \
    "${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/"
sudo cp -f "${ISAAC_ROS_WS}/scripts/docker/patched_dockerfiles/Dockerfile.ros2_humble" \
    "${ISAAC_ROS_WS}/src/isaac_ros_common/docker/"
sudo cp -f "${ISAAC_ROS_WS}/scripts/docker/patched_dockerfiles/Dockerfile.aarch64" \
    "${ISAAC_ROS_WS}/src/isaac_ros_common/docker/"

# Copy patched run_dev.sh script to keep persistent docker container
sudo cp -f "${ISAAC_ROS_WS}/scripts/run_dev.sh" \
    "${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/"

# Copy patched workspace-entrypoint.sh script to stop crashing on container start
sudo cp -f "${ISAAC_ROS_WS}/scripts/workspace-entrypoint.sh" \
    "${ISAAC_ROS_WS}/src/isaac_ros_common/docker/scripts/"

# Ensure mavlink-router config directory exists
mkdir -p "${HOME}/.local/share/mavlink-router"
# Copy patched mavlink-router configuration file for TCP server
sudo cp -f "${ISAAC_ROS_WS}/auxiliary/mavlink_router_config/main.conf" \
    "${HOME}/.local/share/mavlink-router"

# Function to append to .bashrc if not already present
echo "Adding aliases..."
append_if_not_exists() {
    local line="$1"
    if ! grep -qF "$line" "$BASHRC_FILE"; then
        echo "$line" >> "$BASHRC_FILE"
    fi
}

# Write everything to .bashrc if it does not already exist
append_if_not_exists "$EXPORT_DISPLAY"
append_if_not_exists "$EXPORT_ISAAC_WS"
append_if_not_exists "$ISAAC_BUILD_ALIAS"
append_if_not_exists "$ISAAC_RUN_ALIAS"
append_if_not_exists "$ISAAC_START_ALIAS"
append_if_not_exists "$ISAAC_STOP_ALIAS"
append_if_not_exists "$ISAAC_BASH_ALIAS"
append_if_not_exists "$EXPORT_ROS_DOMAIN_ID"

# Handle local_ws
if [ ! -d "${LOCAL_WS}/.git" ]; then
    echo "Cloning local_ws..."
    git clone --recurse-submodules \
        https://github.com/indro-robotics/cypher_drone_local_ws.git \
        "${LOCAL_WS}"
    sudo chmod +x "${LOCAL_WS}/scripts/setup.sh"
else
    echo "${LOCAL_WS} already exists. Pulling latest changes..."
    git -C "${LOCAL_WS}" pull --rebase --autostash
    git -C "${LOCAL_WS}" submodule update --init --recursive
fi

# Reload shell config for aliases / exports
# (safe even when sourced; if run as standalone script, it's just a no-op for current shell)
if [ -f "${BASHRC_FILE}" ]; then
    # shellcheck disable=SC1090
    source "${BASHRC_FILE}"
fi

echo "Copying system service files..."
sudo cp -f "${ISAAC_ROS_WS}/scripts/services/start_isaac_docker.service" "/etc/systemd/system/"
sudo cp -f "${ISAAC_ROS_WS}/scripts/services/jetson-clocks.service" "/etc/systemd/system/"

# Add the rule if not already present
if sudo grep -Fxq "$SUDOERS_LINE" "$SUDOERS_FILE" 2>/dev/null; then
    echo "Rule already present in $SUDOERS_FILE"
else
    echo "$SUDOERS_LINE" | sudo tee "$SUDOERS_FILE" > /dev/null
    sudo chmod 440 "$SUDOERS_FILE"
    echo "Rule added to $SUDOERS_FILE"
fi

echo "Enable Docker socker..."
sudo systemctl enable docker.socket

echo "Enable Isaac ROS docker service..."
sudo systemctl enable start_isaac_docker.service

echo "Enable Jetson high-perf mode..."
sudo systemctl enable jetson-clocks.service

if [[ "$setup_type" == "New Setup" ]]; then
    echo "Installing and configuring Docker..."
    curl https://get.docker.com | sh -s -- --version 27.5.1
    sudo systemctl --now enable docker
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
    sudo usermod -aG docker "$USER"
    newgrp docker
    sudo systemctl daemon-reload
    sudo systemctl restart docker
    sudo apt-get update
    sudo apt-get install -y ca-certificates curl gnupg
    sudo install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
        | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
    sudo chmod a+r /etc/apt/keyrings/docker.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
    $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
    sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    sudo apt-get update
    sudo apt install -y docker-buildx-plugin
    sudo systemctl restart docker
    echo "Setup complete. Rebooting system..."
else
    echo "Patch complete. Rebooting system..."
fi

sudo reboot
