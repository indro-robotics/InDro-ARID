#!/bin/bash

USERNAME="jetson"
BASHRC_FILE=${HOME}/.bashrc
EXPORT_DISPLAY="export DISPLAY=:1001"
ISAAC_ROS_WS="${HOME}/workspaces/isaac_ros-dev"
EXPORT_ISAAC_WS="export ISAAC_ROS_WS=${ISAAC_ROS_WS}"
ISAAC_RUN_ALIAS='alias run_isaac="/bin/bash $ISAAC_ROS_WS/scripts/run_isaac_docker.sh"'
ISAAC_BUILD_ALIAS='alias build_isaac="/bin/bash $ISAAC_ROS_WS/scripts/build_isaac_docker.sh"'
ISAAC_START_ALIAS='alias start_isaac="/bin/bash $ISAAC_ROS_WS/scripts/start_isaac_docker.sh"'
ISAAC_STOP_ALIAS='alias stop_isaac="docker stop isaac_ros_dev-aarch64-container"'
ISAAC_BASH_ALIAS='alias isaac_bash="/bin/bash $ISAAC_ROS_WS/scripts/isaac_bash.sh"'
SUDOERS_FILE="/etc/sudoers.d/${USERNAME}_systemctl"
SUDOERS_LINE="$USERNAME ALL=(ALL) NOPASSWD: \\
    /usr/sbin/uhubctl, \\
    /usr/bin/gpioset, \\
    /bin/systemctl start *, \\
    /bin/systemctl stop *, \\
    /bin/systemctl restart *"

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

# Script permissions
echo "Setting script permissions..."
find ${ISAAC_ROS_WS}/scripts/ -type f -iname "*.sh" -exec chmod +x {} +

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

source ~/.bashrc

echo "Copying system service files..."
sudo cp -f "${ISAAC_ROS_WS}/services/start_isaac_docker.service" "/etc/systemd/system/"

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

if [[ "$setup_type" == "New Setup" ]]; then
    echo "Installing and configuring Docker..."
    curl https://get.docker.com | sh -s -- --version 27.5.1
    sudo systemctl --now enable docker
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
    sudo usermod -aG docker $USER
    newgrp docker
    sudo systemctl daemon-reload
    sudo systemctl restart docker
    sudo apt-get update
    sudo apt-get install -y ca-certificates curl gnupg
    sudo install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
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
