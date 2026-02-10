#!/bin/bash

set -euo pipefail

PS4='+ ${BASH_SOURCE}:${LINENO}:${FUNCNAME[0]:-main}: '

failure() {
  local exit_code=$?
  local line=$1
  echo "Error: command failed at ${BASH_SOURCE[0]}:${line}: '${BASH_COMMAND}' (exit: ${exit_code})" >&2
  exit "$exit_code"
}

trap 'failure ${LINENO}' ERR

USERNAME="jetson"
BASHRC_FILE=${HOME}/.bashrc
EXPORT_ROS_DOMAIN_ID="export ROS_DOMAIN_ID=23"
WORKSPACES="${HOME}/workspaces"
EXPORT_WORKSPACES="export WORKSPACES=${WORKSPACES}"
LOCAL_WS="${WORKSPACES}/local_ws"
EXPORT_LOCAL_WS="export LOCAL_WS=${LOCAL_WS}"
SOURCE_LOCAL_WS="source ${LOCAL_WS}/install/setup.bash"
ISAAC_ROS_WS="${WORKSPACES}/isaac_ros-dev"
EXPORT_ISAAC_WS="export ISAAC_ROS_WS=${ISAAC_ROS_WS}"
PX4_DIR="${LOCAL_WS}/auxiliary/PX4-Autopilot"
ISAAC_RUN_ALIAS='alias run_isaac="/bin/bash $ISAAC_ROS_WS/container_scripts/run_isaac_docker.sh"'
ISAAC_BUILD_ALIAS='alias build_isaac="/bin/bash $ISAAC_ROS_WS/container_scripts/build_isaac_docker.sh"'
ISAAC_START_ALIAS='alias start_isaac="/bin/bash $ISAAC_ROS_WS/container_scripts/start_isaac_docker.sh"'
ISAAC_STOP_ALIAS='alias stop_isaac="docker stop isaac_ros_dev-aarch64-container"'
ISAAC_BASH_ALIAS='alias isaac_bash="/bin/bash $ISAAC_ROS_WS/container_scripts/isaac_bash.sh"'
POLKIT_RULE_FILE="/etc/polkit-1/rules.d/10-reset-usb.rules"
RESET_USB_ALIAS='alias reset_usb="/bin/bash $WORKSPACES/scripts/usb_reset.sh"'
ROSDEP_ALIAS="alias rosdep_local='rosdep install --from-paths \${LOCAL_WS}/src/ --ignore-src -y'"
COLCON_ALIAS="alias colcon_local='cd \${LOCAL_WS} && colcon build --symlink-install --base-paths src && source ./install/setup.bash'"
CLEAN_ALIAS="alias clean_local='cd \${LOCAL_WS} && colcon clean workspace --base-select build install log'"
EXPORT_X11_LOCAL='xhost +local: >/dev/null 2>&1 || true'
ADD_ENTRY_DIR="${ISAAC_ROS_WS}/src/isaac_ros_common/docker/scripts/entrypoint_additions"
ADD_ENTRY_FILE="${ADD_ENTRY_DIR}/additional_entry.user.sh"

EXPORT_DISPLAY='if [ -d /tmp/.X11-unix ]; then
    sock=$(ls /tmp/.X11-unix/X* 2>/dev/null | head -n1)
    if [ -n "$sock" ]; then
        num=${sock##*/X}
        export DISPLAY=":${num}"
    fi
fi'

SUDOERS_FILE="/etc/sudoers.d/${USERNAME}_systemctl"
SUDOERS_LINE="$USERNAME ALL=(ALL) NOPASSWD: \
    /usr/sbin/uhubctl, \
    /usr/bin/gpioset, \
    /bin/systemctl start *, \
    /bin/systemctl stop *, \
    /bin/systemctl restart *, \
    /bin/systemctl kill *, \
    $WORKSPACES/scripts/usb_reset.sh"
REPO_ROOT="$(git rev-parse --show-toplevel)"


cd "$REPO_ROOT" || exit


# set power mode
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



if [ -d "${PX4_DIR}" ]; then
    while true; do
        read -r -p "Install PX4 build dependencies? (y/n)" px4_setup
        case "$px4_setup" in
            [yY] )
                echo "Running PX4 Tools/setup/ubuntu.sh..."
                (   
                    cd "${PX4_DIR}/Tools/setup"
                    bash ubuntu.sh #</dev/null
                )
                echo "PX4 build dependencies installation complete."
                break
                ;;
            [nN] )
                echo "Skipping PX4 build dependencies installation."
                break
                ;;
            * )
                echo "choose y/n."
                ;;
        esac
    done
else
    echo "PX4-Autopilot not found at ${PX4_DIR}, skipping PX4 setup."
fi



# Cache credentials in memory for 7 days
git config --global credential.helper "cache --timeout=604800"


# ========== ISAAC ROS WORKSPACE SETUP ==========

# Script permissions
echo "Setting script permissions..."
find ${WORKSPACES}/scripts -type f \( -name "*.bash" -o -name "*.sh" \) -exec chmod +x {} \;
find ${ISAAC_ROS_WS}/container_scripts -type f \( -name "*.bash" -o -name "*.sh" \) -exec chmod +x {} \;


echo "Update/checkout submodules..."
"${WORKSPACES}/scripts/update_isaac_submods.sh"

# Ensure no changes to config folders (update apriltag and camera calibration paths)
git update-index --assume-unchanged ${ISAAC_ROS_WS}/src/px4_vslam/config/


# Copy custom docker-config files
echo "Patching dockerfiles..."
sudo cp -f "${ISAAC_ROS_WS}/docker_resources/patched_dockerfiles/.isaac_ros_common-config" \
    "${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/"
    
sudo cp -f \
  "${ISAAC_ROS_WS}/docker_resources/dockerfiles/Dockerfile.user" \
  "${ISAAC_ROS_WS}/docker_resources/dockerfiles/Dockerfile.cypher" \
  "${ISAAC_ROS_WS}/docker_resources/patched_dockerfiles/Dockerfile.aarch64" \
  "${ISAAC_ROS_WS}/src/isaac_ros_common/docker/"


# Copy patched run_dev.sh script to keep persistent docker container
sudo cp -f "${ISAAC_ROS_WS}/container_scripts/run_dev.sh" \
    "${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/"


# Copy patched workspace-entrypoint.sh script to stop crashing on container start
sudo mkdir -p "$ADD_ENTRY_DIR"
sudo cp -f "${ISAAC_ROS_WS}/container_scripts/additional_entry.sh" "$ADD_ENTRY_FILE"
sudo chmod +x "$ADD_ENTRY_FILE"

# ========== START BASHRC ALIASES (ISAAC + LOCAL) ==========
echo "Adding aliases..."

if ! grep -qF 'export DISPLAY=' "$BASHRC_FILE"; then
    echo "$EXPORT_DISPLAY" >> "$BASHRC_FILE"
fi

append_if_not_exists() {
    local line="$1"
    if ! grep -qF "$line" "$BASHRC_FILE"; then
        echo "$line" >> "$BASHRC_FILE"
    fi
}

append_if_not_exists "$EXPORT_X11_LOCAL"
append_if_not_exists "$EXPORT_ISAAC_WS"
append_if_not_exists "$ISAAC_BUILD_ALIAS"
append_if_not_exists "$ISAAC_RUN_ALIAS"
append_if_not_exists "$ISAAC_START_ALIAS"
append_if_not_exists "$ISAAC_STOP_ALIAS"
append_if_not_exists "$ISAAC_BASH_ALIAS"
append_if_not_exists "$EXPORT_ROS_DOMAIN_ID"
append_if_not_exists "$EXPORT_LOCAL_WS"
append_if_not_exists "$SOURCE_LOCAL_WS"
append_if_not_exists "$RESET_USB_ALIAS"
append_if_not_exists "$ROSDEP_ALIAS"
append_if_not_exists "$COLCON_ALIAS"
append_if_not_exists "$CLEAN_ALIAS"


if [ -f "${BASHRC_FILE}" ]; then
    source "${BASHRC_FILE}"
fi
# ========== END BASHRC ALIASES (ISAAC + LOCAL) ==========



# ========== START SUDOER RULES ==========
if sudo grep -Fxq "$SUDOERS_LINE" "$SUDOERS_FILE" 2>/dev/null; then
    echo "Rule already present in $SUDOERS_FILE"
else
    echo "$SUDOERS_LINE" | sudo tee "$SUDOERS_FILE" > /dev/null
    sudo chmod 440 "$SUDOERS_FILE"
    echo "Rule added to $SUDOERS_FILE"
fi
# ========== END SUDOER RULES ==========


# For previous jetpack, evaluate necessity
# ========== START CRITICAL L4T COMPATIBILITY ==========
# echo "Synchronizing L4T firmware + core..."
#     sudo apt update && sudo apt install --allow-downgrades --reinstall \
#       nvidia-l4t-firmware=36.3.0-20240719161631 \
#       nvidia-l4t-core=36.3.0-20240719161631
# ========== END CRITICAL L4T COMPATIBILITY ==========



# ========== START CRITICAL USB CNTL ==========
# Install dependencies for USB control
sudo apt-get install -y libusb-1.0-0-dev pkgconf
sudo apt install -y gpiod

# Handle uhubctl (skip if exists)
UHUBCTL_DIR="${HOME}/uhubctl"
if [ ! -d "${UHUBCTL_DIR}" ]; then
    git clone https://github.com/mvp/uhubctl ${UHUBCTL_DIR}
else
    echo "${UHUBCTL_DIR} already exists. Skipping clone."
fi

cd ${UHUBCTL_DIR}
make
sudo make install
cd ~

echo "Configuring USB/GPIO permissions..."

# Create USB udev rules
sudo tee /etc/udev/rules.d/52-usb.rules <<'EOL'
# USB2/3 hub permissions
SUBSYSTEM=="usb", DRIVER=="usb", MODE="0664", GROUP="dialout", ATTR{idVendor}=="2109"
SUBSYSTEM=="usb", DRIVER=="usb", MODE="0664", GROUP="dialout", ATTR{idVendor}=="1d6b"
# Linux 6.0+ interface
SUBSYSTEM=="usb", DRIVER=="usb", \
  RUN+="/bin/sh -c \"chown -f root:dialout \$sys\$devpath/*port*/disable || true\"", \
  RUN+="/bin/sh -c \"chmod -f 660 \$sys\$devpath/*port*/disable || true\""
EOL

# Create GPIO udev rule
sudo tee /etc/udev/rules.d/99-gpio.rules <<EOL
SUBSYSTEM=="gpio", GROUP="gpio", MODE="0660"
EOL

# Add user to required groups
sudo usermod -aG dialout,gpio ${USERNAME}

# Create polkit rule
echo "Creating Polkit rule..."
sudo tee "$POLKIT_RULE_FILE" > /dev/null <<EOL
polkit.addRule(function(action, subject) {RAV4
    if (action.id == "org.freedesktop.systemd1.manage-units" &&
        action.lookup("unit") == "reset_usb.service" &&
        subject.user == "$USERNAME") {
        return polkit.Result.YES;
    }
});
EOL

# Set proper permissions
sudo chmod 644 "$POLKIT_RULE_FILE"
# ========== END CRITICAL USB CNTL ==========



# ========== START SYSTEM SERVICES ==========
echo "Copying system service files..."
sudo cp -f "${ISAAC_ROS_WS}/services/"*.service "/etc/systemd/system/"
sudo cp -f "${LOCAL_WS}/services/"*.service "/etc/systemd/system/"

sudo systemctl enable usb_ros_reset.service
sudo systemctl enable uwb_ros_node.service
sudo systemctl enable start_isaac_docker.service
sudo systemctl enable jetson-clocks.service
sudo systemctl daemon-reload
# ========== END SYSTEM SERVICES ==========



# ========== START ROS2 / COLCON / DEPENDENCIES FOR LOCAL_WS ==========
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo apt update

python3 -m pip install \
  "websockets==15.0.1" \
  "pyudev==0.24.3" \
  "pyserial==3.5" \
  "colcon-clean==0.2.1" \
  "empy<4" \
  --force-reinstall --no-deps

cd ${LOCAL_WS}
sudo rosdep init || true
rosdep update

rosdep install --from-paths ${LOCAL_WS}/src/ --ignore-src -y
colcon build --symlink-install --base-paths ${LOCAL_WS}/src
# ========== END ROS2 / COLCON / DEPENDENCIES FOR LOCAL_WS ==========


# ========== DOCKER INSTALL / ENABLE / REBOOT ==========
if [[ "$setup_type" == "New Setup" ]]; then
    echo "Installing and configuring Docker..."
    (
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
    )
    echo "Setup complete. Rebooting system..."
else
    echo "Patch complete. Re-applying Docker enables if Docker is installed..."
    if command -v docker >/dev/null 2>&1; then
        sudo systemctl --now enable docker || true
    else
        echo "Docker is not installed, skipping docker.service and docker.socket enable."
    fi
    echo "Patch complete. Rebooting system..."
fi


sudo reboot