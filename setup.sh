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
BASHRC_FILE="${HOME}/.bashrc"
WORKSPACES="${HOME}/workspaces"
LOCAL_WS="${WORKSPACES}/local_ws"
ISAAC_ROS_WS="${WORKSPACES}/isaac_ros-dev"
PX4_DIR="${LOCAL_WS}/auxiliary/PX4-Autopilot"
POLKIT_RULE_FILE="/etc/polkit-1/rules.d/10-reset-usb.rules"
SUDOERS_FILE="/etc/sudoers.d/${USERNAME}_systemctl"
REPO_ROOT="$(git rev-parse --show-toplevel)"

cd "$REPO_ROOT"

###############################################################################
# LOG SETUP
###############################################################################
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${SCRIPT_DIR}/log"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/setup_log_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee -a "${LOG_FILE}") 2>&1

###############################################################################
# PROMPT: NEW SETUP vs PATCH
###############################################################################
echo "Select setup type:"
select setup_type in "New Setup" "Patch"; do
  case $setup_type in
    "New Setup"|"Patch") break ;;
    *) echo "Invalid option, choose 1 or 2." ;;
  esac
done

while true; do
  read -r -p "Install PX4 build dependencies? (y/n) " PX4_INSTALL_DEPS
  case "$PX4_INSTALL_DEPS" in
    [yYnN]) break ;;
    *) echo "Invalid option, choose y or n." ;;
  esac
done

###############################################################################
# POWER & HOLDS
###############################################################################
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

###############################################################################
# REPOSITORIES (ROS, JETSON, DOCKER) – NO INSTALLS YET
###############################################################################
# ROS key
if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
  sudo curl -sSL \
    https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
fi

# PVA
sudo nvidia-ctk cdi generate --mode=csv --output=/etc/cdi/nvidia.yaml
if ! grep -q "repo.download.nvidia.com/jetson/common" /etc/apt/sources.list.d/nvidia-l4t-apt-source.list 2>/dev/null; then
  sudo apt-key adv --fetch-key https://repo.download.nvidia.com/jetson/jetson-ota-public.asc
  echo "deb https://repo.download.nvidia.com/jetson/common r36.4 main" \
    | sudo tee /etc/apt/sources.list.d/nvidia-l4t-apt-source.list >/dev/null
  echo "deb https://repo.download.nvidia.com/jetson/t234 r36.4 main" \
    | sudo tee -a /etc/apt/sources.list.d/nvidia-l4t-apt-source.list >/dev/null
fi

# Docker official repo (key + list); engine itself handled later
if [ ! -f /etc/apt/keyrings/docker.gpg ]; then
  sudo install -m 0755 -d /etc/apt/keyrings
  curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
    | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
  sudo chmod a+r /etc/apt/keyrings/docker.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
https://download.docker.com/linux/ubuntu \
$(. /etc/os-release && echo "$VERSION_CODENAME") stable" \
    | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null
fi

###############################################################################
# SINGLE GLOBAL APT REFRESH + BULK INSTALLS
###############################################################################
sudo apt-get update

# Base system deps used across the script (no docker-buildx-plugin here)
sudo apt-get install -y \
  software-properties-common \
  ca-certificates curl gnupg \
  libusb-1.0-0-dev pkgconf gpiod \
  pva-allow-2 \
  python3-colcon-clean

###############################################################################
# OPTIONAL PX4 DEPS
###############################################################################
if [ -d "${PX4_DIR}" ]; then
  if [[ "$PX4_INSTALL_DEPS" == [yY] ]]; then
    echo "Running PX4 Tools/setup/ubuntu.sh..."
    (
      cd "${PX4_DIR}/Tools/setup"
      bash ubuntu.sh
    )
    echo "PX4 build dependencies installation complete."
  else
    echo "Skipping PX4 build dependencies installation."
  fi
else
  echo "PX4-Autopilot not found at ${PX4_DIR}, skipping PX4 setup."
fi

###############################################################################
# GIT / ISAAC PATCHING
###############################################################################
git config --global credential.helper "cache --timeout=604800"

echo "Setting script permissions..."
find "${WORKSPACES}/scripts" -type f \( -name "*.bash" -o -name "*.sh" \) -exec chmod +x {} \;
find "${ISAAC_ROS_WS}/container_scripts" -type f \( -name "*.bash" -o -name "*.sh" \) -exec chmod +x {} \;

echo "Update/checkout submodules..."
"${WORKSPACES}/scripts/update_isaac_submods.sh"

git update-index --assume-unchanged \
  "${ISAAC_ROS_WS}/src/px4_vslam/config/"

echo "Patching dockerfiles..."
sudo cp -f "${ISAAC_ROS_WS}/docker_resources/patched_dockerfiles/.isaac_ros_common-config" \
  "${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/"

sudo cp -f "${ISAAC_ROS_WS}/docker_resources/dockerfiles/Dockerfile.cypher" \
  "${ISAAC_ROS_WS}/src/isaac_ros_common/docker/"

sudo cp -f "${ISAAC_ROS_WS}/container_scripts/run_dev.sh" \
  "${ISAAC_ROS_WS}/src/isaac_ros_common/scripts/"

sudo cp -f "${ISAAC_ROS_WS}/container_scripts/cypher_env.sh" \
  "${ISAAC_ROS_WS}/src/isaac_ros_common/docker/scripts"

###############################################################################
# BASHRC ALIASES / ENV
###############################################################################
EXPORT_DISPLAY='if [ -d /tmp/.X11-unix ]; then
    sock=$(ls /tmp/.X11-unix/X* 2>/dev/null | head -n1)
    if [ -n "$sock" ]; then
        num=${sock##*/X}
        export DISPLAY=":${num}"
    fi
fi'

EXPORT_X11_LOCAL='xhost +local: >/dev/null 2>&1 || true'
EXPORT_ROS_DOMAIN_ID='export ROS_DOMAIN_ID=23'
EXPORT_WORKSPACES="export WORKSPACES=${WORKSPACES}"
EXPORT_LOCAL_WS="export LOCAL_WS=${LOCAL_WS}"
SOURCE_LOCAL_WS="source ${LOCAL_WS}/install/setup.bash"
EXPORT_ISAAC_WS="export ISAAC_ROS_WS=${ISAAC_ROS_WS}"

ISAAC_RUN_ALIAS="alias run_isaac='/bin/bash ${ISAAC_ROS_WS}/container_scripts/run_isaac_docker.sh'"
ISAAC_BUILD_ALIAS="alias build_isaac='/bin/bash ${ISAAC_ROS_WS}/container_scripts/build_isaac_docker.sh'"
ISAAC_START_ALIAS="alias start_isaac='/bin/bash ${ISAAC_ROS_WS}/container_scripts/start_isaac_docker.sh'"
ISAAC_STOP_ALIAS="alias stop_isaac='docker stop isaac_ros_dev-aarch64-container'"
ISAAC_BASH_ALIAS="alias isaac_bash='/bin/bash ${ISAAC_ROS_WS}/container_scripts/isaac_bash.sh'"
RESET_USB_ALIAS="alias reset_usb='/bin/bash ${WORKSPACES}/scripts/usb_reset.sh'"
ROSDEP_ALIAS="alias rosdep_local=\"rosdep install --from-paths \${LOCAL_WS}/src/ --ignore-src -y\""
COLCON_ALIAS="alias colcon_local=\"cd \${LOCAL_WS} && colcon build --symlink-install --base-paths src && source ./install/setup.bash\""
CLEAN_ALIAS="alias clean_local=\"cd \${LOCAL_WS} && colcon clean workspace --base-select build install log\""

append_if_not_exists() {
  local line="$1"
  if ! grep -qF "$line" "$BASHRC_FILE" 2>/dev/null; then
    echo "$line" >> "$BASHRC_FILE"
  fi
}

append_if_not_exists "$EXPORT_DISPLAY"
append_if_not_exists "$EXPORT_X11_LOCAL"
append_if_not_exists "$EXPORT_WORKSPACES"
append_if_not_exists "$EXPORT_LOCAL_WS"
append_if_not_exists "$SOURCE_LOCAL_WS"
append_if_not_exists "$EXPORT_ISAAC_WS"
append_if_not_exists "$ISAAC_BUILD_ALIAS"
append_if_not_exists "$ISAAC_RUN_ALIAS"
append_if_not_exists "$ISAAC_START_ALIAS"
append_if_not_exists "$ISAAC_STOP_ALIAS"
append_if_not_exists "$ISAAC_BASH_ALIAS"
append_if_not_exists "$EXPORT_ROS_DOMAIN_ID"
append_if_not_exists "$RESET_USB_ALIAS"
append_if_not_exists "$ROSDEP_ALIAS"
append_if_not_exists "$COLCON_ALIAS"
append_if_not_exists "$CLEAN_ALIAS"

[ -f "${BASHRC_FILE}" ] && source "${BASHRC_FILE}"

###############################################################################
# SUDOERS / UDEV / POLKIT / GROUPS
###############################################################################
SUDOERS_LINE="$USERNAME ALL=(ALL) NOPASSWD: \
    /usr/sbin/uhubctl, \
    /usr/bin/gpioset, \
    /bin/systemctl start *, \
    /bin/systemctl stop *, \
    /bin/systemctl restart *, \
    /bin/systemctl kill *, \
    ${WORKSPACES}/scripts/usb_reset.sh"

if sudo grep -Fxq "$SUDOERS_LINE" "$SUDOERS_FILE" 2>/dev/null; then
  echo "Rule already present in $SUDOERS_FILE"
else
  echo "$SUDOERS_LINE" | sudo tee "$SUDOERS_FILE" >/dev/null
  sudo chmod 440 "$SUDOERS_FILE"
  echo "Rule added to $SUDOERS_FILE"
fi

UHUBCTL_DIR="${HOME}/uhubctl"
if [ ! -d "${UHUBCTL_DIR}" ]; then
  git clone https://github.com/mvp/uhubctl "${UHUBCTL_DIR}"
fi
cd "${UHUBCTL_DIR}"
make
sudo make install
cd ~

sudo tee /etc/udev/rules.d/52-usb.rules >/dev/null <<'EOL'
# USB2/3 hub permissions
SUBSYSTEM=="usb", DRIVER=="usb", MODE="0664", GROUP="dialout", ATTR{idVendor}=="2109"
SUBSYSTEM=="usb", DRIVER=="usb", MODE="0664", GROUP="dialout", ATTR{idVendor}=="1d6b"
# Linux 6.0+ interface
SUBSYSTEM=="usb", DRIVER=="usb", \
  RUN+="/bin/sh -c \"chown -f root:dialout \$sys\$devpath/*port*/disable || true\"", \
  RUN+="/bin/sh -c \"chmod -f 660 \$sys\$devpath/*port*/disable || true\""
EOL

sudo tee /etc/udev/rules.d/99-gpio.rules >/dev/null <<'EOL'
SUBSYSTEM=="gpio", GROUP=="gpio", MODE=="0660"
EOL

sudo usermod -aG dialout,gpio "${USERNAME}"

echo "Creating Polkit rule..."
sudo tee "$POLKIT_RULE_FILE" >/dev/null <<EOL
polkit.addRule(function(action, subject) {
    if (action.id == "org.freedesktop.systemd1.manage-units" &&
        action.lookup("unit") == "reset_usb.service" &&
        subject.user == "$USERNAME") {
        return polkit.Result.YES;
    }
});
EOL
sudo chmod 644 "$POLKIT_RULE_FILE"

###############################################################################
# SYSTEMD SERVICES
###############################################################################
echo "Copying system service files..."
sudo cp -f "${ISAAC_ROS_WS}/services/"*.service "/etc/systemd/system/"
sudo cp -f "${LOCAL_WS}/services/"*.service "/etc/systemd/system/"

sudo systemctl enable usb_ros_reset.service
sudo systemctl enable uwb_ros_node.service
sudo systemctl enable start_isaac_docker.service
sudo systemctl enable jetson-clocks.service
sudo systemctl daemon-reload

###############################################################################
# ROS2 / LOCAL_WS DEPS (apt already refreshed)
###############################################################################
ensure_pip_pkg() {
  local pkg="$1"       # e.g. websockets==15.0.1 or empy<4
  local name="$2"      # e.g. websockets
  local want_version="$3" # e.g. 15.0.1 or 3.3.4 or empty for unconstrained

  local have
  have=$(python3 -m pip show "$name" 2>/dev/null | awk '/^Version: / {print $2}' || true)

  if [ -z "$have" ]; then
    echo "Installing $pkg (not currently installed)..."
    python3 -m pip install "$pkg" --no-deps
  elif [ -n "$want_version" ] && [ "$have" != "$want_version" ]; then
    echo "Upgrading $name from $have to $want_version..."
    python3 -m pip install "$pkg" --no-deps
  else
    echo "$name==$have already satisfies requirement $pkg, skipping."
  fi
}

# websockets==15.0.1
ensure_pip_pkg "websockets==15.0.1" "websockets" "15.0.1"
# pyudev==0.24.3
ensure_pip_pkg "pyudev==0.24.3" "pyudev" "0.24.3"
# pyserial==3.5
ensure_pip_pkg "pyserial==3.5" "pyserial" "3.5"
# empy<4 (just ensure installed; version bound is loose)
ensure_pip_pkg "empy<4" "empy" ""

cd "${LOCAL_WS}"
# rosdep init only if not already initialized
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
else
  echo "rosdep already initialized, skipping init."
fi

rosdep update
rosdep install --from-paths "${LOCAL_WS}/src/" --ignore-src -y

colcon build \
  --symlink-install \
  --base-paths "${LOCAL_WS}/src" \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON

###############################################################################
# NVIDIA CDI + DOCKER ENGINE + BUILDX
###############################################################################
if [[ "$setup_type" == "New Setup" ]]; then
  echo "Installing and configuring Docker..."
  (
    if ! command -v docker >/dev/null 2>&1; then
      curl https://get.docker.com | sh -s -- --version 29.2.1
    fi

    sudo systemctl --now enable docker
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
    sudo usermod -aG docker "$USER"

    sudo apt-get install -y docker-buildx-plugin
    sudo systemctl --now enable docker || true
  )
else
  if command -v docker >/dev/null 2>&1; then
    sudo systemctl --now enable docker || true
  else
    echo "Docker is not installed... repeat and choose 'New Setup'"
  fi
fi

echo "Rebooting system..."
sudo reboot