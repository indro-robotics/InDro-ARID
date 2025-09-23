#!/bin/bash

USERNAME="jetson"
BASHRC_FILE=${HOME}/.bashrc
LOCAL_WS="${HOME}/workspaces/local_ws"
EXPORT_LOCAL_WS="export LOCAL_WS=${LOCAL_WS}"
SOURCE_LOCAL_WS="source ${LOCAL_WS}/install/setup.bash"
POLKIT_RULE_FILE="/etc/polkit-1/rules.d/10-reset-usb.rules"
RESET_USB_ALIAS='alias reset_usb="/bin/bash $LOCAL_WS/scripts/usb_reset.sh"'
SUDOERS_LINE="$USERNAME ALL=(ALL) NOPASSWD: /bin/systemctl start *, /bin/systemctl stop *, /bin/systemctl kill *, $LOCAL_WS/scripts/usb_reset.sh"

# Function to append to .bashrc if not already present
echo "Adding aliases..."
append_if_not_exists() {
    local line="$1"
    if ! grep -qF "$line" "$BASHRC_FILE"; then
        echo "$line" | sudo tee -a "$BASHRC_FILE" > /dev/null
    fi
}

append_if_not_exists "$EXPORT_LOCAL_WS"
append_if_not_exists "$SOURCE_LOCAL_WS"
append_if_not_exists "$RESET_USB_ALIAS"

source ~/.bashrc

# File to create in /etc/sudoers.d
SUDOERS_FILE="/etc/sudoers.d/${USERNAME}_systemctl"

# Add the rule if not already present
if sudo grep -Fxq "$SUDOERS_LINE" "$SUDOERS_FILE" 2>/dev/null; then
    echo "Rule already present in $SUDOERS_FILE"
else
    echo "$SUDOERS_LINE" | sudo tee "$SUDOERS_FILE" > /dev/null
    sudo chmod 440 "$SUDOERS_FILE"
    echo "Rule added to $SUDOERS_FILE"
fi

sudo cp -f "${LOCAL_WS}/services/"*.service "/etc/systemd/system/"
sudo cp -f "${LOCAL_WS}/src/csi_drone_camera/source/"*.service "/etc/systemd/system/"
sudo cp -f "${LOCAL_WS}/services/jetson-clocks.service" "/etc/systemd/system/"
chmod +x /home/jetson/workspaces/local_ws/scripts/usb_reset.sh


# ========== BEGIN CRITICAL USB CNTL ==========
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
polkit.addRule(function(action, subject) {
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

sudo systemctl enable usb_ros_reset.service
sudo systemctl enable cam_manager.service
sudo systemctl enable jetson-clocks.service
sudo systemctl daemon-reload

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo apt update
python3 -m pip install "setuptools<66" websockets==15.0.1 pyudev==0.24.3 pyserial==3.5

cd ${LOCAL_WS}
sudo rosdep init
rosdep update

rosdep install --from-paths ${LOCAL_WS}/src/ --ignore-src -y
colcon build --symlink-install --base-paths ${LOCAL_WS}/src
