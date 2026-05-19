#!/bin/bash
# Run this once on a fresh Ubuntu 22.04 RPi after cloning the repo.
# Usage: bash setup.sh
set -e

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASHRC="$HOME/.bashrc"

add_to_bashrc() {
    grep -qF "$1" "$BASHRC" || echo "$1" >> "$BASHRC"
}

add_to_config() {
    grep -q "^$1" /boot/firmware/config.txt || echo "$1" | sudo tee -a /boot/firmware/config.txt
}

echo ""
echo "========================================"
echo "  pies-lipad setup"
echo "========================================"
echo ""

# ── 1. ROS2 Humble ──────────────────────────────────────────────────────────
if [ ! -d "/opt/ros/humble" ]; then
    echo ">> Installing ROS2 Humble..."
    sudo apt update && sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    sudo apt install -y software-properties-common curl
    sudo add-apt-repository universe -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update && sudo apt upgrade -y
    sudo apt install -y ros-humble-desktop ros-dev-tools
else
    echo ">> ROS2 Humble already installed, skipping."
fi

# ── 2. System packages ───────────────────────────────────────────────────────
echo ">> Installing system packages..."
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    python3-pip \
    ros-dev-tools \
    python3-lgpio \
    ros-humble-camera-ros \
    ros-humble-cv-bridge \
    python3-opencv \
    v4l-utils \
    libcamera-tools \
    network-manager \
    tesseract-ocr

# dialout: /dev/serial0 (Pixhawk) and /dev/gpiochip0 (lgpio)
# video:   /dev/video* and /dev/media* (camera)
echo ">> Adding $USER to dialout and video groups..."
sudo usermod -aG dialout "$USER"
sudo usermod -aG video "$USER"

# ── 3. Python dependencies ───────────────────────────────────────────────────
echo ">> Installing Python dependencies..."
pip install --user \
    "numpy<2" \
    "setuptools==59.6.0" \
    "empy==3.3.4" \
    pyros-genmsg \
    pymavlink \
    pytesseract

# ── 4. Boot config (camera + serial) ────────────────────────────────────────
echo ">> Configuring /boot/firmware/config.txt..."
# Camera
add_to_config "camera_auto_detect=1"
add_to_config "gpu_mem=128"
# UART for Pixhawk on /dev/serial0 (TELEM2 at 921600 baud)
add_to_config "enable_uart=1"
# Disable Bluetooth to free the primary UART (wlan0 not affected)
add_to_config "dtoverlay=disable-bt"

# ── 5. NetworkManager + field connections ────────────────────────────────────
# Safe to run over SSH — netplan apply transfers management to NM without
# dropping active connections.
echo ">> Setting up NetworkManager..."

# Disable cloud-init network management (conflicts with NM)
if [ ! -f /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg ]; then
    sudo bash -c 'echo "network: {config: disabled}" > /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg'
fi

# Switch netplan renderer to NetworkManager
if [ ! -f /etc/netplan/99-nm.yaml ]; then
    sudo bash -c 'printf "network:\n  version: 2\n  renderer: NetworkManager\n" > /etc/netplan/99-nm.yaml'
    sudo chmod 600 /etc/netplan/99-nm.yaml
    sudo netplan apply
    sleep 3  # give NM time to take over before running nmcli
fi

# Ethernet static IP — direct cable connection to laptop (10.42.0.1 ↔ 10.42.0.2)
if [ ! -f /etc/netplan/99-eth0-static.yaml ]; then
    sudo bash -c 'cat > /etc/netplan/99-eth0-static.yaml << EOF
network:
  version: 2
  ethernets:
    eth0:
      addresses: [10.42.0.2/24]
      dhcp4: false
EOF'
    sudo chmod 600 /etc/netplan/99-eth0-static.yaml
    sudo netplan apply
fi

# WiFi hotspot for field use
# autoconnect yes — starts automatically on boot at the field.
# At home: run "sudo nmcli con down pies-hotspot" to restore internet.
if ! sudo nmcli con show pies-hotspot &>/dev/null; then
    sudo nmcli con add type wifi ifname wlan0 con-name pies-hotspot ssid "piesdrone" mode ap \
        ipv4.method shared ipv4.addresses 172.16.0.1/24 \
        wifi-sec.key-mgmt wpa-psk wifi-sec.psk "piesdrone123" autoconnect yes
    echo "   Hotspot created: SSID=piesdrone, IP=172.16.0.1, autoconnect=yes"
else
    echo "   pies-hotspot already exists, skipping."
fi

# ── 6. Micro-XRCE-DDS-Agent ─────────────────────────────────────────────────
echo ">> Cloning Micro-XRCE-DDS-Agent..."
if [ ! -d "$REPO/Micro-XRCE-DDS-Agent" ]; then
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$REPO/Micro-XRCE-DDS-Agent"
else
    echo "   Micro-XRCE-DDS-Agent already present, skipping."
fi

# ── 7. px4_msgs + px4_ros_com ────────────────────────────────────────────────
echo ">> Cloning PX4 ROS2 packages..."
PX4_SRC="$REPO/ros2_ws/src"
if [ ! -d "$PX4_SRC/px4_msgs" ]; then
    git clone https://github.com/PX4/px4_msgs.git "$PX4_SRC/px4_msgs"
else
    echo "   px4_msgs already present, skipping."
fi
if [ ! -d "$PX4_SRC/px4_ros_com" ]; then
    git clone https://github.com/PX4/px4_ros_com.git "$PX4_SRC/px4_ros_com"
else
    echo "   px4_ros_com already present, skipping."
fi

# ── 8. rosdep ────────────────────────────────────────────────────────────────
echo ">> Running rosdep..."
sudo rosdep init 2>/dev/null || true  # already initialized is not an error
rosdep update
rosdep install --from-paths "$REPO/ros2_ws/src" --ignore-src -r -y

# ── 9. Build Micro-XRCE-DDS-Agent ───────────────────────────────────────────
echo ">> Building Micro-XRCE-DDS-Agent (this takes a while)..."
mkdir -p "$REPO/Micro-XRCE-DDS-Agent/build"
cmake -S "$REPO/Micro-XRCE-DDS-Agent" -B "$REPO/Micro-XRCE-DDS-Agent/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$REPO/Micro-XRCE-DDS-Agent/build" --target MicroXRCEAgent -- -j$(nproc)

# ── 10. Build ROS2 workspace ─────────────────────────────────────────────────
echo ">> Building ROS2 workspace (this takes a while)..."
source /opt/ros/humble/setup.bash
cd "$REPO/ros2_ws"
colcon build --symlink-install

# ── 11. Update .bashrc ───────────────────────────────────────────────────────
echo ">> Updating .bashrc..."
add_to_bashrc "source /opt/ros/humble/setup.bash"
add_to_bashrc "source $REPO/ros2_ws/install/local_setup.bash"

echo ""
echo "========================================"
echo "  Setup complete!"
echo ""
echo "  IMPORTANT: reboot before first use so group memberships"
echo "  (dialout, video) and UART/BT overlay take effect."
echo ""
echo "  After reboot:"
echo "    source ~/.bashrc"
echo ""
echo "  To start the system (see docs/startup.md for full details):"
echo "    sudo $REPO/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent serial --dev /dev/serial0 -b 921600"
echo "    ros2 launch pies_mission autonomous.launch.py"
echo ""
echo "  Field connection: hotspot starts automatically on boot."
echo "  At home after field use: sudo nmcli con down pies-hotspot"
echo "========================================"
