#!/bin/bash
# Run this once on a fresh Ubuntu 22.04 RPi after cloning the repo.
# Usage: bash setup.sh
set -e

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASHRC="$HOME/.bashrc"

add_to_bashrc() {
    grep -qF "$1" "$BASHRC" || echo "$1" >> "$BASHRC"
}

echo ""
echo "========================================"
echo "  pies-lipad setup"
echo "========================================"
echo ""

# ── 1. ROS2 Humble ──────────────────────────────────────────────────────────
if ! command -v ros2 &>/dev/null; then
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
    python3-lgpio \
    ros-humble-camera-ros \
    ros-humble-cv-bridge \
    python3-opencv \
    v4l-utils \
    libcamera-tools

# ── 3. Python dependencies ───────────────────────────────────────────────────
echo ">> Installing Python dependencies..."
pip install --user "numpy<2" "setuptools==59.6.0" "empy==3.3.4" pyros-genmsg

# ── 4. Camera config ─────────────────────────────────────────────────────────
echo ">> Configuring camera in /boot/firmware/config.txt..."
if ! grep -q "^camera_auto_detect=1" /boot/firmware/config.txt; then
    echo "camera_auto_detect=1" | sudo tee -a /boot/firmware/config.txt
fi
if ! grep -q "^gpu_mem=128" /boot/firmware/config.txt; then
    echo "gpu_mem=128" | sudo tee -a /boot/firmware/config.txt
fi

# ── 5. Micro-XRCE-DDS-Agent ─────────────────────────────────────────────────
echo ">> Cloning Micro-XRCE-DDS-Agent..."
if [ ! -d "$REPO/Micro-XRCE-DDS-Agent" ]; then
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$REPO/Micro-XRCE-DDS-Agent"
else
    echo "   Micro-XRCE-DDS-Agent already present, skipping."
fi

# ── 6. px4_msgs + px4_ros_com ────────────────────────────────────────────────
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

# ── 7. Build Micro-XRCE-DDS-Agent ───────────────────────────────────────────
echo ">> Building Micro-XRCE-DDS-Agent (this takes a while)..."
mkdir -p "$REPO/Micro-XRCE-DDS-Agent/build"
cmake -S "$REPO/Micro-XRCE-DDS-Agent" -B "$REPO/Micro-XRCE-DDS-Agent/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$REPO/Micro-XRCE-DDS-Agent/build" --target MicroXRCEAgent -- -j$(nproc)

# ── 8. Build ROS2 workspace ──────────────────────────────────────────────────
echo ">> Building ROS2 workspace (this takes a while)..."
source /opt/ros/humble/setup.bash
cd "$REPO/ros2_ws"
colcon build --symlink-install

# ── 9. Update .bashrc ────────────────────────────────────────────────────────
echo ">> Updating .bashrc..."
add_to_bashrc "source /opt/ros/humble/setup.bash"
add_to_bashrc "source $REPO/ros2_ws/install/local_setup.bash"

echo ""
echo "========================================"
echo "  Setup complete!"
echo "  Run: source ~/.bashrc"
echo ""
echo "  To start the system (see docs/startup.md for details):"
echo "    sudo $REPO/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent serial --dev /dev/serial0 -b 921600"
echo "    ros2 run pies_servo servo_node"
echo "    ros2 run camera_ros camera_node"
echo "========================================"
