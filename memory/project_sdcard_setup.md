---
name: project-sdcard-setup
description: Fresh SD card install verification checklist and recovery steps for incomplete installs
metadata:
  type: project
  originSessionId: 98654d98-cc2b-4d1c-9a47-e594bc788121
---

Fresh installs are done manually following [docs/startup.md — Fresh SD Card Setup](../docs/startup.md#fresh-sd-card-setup). The procedure is long and network-dependent; partial failures (apt error, network hiccup) leave the system in an unclear state.

**History:** setup.sh was removed 2026-05-27 — it was brittle in practice (failed partway on second SD card, 2026-05-20). Steps are now documented manually in startup.md.

**Why:** Long sequential installs can fail silently mid-way. Always verify key outputs before rebooting.

**How to apply:** After running the fresh install steps, verify the checklist below before rebooting. If anything is missing, run the recovery commands.

## Verification checklist after install

```bash
# ROS2 installed?
ls /opt/ros/humble/

# px4_msgs and px4_ros_com cloned?
ls ~/pies-lipad/ros2_ws/src/

# ros2_ws built?
ls ~/pies-lipad/ros2_ws/install/

# .bashrc has source lines?
grep -c 'ros/humble\|local_setup' ~/.bashrc   # should print 2

# pigpiod installed and enabled?
systemctl is-enabled pigpiod   # should print "enabled"

# Claude memory symlinked (not a real dir)?
ls -la ~/.claude/projects/-home-lipad-pies-lipad/memory
```

## Recovery if install failed partway

```bash
# 1. Clone missing PX4 packages
cd ~/pies-lipad/ros2_ws/src
[ -d px4_msgs ]  || git clone https://github.com/PX4/px4_msgs.git
[ -d px4_ros_com ] || git clone https://github.com/PX4/px4_ros_com.git

# 2. Pin Python versions (colcon build fails otherwise)
pip3 install --user "setuptools==59.6.0" "numpy<2"

# 3. Build the workspace (~33 min on RPi 4)
source /opt/ros/humble/setup.bash
cd ~/pies-lipad/ros2_ws
colcon build --symlink-install

# 4. Add .bashrc source lines if missing
grep -qF 'ros/humble' ~/.bashrc || echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
grep -qF 'local_setup' ~/.bashrc || echo 'source ~/pies-lipad/ros2_ws/install/local_setup.bash 2>/dev/null || true' >> ~/.bashrc

# 5. Fix Claude memory symlink if it's a real directory
MDIR=~/.claude/projects/-home-lipad-pies-lipad/memory
[ -d "$MDIR" ] && rmdir "$MDIR" && ln -s ~/pies-lipad/memory "$MDIR"

# 6. Install and enable pigpiod (servo driver dependency)
sudo apt install -y pigpio
sudo systemctl enable --now pigpiod

# 7. rosdep init (needs interactive terminal — run manually)
sudo rosdep init && rosdep update
```

## Python version pinning rationale

- `setuptools==59.6.0` — ROS2 ament_python packages fail to build with setuptools ≥60 (changed `find_packages` behaviour).
- `numpy<2` — Several ROS2/OpenCV bindings were compiled against numpy 1.x ABI; numpy 2.x breaks them at import time.
