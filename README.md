# pies-lipad

Autonomous drone system for the [C-UASC competition](docs/competition.md) (June 5–7, 2026, Mojave Air & Space Port).

Built on a Raspberry Pi 4 companion computer running Ubuntu 22.04, connected to a Pixhawk flight controller via serial.

## Hardware

| Component | Details |
|---|---|
| Flight controller | Pixhawk — TELEM2 serial at 921600 baud |
| Companion computer | Raspberry Pi 4, Ubuntu 22.04 |
| Camera | Sunny P5V04A (OV5647, 5MP CSI) |
| Gripper | Rack-and-pinion servo, GPIO 12, 270° range |

## Fresh Setup (new SD card)

```bash
git clone https://github.com/pinnoce/pies-lipad.git ~/pies-lipad
cd ~/pies-lipad
bash setup.sh
```

`setup.sh` installs ROS2 Humble, all dependencies, clones and builds Micro-XRCE-DDS-Agent and the PX4 ROS2 packages, builds the ROS2 workspace, and configures `.bashrc`. Takes ~30–45 minutes on first run.

> **After setup:** reboot before first use (dialout group membership needs a fresh login).

## Daily Startup

See [docs/startup.md](docs/startup.md) for the full sequence. Short version:

```bash
# Terminal 1 — bridge Pixhawk to ROS2
sudo ~/pies-lipad/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent serial --dev /dev/serial0 -b 921600

# Terminal 2 — servo/gripper
ros2 run pies_servo servo_node

# Terminal 3 — camera
ros2 run camera_ros camera_node
```

## What's Working

| Component | Status |
|---|---|
| Micro-XRCE-DDS-Agent (Pixhawk bridge) | ✅ |
| ROS2 Humble + PX4 topics | ✅ |
| Servo/gripper (`/servo/angle`, 0–270°) | ✅ |
| Camera (`/camera/image_raw`, ~16 FPS, 800×600) | ✅ |
| Geofence (QGroundControl + PX4 native RTL) | ✅ |
| Mission sequencer | ❌ |
| Computer vision (green X + marker detection) | ❌ |
| Package recovery sequence | ❌ |

## Repo Structure

```
setup.sh                  # One-shot bootstrap for a fresh RPi
docs/
  competition.md          # C-UASC rules, missions, scoring, build order
  startup.md              # Daily startup sequence
ros2_ws/src/
  pies_servo/             # Servo/gripper ROS2 package
    pies_servo/
      user_main.py        # ← edit this to change servo behaviour
      servo_driver.py     # lgpio hardware abstraction (don't edit)
      servo_node.py       # ROS2 boilerplate (don't edit)
tools/
  capture_frame.py        # Save one camera frame to captured_frame.jpg
```

Third-party repos (`Micro-XRCE-DDS-Agent`, `px4_msgs`, `px4_ros_com`) are gitignored and cloned automatically by `setup.sh`.

## Key Commands

```bash
# Test servo (degrees, 0–270)
ros2 topic pub --once /servo/angle std_msgs/msg/Float32 "data: 90.0"

# Check camera framerate
ros2 topic hz /camera/image_raw

# Capture a test image (saves to captured_frame.jpg)
python3 tools/capture_frame.py

# Rebuild after code changes
cd ros2_ws && colcon build --symlink-install
```
