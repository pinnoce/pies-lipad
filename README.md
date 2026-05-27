# pies-lipad

Autonomous drone system for the [C-UASC competition](docs/competition.md) (June 5–7, 2026, Mojave Air & Space Port).

Built on a Raspberry Pi 4 companion computer running Ubuntu 22.04, connected to a Pixhawk flight controller via serial.

## Hardware

| Component | Details |
|---|---|
| Flight controller | Pixhawk — TELEM2 serial at 921600 baud |
| Companion computer | Raspberry Pi 4, Ubuntu 22.04 |
| Camera | Sunny P5V04A (OV5647, 5MP CSI) |
| Gripper | Rack-and-pinion servo, GPIO 12, 270° range — 0°=CCW=close, 270°=CW=open |

## Fresh Setup (new SD card)

**1. Flash the SD card** using [Raspberry Pi Imager](https://www.raspberrypi.com/software/):
- OS: **Ubuntu Server 22.04 LTS (64-bit)**
- Click the **gear icon ⚙** before writing and set:
  - Hostname: `rpi`
  - Username: `lipad` / password: your choice
  - WiFi: home SSID and password
  - Enable SSH: yes, password authentication

**2. First boot** — insert SD card, power on, wait ~60 s, then SSH in:
```bash
ssh lipad@rpi
```

**3. Clone the repo and follow the setup steps** in [docs/startup.md — Fresh SD Card Setup](docs/startup.md#fresh-sd-card-setup) (~30–45 min):
```bash
git clone https://github.com/pinnoce/pies-lipad.git ~/pies-lipad
```

Covers ROS2 Humble, all dependencies, boot config, NetworkManager, hotspot, Ethernet IP, building everything, `.bashrc`, and Claude Code memory.

**4. Reboot:**
```bash
sudo reboot
```
> Required for `dialout`/`video` group memberships and the Bluetooth-disable overlay to take effect.

## Connecting to the RPi

| Method | When | Command |
|---|---|---|
| Home WiFi | Development at home | `ssh lipad@rpi` |
| Ethernet cable | Development / hotspot setup | `ssh lipad@10.42.0.2` |
| RPi hotspot | At the field (no WiFi infrastructure) | `ssh lipad@172.16.0.1` |

**Hotspot:** starts automatically on boot at the field (`autoconnect yes`). Laptop joins `piesdrone` (password: `piesdrone123`) → `ssh lipad@172.16.0.1`. At home after field use, run `sudo nmcli con down pies-hotspot` to restore internet. Always bring the Ethernet cable as a field backup. See [docs/startup.md](docs/startup.md) for full setup.

## Daily Startup

See [docs/startup.md](docs/startup.md) for the full sequence. Short version:

```bash
# Terminal 1 — bridge Pixhawk to ROS2
sudo ~/pies-lipad/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent serial --dev /dev/serial0 -b 921600

# Terminal 2 — full autonomous mission (edit mission_config.py first)
ros2 launch pies_mission autonomous.launch.py

# Trigger the mission when ready
ros2 topic pub --once /mission/go std_msgs/msg/Empty "{}"
```

## What's Working

| Component | Status |
|---|---|
| Micro-XRCE-DDS-Agent (Pixhawk bridge) | ✅ |
| ROS2 Humble + PX4 topics | ✅ |
| Servo/gripper (`/servo/angle`, 0–270°) | ✅ |
| Camera (`/camera/image_raw`, ~16 FPS, 800×600) | ✅ |
| Geofence (QGroundControl + PX4 native RTL) | ✅ |
| Computer vision (bucket, green X, markers) | ✅ |
| Package recovery — visual centering + grab | ✅ |
| Package recovery — full autonomous mission | ✅ |
| QGC state monitoring via SiK telemetry | ✅ |

## Repo Structure

```
docs/
  competition.md                # C-UASC rules, missions, scoring
  startup.md                    # Daily startup + calibration sequence
ros2_ws/src/
  pies_servo/                   # Servo/gripper ROS2 package
    pies_servo/
      user_main.py              # ← edit to change servo behaviour
      servo_driver.py           # pigpio hardware abstraction
      servo_node.py             # ROS2 boilerplate
  pies_vision/                  # Computer vision nodes
  pies_mission/                 # Autonomous mission stack
    pies_mission/
      mission_config.py         # ← all flight parameters (edit before each flight)
      autonomous_mission.py     # Full state machine: takeoff → pickup → drop → RTL
      visual_centering.py       # Standalone OFFBOARD centering node
      calibrate_gains.py        # Sets KP_X/KP_Y signs via live nudge test
    launch/
      autonomous.launch.py      # Full autonomous stack
      mission.launch.py         # Visual-centering-only stack
tools/
  sim_mission.py                # Simulate full mission without hardware
  sim_calibrate.py              # Validate calibration for all camera mount angles
  sim_centering.py              # Simulate visual centering descent loop only
  test_statustext.py            # Preview mission STATUSTEXT in QGC via pymavlink UDP
  capture_frame.py              # Save one camera frame to captured_frame.jpg
```

Third-party repos (`Micro-XRCE-DDS-Agent`, `px4_msgs`, `px4_ros_com`) are gitignored and cloned during fresh install (step 8 in [docs/startup.md](docs/startup.md#fresh-sd-card-setup)).

## Key Commands

```bash
# Run full autonomous mission
ros2 launch pies_mission autonomous.launch.py
ros2 topic pub --once /mission/go std_msgs/msg/Empty "{}"

# Calibrate gain signs after mounting camera (run once)
ros2 run pies_mission calibrate_gains

# Simulate mission before flying
python3 tools/sim_mission.py

# Preview QGC mission messages via SiK telemetry (no hardware needed)
python3 tools/test_statustext.py

# Test servo (degrees, 0–270)
ros2 topic pub --once /servo/angle std_msgs/msg/Float32 "data: 90.0"

# Check camera framerate
ros2 topic hz /camera/image_raw

# Rebuild after code changes
cd ros2_ws && colcon build --symlink-install
```
