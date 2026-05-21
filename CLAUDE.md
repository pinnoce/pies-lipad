# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**pies-lipad** is an autonomous drone system built for the C-UASC competition (June 5–7, 2026). See [docs/competition.md](docs/competition.md) for full competition rules, missions, scoring, and build priorities. See [docs/startup.md](docs/startup.md) for the daily startup sequence.

Two major components:

1. **Micro-XRCE-DDS-Agent** — a C++11 DDS-XRCE broker (eProsima) that bridges the Pixhawk (running Micro XRCE-DDS Client) to the ROS2 DDS network.
2. **ros2_ws** — a ROS2 colcon workspace. Authored packages in `src/`:
   - `pies_servo` — servo/gripper control via pigpio (pigpiod daemon) on GPIO 12
   - `pies_vision` — computer vision nodes (bucket_detector, green_x_detector, etc.)
   - `pies_mission` — full autonomous mission stack (see below)
   - `px4_msgs` / `px4_ros_com` — PX4 ROS2 message definitions, cloned by `setup.sh`

   `camera_ros` is installed as a system package (`ros-humble-camera-ros`), not in `src/`. OV5647 CSI camera, 800×600 at ~17 FPS.

The full data path for autonomous package recovery:
```
Pixhawk (XRCE-DDS Client, TELEM2 921600 baud)
    → MicroXRCEAgent (serial bridge, /dev/serial0)
        → ROS2 DDS network
            ├── camera_ros       → /camera/image_raw
            │       └── bucket_detector (/vision/bucket JSON)
            │               └── autonomous_mission or visual_centering
            │                       → /fmu/in/trajectory_setpoint  (OFFBOARD velocity)
            │                       → /fmu/in/offboard_control_mode
            ├── pies_servo       ← /servo/angle (Float32, degrees)
            └── /fmu/out/vehicle_local_position  (NED pos + heading)
```

## RPi Connection Methods

| Method | Address | When to use |
|---|---|---|
| Home WiFi | `ssh lipad@rpi` | General development |
| Ethernet cable | `ssh lipad@10.42.0.2` | Safe for hotspot setup; no internet dependency |
| RPi hotspot | `ssh lipad@172.16.0.1` | At the field (no WiFi infrastructure) |

Hotspot SSID: `piesdrone` / password: `piesdrone123`. Set to `autoconnect yes` — starts automatically on boot at the field, no manual step needed. At home after field use: `sudo nmcli con down pies-hotspot` to restore internet (RPi has one WiFi radio; hotspot drops internet). Always bring Ethernet cable to the field as backup — if hotspot fails, `ssh lipad@10.42.0.2` and start hotspot manually.

**Indoor arming:** Position mode requires GPS lock. Use **Stabilized** mode for indoor motor tests — no GPS needed.

## Fresh SD Card Setup

Flash **Ubuntu Server 22.04 LTS (64-bit)** with Raspberry Pi Imager. Click the gear icon ⚙ and set: hostname `rpi`, username `lipad`, WiFi credentials, enable SSH. Then:

```bash
ssh lipad@rpi
git clone https://github.com/pinnoce/pies-lipad.git ~/pies-lipad
cd ~/pies-lipad
bash setup.sh   # ~30–45 min — run as lipad, NOT sudo
sudo rosdep init && rosdep update   # rosdep init needs a real terminal; run once
sudo reboot
```

`setup.sh` handles everything: ROS2, all packages, camera config, UART config, ModemManager removal, serial getty disable, NetworkManager, hotspot (`piesdrone`), Ethernet static IP (`10.42.0.2`), clones `px4_msgs`/`px4_ros_com`, builds, `.bashrc`, and Claude memory symlink.

**If setup.sh fails partway:** the most likely gap is `px4_msgs` and `px4_ros_com` not being cloned. Check `ls ros2_ws/src/` — if they're missing, clone manually:
```bash
cd ~/pies-lipad/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
cd ~/pies-lipad/ros2_ws
source /opt/ros/humble/setup.bash && colcon build --symlink-install
```
Also verify `.bashrc` has the two source lines (added by setup.sh):
```
source /opt/ros/humble/setup.bash
source ~/pies-lipad/ros2_ws/install/local_setup.bash
```

## Serial Port (`/dev/serial0`)

Pixhawk connects via TELEM2 → `/dev/serial0` (= `/dev/ttyAMA0`) at 921600 baud. Two things that silently break this if present:

- **ModemManager** — probes serial ports on attach, sends AT commands to the Pixhawk. `setup.sh` removes it.
- **serial-getty** — Ubuntu may run a login shell on `ttyAMA0`. `setup.sh` disables `serial-getty@ttyAMA0` and `serial-getty@ttyS0`.

If the DDS agent connects but no `/fmu/` topics appear, check: `systemctl status serial-getty@ttyAMA0` and `which ModemManager`.

---

## ROS2 Workspace (pies_servo)

### Build & Run

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 run pies_servo servo_node
```

### Send a command manually

```bash
ros2 topic pub /servo/angle std_msgs/msg/Float32 "data: 90.0"
```

### Lint / test

```bash
cd ros2_ws
colcon test
colcon test-result --verbose
```

### Architecture

`pies_servo` has three layers:

- **`servo_driver.py`** — raw hardware: connects to the `pigpiod` daemon via `pigpio`, maps 0–270° to 500–2500 µs pulse width on GPIO pin 12. Requires `pigpiod` running (`sudo systemctl enable --now pigpiod`).
- **`servo_node.py`** — ROS2 node `servo_node`; subscribes to `/servo/angle` (`std_msgs/Float32`) and calls into `user_main`.
- **`user_main.py`** — the intended customisation point. Instantiates `ServoDriver` on GPIO 12, moves the servo on each command, and auto-releases the PWM signal after a scaled delay (0.2–0.8 s based on move distance) to prevent jitter while holding position.

`user_main.py` is the file to edit when changing servo behaviour (pin, timing, motion profiles). `servo_driver.py` is pure hardware abstraction.

---

## ROS2 Workspace (pies_mission)

The main mission package. All parameters live in one file — edit before each flight, no rebuild needed (symlink-install).

### Key files

Paths relative to `ros2_ws/src/pies_mission/` unless noted otherwise.

| File | Purpose |
|------|---------|
| `pies_mission/mission_config.py` | Single source of truth for all parameters |
| `pies_mission/autonomous_mission.py` | Full state machine: IDLE→ARMING→TAKEOFF→FLY_PICKUP→VISUAL→CLIMB→FLY_DROP→DROP→RTL→DONE |
| `pies_mission/visual_centering.py` | Standalone OFFBOARD centering node (manual takeoff, then hand off) |
| `pies_mission/calibrate_gains.py` | Sets KP_X/KP_Y signs via live nudge test — run once after mounting |
| `launch/autonomous.launch.py` | Launches full autonomous stack |
| `launch/mission.launch.py` | Launches visual-centering-only stack |
| `tools/sim_mission.py` *(repo root)* | Simulates the full mission state machine without hardware |
| `tools/sim_calibrate.py` *(repo root)* | Validates calibration sign logic for all camera mount angles |
| `tools/sim_centering.py` *(repo root)* | Simulates visual centering descent loop only |
| `tools/test_statustext.py` *(repo root)* | pymavlink UDP server that replays mission STATUSTEXT in QGC — use to preview Messages panel before flying |

### Run autonomous mission

```bash
ros2 launch pies_mission autonomous.launch.py
# then trigger:
ros2 topic pub --once /mission/go std_msgs/msg/Empty "{}"
```

### Run visual centering only (manual-assist mode)

```bash
ros2 launch pies_mission mission.launch.py
# arm + fly to bucket manually, then switch to Offboard mode in QGC
```

### Calibrate gain signs (once after mounting camera)

```bash
ros2 run pies_mission calibrate_gains
# hover in Offboard with bucket visible; follow prompts; writes KP_X/KP_Y to mission_config.py
```

### Key parameters (mission_config.py)

| Parameter | Default | Notes |
|-----------|---------|-------|
| `CAM_ROT_DEG` | `0` | 0 = nose→image top; use 90/180/270 for other mounts |
| `KP_X`, `KP_Y` | `0.004` | Set by `calibrate_gains`; signs depend on mount |
| `GRAB_ALT` | `0.2` m | AGL altitude where gripper fires |
| `DROP_ALT` | `0.2` m | AGL altitude for bucket release |
| `CRUISE_ALT` | `5.0` m | Navigation altitude |
| `BLIND_GRAB_S` | `4.0` s | Grab window after camera loses bucket overhead |
| `PICKUP_LAT/LON` | `0.0` | **Set day-of** from QGroundControl |
| `DROP_LAT/LON` | `0.0` | **Set day-of** from QGroundControl |

### Camera / control law

Pixel error is rotated to drone body-frame axes with `phi = 90° − CAM_ROT_DEG`, then velocity
is rotated to NED using the live heading from `VehicleLocalPosition`. This means the controller
works regardless of which direction the drone is facing.

### QGC monitoring via SiK telemetry

`autonomous_mission.py` publishes each state transition as a MAVLink STATUSTEXT via
`/fmu/in/log_message` (`px4_msgs/LogMessage`). Messages appear in QGC's Messages panel over
the SiK 915 MHz radio at full range — no SSH needed during flight.

`LogMessage.text` is a fixed `uint8[128]` array — always pad to exactly 128 bytes.

To preview the full sequence in QGC without hardware:
```bash
python3 tools/test_statustext.py
# QGC: Application Settings → Comm Links → Add → UDP, port 14550, RPi IP
```

### Servo direction

Confirmed 2026-05-16: **0° → CCW → closes gripper; 270° → CW → opens gripper.**
`GRAB_ANGLE = 30` and `RELEASE_ANGLE = 240` are in the correct directions.
Final values need tuning once the rack-and-pinion mechanism is physically attached — sweep in
10° steps from each limit and set `GRAB_ANGLE` = closed stop + 10°, `RELEASE_ANGLE` = open stop − 10°.

### Simulation

```bash
python3 tools/sim_mission.py           # full mission, default waypoints
python3 tools/sim_calibrate.py         # calibration sign check for all 8 mount angles
```

Expected: `sim_calibrate` shows 0°/90°/180°/270° converging in ~2.7 s; diagonal mounts diverge with a warning. `sim_mission` completes in ~66 s.

---

## Micro-XRCE-DDS-Agent

### Build Commands

The project uses a superbuild CMake pattern that downloads and compiles all dependencies (Fast-CDR, Fast-DDS, foonathan_memory, spdlog, Micro XRCE-DDS Client) as ExternalProjects into `build/temp_install/`. The source is cloned by `setup.sh`; `build/` is gitignored.

**First-time build:**
```bash
cd Micro-XRCE-DDS-Agent
mkdir -p build && cd build
cmake ..
cmake --build . --target MicroXRCEAgent
```

**Rebuild after source changes (deps already built):**
```bash
cd Micro-XRCE-DDS-Agent/build
cmake --build . --target MicroXRCEAgent
```

**Build with tests:**
```bash
cmake .. -DUAGENT_BUILD_TESTS=ON
cmake --build .
ctest
ctest -R <test-name>   # single test
```

### Running the Agent

```bash
./build/MicroXRCEAgent udp4 -p 8888
./build/MicroXRCEAgent serial --dev /dev/ttyUSB0
./build/MicroXRCEAgent canfd --dev can0
```

### Key CMake Options

| Option | Default | Purpose |
|--------|---------|---------|
| `UAGENT_SUPERBUILD` | ON | Fetch & build all deps via ExternalProject |
| `UAGENT_BUILD_TESTS` | OFF | Build unit tests (GTest/GMock) |
| `UAGENT_FAST_PROFILE` | ON | Enable Fast-DDS middleware |
| `UAGENT_CED_PROFILE` | ON | Enable built-in CED middleware |
| `UAGENT_P2P_PROFILE` | ON | Enable P2P agent discovery (requires CED) |
| `UAGENT_DISCOVERY_PROFILE` | ON | Enable UDP multicast discovery |
| `UAGENT_SOCKETCAN_PROFILE` | ON | Enable CAN FD transport (Linux only) |
| `UAGENT_SECURITY_PROFILE` | OFF | Enable DDS Security |
| `UAGENT_BUILD_USAGE_EXAMPLES` | OFF | Build `examples/custom_agent` |

### Architecture

```
Transport (UDP/TCP/Serial/CAN/Custom)
    └── Server<EndPoint>           (src/cpp/transport/Server.cpp)
          └── Processor<EndPoint>  (src/cpp/processor/Processor.cpp)
                ├── Root           (src/cpp/Root.cpp)        — registry of ProxyClients
                └── ProxyClient    (src/cpp/client/)         — one per connected XRCE client
                      └── Middleware (abstract interface)    — DDS operations
                            ├── FastDDSMiddleware
                            └── CedMiddleware
```

**Key classes:**
- **`Root`** (`include/uxr/agent/Root.hpp`): registry keyed on `ClientKey`; creates/destroys `ProxyClient` instances.
- **`ProxyClient`** (`include/uxr/agent/client/ProxyClient.hpp`): one per connected XRCE client; owns the client's DDS entity tree.
- **`Processor<EndPoint>`** (`include/uxr/agent/processor/Processor.hpp`): decodes XRCE submessages, dispatches create/delete/write/read to `Root`/`ProxyClient`.
- **`Middleware`** (`include/uxr/agent/middleware/Middleware.hpp`): pure-virtual interface; implemented by `FastDDSMiddleware` and `CedMiddleware`.
- **`AgentInstance`** (`include/uxr/agent/AgentInstance.hpp`): singleton entry point; parses CLI args, selects transport+middleware, runs the agent loop.
- **`CustomAgent`** (`include/uxr/agent/transport/custom/CustomAgent.hpp`): user-defined transport via four lambda callbacks (init, fini, send, recv).

**XRCE object hierarchy within a ProxyClient:**
`Participant → Topic → Publisher/Subscriber → DataWriter/DataReader`
and independently: `Participant → Requester / Replier`

Each entity is created from a **ref** (looked up in `agent.refs` XML), inline **XML**, or **binary**.

Platform-specific transport files are selected by CMake (`*Linux.cpp` vs `*Windows.cpp`). All code is in namespace `eprosima::uxr`.

### Custom Transport Development

See `examples/custom_agent/custom_agent.cpp`. Implement four `std::function` callbacks and instantiate `eprosima::uxr::CustomAgent`. Define a `CustomEndPoint` with named typed members for your addressing scheme.
