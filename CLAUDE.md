# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**pies-lipad** is an autonomous drone system built for the C-UASC competition (June 5–7, 2026). See [docs/competition.md](docs/competition.md) for full competition rules, missions, scoring, and build priorities. See [docs/startup.md](docs/startup.md) for the daily startup sequence.

Two major components:

1. **Micro-XRCE-DDS-Agent** — a C++11 DDS-XRCE broker (eProsima) that bridges a microcontroller running Micro XRCE-DDS Client to the full ROS2 DDS network.
2. **ros2_ws** — a ROS2 colcon workspace containing `pies_servo` (servo/gripper control) and `camera_ros` (OV5647 CSI camera, 640×480 at 17 FPS).

The full data path is:
```
Microcontroller (XRCE-DDS Client)
    → MicroXRCEAgent (UDP/serial bridge)
        → ROS2 DDS network
            → pies_servo node (/servo/angle)
                → lgpio PWM on GPIO 12
                    → Servo motor
```

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

- **`servo_driver.py`** — raw hardware: opens `/dev/gpiochip0` via `lgpio`, maps 0–270° to 500–2500 µs pulse width on GPIO pin 12.
- **`servo_node.py`** — ROS2 node `servo_node`; subscribes to `/servo/angle` (`std_msgs/Float32`) and calls into `user_main`.
- **`user_main.py`** — the intended customisation point. Instantiates `ServoDriver` on GPIO 12, moves the servo on each command, and auto-releases the PWM signal 0.5 s later (prevents jitter while holding position).

`user_main.py` is the file to edit when changing servo behaviour (pin, timing, motion profiles). `servo_driver.py` is pure hardware abstraction.

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
