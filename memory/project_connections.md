---
name: project-connections
description: RPi SSH connection methods — WiFi, Ethernet, and hotspot addresses, autoconnect behaviour, and field lessons
metadata:
  node_type: memory
  type: project
  originSessionId: 755c71f3-6f41-4146-863e-325b7b333604
---

All three connection methods confirmed working. Hotspot updated to `autoconnect yes` on 2026-05-18.

**Why:** Each method serves a different scenario; easy to forget addresses and constraints under field pressure.

**How to apply:** Use Ethernet at home for development. After any field session, bring hotspot down at home or RPi loses internet. Always bring Ethernet cable to the field as backup.

## Methods

| Method | Address | When |
|---|---|---|
| Home WiFi | `ssh lipad@rpi` | Home development |
| Ethernet cable | `ssh lipad@10.42.0.2` | Home dev / field backup |
| RPi hotspot | `ssh lipad@172.16.0.1` | Field (no WiFi infrastructure) |

## Hotspot details

- SSID: `piesdrone` / password: `piesdrone123`
- **autoconnect yes** — starts automatically on boot at the field; no manual step needed
- Stop: `sudo nmcli con down pies-hotspot` — run this at home after every field session
- RPi hotspot IP: `172.16.0.1/24` — pinned explicitly to avoid subnet conflicts
- **Single radio limitation:** wlan0 can't be client + AP simultaneously. Hotspot drops RPi internet. Fine at field, never leave active at home.
- **Laptop auto-connect warning:** Windows saves piesdrone and auto-joins it next time hotspot starts. Forget the network after testing to prevent accidental internet loss.
- **Field lesson (2026-05-18):** Hotspot failed at field because RPi rebooted and `autoconnect no` meant it didn't restart. Fix: changed to `autoconnect yes`. Command: `sudo nmcli con modify pies-hotspot connection.autoconnect yes`
- **If hotspot fails at field:** plug in Ethernet cable → `ssh lipad@10.42.0.2` → `sudo nmcli con up pies-hotspot`

## Subnet layout (no conflicts)

- `10.42.0.x` — Ethernet cable (RPi=.2, laptop=.1)
- `172.16.0.x` — RPi hotspot (RPi=.1, clients get DHCP)
- `192.168.1.x` — home router (typical)

## Indoor arming note

Position mode requires GPS lock — won't arm indoors. Use **Stabilized** mode for indoor motor tests (no GPS needed).

## Camera CSI ribbon cable

OV5647 ribbon cable must be fully seated in both the RPi CAM port and camera module connector. Confirmed issue 2026-05-18: cable was loose → "no cameras available" from libcamera. After reseating, camera came up immediately. Diagnostic: `dmesg | grep -i 'ov5647\|unicam'` — should show driver messages at boot.

## SD card history

- **2026-05-20:** Fresh SD card set up. `setup.sh` failed partway through; user manually followed PX4 guide. `px4_msgs` and `px4_ros_com` had to be cloned by hand. `numpy` downgraded to <2, `setuptools` pinned to 59.6.0. `ros2_ws` built (~33 min). `.bashrc` updated with ROS2 source lines. `rosdep init` still pending (needs interactive terminal). See [[project-sdcard-setup]] for recovery steps.

## One-time setup (already done 2026-05-16)

Setup was run safely over Ethernet SSH (not keyboard/monitor):
- NetworkManager installed, cloud-init network disabled, netplan renderer switched to NM
- Hotspot connection created with `ipv4.addresses 172.16.0.1/24` to pin the subnet
- Updated to `autoconnect yes` on 2026-05-18
- Key: must use `sudo nmcli` not `nmcli` — insufficient privileges otherwise
