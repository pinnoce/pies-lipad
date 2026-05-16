# Startup Guide

Run these commands each time the Raspberry Pi boots.

## 1. Source the workspace

```bash
source ~/.bashrc
```

## 2. Start the Micro-XRCE-DDS-Agent (bridges Pixhawk → ROS2)

Pixhawk must be powered and connected via serial before running this.

```bash
sudo ~/pies-lipad/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent serial --dev /dev/serial0 -b 921600
```

Leave this terminal open. Open a new terminal for everything below.

## 3. Start the servo node

```bash
ros2 run pies_servo servo_node
```

## 4. Start the camera node

```bash
ros2 run camera_ros camera_node
```

## 5. Pre-flight calibration (do once after mounting, not every boot)

Do these steps in order — each one builds on the previous.

### 5a. Servo angles

**Direction confirmed (2026-05-16):** 0° → CCW → closes gripper; 270° → CW → opens gripper.

**Still needed — rack-and-pinion mechanism not yet attached.** Once mounted, with the servo node running (step 3):

**Find the closed (grab) limit** — sweep down in 10° steps from the default:
```bash
ros2 topic pub --once /servo/angle std_msgs/msg/Float32 "data: 30.0"
ros2 topic pub --once /servo/angle std_msgs/msg/Float32 "data: 20.0"
ros2 topic pub --once /servo/angle std_msgs/msg/Float32 "data: 10.0"
ros2 topic pub --once /servo/angle std_msgs/msg/Float32 "data: 0.0"
```
Stop at the first sign of grinding or resistance. Note the last safe angle — this is the closed limit.

**Find the open (release) limit** — sweep up in 10° steps:
```bash
ros2 topic pub --once /servo/angle std_msgs/msg/Float32 "data: 240.0"
ros2 topic pub --once /servo/angle std_msgs/msg/Float32 "data: 250.0"
ros2 topic pub --once /servo/angle std_msgs/msg/Float32 "data: 260.0"
ros2 topic pub --once /servo/angle std_msgs/msg/Float32 "data: 270.0"
```
Stop at the first sign of grinding or resistance. Note the last safe angle — this is the open limit.

**Update mission_config.py:**
```bash
nano ~/pies-lipad/ros2_ws/src/pies_mission/pies_mission/mission_config.py
# GRAB_ANGLE    = <closed limit> + 10
# RELEASE_ANGLE = <open limit>   - 10
```

---

### 5b. Camera offset

**What it is:** if the camera is not mounted exactly at the drone's center of mass, the bucket will appear offset from image-center even when the drone is directly above it. `CAM_OFFSET_X/Y_PX` corrects for this.

**Prerequisites:** DDS agent (step 2), camera node (step 4), and bucket_detector running.

Start bucket_detector alone if the full stack isn't up:
```bash
ros2 run pies_vision bucket_detector
```

**Steps:**
1. Arm and take off in **Position** mode
2. Fly to hover directly above the bucket — use GPS position hold to stay still
3. In another terminal, watch the detector:
```bash
ros2 topic echo /vision/bucket
```
4. Read the `offset_x` and `offset_y` fields (pixels from image centre)
5. Update `mission_config.py`:
```
CAM_OFFSET_X_PX = <offset_x value>
CAM_OFFSET_Y_PX = <offset_y value>
```

---

### 5c. Gain sign calibration

**What it is:** determines the correct sign (±) for `KP_X` and `KP_Y` based on camera mount orientation. Wrong signs cause the drone to fly *away* from the bucket instead of toward it. Run once per camera mount.

**Prerequisites:** DDS agent (step 2), camera node (step 4), bucket_detector running, bucket visible below.

**Steps:**
1. Arm and take off in **Position** mode
2. Fly to ~3 m above the bucket — bucket must be clearly visible in the camera feed
3. In another terminal, run:
```bash
ros2 run pies_mission calibrate_gains
```
4. Follow the two prompts:
   - **Nudge 1 (forward):** script briefly commands forward movement, measures bucket pixel shift
   - **Nudge 2 (right):** script briefly commands rightward movement, measures bucket pixel shift
5. Script writes `KP_X` and `KP_Y` (with correct signs) directly to `mission_config.py`

**Expected result** with `CAM_ROT_DEG = 0` (nose → image top): `KP_X ≈ −0.004`, `KP_Y ≈ +0.004`

> During nudges the drone briefly enters Offboard mode — stay ready to take manual control.

---

### 5d. GPS coordinates (day-of)

In QGroundControl, right-click the pickup location on the map → **Copy coordinates**. Repeat for the drop zone. Then:

```bash
nano ~/pies-lipad/ros2_ws/src/pies_mission/pies_mission/mission_config.py
# Set:
#   PICKUP_LAT = <lat>
#   PICKUP_LON = <lon>
#   DROP_LAT   = <lat>
#   DROP_LON   = <lon>
```
No rebuild needed (symlink-install).

---

## 6. Run the Package Recovery mission

**First, set the GPS coords** (day-of — get from QGroundControl, right-click → *Copy coordinates*):

```bash
nano ~/pies-lipad/ros2_ws/src/pies_mission/pies_mission/mission_config.py
```

Set `PICKUP_LAT`, `PICKUP_LON`, `DROP_LAT`, `DROP_LON`. Save and close. No rebuild needed.

---

### Option A — Fully autonomous

Launch everything (servo + camera + bucket detector + autonomous mission):

```bash
ros2 launch pies_mission autonomous.launch.py
```

Place the drone at the takeoff point, then trigger the mission:

```bash
ros2 topic pub --once /mission/go std_msgs/msg/Empty "{}"
```

The drone arms, takes off, flies to the pickup GPS coords, centres visually, grabs, flies to the
drop zone, releases, and returns home — all autonomously. Watch `ros2 topic echo /mission/status`
for state updates: `ARMING → TAKEOFF → FLY_PICKUP → VISUAL → CLIMB → FLY_DROP → DROP → RTL → DONE`.

**QGC state monitoring via SiK telemetry** — each state transition is also published as a MAVLink
STATUSTEXT message over the SiK 915 MHz radio. Open QGC → Messages panel (bell icon, top-right)
to watch the mission progress at full radio range without SSH.

To preview the full message sequence in QGC before flying:
```bash
python3 tools/test_statustext.py
# Then in QGC: Application Settings → Comm Links → Add → UDP, port 14550, RPi IP
```

> To abort at any time: flip RC failsafe or take manual control. To re-open the gripper before
> an unplanned landing: `ros2 topic pub --once /gripper/open std_msgs/msg/Empty "{}"`

---

### Option B — Manual assist (fly to bucket, then hand off)

Launch servo + camera + bucket detector + visual centering node only:

```bash
ros2 launch pies_mission mission.launch.py
```

**In QGroundControl:**
1. Arm the drone
2. Take off and fly toward the bucket area manually
3. Switch flight mode to **Offboard** — the centering node takes over horizontal position
4. Watch logs: `SEARCHING → CENTERING → CENTERED → GRABBED`
5. After grab, switch back to **Position** or **Mission** mode for the return flight

> The gripper fires automatically. After `GRABBED` the node sends zero velocity (hover).
> Switch out of Offboard as soon as you see `GRABBED` in the logs.

---

## Verify everything is working

```bash
# Should show /fmu/*, /servo/angle, /camera/image_raw, etc.
ros2 topic list

# Check camera is publishing at ~17 FPS
ros2 topic hz /camera/image_raw

# Test the servo manually (0–270°)
ros2 topic pub --once /servo/angle std_msgs/msg/Float32 "data: 90.0"
```

## Notes

- Each command needs its own terminal (or use `tmux` / `screen`).
- The agent must be running before PX4 topics appear in `ros2 topic list`.
- The Pixhawk serial port is `/dev/serial0` at 921600 baud (TELEM2).
- Servo is on GPIO 12, range 0–270°.
- If the servo node fails with **permission denied**, the user isn't in the `dialout` group yet — run `sudo usermod -aG dialout $USER` and reboot.

---

## Field Connection (no WiFi infrastructure)

### Option A — RPi WiFi Hotspot ✅ confirmed working

**One-time setup** (safe to run over Ethernet SSH — do this once, already done):

```bash
sudo apt install -y network-manager
sudo bash -c 'echo "network: {config: disabled}" > /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg'
sudo bash -c 'printf "network:\n  version: 2\n  renderer: NetworkManager\n" > /etc/netplan/99-nm.yaml'
sudo chmod 600 /etc/netplan/99-nm.yaml
sudo netplan apply
sudo nmcli con add type wifi ifname wlan0 con-name pies-hotspot ssid "piesdrone" mode ap \
  ipv4.method shared ipv4.addresses 172.16.0.1/24 \
  wifi-sec.key-mgmt wpa-psk wifi-sec.psk "piesdrone123" autoconnect no
```

> **Subnet note:** `172.16.0.x` is used deliberately — `10.42.0.x` is the Ethernet cable subnet
> and `192.168.1.x` is typically the home router. Keeping all three separate avoids routing conflicts.

**At the field — start the hotspot:**

```bash
sudo nmcli con up pies-hotspot
```

> **Important:** The RPi has one WiFi radio. When the hotspot starts, `wlan0` switches from
> client mode (home WiFi = internet) to AP mode (broadcasting piesdrone). The RPi loses internet
> while the hotspot is active — this is expected and fine at the field.
> Do **not** start the hotspot at home if you need internet on the RPi (e.g. Claude Code).

On your laptop — join WiFi `piesdrone` (password: `piesdrone123`), then:

```bash
ssh lipad@172.16.0.1
```

> **Auto-connect warning:** If your laptop has previously connected to `piesdrone`, Windows may
> auto-join it next time the hotspot starts, cutting your internet. Right-click `piesdrone` in
> the WiFi list → **Forget** to prevent this. Re-join manually at the field when needed.

**Stop the hotspot** (RPi reconnects to home WiFi automatically):

```bash
sudo nmcli con down pies-hotspot
```

---

### Option B — Direct Ethernet Cable (fallback)

Plug an Ethernet cable directly between your laptop and the RPi. Because there's no router, both ends need a manually assigned IP address so they can find each other.

> **Important:** Use the `10.42.0.x` subnet, not `192.168.1.x`. Most home routers hand out `192.168.1.x` addresses, so using that range on the Ethernet link causes a routing conflict — the RPi ends up with the same subnet on both wlan0 and eth0, which drops your WiFi SSH connection when the cable is plugged in.

**On the RPi** — run this once (over existing WiFi or with keyboard/monitor):

```bash
sudo bash -c 'cat > /etc/netplan/99-eth0-static.yaml << EOF
network:
  version: 2
  ethernets:
    eth0:
      addresses: [10.42.0.2/24]
      dhcp4: false
EOF
chmod 600 /etc/netplan/99-eth0-static.yaml'
sudo netplan apply
```

This tells the RPi: "whenever something is plugged into the Ethernet port, use IP `10.42.0.2`." The subnet is separate from your home WiFi, so plugging in the cable won't affect your WiFi SSH connection.

**On your laptop** — set a static IP on the Ethernet port:

- **macOS:** System Settings → Network → Ethernet → Details → TCP/IP → Configure IPv4: Manually → IP: `10.42.0.1`, Subnet: `255.255.255.0`, Router: *(leave blank)*
- **Windows 10:** When you plug in the cable, Windows will show "Unidentified network / No internet" — that's expected (no DHCP router). Then:
  Settings → Network & Internet → Change adapter options → right-click the Ethernet adapter → Properties → Internet Protocol Version 4 (TCP/IPv4) → Properties → Use the following IP address:
  IP: `10.42.0.1`, Subnet mask: `255.255.255.0`, Default gateway: *(leave blank)*

Then SSH in from your laptop:

```bash
ssh lipad@10.42.0.2
```

**Bring both a USB keyboard + HDMI cable to the competition as backup.**

---

## Field Day: Complete Sequence

Consolidated ordered checklist for competition or field testing.

### Step 1 — Before leaving home

**Set GPS coordinates** (right-click in QGC → *Copy coordinates*):
```bash
nano ~/pies-lipad/ros2_ws/src/pies_mission/pies_mission/mission_config.py
# Set PICKUP_LAT, PICKUP_LON, DROP_LAT, DROP_LON
```

**Start the hotspot** (while still on home WiFi or Ethernet):
```bash
sudo nmcli con up pies-hotspot
```
> RPi internet drops — expected. Ethernet SSH stays alive if the cable is still plugged in.

---

### Step 2 — At the field: connect

On your laptop, join WiFi `piesdrone` (password: `piesdrone123`). Open three separate SSH sessions:
```bash
ssh lipad@172.16.0.1
```

> **No hotspot?** Plug in the Ethernet cable, set laptop static IP to `10.42.0.1` (see [Option B — Direct Ethernet Cable](#option-b--direct-ethernet-cable-fallback) above), then `ssh lipad@10.42.0.2`.

---

### Step 3 — Terminal 1: Pixhawk bridge

Pixhawk must be powered first.
```bash
source ~/.bashrc
sudo ~/pies-lipad/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent serial --dev /dev/serial0 -b 921600
```
Leave open.

---

### Step 4 — Terminal 2: Launch autonomous stack

```bash
source ~/.bashrc
ros2 launch pies_mission autonomous.launch.py
```
Wait until `bucket_detector`, `autonomous_mission`, and `servo_node` all report startup.

---

### Step 5 — Terminal 3: Monitor mission state

```bash
source ~/.bashrc
ros2 topic echo /mission/status
```
Also open QGC **Messages panel** (bell icon, top-right) — state transitions stream here over SiK radio at full range.

---

### Step 6 — QGC pre-flight checks

- SiK 915 MHz radio connected → vehicle visible in QGC
- Geofence set with a safe ceiling (e.g. **15 m** relative to launch)
- Altitude mode: **Relative to Launch**
- Messages panel open (bell icon) to watch STATUSTEXT over radio

---

### Step 7 — Trigger the mission

Place drone at the takeoff point, then:
```bash
ros2 topic pub --once /mission/go std_msgs/msg/Empty "{}"
```
State machine: `ARMING → TAKEOFF → FLY_PICKUP → VISUAL → CLIMB → FLY_DROP → DROP → RTL → DONE`

---

### Step 8 — Abort

Flip RC failsafe **or** switch to **Position** mode in QGC — immediately interrupts OFFBOARD.

To re-open gripper before an unplanned landing:
```bash
ros2 topic pub --once /gripper/open std_msgs/msg/Empty "{}"
```
