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

### Option A — RPi WiFi Hotspot

Run this **once** with a keyboard/monitor plugged into the RPi (not over SSH — it will drop the connection):

```bash
sudo apt install network-manager
sudo bash -c 'echo "network: {config: disabled}" > /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg'
sudo bash -c 'printf "network:\n  version: 2\n  renderer: NetworkManager\n" > /etc/netplan/99-nm.yaml'
sudo netplan apply
nmcli con add type wifi ifname wlan0 con-name pies-hotspot ssid "piesdrone" mode ap \
  ipv4.method shared wifi-sec.key-mgmt wpa-psk wifi-sec.psk "piesdrone123" autoconnect no
```

At the field, start the hotspot:

```bash
nmcli con up pies-hotspot
```

Then SSH in from your laptop:

```bash
ssh lipad@10.42.0.1
```

Stop the hotspot (to reconnect to home WiFi):

```bash
nmcli con down pies-hotspot
```

---

### Option B — Direct Ethernet Cable (fallback)

Plug an Ethernet cable directly between your laptop and the RPi. Because there's no router, both ends need a manually assigned IP address so they can find each other.

**On your laptop** — set a static IP on the Ethernet port:

- **macOS:** System Settings → Network → Ethernet → Details → TCP/IP → Configure IPv4: Manually → IP: `192.168.1.1`, Subnet: `255.255.255.0`
- **Windows:** Settings → Network → Ethernet → Edit → Manual → IPv4 on → IP: `192.168.1.1`, Subnet: `255.255.255.0`

**On the RPi** — run this once (over existing WiFi or with keyboard/monitor):

```bash
sudo bash -c 'cat > /etc/netplan/99-eth0-static.yaml << EOF
network:
  version: 2
  ethernets:
    eth0:
      addresses: [192.168.1.2/24]
      dhcp4: false
EOF
chmod 600 /etc/netplan/99-eth0-static.yaml'
sudo netplan apply
```

This tells the RPi: "whenever something is plugged into the Ethernet port, use IP `192.168.1.2`." Your laptop is `192.168.1.1`, RPi is `192.168.1.2` — they can now talk directly.

Then SSH in from your laptop:

```bash
ssh lipad@192.168.1.2
```

**Bring both a USB keyboard + HDMI cable to the competition as backup.**
