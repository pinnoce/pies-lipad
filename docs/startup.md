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
