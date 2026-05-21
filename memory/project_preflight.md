---
name: project-preflight
description: Hardware calibration and day-of steps required before competition flight
metadata: 
  node_type: memory
  type: project
  originSessionId: 755c71f3-6f41-4146-863e-325b7b333604
---

Steps that must be completed before the first competition flight.

**Why:** These require physical hardware and can't be done in software alone; easy to forget under competition pressure.

**How to apply:** Treat this as a blocking checklist — don't fly the autonomous mission until all steps are done.

## One-time after hardware mounting (bench, before field)

1. **Servo bench test** — direction confirmed 2026-05-16: 0° = CCW = closes gripper; 270° = CW = opens gripper. Defaults (GRAB=30°, RELEASE=240°) are in the correct direction.
   **Prerequisite:** `pigpiod` must be running — `sudo systemctl enable --now pigpiod` (should be persistent after this; verify with `systemctl is-active pigpiod`).
   **Remaining:** rack-and-pinion mechanism not yet attached to servo. Once mounted, sweep in 10° steps to find the mechanical stops on each end. Set `GRAB_ANGLE` = closed limit + 10° and `RELEASE_ANGLE` = open limit − 10° in `mission_config.py`. Stop immediately if you hear grinding.

2. **Camera check** — confirmed working 2026-05-18 after reseating CSI ribbon cable. Before each field session: verify `dmesg | grep -i ov5647` shows the driver loaded. If "no cameras available", reseat the ribbon cable (both ends — RPi CAM port and camera module).

3. **Camera offset calibration**
   - Hover above a visible coloured object in Position mode
   - `ros2 topic echo /vision/bucket` and read `offset_x` / `offset_y`
   - Set `CAM_OFFSET_X_PX` and `CAM_OFFSET_Y_PX` in `mission_config.py`

4. **Gain sign calibration**
   - Hover in OFFBOARD mode with bucket visible below
   - `ros2 run pies_mission calibrate_gains`
   - Follow prompts (two nudges: forward then right); writes KP_X/KP_Y signs to `mission_config.py`
   - Expected result for `CAM_ROT_DEG=0`: KP_X ≈ −0.004, KP_Y ≈ +0.004

## Day-of (at competition site)

5. **GPS coordinates** — right-click in QGroundControl to get decimal degrees:
   - `PICKUP_LAT`, `PICKUP_LON` — bucket / green X location
   - `DROP_LAT`, `DROP_LON` — drop zone location

6. **Confirm `CAM_ROT_DEG = 0`** is still correct for the physical mount (camera nose = image top).

7. **Run `python3 tools/sim_mission.py`** with the real GPS coords (convert to NED offsets) to sanity-check timing before flight.

## Field connection checklist
- Hotspot auto-starts on boot (`autoconnect yes`) — just power on RPi and join `piesdrone`
- **Bring Ethernet cable** — if hotspot fails, plug cable, set laptop to `10.42.0.1`, SSH to `10.42.0.2`, start hotspot manually
- At home after field: `sudo nmcli con down pies-hotspot` to restore internet
- Indoor arming test: use **Stabilized** mode (Position mode needs GPS)
