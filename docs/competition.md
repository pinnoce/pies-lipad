# C-UASC Competition

**Event:** CSU California Unmanned Aerial System Competition  
**Date:** June 5–7, 2026  
**Location:** Mojave Air & Space Port, Rutan Field, Mojave, CA

## Format

- 3 flight slots × 15 min each (5 min setup + 10 min flight) over 2 flying days
- One aerial mission + one ground mission per slot
- Final score = top 4 mission scores combined
- **25% autonomous bonus** on: Object Localization, Package Delivery, Payload Drop, Package Recovery

## Missions

| Mission | Max Points | Autonomous Bonus | Notes |
|---|---|---|---|
| Waypoint Navigation | 10 | — | 7 waypoints, provided morning of competition |
| Circuit Time Trial | 8–12 | — | Speed run through same waypoints |
| Package Drop | 14 | +25% | Drop beanbag from ≥6ft onto red bullseye |
| Package Delivery | 13 | +25% | Deliver cube gently to red bullseye |
| Target Localization | 14 | +25% | Find and number 5–10 black/white square markers |
| Package Recovery | **16** | **+25%** | Grab bucket at green X, return to takeoff point |

## Key Rules

- Waypoints given morning of competition — must be uploadable day-of, not hardcoded
- Geofence breach → autonomous RTL must be demonstrated before first flight (video deadline May 5 passed → shakedown flight required, uses one slot)
- Max 400ft AGL, always visual line of sight
- Max 55 lbs (25 kg) fully loaded
- Must turn in ≤150ft (46m) radius
- Mojave is hot (~6500ft density altitude) and windy (gusts to 14 kts)

## Geofence (Appendix)

| Point | Latitude | Longitude | Altitude (m) |
|---|---|---|---|
| 1 | 35°03'11.60"N | 118°09'06.84"W | 833.87 |
| 2 | 35°03'00.83"N | 118°09'07.13"W | 831.11 |
| 3 | 35°03'00.95"N | 118°09'12.54"W | 831.94 |
| 4 | 35°02'49.58"N | 118°09'12.75"W | 829.7 |
| 5 | 35°02'49.53"N | 118°09'01.91"W | 828.59 |
| 6 | 35°02'56.45"N | 118°09'01.94"W | 830.14 |
| 7 | 35°02'56.44"N | 118°08'54.90"W | 828.62 |
| 8 | 35°03'11.52"N | 118°08'54.75"W | 831.09 |

## Hardware

- **Gripper:** Rack-and-pinion servo (GPIO 12, 270° range) — grabs bucket by its upward-facing handle for Package Recovery
- **Flight controller:** Pixhawk connected to RPi via serial (TELEM2, 921600 baud)
- **Companion computer:** Raspberry Pi running Ubuntu 22.04
- **Camera:** Sunny P5V04A (OV5647, 5MP CSI) — publishing at 17 FPS via `/camera/image_raw`

## Suggested Flight Slot Strategy

- **Slot 1:** Shakedown (geofence RTL demo) + Waypoint Navigation
- **Slot 2:** Package Recovery (gripper) + Target Localization (camera)
- **Slot 3:** Re-attempt lowest score + Circuit Time Trial

## Software Build Order

1. **Mission sequencer** — accepts GPS waypoints day-of, flies them autonomously
2. **Computer vision** — green X detection (package recovery alignment) + numbered marker detection (target localization)
3. **Package recovery sequence** — combines sequencer + CV + gripper into one autonomous run

## Software Status

| Component | Status | Notes |
|---|---|---|
| Micro-XRCE-DDS-Agent | ✅ Done | Serial to Pixhawk at 921600 baud, `/dev/serial0` |
| ROS2 Humble | ✅ Done | PX4 `/fmu/` topics flowing |
| pies_servo | ✅ Done | `/servo/angle` (Float32, 0–270°), GPIO 12, lgpio |
| camera_ros | ✅ Done | `/camera/image_raw` at 17 FPS, 640×480, OV5647 |
| Geofence | ✅ Done | Configured in QGroundControl — PX4 enforces RTL on breach natively, no custom code needed |
| Mission sequencer | ❌ TODO | Accepts day-of waypoints, flies autonomously |
| Computer vision | ❌ TODO | Green X detection + numbered marker detection |
| Package recovery | ❌ TODO | Sequencer + CV + gripper combined autonomous run |
