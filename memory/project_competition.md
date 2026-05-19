---
name: project-competition
description: "C-UASC competition details, missions, scoring, and strategy for the pies-lipad drone system"
metadata: 
  node_type: memory
  type: project
  originSessionId: 755c71f3-6f41-4146-863e-325b7b333604
---

Full competition details, geofence coordinates, missions, scoring, and build order are in the repo at `docs/competition.md`. Daily startup sequence (agent + servo + camera) is in `docs/startup.md`.

**Why:** Building toward full autonomous mission capability for C-UASC (June 5–7, 2026).
**How to apply:** Always refer to docs/competition.md for rules and scoring. Prioritize autonomous operation for the 25% bonus. Package Recovery (16 pts) is highest value and the gripper is purpose-built for it.

## Software Status (as of 2026-05-18)
- ✅ Micro-XRCE-DDS-Agent built and running (serial to Pixhawk)
- ✅ ROS2 Humble installed, PX4 topics flowing
- ✅ pies_servo ROS2 package — servo controlled via /servo/angle
- ✅ Camera node working — /camera/image_raw at ~16 FPS, 800×600 NV21 (ros-humble-camera-ros); CSI ribbon cable confirmed seated 2026-05-18
- ✅ Geofence — handled by PX4/QGroundControl natively, no custom code needed
- ✅ tools/capture_frame.py — saves one JPEG from the camera for testing
- ✅ gripper_trigger (pies_mission) — GPS-proximity servo trigger, coords via --ros-args day-of
- ✅ green_x_detector — HSV blob, /vision/green_x (geometry_msgs/Point)
- ✅ red_bullseye_detector — dual-band HSV + circularity, /vision/red_bullseye
- ✅ marker_detector — Canny+perspective warp+triangle pattern, /vision/markers (JSON), optional pytesseract OCR
- ✅ bucket_detector — HSV colour detection for blue/orange/green bucket, /vision/bucket (JSON)
- ✅ Package recovery sequence — full stack complete, simulated and verified (see [[project-visual-centering]])

## Package Recovery Stack (pies_mission)
- `visual_centering.py` — standalone OFFBOARD node, centering + descent + grab
- `autonomous_mission.py` — full state machine: IDLE→ARMING→TAKEOFF→FLY_PICKUP→VISUAL→CLIMB→FLY_DROP→DROP→RTL→DONE
- `calibrate_gains.py` — ROS2 node that sets KP_X/KP_Y signs via live nudge test; run once after mounting
- `mission_config.py` — single config file for all parameters; no CLI flags needed
- `mission.launch.py` / `autonomous.launch.py` — launch files for each mode
- `tools/sim_mission.py` — full mission state machine simulator (no hardware)
- `tools/sim_calibrate.py` — calibration sign simulator for all 8 camera mount angles
- `tools/test_statustext.py` — pymavlink UDP server that replays full mission STATUSTEXT sequence in QGC; use before field to preview Messages panel; COMP_ID=191, MAV_AUTOPILOT_INVALID to suppress QGC voice

## Pre-flight steps still needed (hardware, day-of)
See [[project-preflight]] for the full checklist.
