---
name: project-visual-centering
description: Technical decisions and key parameters for the visual centering / package recovery system
metadata: 
  node_type: memory
  type: project
  originSessionId: 755c71f3-6f41-4146-863e-325b7b333604
---

Core design decisions that are non-obvious from reading the code.

**Why:** These were deliberated choices made across multiple sessions; useful context for future changes.

**How to apply:** When modifying the control law, calibration flow, or camera model, check this first.

## Camera model

Camera rotation is parameterised by `CAM_ROT_DEG` in `mission_config.py`:
- `0` = drone nose → image top (user's chosen mount)
- `90` = nose → image right
- `180` / `270` also supported; 45°-increment mounts are mathematically ambiguous (calibration fails)

Pixel error is rotated to drone body-frame control axes using:
```
phi = 90° - CAM_ROT_DEG
ctrl_x =  cos(phi)*err_x + sin(phi)*err_y   (drives vel_fwd)
ctrl_y = -sin(phi)*err_x + cos(phi)*err_y   (drives vel_right)
```

Body-frame velocity is then rotated to NED via the live heading:
```
vel_n = vel_fwd*cos(hdg) - vel_right*sin(hdg)
vel_e = vel_fwd*sin(hdg) + vel_right*cos(hdg)
```
This makes the gains work regardless of which direction the drone is facing.

## Gain sign convention

After running `calibrate_gains`:
- For `CAM_ROT_DEG=0`: KP_X becomes **negative** (−0.004), KP_Y stays **positive** (+0.004)
- For `CAM_ROT_DEG=90`: KP_X positive, KP_Y negative
- Pattern: `sign = -1 if cos(2*CAM_ROT_DEG) > 0 else +1`; KP_X uses this sign, KP_Y uses the opposite

The sign logic in `calibrate_gains._determine_kpx`: `proj < 0 → kpx positive`.
The sign logic in `calibrate_gains._determine_kpy`: `proj > 0 → kpy positive` (opposite condition — intentional).

## Key parameter values (current config)
- `GRAB_ALT = 0.2` m — gripper fires at this AGL altitude
- `DROP_ALT = 0.2` m — bucket released at this AGL altitude
- `GRAB_ANGLE = 30°`, `RELEASE_ANGLE = 240°` — direction confirmed correct (0°=CCW=close, 270°=CW=open) but final values need tuning once rack-and-pinion mechanism is physically attached (sweep to find mechanical limits, add 10° margin each side)
- `CRUISE_ALT = 5.0` m
- `DESCENT_SPEED = 0.3` m/s (NED vd, positive = down)
- `BLIND_GRAB_S = 4.0` s — camera loses bucket when directly overhead; grab if we were centred within this window
- `CENTER_THRESHOLD_PX = 20` px, `CONFIRM_FRAMES = 10` (0.5 s at 20 Hz)
- `CAM_ROT_DEG = 0` (nose → image top)

## Simulation tools
- `tools/sim_calibrate.py` — tests calibration sign logic for all 8 mount angles; expected output: 0°/90°/180°/270° converge ✓, 45°/135°/225°/315° diverge with warning
- `tools/sim_mission.py` — full mission state machine; applies calibrated gain signs automatically; expected output: 66 s, all states ✓
- `tools/test_statustext.py` — replays mission STATUSTEXT sequence in QGC over UDP without hardware; use COMP_ID=191 (MAV_COMP_ID_ONBOARD_COMPUTER) + MAV_AUTOPILOT_INVALID to prevent QGC voice announcements

The sim pixel model (`offset_x = (PICKUP_E−de)*px_per_m`, `offset_y = −(PICKUP_N−dn)*px_per_m`) is only correct for `CAM_ROT_DEG=0` with heading=0. Valid for sim purposes since heading is not modelled.

## QGC monitoring via SiK telemetry

`autonomous_mission.py` publishes state transitions as MAVLink STATUSTEXT via `/fmu/in/log_message` (px4_msgs/LogMessage). Flow: RPi → PX4 DDS bridge → uORB log_message → MAVLink STATUSTEXT → SiK radio → QGC Messages panel. Wired into `_transition()`, `_grab()`, and `_release()`.

`LogMessage.text` is `uint8[128]` — must provide exactly 128 bytes. The `_statustext()` method pads with nulls to 128 chars: `(text + '\x00' * 128)[:128]`.

**Still needs real-hardware verification**: confirm PX4's DDS bridge actually forwards `/fmu/in/log_message` with this firmware build. If messages don't appear in QGC, check that `log_message` is in the uXRCE-DDS subscription list (`uxrce_dds_client` module, `dds_topics.yaml`).

## Bugs fixed across reviews (2026-05-16)

- **`tools/test_statustext.py`** — printed hardcoded `192.168.1.161` in QGC setup instructions instead of the computed `{ip}` variable. Fixed.
- **`tools/sim_calibrate.py`** — footer said "All orientations now converge" which contradicts the table showing diagonal mounts (45°/135°/225°/315°) diverge. Fixed to accurate wording.
- **`autonomous_mission.py` `_statustext()`** — padded text to 127 chars with `[:127]` but `LogMessage.text` is `uint8[128]` (fixed-size). Would raise `ValueError` at runtime on first STATUSTEXT send. Fixed to `[:128]`.
