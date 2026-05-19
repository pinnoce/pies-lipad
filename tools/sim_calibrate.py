#!/usr/bin/env python3
"""
Simulate the gain-sign calibration for every camera mount angle.

'cam_rot' is how many degrees the camera is rotated CCW (viewed from above)
relative to the drone body, where 0° = camera-up faces drone-forward.

  0°  = drone nose → image top    (forward = image-up)
  90° = drone nose → image right  (forward = image-right)  ← ideal for this controller
 180° = drone nose → image bottom
 270° = drone nose → image left

For each angle the sim:
  1. Computes the bucket pixel shift expected during each calibration nudge
  2. Determines KP_X / KP_Y signs using the same logic as calibrate_gains.py
  3. Runs a closed-loop simulation to check whether the gains converge

Usage:
    python3 tools/sim_calibrate.py
"""
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'ros2_ws', 'src', 'pies_mission'))
try:
    from pies_mission import mission_config as cfg
    KP_MAG    = abs(cfg.KP_X) or 0.004
    MAX_VEL   = cfg.MAX_VEL
    THRESHOLD = cfg.CENTER_THRESHOLD_PX
    CONFIRM   = cfg.CONFIRM_FRAMES
    print("Config: mission_config.py")
except ImportError:
    KP_MAG = 0.004; MAX_VEL = 0.3; THRESHOLD = 20.0; CONFIRM = 10
    print("Config: built-in defaults")

ALT       = 3.0     # hover altitude during calibration (m)
HFOV_DEG  = 54.0    # OV5647 camera HFOV
IMG_W     = 800
PX_PER_M  = IMG_W / (2 * ALT * math.tan(math.radians(HFOV_DEG / 2)))
DT        = 0.05    # 20 Hz control loop

NUDGE_VEL = 0.15    # matches calibrate_gains.py _NUDGE_VEL
NUDGE_S   = 0.8     # matches calibrate_gains.py _NUDGE_S
MIN_DISP  = 3.0     # px — matches calibrate_gains.py _MIN_PROJ_PX

START_ERR_X = 150.0  # bucket start error for convergence test (pixels)
START_ERR_Y = 100.0


# ── Camera model ──────────────────────────────────────────────────────────────
# When the drone moves at body-frame velocity (v_fwd, v_right), the bucket
# (fixed on the ground) appears to shift in the image by:
#   dpx/dt = (-sin(θ) * v_fwd  - cos(θ) * v_right) * PX_PER_M
#   dpy/dt = ( cos(θ) * v_fwd  - sin(θ) * v_right) * PX_PER_M
# where θ = cam_rot in radians.
#
# Verification at θ=0 (camera-up = drone-forward):
#   forward nudge → dpy = +v_fwd * PX_PER_M (bucket drifts to image bottom) ✓
#   right nudge   → dpx = -v_right * PX_PER_M (bucket drifts to image left)  ✓

def pixel_vel(cam_rot_deg, v_fwd, v_right):
    t = math.radians(cam_rot_deg)
    dpx = (-math.sin(t) * v_fwd - math.cos(t) * v_right) * PX_PER_M
    dpy = ( math.cos(t) * v_fwd - math.sin(t) * v_right) * PX_PER_M
    return dpx, dpy


# ── Calibration sign logic (mirrors calibrate_gains.py) ──────────────────────

def calibrate(cam_rot_deg):
    phi = math.radians(90.0 - cam_rot_deg)
    c, s = math.cos(phi), math.sin(phi)

    # Forward nudge — project both axes onto ctrl_x
    dpx_f, dpy_f = pixel_vel(cam_rot_deg, NUDGE_VEL, 0.0)
    proj_fwd = (c * dpx_f + s * dpy_f) * NUDGE_S

    # Right nudge — project both axes onto ctrl_y
    dpx_r, dpy_r = pixel_vel(cam_rot_deg, 0.0, NUDGE_VEL)
    proj_right = (-s * dpx_r + c * dpy_r) * NUDGE_S

    if abs(proj_fwd) >= MIN_DISP:
        kpx = (+1 if proj_fwd < 0 else -1) * KP_MAG
        kpx_note = f'proj_fwd={proj_fwd:+.1f}px'
    else:
        kpx = KP_MAG
        kpx_note = f'proj_fwd={proj_fwd:+.1f}px < {MIN_DISP:.0f} → default'

    if abs(proj_right) >= MIN_DISP:
        kpy = (+1 if proj_right > 0 else -1) * KP_MAG
        kpy_note = f'proj_right={proj_right:+.1f}px'
    else:
        kpy = KP_MAG
        kpy_note = f'proj_right={proj_right:+.1f}px < {MIN_DISP:.0f} → default'

    return kpx, kpy, kpx_note, kpy_note


# ── Closed-loop convergence test ──────────────────────────────────────────────

def converge(cam_rot_deg, kpx, kpy, max_steps=3000):
    err_x, err_y = START_ERR_X, START_ERR_Y
    start_dist   = math.hypot(err_x, err_y)
    centered     = 0
    phi          = math.radians(90.0 - cam_rot_deg)
    c, s         = math.cos(phi), math.sin(phi)

    for step in range(max_steps):
        ctrl_x =  c * err_x + s * err_y
        ctrl_y = -s * err_x + c * err_y
        vel_fwd   = max(-MAX_VEL, min(MAX_VEL,  kpx * ctrl_x))
        vel_right = max(-MAX_VEL, min(MAX_VEL, -kpy * ctrl_y))

        dpx, dpy = pixel_vel(cam_rot_deg, vel_fwd, vel_right)
        err_x   += dpx * DT
        err_y   += dpy * DT

        # Divergence guard
        if math.hypot(err_x, err_y) > start_dist * 20:
            return False, step, math.hypot(err_x, err_y)

        dist = math.hypot(err_x, err_y)
        if dist < THRESHOLD:
            centered += 1
            if centered >= CONFIRM:
                return True, step, dist
        else:
            centered = 0

    return False, max_steps, math.hypot(err_x, err_y)


# ── Main ──────────────────────────────────────────────────────────────────────

angles = list(range(0, 360, 45))

print(f"\n=== Calibration Simulation ===")
print(f"  Alt {ALT} m  |  {PX_PER_M:.0f} px/m  |  nudge {NUDGE_VEL} m/s × {NUDGE_S} s = {NUDGE_VEL*NUDGE_S*PX_PER_M:.0f} px max shift")
print(f"  Bucket start: ({START_ERR_X:+.0f}, {START_ERR_Y:+.0f}) px  |  "
      f"threshold {THRESHOLD:.0f} px  |  confirm {CONFIRM} frames\n")

print(f"{'Angle':>6}  {'Camera orientation':<26}  {'KP_X':>7}  {'KP_Y':>7}  "
      f"{'Converge':>10}  {'Steps':>6}  Notes")
print("─" * 100)

for deg in angles:
    # Human-readable orientation description
    orientations = {
        0:   'nose → image top',
        45:  'nose → top-right',
        90:  'nose → image right',
        135: 'nose → bottom-right',
        180: 'nose → image bottom',
        225: 'nose → bottom-left',
        270: 'nose → image left',
        315: 'nose → top-left',
    }
    label = orientations.get(deg, '')

    kpx, kpy, kpx_note, kpy_note = calibrate(deg)
    ok, steps, final_dist = converge(deg, kpx, kpy)

    status = '✓ YES' if ok else '✗ NO '
    t_conv = f'{steps * DT:.1f} s' if ok else f'div {final_dist:.0f}px'

    # Brief note
    # coupling factor = |cos(2θ)|; zero at 45°/135°/225°/315°
    coupling = abs(math.cos(2 * math.radians(deg)))
    if coupling < 0.1:
        note = '⚠ off-axis mount — calibration ambiguous; use nearest 90°'
    else:
        note = ''

    print(f"  {deg:>3}°  {label:<26}  {kpx:>+7.4f}  {kpy:>+7.4f}  "
          f"{status:>10}  {t_conv:>6}  {note}")
    if 'default' in kpx_note:
        print(f"         KP_X: {kpx_note}")
    if 'default' in kpy_note:
        print(f"         KP_Y: {kpy_note}")

print()
print("  0°/90°/180°/270° mounts converge. Diagonal mounts (45°/135°/225°/315°) diverge (⚠ above).")
print("  Set CAM_ROT_DEG in mission_config.py, then run calibrate_gains to set KP signs.")
