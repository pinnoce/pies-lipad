#!/usr/bin/env python3
"""
Simulate the full autonomous_mission state machine — no hardware needed.

Reads all parameters from mission_config.py.
Uses example NED positions for pickup/drop (since GPS coords are set day-of).

Usage:
  python3 tools/sim_mission.py                            # text output
  python3 tools/sim_mission.py --pickup 30 20 --drop 55 -10  # custom waypoints (N E metres)
  python3 tools/sim_mission.py --pickup 30 20 --plot      # matplotlib trajectory
"""
import math, sys, os

# ── Load mission_config ────────────────────────────────────────────────────────
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                '..', 'ros2_ws', 'src', 'pies_mission'))
try:
    from pies_mission import mission_config as cfg
    CRUISE_ALT      = cfg.CRUISE_ALT
    ARRIVAL_RADIUS  = cfg.ARRIVAL_RADIUS
    ARM_COUNTDOWN_S = cfg.ARM_COUNTDOWN_S
    DESCENT_SPEED   = cfg.DESCENT_SPEED
    GRAB_ALT        = cfg.GRAB_ALT
    DROP_ALT        = cfg.DROP_ALT
    KP_X = cfg.KP_X;  KP_Y = cfg.KP_Y
    MAX_VEL         = cfg.MAX_VEL
    THRESHOLD       = cfg.CENTER_THRESHOLD_PX
    CONFIRM         = cfg.CONFIRM_FRAMES
    BLIND_GRAB_S    = cfg.BLIND_GRAB_S
    CAM_OFFSET_X    = cfg.CAM_OFFSET_X_PX
    CAM_OFFSET_Y    = cfg.CAM_OFFSET_Y_PX
    CAM_ROT_DEG     = cfg.CAM_ROT_DEG
    print("Config: mission_config.py")
except ImportError:
    CRUISE_ALT = 5.0;  ARRIVAL_RADIUS = 1.5;  ARM_COUNTDOWN_S = 5.0
    DESCENT_SPEED = 0.3;  GRAB_ALT = 0.5;  DROP_ALT = 1.0
    KP_X = KP_Y = 0.004;  MAX_VEL = 0.3
    THRESHOLD = 20.0;  CONFIRM = 10;  BLIND_GRAB_S = 4.0
    CAM_OFFSET_X = CAM_OFFSET_Y = 0.0;  CAM_ROT_DEG = 0
    print("Config: built-in defaults")

# ── Parse args ─────────────────────────────────────────────────────────────────
args = sys.argv[1:]
PICKUP_N, PICKUP_E = 30.0, 20.0
DROP_N,   DROP_E   = 55.0, -10.0
DO_PLOT = '--plot' in args
for i, a in enumerate(args):
    if a == '--pickup' and i + 2 < len(args):
        PICKUP_N, PICKUP_E = float(args[i+1]), float(args[i+2])
    elif a == '--drop' and i + 2 < len(args):
        DROP_N, DROP_E = float(args[i+1]), float(args[i+2])

# ── Calibrated gain signs ──────────────────────────────────────────────────────
# calibrate_gains.py sets signs from hardware nudge tests.  The sign for any
# 90°-increment mount is determined by cos(2*cam_rot): negative cos → positive KP_X.
_sign = -1 if math.cos(2 * math.radians(CAM_ROT_DEG)) > 0 else 1
KP_X  = _sign        * abs(KP_X)
KP_Y  = (-_sign)     * abs(KP_Y)

# ── Sim physics ────────────────────────────────────────────────────────────────
DT           = 0.05   # 20 Hz (matches real node)
MAX_NAV_H    = 4.0    # m/s horizontal in position mode
MAX_CLIMB    = 2.5    # m/s climb rate
MAX_SINK_RTL = 1.5    # m/s descent during RTL
KP_POS       = 1.5    # position controller gain for navigation
HFOV_DEG     = 54.0   # OV5647 camera horizontal FOV
IMG_W        = 800
_PX1M        = IMG_W / (2 * math.tan(math.radians(HFOV_DEG / 2)))  # px/m at 1 m alt

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))

def _nav_vel(cur, tgt, max_speed):
    return _clamp(KP_POS * (tgt - cur), -max_speed, max_speed)

# ── Drone state ────────────────────────────────────────────────────────────────
dn   = de  = 0.0    # north, east position (metres from home)
alt  = 0.0          # altitude above home (metres, positive up)

# Visual centering state
centered_count  = 0
grabbed         = False
last_centered_t = None

# Timing
drop_timer  = None
dropped     = False   # True once gripper has been released

# ── Trajectory recording ────────────────────────────────────────────────────────
rec_n, rec_e, rec_alt, rec_state, rec_t = [], [], [], [], []
events = []   # (t, label) for annotations / plot

# ── Output helpers ─────────────────────────────────────────────────────────────
STATE_PAD = 12

def log(t, state, note=''):
    print(f" {t:>6.1f}  {state:<{STATE_PAD}} {dn:>+7.1f} {de:>+7.1f} {alt:>6.2f}  {note}")

# ── Header ─────────────────────────────────────────────────────────────────────
pickup_dist = math.hypot(PICKUP_N, PICKUP_E)
drop_dist   = math.hypot(DROP_N - PICKUP_N, DROP_E - PICKUP_E)
rtl_dist    = math.hypot(DROP_N, DROP_E)
approx_time = (ARM_COUNTDOWN_S
               + CRUISE_ALT / MAX_CLIMB
               + pickup_dist / MAX_NAV_H
               + drop_dist / MAX_NAV_H
               + rtl_dist / MAX_NAV_H
               + CRUISE_ALT / MAX_CLIMB * 2
               + 30)   # rough visual + climb buffer

print(f"\n=== Autonomous Mission Simulation ===")
print(f"  Home    : (0, 0)")
print(f"  Pickup  : ({PICKUP_N:+.0f} m N, {PICKUP_E:+.0f} m E)  dist {pickup_dist:.0f} m from home")
print(f"  Drop    : ({DROP_N:+.0f} m N, {DROP_E:+.0f} m E)  dist {drop_dist:.0f} m from pickup")
print(f"  CRUISE_ALT={CRUISE_ALT} m  ARRIVAL_RADIUS={ARRIVAL_RADIUS} m  "
      f"DESCENT_SPEED={DESCENT_SPEED} m/s")
print(f"  ARM_COUNTDOWN={ARM_COUNTDOWN_S:.0f} s  CONFIRM={CONFIRM} frames  "
      f"GRAB_ALT={GRAB_ALT} m  DROP_ALT={DROP_ALT} m  BLIND_GRAB={BLIND_GRAB_S} s")
print(f"  Estimated total time: ~{approx_time:.0f} s")
print(f"\n {'t(s)':>6}  {'state':<{STATE_PAD}} {'N(m)':>7} {'E(m)':>7} {'alt(m)':>6}  note")
print("─" * 72)

# ── Main simulation loop ───────────────────────────────────────────────────────
state = 'ARMING'
events.append((0.0, 'ARMING'))
log(0.0, state, f'/mission/go received — arming in {ARM_COUNTDOWN_S:.0f} s')

for tick in range(60_000):   # safety limit: 3000 s
    t = (tick + 1) * DT

    rec_n.append(dn); rec_e.append(de); rec_alt.append(alt)
    rec_state.append(state); rec_t.append(t)

    # ── ARMING: count down, then transition to TAKEOFF ─────────────────────────
    if state == 'ARMING':
        if t >= ARM_COUNTDOWN_S:
            state = 'TAKEOFF'
            events.append((t, 'TAKEOFF'))
            log(t, state, 'armed — climbing to cruise altitude')
        elif tick % 20 == 0:   # log once per second during countdown
            log(t, state, f'countdown {ARM_COUNTDOWN_S - t:.0f} s ...')

    # ── TAKEOFF: climb to CRUISE_ALT ──────────────────────────────────────────
    elif state == 'TAKEOFF':
        vz = _clamp(KP_POS * (CRUISE_ALT - alt), 0.0, MAX_CLIMB)
        alt += vz * DT
        if abs(alt - CRUISE_ALT) < 0.3:
            state = 'FLY_PICKUP'
            events.append((t, 'FLY_PICKUP'))
            log(t, state, f'at {alt:.1f} m — flying to pickup')
        elif tick % 20 == 0:
            log(t, state, f'climbing {alt:.1f} → {CRUISE_ALT} m')

    # ── FLY_PICKUP: fly to bucket GPS location ─────────────────────────────────
    elif state == 'FLY_PICKUP':
        vn = _nav_vel(dn, PICKUP_N, MAX_NAV_H)
        ve = _nav_vel(de, PICKUP_E, MAX_NAV_H)
        dn += vn * DT;  de += ve * DT
        dist = math.hypot(dn - PICKUP_N, de - PICKUP_E)
        if dist < ARRIVAL_RADIUS:
            state = 'VISUAL'
            events.append((t, 'VISUAL'))
            log(t, state, f'arrived at pickup ({dist:.1f} m) — starting visual centering')
        elif tick % 20 == 0:
            log(t, state, f'dist to pickup {dist:.1f} m  speed {math.hypot(vn,ve):.1f} m/s')

    # ── VISUAL: camera P-controller + descent + grab ───────────────────────────
    elif state == 'VISUAL':
        px_per_m = _PX1M / max(alt, 0.1)
        offset_x = (PICKUP_E - de) * px_per_m
        offset_y = -(PICKUP_N - dn) * px_per_m   # image-down = south = −north

        # Bucket visible if within camera frame and not in blind spot
        in_frame = (abs(offset_x) < 400 and abs(offset_y) < 300
                    and not (alt < 0.8 and math.hypot(PICKUP_N - dn, PICKUP_E - de) < 0.25))

        if not in_frame:
            if (last_centered_t is not None
                    and (t - last_centered_t) < BLIND_GRAB_S):
                # Keep descending toward grab alt; fire when we arrive
                alt = max(alt - DESCENT_SPEED * DT, 0.0)
                if alt <= GRAB_ALT:
                    grabbed = True
                    events.append((t, 'GRABBED'))
                    log(t, state, f'BLIND-SPOT GRAB at {alt:.2f} m  '
                                  f'(centred {t-last_centered_t:.1f}s ago)')
                    state = 'CLIMB'
                    events.append((t, 'CLIMB'))
                elif tick % 20 == 0:
                    log(t, state, f'blind-spot descending  alt={alt:.2f} m  '
                                  f'(window {BLIND_GRAB_S-(t-last_centered_t):.1f}s left)')
            else:
                centered_count = 0
                if tick % 20 == 0:
                    log(t, state, 'SEARCHING — bucket not in frame')
        else:
            err_x = offset_x - CAM_OFFSET_X
            err_y = offset_y - CAM_OFFSET_Y
            dist_px = math.hypot(err_x, err_y)

            _phi = math.radians(90.0 - CAM_ROT_DEG)
            _c, _s   = math.cos(_phi), math.sin(_phi)
            ctrl_x   =  _c * err_x + _s * err_y
            ctrl_y   = -_s * err_x + _c * err_y
            vel_fwd   = _clamp( KP_X * ctrl_x, -MAX_VEL, MAX_VEL)
            vel_right = _clamp(-KP_Y * ctrl_y, -MAX_VEL, MAX_VEL)
            # sim has no yaw → heading = 0, so vel_n = vel_fwd, vel_e = vel_right
            vel_n, vel_e = vel_fwd, vel_right

            if dist_px < THRESHOLD:
                centered_count += 1
                last_centered_t = t
                alt = max(alt - DESCENT_SPEED * DT, 0.0)
                if tick % 20 == 0 or centered_count == 1:
                    log(t, state, f'CENTERED ({centered_count:>2}/{CONFIRM})  '
                                  f'dist={dist_px:.0f}px  alt={alt:.2f}m  '
                                  f'(grab at ≤{GRAB_ALT}m)')
                if centered_count >= CONFIRM and alt <= GRAB_ALT:
                    grabbed = True
                    events.append((t, 'GRABBED'))
                    log(t, state, f'══ GRABBED ══  alt={alt:.2f} m  '
                                  f'(handle ~0.2m off ground)')
                    state = 'CLIMB'
                    events.append((t, 'CLIMB'))
            else:
                centered_count = 0
                dn += vel_n * DT;  de += vel_e * DT
                if tick % 20 == 0:
                    log(t, state, f'centering  err=({err_x:+.0f},{err_y:+.0f})px  '
                                  f'dist={dist_px:.0f}px  vel=({vel_n:+.2f},{vel_e:+.2f})')

    # ── CLIMB: go back to cruise altitude ──────────────────────────────────────
    elif state == 'CLIMB':
        vz = _clamp(KP_POS * (CRUISE_ALT - alt), 0.0, MAX_CLIMB)
        alt += vz * DT
        if abs(alt - CRUISE_ALT) < 0.3:
            state = 'FLY_DROP'
            events.append((t, 'FLY_DROP'))
            log(t, state, f'at {alt:.1f} m — flying to drop zone')
        elif tick % 20 == 0:
            log(t, state, f'climbing {alt:.1f} → {CRUISE_ALT} m')

    # ── FLY_DROP: fly to drop zone ─────────────────────────────────────────────
    elif state == 'FLY_DROP':
        vn = _nav_vel(dn, DROP_N, MAX_NAV_H)
        ve = _nav_vel(de, DROP_E, MAX_NAV_H)
        dn += vn * DT;  de += ve * DT
        dist = math.hypot(dn - DROP_N, de - DROP_E)
        if dist < ARRIVAL_RADIUS:
            state = 'DROP'
            events.append((t, 'DROP'))
            log(t, state, f'arrived at drop zone ({dist:.1f} m) — descending to {DROP_ALT} m')
        elif tick % 20 == 0:
            log(t, state, f'dist to drop {dist:.1f} m')

    # ── DROP: descend to DROP_ALT, release gripper, hold 2 s ─────────────────
    elif state == 'DROP':
        if not dropped:
            # Phase 1: descend to DROP_ALT
            vz = _clamp(KP_POS * (alt - DROP_ALT), 0.0, MAX_SINK_RTL)
            alt = max(alt - vz * DT, DROP_ALT)
            if alt <= DROP_ALT + 0.1:
                dropped = True
                drop_timer = t
                events.append((t, 'RELEASED'))
                log(t, state, f'RELEASED at {alt:.2f} m  '
                              f'(bucket falls {DROP_ALT:.1f} m, much gentler)')
            elif tick % 20 == 0:
                log(t, state, f'descending to drop alt  {alt:.2f} → {DROP_ALT} m')
        else:
            # Phase 2: hold position 2 s, then RTL
            if t - drop_timer > 2.0:
                state = 'RTL'
                events.append((t, 'RTL'))
                log(t, state, 'returning home')

    # ── RTL: fly home and land ─────────────────────────────────────────────────
    elif state == 'RTL':
        vn = _nav_vel(dn, 0.0, MAX_NAV_H)
        ve = _nav_vel(de, 0.0, MAX_NAV_H)
        dn += vn * DT;  de += ve * DT
        home_dist = math.hypot(dn, de)
        if home_dist < 1.0:
            alt = max(alt - MAX_SINK_RTL * DT, 0.0)
            if alt < 0.05:
                state = 'DONE'
                events.append((t, 'DONE'))
                log(t, state, f'landed — mission complete in {t:.0f} s')
                break
        elif tick % 20 == 0:
            log(t, state, f'dist to home {home_dist:.1f} m')

print(f"\n  States logged: {' → '.join(e[1] for e in events)}")
print(f"  Total time   : {t:.1f} s")

# ── Matplotlib plot ────────────────────────────────────────────────────────────
if not DO_PLOT:
    sys.exit(0)

try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    import numpy as np
except ImportError:
    print("\nmatplotlib not available — install with: pip install matplotlib")
    sys.exit(0)

STATE_COLOR = {
    'ARMING': '#FF6600', 'TAKEOFF': '#FFAA00', 'FLY_PICKUP': '#00AAFF',
    'VISUAL': '#00FF88', 'CLIMB': '#FFFF00', 'FLY_DROP': '#AA44FF',
    'DROP': '#FF4444', 'RTL': '#FF88CC', 'DONE': '#AAAAAA',
}

fig, (ax, ax_alt) = plt.subplots(1, 2, figsize=(14, 6))
fig.patch.set_facecolor('#111122')
for a in (ax, ax_alt):
    a.set_facecolor('#1a1a2e')
    a.tick_params(colors='white'); a.xaxis.label.set_color('white')
    a.yaxis.label.set_color('white'); a.title.set_color('white')
    for spine in a.spines.values(): spine.set_edgecolor('#444')

# ── Bird's-eye ────────────────────────────────────────────────────────────────
ax.set_title("Mission trajectory — bird's eye view")
ax.set_xlabel("East (m)");  ax.set_ylabel("North (m)")
ax.set_aspect('equal');     ax.grid(True, alpha=0.2, color='white')

E = np.array(rec_e);  N = np.array(rec_n)
for i in range(len(rec_t) - 1):
    c = STATE_COLOR.get(rec_state[i], 'white')
    ax.plot(E[i:i+2], N[i:i+2], color=c, lw=2)

ax.plot(0, 0,         'ws',  ms=12, zorder=6, label='Home')
ax.plot(PICKUP_E, PICKUP_N, 'g^',  ms=14, zorder=6, label='Pickup')
ax.plot(DROP_E,   DROP_N,   'rv',  ms=14, zorder=6, label='Drop')

for ev_t, ev_label in events:
    if ev_label in ('GRABBED', 'VISUAL', 'RTL', 'DONE'):
        idx = min(range(len(rec_t)), key=lambda i: abs(rec_t[i] - ev_t))
        ax.annotate(ev_label,
                    xy=(rec_e[idx], rec_n[idx]),
                    xytext=(6, 6), textcoords='offset points',
                    color='white', fontsize=7,
                    arrowprops=dict(arrowstyle='->', color='white', lw=0.6))

patches = [mpatches.Patch(color=c, label=s)
           for s, c in STATE_COLOR.items() if s in set(rec_state)]
ax.legend(handles=patches + [
    plt.Line2D([0],[0], marker='s', color='w', ms=8, label='Home'),
    plt.Line2D([0],[0], marker='^', color='g', ms=8, label='Pickup'),
    plt.Line2D([0],[0], marker='v', color='r', ms=8, label='Drop'),
], facecolor='#222', labelcolor='white', fontsize=7, loc='best')

# ── Altitude vs time ─────────────────────────────────────────────────────────
ax_alt.set_title("Altitude over time")
ax_alt.set_xlabel("Time (s)");  ax_alt.set_ylabel("Altitude (m)")
ax_alt.grid(True, alpha=0.2, color='white')

T = np.array(rec_t);  A = np.array(rec_alt)
for i in range(len(rec_t) - 1):
    c = STATE_COLOR.get(rec_state[i], 'white')
    ax_alt.plot(T[i:i+2], A[i:i+2], color=c, lw=2)

ax_alt.axhline(CRUISE_ALT, color='white', lw=0.5, ls='--', alpha=0.4,
               label=f'cruise {CRUISE_ALT} m')
for ev_t, ev_label in events:
    ax_alt.axvline(ev_t, color='white', lw=0.4, alpha=0.3)
    ax_alt.text(ev_t + 0.3, CRUISE_ALT * 0.05, ev_label,
                color='white', fontsize=6, rotation=45, va='bottom')
ax_alt.legend(facecolor='#222', labelcolor='white', fontsize=7)

plt.tight_layout()
plt.show()
