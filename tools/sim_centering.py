#!/usr/bin/env python3
"""
Simulate visual_centering.py without any hardware or ROS.

Reads gains / thresholds from mission_config.py automatically.
Models the P-controller loop and shows how the drone converges on the bucket.

Usage:
    python3 tools/sim_centering.py                    # bucket at (+200, +150) px, 3 m alt
    python3 tools/sim_centering.py 100 -80            # custom start pixel offset (x, y)
    python3 tools/sim_centering.py 200 150 --alt 5    # simulate at 5 m altitude
    python3 tools/sim_centering.py 200 150 --plot     # matplotlib animation
"""
import math
import sys
import os

# ── Load mission_config ───────────────────────────────────────────────────────
try:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                    '..', 'ros2_ws', 'src', 'pies_mission'))
    from pies_mission import mission_config as cfg
    CAM_OFFSET_X = cfg.CAM_OFFSET_X_PX
    CAM_OFFSET_Y = cfg.CAM_OFFSET_Y_PX
    KP_X         = cfg.KP_X
    KP_Y         = cfg.KP_Y
    MAX_VEL      = cfg.MAX_VEL
    THRESHOLD    = cfg.CENTER_THRESHOLD_PX
    CONFIRM      = cfg.CONFIRM_FRAMES
    print("Config loaded from mission_config.py")
except ImportError:
    CAM_OFFSET_X = 0.0
    CAM_OFFSET_Y = 0.0
    KP_X = 0.004
    KP_Y = 0.004
    MAX_VEL = 0.3
    THRESHOLD = 20.0
    CONFIRM = 10
    print("mission_config not found — using built-in defaults")

# ── Parse arguments ───────────────────────────────────────────────────────────
args = sys.argv[1:]

def _floatarg(args, idx, default):
    try:
        v = args[idx]
        return float(v) if not v.startswith('--') else default
    except (IndexError, ValueError):
        return default

start_x  = _floatarg(args, 0, 200.0)
start_y  = _floatarg(args, 1, 150.0)

alt_idx  = next((i for i, a in enumerate(args) if a == '--alt'), None)
altitude = float(args[alt_idx + 1]) if alt_idx is not None else 3.0

do_plot  = '--plot' in args

# ── Camera scale: OV5647 54° HFOV, 800 px wide ───────────────────────────────
HFOV_DEG     = 54.0
frame_w_m    = 2 * altitude * math.tan(math.radians(HFOV_DEG / 2))
PX_PER_M     = 800 / frame_w_m
DT           = 0.05   # 20 Hz control loop

# ── World positions (east-positive, south-positive) ───────────────────────────
bucket_east  =  start_x / PX_PER_M
bucket_south =  start_y / PX_PER_M

drone_east   = 0.0
drone_south  = 0.0

centered_count = 0
start_dist = math.hypot(start_x - CAM_OFFSET_X, start_y - CAM_OFFSET_Y)

# ── Print header ──────────────────────────────────────────────────────────────
print(f"\n=== Visual Centering Simulation ===")
print(f"  Bucket start : ({start_x:+.0f}, {start_y:+.0f}) px  "
      f"→  dist {start_dist:.0f} px from gripper target")
print(f"  Altitude     : {altitude:.1f} m  →  {PX_PER_M:.0f} px/m  "
      f"({bucket_east:.2f} m east, {bucket_south:.2f} m south)")
print(f"  Gains        : kp_x={KP_X}  kp_y={KP_Y}  max_vel={MAX_VEL} m/s")
print(f"  Threshold    : {THRESHOLD:.0f} px  │  confirm {CONFIRM} frames "
      f"({CONFIRM * DT:.2f} s at 20 Hz)")
print(f"  Cam offset   : ({CAM_OFFSET_X:+.0f}, {CAM_OFFSET_Y:+.0f}) px\n")

# ── Matplotlib animation ──────────────────────────────────────────────────────
if do_plot:
    try:
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
        from matplotlib.animation import FuncAnimation

        FW, FH = 800, 600
        fig, ax = plt.subplots(figsize=(8, 6))
        ax.set_xlim(0, FW); ax.set_ylim(FH, 0)   # image coords (y-down)
        ax.set_facecolor('#8B7355')                # Mojave terrain colour
        ax.set_title('Visual Centering Simulation — camera view')
        ax.set_xlabel('pixels (east →)')
        ax.set_ylabel('pixels (↓ south)')

        cx, cy = FW // 2 + CAM_OFFSET_X, FH // 2 + CAM_OFFSET_Y
        ax.axhline(cy, color='white', lw=0.5, alpha=0.4)
        ax.axvline(cx, color='white', lw=0.5, alpha=0.4)
        gripper_cross, = ax.plot(cx, cy, 'w+', ms=18, mew=2, label='gripper target')

        bucket_dot, = ax.plot(FW // 2 + start_x, FH // 2 + start_y,
                              'o', ms=22, color='#FF6A00', label='bucket', zorder=5)
        error_line, = ax.plot([], [], 'r-', lw=1.5, alpha=0.7)
        info_txt = ax.text(10, 20, '', color='white', fontsize=9,
                           fontfamily='monospace', va='top')
        ax.legend(loc='lower right', fontsize=8)

        _drone_e = [0.0]; _drone_s = [0.0]; _count = [0]; _state = ['SEARCHING']

        def _step(frame):
            de, ds = _drone_e[0], _drone_s[0]
            ox = (bucket_east  - de) * PX_PER_M
            oy = (bucket_south - ds) * PX_PER_M
            ex, ey = ox - CAM_OFFSET_X, oy - CAM_OFFSET_Y
            dist = math.hypot(ex, ey)

            ve =  KP_X * ex;  ve = max(-MAX_VEL, min(MAX_VEL, ve))
            vn = -KP_Y * ey;  vn = max(-MAX_VEL, min(MAX_VEL, vn))

            if dist < THRESHOLD:
                _count[0] += 1; _state[0] = 'CENTERED'; ve = vn = 0.0
            else:
                _count[0] = 0; _state[0] = 'CENTERING'

            bpx, bpy = FW // 2 + ox, FH // 2 + oy
            bucket_dot.set_data([bpx], [bpy])
            error_line.set_data([cx, bpx], [cy, bpy])
            info_txt.set_text(
                f"step {frame:>4}   t={frame*DT:.2f} s\n"
                f"err  ({ex:+.0f}, {ey:+.0f}) px   dist {dist:.0f} px\n"
                f"vel  N={vn:+.3f}  E={ve:+.3f} m/s\n"
                f"state  {_state[0]}  ({_count[0]}/{CONFIRM})"
            )

            if _count[0] >= CONFIRM:
                info_txt.set_text(info_txt.get_text() + '\n✓ GRABBED')
                ani.event_source.stop()

            _drone_e[0] += ve * DT
            _drone_s[0] -= vn * DT
            return bucket_dot, error_line, info_txt

        ani = FuncAnimation(fig, _step, interval=50, blit=True, cache_frame_data=False)
        plt.tight_layout()
        plt.show()
        sys.exit(0)

    except ImportError:
        print("matplotlib not available — falling back to text mode\n")

# ── Text simulation ───────────────────────────────────────────────────────────
BAR_W = 20
prev_state = None

print(f"{'step':>5} {'t(s)':>6}  {'err_x':>7} {'err_y':>7} {'dist':>7}  "
      f"{'vel_n':>7} {'vel_e':>7}  state")
print("─" * 82)

for step in range(1000):
    offset_x = (bucket_east  - drone_east)  * PX_PER_M
    offset_y = (bucket_south - drone_south) * PX_PER_M
    err_x    = offset_x - CAM_OFFSET_X
    err_y    = offset_y - CAM_OFFSET_Y
    dist     = math.hypot(err_x, err_y)

    vel_e =  KP_X * err_x;  vel_e = max(-MAX_VEL, min(MAX_VEL, vel_e))
    vel_n = -KP_Y * err_y;  vel_n = max(-MAX_VEL, min(MAX_VEL, vel_n))

    if dist < THRESHOLD:
        centered_count += 1;  state = 'CENTERED';  vel_e = vel_n = 0.0
    else:
        centered_count = 0;   state = 'CENTERING'

    state_changed = state != prev_state
    if step == 0 or step % 10 == 0 or state_changed:
        filled = int(min(dist, start_dist) / max(start_dist, 1) * BAR_W)
        bar = '█' * filled + '░' * (BAR_W - filled)
        t = step * DT
        label = f"{state:<10}"
        if state == 'CENTERED':
            label += f"({centered_count}/{CONFIRM})"
        print(f"{step:>5} {t:>6.2f}  {err_x:>+7.1f} {err_y:>+7.1f} {dist:>7.1f}  "
              f"{vel_n:>+7.3f} {vel_e:>+7.3f}  {label}  |{bar}|")

    prev_state = state

    if centered_count >= CONFIRM:
        print(f"\n  GRABBED at t={step * DT:.2f} s  (step {step})")
        print(f"  Final offset: ({offset_x:+.1f}, {offset_y:+.1f}) px  dist={dist:.1f} px")
        sys.exit(0)

    drone_east  += vel_e * DT
    drone_south -= vel_n * DT   # south = −north

print("\n  Did not converge within 1000 steps — check gains or threshold")
