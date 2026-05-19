#!/usr/bin/env python3
"""
Day-of mission configuration — edit this file before each flight.

Both autonomous_mission and visual_centering read their defaults from here.
No terminal flags needed; just save and restart the nodes.

With `colcon build --symlink-install` you do NOT need to rebuild after editing.
"""

# ── GPS waypoints (decimal degrees) ──────────────────────────────────────────
# Get these from QGroundControl (right-click map → Copy coordinates) or a GPS app.

PICKUP_LAT =  0.0   # latitude  of bucket / green X
PICKUP_LON =  0.0   # longitude of bucket / green X

DROP_LAT   =  0.0   # latitude  of drop zone
DROP_LON   =  0.0   # longitude of drop zone

# ── Servo angles — calibrate on the bench before each event ──────────────────
# Direction confirmed: 0° = CCW = close, 270° = CW = open.
# Sweep to mechanical limits in 10° steps, then add/subtract 10° margin.

GRAB_ANGLE    =  30.0   # servo angle that CLOSES the gripper
RELEASE_ANGLE = 240.0   # servo angle that OPENS  the gripper

# ── Camera / visual centering ─────────────────────────────────────────────────
# Rotation of the camera relative to the drone body (degrees, CCW viewed from above).
#   0   = drone nose → image top    (camera upright on drone)
#  90   = drone nose → image right
# 180   = drone nose → image bottom
# 270   = drone nose → image left
# Set to whichever 90° increment matches your actual mount.
CAM_ROT_DEG = 0
# Calibrate cam_offset after mounting:
#   1. Hover above a visible object in Position mode.
#   2. Read offset_x / offset_y from bucket_detector logs.
#   3. Those numbers are the camera centre's pixel offset from the gripper footprint.
#      Paste them below.

CAM_OFFSET_X_PX =   0.0   # positive = image centre is to the RIGHT of the gripper
CAM_OFFSET_Y_PX =   0.0   # positive = image centre is BELOW the gripper (image y-down)

# P-gains: m/s per pixel error, applied in the drone's body frame.
# Signs depend on camera mount — run 'ros2 run pies_mission calibrate_gains'.
KP_X = 0.004
KP_Y = 0.004

# Hard speed cap during visual centering (m/s)
MAX_VEL = 0.3

# Pixel radius considered "centred" (smaller = more precise but twitchier)
CENTER_THRESHOLD_PX = 20.0

# Consecutive centred frames required before grabbing (at 20 Hz: 10 = 0.5 s)
CONFIRM_FRAMES = 10

# Descend at this speed (m/s NED-down) while centred during the visual phase.
DESCENT_SPEED = 0.3

# Altitude (metres AGL) at which the gripper fires.
# Handle is ~0.2 m off the ground.  Start conservative (0.5 m) and lower it
# once you confirm the drone hovers stably at that height.
GRAB_ALT = 0.2

# Altitude (metres AGL) to descend to before releasing the bucket at the drop zone.
# Lower = gentler drop.  1.0 m is a reasonable starting point.
DROP_ALT = 0.2

# Which bucket colour to track: "any" | "blue" | "orange" | "green"
TARGET_COLOR = "any"

# ── Autonomous mission ────────────────────────────────────────────────────────

# Altitude for all GPS navigation legs (metres above home / takeoff point)
CRUISE_ALT = 5.0

# Horizontal distance at which a GPS waypoint is considered "reached" (metres)
ARRIVAL_RADIUS = 1.5

# Seconds to count down after /mission/go before sending the arm command.
# Use this time to verify the area is clear.
ARM_COUNTDOWN_S = 5.0

# ─────────────────────────────────────────────────────────────────────────────

# Blind-spot grab window (seconds).
# The side-mounted camera loses sight of the bucket when the drone is directly
# above it.  If the bucket disappears within this many seconds of being centred,
# treat it as "we're right above it" and fire the gripper.
# Set to 0.0 to disable (grab only via confirm_frames).
BLIND_GRAB_S = 4.0
