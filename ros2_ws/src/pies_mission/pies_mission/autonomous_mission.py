#!/usr/bin/env python3
"""
Full autonomous package recovery mission.

State machine:
  IDLE       — streaming setpoints, waiting for /mission/go
  ARMING     — sending arm + OFFBOARD mode commands until PX4 confirms
  TAKEOFF    — climbing to CRUISE_ALT above home
  FLY_PICKUP — flying to pickup GPS coords at cruise altitude
  VISUAL     — visual P-controller centers over bucket, descends, grabs
  CLIMB      — climbing back to cruise altitude after grab
  FLY_DROP   — flying to drop GPS coords at cruise altitude
  DROP       — descend to DROP_ALT, release gripper, hold position briefly
  RTL        — commanding return-to-launch
  DONE       — mission complete, node idles

To start:
  ros2 topic pub --once /mission/go std_msgs/msg/Empty "{}"

All parameters come from mission_config.py — edit that file before each flight.
"""
import json
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float32, String, Empty

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleGlobalPosition,
    VehicleStatus,
    LogMessage,
)

from . import mission_config as cfg

_PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE,
    depth=1,
)

NAN = math.nan
_DETECT_TIMEOUT = 0.5   # seconds before a bucket detection is considered stale

# ── State labels ──────────────────────────────────────────────────────────────
IDLE       = 'IDLE'
ARMING     = 'ARMING'
TAKEOFF    = 'TAKEOFF'
FLY_PICKUP = 'FLY_PICKUP'
VISUAL     = 'VISUAL'
CLIMB      = 'CLIMB'
FLY_DROP   = 'FLY_DROP'
DROP       = 'DROP'
RTL        = 'RTL'
DONE       = 'DONE'


class AutonomousMission(Node):

    def __init__(self):
        super().__init__('autonomous_mission')

        # ── Mission state ─────────────────────────────────────────────────────
        self._state          = IDLE
        self._armed          = False
        self._local_pos      = None   # latest VehicleLocalPosition
        self._home_lat       = None   # set on first GPS fix, used as NED origin
        self._home_lon       = None
        self._pickup_n       = None   # pickup waypoint in local NED (metres)
        self._pickup_e       = None
        self._drop_n         = None   # drop waypoint in local NED (metres)
        self._drop_e         = None

        # ── Visual centering state ────────────────────────────────────────────
        self._bucket         = None
        self._bucket_stamp   = self.get_clock().now()
        self._centered_count = 0
        self._grabbed        = False
        self._last_centered  = None   # time of last CENTERED frame

        # ── Timing helpers ────────────────────────────────────────────────────
        self._go_time        = None   # when /mission/go arrived
        self._drop_time      = None   # when gripper was released
        self._loop_count     = 0      # total 20 Hz ticks (used for rate-limiting cmds)
        self._gripper_inited = False  # opened on first loop tick once servo_node connects

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(Empty, '/mission/go',    self._go_cb,     10)
        self.create_subscription(Empty, '/gripper/open',  self._gripper_open_cb, 10)
        self.create_subscription(String, '/vision/bucket', self._bucket_cb, 10)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._local_pos_cb, _PX4_QOS)
        self.create_subscription(
            VehicleGlobalPosition, '/fmu/out/vehicle_global_position',
            self._global_pos_cb, _PX4_QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self._status_cb, _PX4_QOS)

        # ── Publishers ────────────────────────────────────────────────────────
        self._ocm_pub    = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', _PX4_QOS)
        self._sp_pub     = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', _PX4_QOS)
        self._cmd_pub    = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', _PX4_QOS)
        self._log_pub    = self.create_publisher(
            LogMessage, '/fmu/in/log_message', _PX4_QOS)
        self._servo_pub  = self.create_publisher(Float32, '/servo/angle', 10)
        self._status_pub = self.create_publisher(String, '/mission/status', 10)

        self.create_timer(0.05, self._loop)   # 20 Hz

        self.get_logger().info(
            'Autonomous mission ready — gripper (landing gear) will open on first tick.\n'
            '  1. Verify mission_config.py has correct GPS coords.\n'
            '  2. Place drone at takeoff point and power on.\n'
            '  3. Publish to /mission/go when ready:\n'
            '       ros2 topic pub --once /mission/go std_msgs/msg/Empty "{}"\n'
            '  To re-open gripper at any time (e.g. before emergency landing):\n'
            '       ros2 topic pub --once /gripper/open std_msgs/msg/Empty "{}"'
        )

    # ── Subscription callbacks ─────────────────────────────────────────────────

    def _go_cb(self, _):
        if self._state != IDLE:
            return
        if cfg.PICKUP_LAT == 0.0 or cfg.DROP_LAT == 0.0:
            self.get_logger().error(
                'PICKUP_LAT / DROP_LAT are still 0 in mission_config.py — aborting.')
            return
        if self._home_lat is None:
            self.get_logger().error('No GPS fix yet — aborting.')
            return
        self._go_time = self.get_clock().now()
        self._open_gripper()   # ensure gripper is open before takeoff
        self.get_logger().info(
            f'GO received — gripper opened, arming in {cfg.ARM_COUNTDOWN_S:.0f} s ...'
            '  Ctrl-C or flip RC failsafe to abort.')
        self._transition(ARMING)

    def _bucket_cb(self, msg: String):
        data = json.loads(msg.data)
        if data:
            self._bucket       = data
            self._bucket_stamp = self.get_clock().now()
        else:
            self._bucket = None

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        self._local_pos = msg

    def _global_pos_cb(self, msg: VehicleGlobalPosition):
        # Capture the first valid fix as the home / NED origin
        if self._home_lat is None and msg.lat != 0.0:
            self._home_lat = msg.lat
            self._home_lon = msg.lon
            self._pickup_n, self._pickup_e = self._gps_to_ned(cfg.PICKUP_LAT, cfg.PICKUP_LON)
            self._drop_n,   self._drop_e   = self._gps_to_ned(cfg.DROP_LAT,   cfg.DROP_LON)
            self.get_logger().info(
                f'GPS home: ({self._home_lat:.6f}, {self._home_lon:.6f})\n'
                f'  Pickup NED: ({self._pickup_n:+.1f}, {self._pickup_e:+.1f}) m\n'
                f'  Drop   NED: ({self._drop_n:+.1f},   {self._drop_e:+.1f}) m'
            )

    def _status_cb(self, msg: VehicleStatus):
        self._armed = (msg.arming_state == 2)

    def _gripper_open_cb(self, _):
        """Re-open gripper on demand — use before any unplanned landing."""
        self._open_gripper()

    # ── 20 Hz control loop ────────────────────────────────────────────────────

    def _loop(self):
        ts = int(self.get_clock().now().nanoseconds / 1000)
        self._loop_count += 1
        pos = self._local_pos

        # Open gripper (landing gear) on the first tick — servo_node is connected by now
        if not self._gripper_inited:
            self._open_gripper()
            self._gripper_inited = True

        # ── IDLE: stream position hold so PX4 will accept OFFBOARD switch ────
        if self._state == IDLE:
            self._pub_ocm(position=True, ts=ts)
            self._send_pos(0.0, 0.0, cfg.CRUISE_ALT, ts)

        # ── ARMING: send arm + OFFBOARD mode commands until confirmed ─────────
        elif self._state == ARMING:
            self._pub_ocm(position=True, ts=ts)
            self._send_pos(0.0, 0.0, cfg.CRUISE_ALT, ts)

            # Countdown check — don't arm immediately, give time to abort
            elapsed = (self.get_clock().now() - self._go_time).nanoseconds / 1e9
            remaining = cfg.ARM_COUNTDOWN_S - elapsed
            if remaining > 0:
                if self._loop_count % 20 == 0:   # log once per second
                    self.get_logger().info(f'Arming in {remaining:.0f} s ...')
                return

            # Send commands every 0.5 s until armed
            if self._loop_count % 10 == 0:
                self._send_vehicle_cmd(400, param1=1.0)           # ARM
                self._send_vehicle_cmd(176, param1=1.0, param2=6.0)  # → OFFBOARD

            if self._armed:
                self._transition(TAKEOFF)

        # ── TAKEOFF: climb to cruise altitude above home ──────────────────────
        elif self._state == TAKEOFF:
            self._pub_ocm(position=True, ts=ts)
            self._send_pos(0.0, 0.0, cfg.CRUISE_ALT, ts)
            if pos and abs(pos.z + cfg.CRUISE_ALT) < 0.5:
                self._transition(FLY_PICKUP)

        # ── FLY_PICKUP: fly to bucket GPS coords ─────────────────────────────
        elif self._state == FLY_PICKUP:
            self._pub_ocm(position=True, ts=ts)
            self._send_pos(self._pickup_n, self._pickup_e, cfg.CRUISE_ALT, ts)
            if pos:
                dist = math.hypot(pos.x - self._pickup_n, pos.y - self._pickup_e)
                if dist < cfg.ARRIVAL_RADIUS:
                    self._transition(VISUAL)

        # ── VISUAL: camera-based centering + descent + grab ───────────────────
        elif self._state == VISUAL:
            self._pub_ocm(velocity=True, ts=ts)
            self._visual_step(ts)
            if self._grabbed:
                self._transition(CLIMB)

        # ── CLIMB: go back up to cruise altitude ──────────────────────────────
        elif self._state == CLIMB:
            self._pub_ocm(position=True, ts=ts)
            if pos:
                self._send_pos(pos.x, pos.y, cfg.CRUISE_ALT, ts)
                if abs(pos.z + cfg.CRUISE_ALT) < 0.5:
                    self._transition(FLY_DROP)

        # ── FLY_DROP: fly to drop zone ────────────────────────────────────────
        elif self._state == FLY_DROP:
            self._pub_ocm(position=True, ts=ts)
            self._send_pos(self._drop_n, self._drop_e, cfg.CRUISE_ALT, ts)
            if pos:
                dist = math.hypot(pos.x - self._drop_n, pos.y - self._drop_e)
                if dist < cfg.ARRIVAL_RADIUS:
                    self._transition(DROP)

        # ── DROP: descend to DROP_ALT, release gripper, hold 2 s, then RTL ────
        elif self._state == DROP:
            self._pub_ocm(position=True, ts=ts)
            if pos:
                current_alt = -pos.z
                if self._drop_time is None:
                    # Phase 1: descend to drop altitude
                    self._send_pos(pos.x, pos.y, cfg.DROP_ALT, ts)
                    if current_alt <= cfg.DROP_ALT + 0.3:
                        self._release()
                        self._drop_time = self.get_clock().now()
                else:
                    # Phase 2: hold position after release, then RTL
                    self._send_pos(pos.x, pos.y, cfg.DROP_ALT, ts)
                    if (self.get_clock().now() - self._drop_time).nanoseconds / 1e9 > 2.0:
                        self._transition(RTL)

        # ── RTL: command PX4 to return home and land ──────────────────────────
        elif self._state == RTL:
            self._send_vehicle_cmd(20)   # NAV_RETURN_TO_LAUNCH
            self._open_gripper()         # ensure landing gear is open before touchdown
            self._transition(DONE)
            self.get_logger().info('Mission complete — gripper open, drone returning home.')

        # DONE: nothing to do

    # ── Visual centering (runs inside VISUAL state) ───────────────────────────

    def _visual_step(self, ts: int):
        bucket = self._bucket
        if bucket is not None:
            age = (self.get_clock().now() - self._bucket_stamp).nanoseconds / 1e9
            if age > _DETECT_TIMEOUT:
                bucket = None

        if bucket is None:
            # Blind-spot: keep descending toward grab_alt, fire when we reach it.
            # If BLIND_GRAB_S expires before reaching grab_alt, give up.
            if self._last_centered is not None:
                age = (self.get_clock().now() - self._last_centered).nanoseconds / 1e9
                if age < cfg.BLIND_GRAB_S:
                    pos = self._local_pos
                    current_alt = -pos.z if pos else 999.0
                    if current_alt <= cfg.GRAB_ALT:
                        self.get_logger().info(
                            f'Blind-spot grab at {current_alt:.2f} m  '
                            f'({age:.1f} s after centering)')
                        self._grab('blind-spot')
                    else:
                        self._send_vel(0.0, 0.0, cfg.DESCENT_SPEED, ts)
                    return
            self._centered_count = 0
            self._send_vel(0.0, 0.0, 0.0, ts)
            return

        err_x = bucket['offset_x'] - cfg.CAM_OFFSET_X_PX
        err_y = bucket['offset_y'] - cfg.CAM_OFFSET_Y_PX
        dist  = math.hypot(err_x, err_y)

        phi       = math.radians(90.0 - cfg.CAM_ROT_DEG)
        c, s      = math.cos(phi), math.sin(phi)
        ctrl_x    =  c * err_x + s * err_y
        ctrl_y    = -s * err_x + c * err_y
        vel_fwd   = max(-cfg.MAX_VEL, min(cfg.MAX_VEL,  cfg.KP_X * ctrl_x))
        vel_right = max(-cfg.MAX_VEL, min(cfg.MAX_VEL, -cfg.KP_Y * ctrl_y))

        lp = self._local_pos
        hdg = lp.heading if (lp and not math.isnan(lp.heading)) else 0.0
        vel_n = vel_fwd * math.cos(hdg) - vel_right * math.sin(hdg)
        vel_e = vel_fwd * math.sin(hdg) + vel_right * math.cos(hdg)

        if dist < cfg.CENTER_THRESHOLD_PX:
            self._centered_count += 1
            self._last_centered = self.get_clock().now()
            self._send_vel(0.0, 0.0, cfg.DESCENT_SPEED, ts)
            pos = self._local_pos
            current_alt = -pos.z if pos else 999.0   # NED: z negative = above ground
            self.get_logger().info(
                f'CENTERED ({self._centered_count}/{cfg.CONFIRM_FRAMES})  '
                f'dist={dist:.1f} px  alt={current_alt:.2f} m  '
                f'(grab at ≤{cfg.GRAB_ALT} m)',
                throttle_duration_sec=0.5)
            if self._centered_count >= cfg.CONFIRM_FRAMES and current_alt <= cfg.GRAB_ALT:
                self._grab(bucket['color'])
        else:
            self._centered_count = 0
            self._send_vel(vel_n, vel_e, 0.0, ts)
            self.get_logger().info(
                f'CENTERING [{bucket["color"]}]  '
                f'err=({err_x:+.0f},{err_y:+.0f}) px  dist={dist:.1f} px  '
                f'fwd/right=({vel_fwd:+.3f},{vel_right:+.3f}) m/s  hdg={math.degrees(hdg):.0f}°',
                throttle_duration_sec=0.5)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _gps_to_ned(self, lat: float, lon: float):
        """GPS coords → NED offset in metres from home."""
        R = 6_371_000.0
        north = math.radians(lat - self._home_lat) * R
        east  = math.radians(lon - self._home_lon) * R * math.cos(math.radians(self._home_lat))
        return north, east

    def _grab(self, colour: str):
        msg = Float32()
        msg.data = float(cfg.GRAB_ANGLE)
        self._servo_pub.publish(msg)
        self._grabbed = True
        self.get_logger().info(f'══ GRABBED [{colour}] ══  servo → {cfg.GRAB_ANGLE}°')
        self._publish_status('GRABBED')
        self._statustext(f'GRABBED [{colour}]')

    def _open_gripper(self):
        """Open gripper to receive the bucket handle. Called before takeoff."""
        msg = Float32()
        msg.data = float(cfg.RELEASE_ANGLE)
        self._servo_pub.publish(msg)
        self.get_logger().info(f'Gripper open (ready)  servo → {cfg.RELEASE_ANGLE}°')

    def _release(self):
        """Open gripper to drop the bucket at the drop zone."""
        msg = Float32()
        msg.data = float(cfg.RELEASE_ANGLE)
        self._servo_pub.publish(msg)
        self.get_logger().info(f'Gripper released  servo → {cfg.RELEASE_ANGLE}°')
        self._statustext('RELEASED at drop zone')

    def _send_vehicle_cmd(self, command: int, **params):
        msg = VehicleCommand()
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
        msg.command          = command
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        for k, v in params.items():
            setattr(msg, k, float(v))
        self._cmd_pub.publish(msg)

    def _pub_ocm(self, position: bool = False, velocity: bool = False, ts: int = 0):
        ocm = OffboardControlMode()
        ocm.position     = position
        ocm.velocity     = velocity
        ocm.acceleration = False
        ocm.attitude     = False
        ocm.body_rate    = False
        ocm.timestamp    = ts
        self._ocm_pub.publish(ocm)

    def _send_pos(self, north: float, east: float, alt_m: float, ts: int):
        """Position setpoint in local NED. alt_m is metres above home (positive up)."""
        sp = TrajectorySetpoint()
        sp.position  = [north, east, -alt_m]   # NED: up = negative z
        sp.velocity  = [NAN, NAN, NAN]
        sp.yaw       = NAN
        sp.timestamp = ts
        self._sp_pub.publish(sp)

    def _send_vel(self, vn: float, ve: float, vd: float, ts: int):
        """Velocity setpoint in NED (vd positive = descend)."""
        sp = TrajectorySetpoint()
        sp.position  = [NAN, NAN, NAN]
        sp.velocity  = [vn, ve, vd]
        sp.yaw       = NAN
        sp.timestamp = ts
        self._sp_pub.publish(sp)

    def _statustext(self, text: str):
        """Relay a message to QGC via MAVLink STATUSTEXT over the SiK telemetry link."""
        msg = LogMessage()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.severity = 6  # INFO
        padded = (text + '\x00' * 127)[:127]
        msg.text = [ord(c) for c in padded]
        self._log_pub.publish(msg)

    def _transition(self, new_state: str):
        if new_state != self._state:
            self.get_logger().info(f'Mission: {self._state} → {new_state}')
            self._state = new_state
            self._publish_status(new_state)
            self._statustext(f'Mission: {new_state}')

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)


def main():
    rclpy.init()
    node = AutonomousMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
