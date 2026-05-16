#!/usr/bin/env python3
"""
Visual centering — streams PX4 OFFBOARD velocity setpoints to position the
gripper directly above the detected bucket, then closes the servo to grab.

Works in two phases:
  1. CENTERING  — horizontal P-controller drives bucket to the calibrated
                  gripper-pixel until error < center_threshold_px for
                  confirm_frames consecutive frames AND altitude <= grab_alt.
                  Drone descends at descent_speed while centred.
  2. GRABBED    — servo fires, node sends zero velocity (hover).

The pilot arms and switches to OFFBOARD mode when ready; this node takes
over horizontal position and descent from that point.

── Camera-offset calibration (do this once after mounting) ─────────────────
  1. Hover above a visible coloured object in manual/position mode.
  2. Start bucket_detector and note offset_x / offset_y in the logs.
  3. Those values are cam_offset_x_px and cam_offset_y_px — they describe
     where the IMAGE CENTRE is relative to the GRIPPER projected on the
     ground at your hover altitude.
  4. Edit mission_config.py: set CAM_OFFSET_X_PX and CAM_OFFSET_Y_PX.

── Gain sign calibration ────────────────────────────────────────────────────
  Pixel errors are converted to drone body-frame velocities (forward/right),
  then rotated to NED using the drone's live heading.  This means the gains
  work correctly regardless of which direction the drone is facing.

  Default: kp_x > 0 → image-right nudges the drone in one body-axis direction
           kp_y > 0 → image-down  nudges the drone in the other body-axis direction
  If the drone moves the wrong way on one axis, negate that gain.
  Start with small values (0.004) and increase once direction is verified.

Topics:
  sub  /vision/bucket                       std_msgs/String  (JSON from bucket_detector)
  sub  /fmu/out/vehicle_local_position      px4_msgs/msg/VehicleLocalPosition
  sub  /gripper/open                        std_msgs/Empty   (re-open on demand)
  pub  /fmu/in/offboard_control_mode        px4_msgs/msg/OffboardControlMode
  pub  /fmu/in/trajectory_setpoint          px4_msgs/msg/TrajectorySetpoint
  pub  /servo/angle                         std_msgs/Float32
  pub  /vision/centering_status             std_msgs/String
       Values: SEARCHING | CENTERING | CENTERED | GRABBED
"""
import json
import math
import rclpy
from rclpy.node import Node
from . import mission_config as cfg
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float32, String, Empty

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition

_PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE,
    depth=1,
)

# Stale detection older than this (seconds) is ignored → hover in place
_DETECTION_TIMEOUT = 0.5

NAN = math.nan


class VisualCentering(Node):
    def __init__(self):
        super().__init__('visual_centering')

        # ── Parameters ────────────────────────────────────────────────────────
        # Pixel offset of image centre from the gripper footprint.
        # Positive x = gripper is to the RIGHT of image centre.
        # Positive y = gripper is BELOW  image centre (image y-down convention).
        self.declare_parameter('cam_offset_x_px',    cfg.CAM_OFFSET_X_PX)
        self.declare_parameter('cam_offset_y_px',    cfg.CAM_OFFSET_Y_PX)
        self.declare_parameter('kp_x',               cfg.KP_X)
        self.declare_parameter('kp_y',               cfg.KP_Y)
        self.declare_parameter('max_vel',            cfg.MAX_VEL)
        self.declare_parameter('center_threshold_px', cfg.CENTER_THRESHOLD_PX)
        self.declare_parameter('confirm_frames',     cfg.CONFIRM_FRAMES)
        self.declare_parameter('descent_speed',      cfg.DESCENT_SPEED)
        self.declare_parameter('grab_angle',         cfg.GRAB_ANGLE)
        self.declare_parameter('release_angle',      cfg.RELEASE_ANGLE)
        self.declare_parameter('grab_alt',           cfg.GRAB_ALT)
        self.declare_parameter('cam_rot_deg',        cfg.CAM_ROT_DEG)

        # How long after losing the bucket (camera blind spot) to still grab.
        # When the drone is directly above the bucket the side camera can't see it.
        # If we were centered within this window, we're close enough — grab anyway.
        self.declare_parameter('blind_grab_s',       cfg.BLIND_GRAB_S)

        # ── State ─────────────────────────────────────────────────────────────
        self._latest_bucket: dict | None = None
        self._bucket_stamp      = self.get_clock().now()
        self._centered_count    = 0
        self._state             = 'SEARCHING'
        self._grabbed           = False
        self._last_centered_t   = None   # time of last CENTERED frame
        self._gripper_inited    = False  # opened on first loop tick
        self._local_pos         = None   # latest VehicleLocalPosition for alt check

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(String, '/vision/bucket',  self._bucket_cb,       10)
        self.create_subscription(Empty,  '/gripper/open',   self._gripper_open_cb, 10)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._local_pos_cb, _PX4_QOS)

        # ── Publishers ────────────────────────────────────────────────────────
        self._ocm_pub    = self.create_publisher(
            OffboardControlMode,  '/fmu/in/offboard_control_mode',  _PX4_QOS)
        self._sp_pub     = self.create_publisher(
            TrajectorySetpoint,   '/fmu/in/trajectory_setpoint',    _PX4_QOS)
        self._servo_pub  = self.create_publisher(Float32, '/servo/angle', 10)
        self._status_pub = self.create_publisher(String,  '/vision/centering_status', 10)

        # 20 Hz control loop — must stream setpoints to keep PX4 happy in OFFBOARD
        self.create_timer(0.05, self._loop)

        self.get_logger().info(
            'Visual centering ready — gripper (landing gear) will open on first tick.\n'
            'Switch drone to OFFBOARD mode to activate.\n'
            '  cam_offset_x_px / cam_offset_y_px  need calibration after camera mount.\n'
            'To re-open gripper before landing:\n'
            '  ros2 topic pub --once /gripper/open std_msgs/msg/Empty "{}"'
        )

    # ── Subscription callbacks ─────────────────────────────────────────────────

    def _gripper_open_cb(self, _):
        """Re-open gripper on demand — use before any unplanned landing."""
        angle = self._p('release_angle')
        msg = Float32()
        msg.data = float(angle)
        self._servo_pub.publish(msg)
        self.get_logger().info(f'Gripper opened (landing gear)  servo → {angle}°')

    def _local_pos_cb(self, msg: VehicleLocalPosition):
        self._local_pos = msg

    def _bucket_cb(self, msg: String):
        data = json.loads(msg.data)
        if data:
            self._latest_bucket = data
            self._bucket_stamp  = self.get_clock().now()
        else:
            self._latest_bucket = None

    # ── Control loop (20 Hz) ──────────────────────────────────────────────────

    def _loop(self):
        ts = int(self.get_clock().now().nanoseconds / 1000)  # µs for PX4

        # Open gripper (landing gear) on first tick — servo_node is connected by now
        if not self._gripper_inited:
            angle = self._p('release_angle')
            msg = Float32()
            msg.data = float(angle)
            self._servo_pub.publish(msg)
            self._gripper_inited = True
            self.get_logger().info(f'Gripper opened (landing gear)  servo → {angle}°')

        # Always stream OffboardControlMode — PX4 exits OFFBOARD if this stops
        ocm = OffboardControlMode()
        ocm.position     = False
        ocm.velocity     = True
        ocm.acceleration = False
        ocm.attitude     = False
        ocm.body_rate    = False
        ocm.timestamp    = ts
        self._ocm_pub.publish(ocm)

        if self._grabbed:
            self._send_vel(0.0, 0.0, 0.0, ts)
            return

        # Drop stale detections
        bucket = self._latest_bucket
        if bucket is not None:
            age = (self.get_clock().now() - self._bucket_stamp).nanoseconds / 1e9
            if age > _DETECTION_TIMEOUT:
                bucket = None

        if bucket is None:
            # Blind-spot: side camera loses the bucket when directly overhead.
            # Keep descending toward grab_alt — fire when we reach it.
            # If BLIND_GRAB_S expires before reaching grab_alt, give up.
            if self._last_centered_t is not None:
                blind_age = (self.get_clock().now() - self._last_centered_t).nanoseconds / 1e9
                if blind_age < self._p('blind_grab_s'):
                    pos = self._local_pos
                    current_alt = -pos.z if pos else 999.0
                    if current_alt <= self._p('grab_alt'):
                        self.get_logger().info(
                            f'Blind-spot grab at {current_alt:.2f} m  '
                            f'({blind_age:.1f} s after centering)'
                        )
                        self._grab('blind-spot')
                    else:
                        self._send_vel(0.0, 0.0, self._p('descent_speed'), ts)
                    return
            self._centered_count = 0
            self._transition('SEARCHING')
            self._send_vel(0.0, 0.0, 0.0, ts)
            return

        # ── Error: pixel offset of bucket from gripper footprint ──────────────
        err_x = bucket['offset_x'] - self._p('cam_offset_x_px')
        err_y = bucket['offset_y'] - self._p('cam_offset_y_px')
        dist  = math.hypot(err_x, err_y)
        threshold = self._p('center_threshold_px')

        if dist < threshold:
            self._centered_count += 1
            self._last_centered_t = self.get_clock().now()
            self._transition('CENTERED')
            vz = self._p('descent_speed')
            self._send_vel(0.0, 0.0, vz, ts)

            pos = self._local_pos
            current_alt = -pos.z if pos else 999.0   # NED: z negative = above ground
            self.get_logger().info(
                f'CENTERED ({self._centered_count}/{self._p("confirm_frames")})  '
                f'dist={dist:.1f} px  alt={current_alt:.2f} m  '
                f'(grab at ≤{self._p("grab_alt")} m)',
                throttle_duration_sec=0.5)
            if self._centered_count >= self._p('confirm_frames') and current_alt <= self._p('grab_alt'):
                self._grab(bucket['color'])
        else:
            self._centered_count = 0
            self._transition('CENTERING')

            # Rotate pixel error to control axes, then apply gains.
            # phi aligns the error vector with the drone body axes for this camera mount.
            kx, ky    = self._p('kp_x'), self._p('kp_y')
            max_v     = self._p('max_vel')
            phi       = math.radians(90.0 - self._p('cam_rot_deg'))
            c, s      = math.cos(phi), math.sin(phi)
            ctrl_x    =  c * err_x + s * err_y   # drives vel_fwd
            ctrl_y    = -s * err_x + c * err_y   # drives vel_right
            vel_fwd   = max(-max_v, min(max_v,  kx * ctrl_x))
            vel_right = max(-max_v, min(max_v, -ky * ctrl_y))

            lp = self._local_pos
            hdg = lp.heading if (lp and not math.isnan(lp.heading)) else 0.0
            vel_n = vel_fwd * math.cos(hdg) - vel_right * math.sin(hdg)
            vel_e = vel_fwd * math.sin(hdg) + vel_right * math.cos(hdg)
            self._send_vel(vel_n, vel_e, 0.0, ts)

            self.get_logger().info(
                f'CENTERING [{bucket["color"]}]  '
                f'err=({err_x:+.0f},{err_y:+.0f})px  dist={dist:.1f}px  '
                f'fwd/right=({vel_fwd:+.3f},{vel_right:+.3f})m/s  hdg={math.degrees(hdg):.0f}°'
            )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _p(self, name):
        return self.get_parameter(name).value

    def _send_vel(self, vn: float, ve: float, vd: float, ts: int):
        sp = TrajectorySetpoint()
        sp.position  = [NAN, NAN, NAN]   # not controlling position
        sp.velocity  = [vn, ve, vd]       # NED: north, east, down
        sp.yaw       = NAN                # not controlling yaw
        sp.timestamp = ts
        self._sp_pub.publish(sp)

    def _grab(self, colour: str):
        angle = self._p('grab_angle')
        msg   = Float32()
        msg.data = float(angle)
        self._servo_pub.publish(msg)
        self._grabbed = True
        self.get_logger().info(
            f'══ GRABBED [{colour}] ══  servo → {angle}°  '
            f'Switch to RTL/Mission for next phase.'
        )
        self._publish_status('GRABBED')

    def _transition(self, new_state: str):
        if new_state != self._state:
            self._state = new_state
            self.get_logger().info(f'State → {new_state}')
            self._publish_status(new_state)

    def _publish_status(self, status: str):
        msg      = String()
        msg.data = status
        self._status_pub.publish(msg)


def main():
    rclpy.init()
    node = VisualCentering()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
