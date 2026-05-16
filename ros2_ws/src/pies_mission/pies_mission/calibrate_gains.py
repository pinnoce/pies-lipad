#!/usr/bin/env python3
"""
Interactive gain-sign calibrator for visual_centering / autonomous_mission.

Hover the drone in OFFBOARD mode with the bucket centred below, then run:

    ros2 run pies_mission calibrate_gains

The node commands two small test nudges (~10 cm each — forward, then right),
projects the observed pixel shift onto the correct control axis based on
CAM_ROT_DEG, and writes the correct KP_X / KP_Y signs to mission_config.py.

Press Enter in the terminal to advance through each confirmation prompt.
Ctrl-C at any time to abort without writing changes.
"""

import json
import math
import pathlib
import re
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition

from . import mission_config as cfg

_PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE,
    depth=1,
)

NAN          = math.nan
_NUDGE_VEL   = 0.15   # m/s — small enough to be safe, large enough to measure
_NUDGE_S     = 0.8    # seconds per nudge
_RECOVER_S   = 1.2    # seconds to hover still after each nudge
_BASELINE_S  = 1.0    # seconds of still measurement before each nudge
_MIN_PROJ_PX = 3.0    # minimum projected pixel displacement to trust the result


class GainCalibrator(Node):

    def __init__(self):
        super().__init__('calibrate_gains')

        # phi rotates pixel error into the control frame for this camera mount.
        # phi = 90° - cam_rot_deg.  At cam_rot=90° (nose→right): phi=0, no rotation.
        # At cam_rot=0° (nose→top): phi=90°, axes are swapped.
        self._phi = math.radians(90.0 - cfg.CAM_ROT_DEG)

        self._local_pos  = None
        self._bucket     = None

        # Phase state machine
        self._phase      = 'WAIT_BUCKET'
        self._phase_t    = self.get_clock().now()

        # Sample buffers — both x and y collected during every baseline and nudge
        self._base_fwd_x:    list[float] = []
        self._base_fwd_y:    list[float] = []
        self._samples_fwd_x: list[float] = []
        self._samples_fwd_y: list[float] = []

        self._base_right_x:    list[float] = []
        self._base_right_y:    list[float] = []
        self._samples_right_x: list[float] = []
        self._samples_right_y: list[float] = []

        self._result_kpx: float | None = None
        self._result_kpy: float | None = None

        # Background thread reads Enter presses
        self._confirmed = False
        t = threading.Thread(target=self._read_enter, daemon=True)
        t.start()

        self.create_subscription(
            String, '/vision/bucket', self._bucket_cb, 10)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._pos_cb, _PX4_QOS)

        self._ocm_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', _PX4_QOS)
        self._sp_pub  = self.create_publisher(
            TrajectorySetpoint,  '/fmu/in/trajectory_setpoint',   _PX4_QOS)

        self.create_timer(0.05, self._loop)   # 20 Hz

        self.get_logger().info(
            '\n'
            '══ Gain Calibrator ══════════════════════════════════════════════\n'
            'Hover in OFFBOARD mode with the bucket visible below.\n'
            f'CAM_ROT_DEG = {cfg.CAM_ROT_DEG}°  '
            f'(phi = {math.degrees(self._phi):.0f}°)\n'
            f'Each test nudges {_NUDGE_VEL} m/s for {_NUDGE_S} s '
            f'(~{_NUDGE_VEL * _NUDGE_S * 100:.0f} cm).\n'
            'Keep RC ready to take over at any time.  Ctrl-C to abort.\n'
            '═════════════════════════════════════════════════════════════════\n'
            'Waiting for stable bucket detection...'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _read_enter(self):
        while True:
            input()
            self._confirmed = True

    def _bucket_cb(self, msg: String):
        data = json.loads(msg.data)
        self._bucket = data if data else None

    def _pos_cb(self, msg: VehicleLocalPosition):
        self._local_pos = msg

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._phase_t).nanoseconds / 1e9

    def _next_phase(self, phase: str):
        self._phase   = phase
        self._phase_t = self.get_clock().now()
        self._confirmed = False

    def _heading(self) -> float:
        lp = self._local_pos
        return lp.heading if (lp and not math.isnan(lp.heading)) else 0.0

    def _send_vel(self, vel_fwd: float, vel_right: float, ts: int):
        hdg = self._heading()
        sp = TrajectorySetpoint()
        sp.position  = [NAN, NAN, NAN]
        sp.velocity  = [
            vel_fwd * math.cos(hdg) - vel_right * math.sin(hdg),
            vel_fwd * math.sin(hdg) + vel_right * math.cos(hdg),
            0.0,
        ]
        sp.yaw       = NAN
        sp.timestamp = ts
        self._sp_pub.publish(sp)

    def _pub_ocm(self, ts: int):
        ocm = OffboardControlMode()
        ocm.position     = False
        ocm.velocity     = True
        ocm.acceleration = ocm.attitude = ocm.body_rate = False
        ocm.timestamp    = ts
        self._ocm_pub.publish(ocm)

    @staticmethod
    def _mean(lst: list[float]) -> float:
        return sum(lst) / len(lst) if lst else 0.0

    def _proj_fwd(self, dpx: float, dpy: float) -> float:
        """Project pixel displacement onto the ctrl_x (forward-response) axis."""
        c, s = math.cos(self._phi), math.sin(self._phi)
        return c * dpx + s * dpy

    def _proj_right(self, dpx: float, dpy: float) -> float:
        """Project pixel displacement onto the ctrl_y (right-response) axis."""
        c, s = math.cos(self._phi), math.sin(self._phi)
        return -s * dpx + c * dpy

    # ── Control loop ──────────────────────────────────────────────────────────

    def _loop(self):
        ts = int(self.get_clock().now().nanoseconds / 1000)
        self._pub_ocm(ts)

        b  = self._bucket
        px = b['offset_x'] if b else None
        py = b['offset_y'] if b else None

        # ── WAIT_BUCKET ───────────────────────────────────────────────────────
        if self._phase == 'WAIT_BUCKET':
            self._send_vel(0.0, 0.0, ts)
            if b is None:
                self._phase_t = self.get_clock().now()
            elif self._elapsed() >= 2.0:
                self.get_logger().info(
                    f'Bucket stable at ({px:+.0f}, {py:+.0f}) px.\n'
                    'Press Enter to start the FORWARD test...'
                )
                self._next_phase('CONFIRM_FWD')

        # ── CONFIRM_FWD ───────────────────────────────────────────────────────
        elif self._phase == 'CONFIRM_FWD':
            self._send_vel(0.0, 0.0, ts)
            if self._confirmed:
                self._base_fwd_x = []; self._base_fwd_y = []
                self.get_logger().info('Collecting forward baseline...')
                self._next_phase('BASELINE_FWD')

        # ── BASELINE_FWD ──────────────────────────────────────────────────────
        elif self._phase == 'BASELINE_FWD':
            self._send_vel(0.0, 0.0, ts)
            if b:
                self._base_fwd_x.append(px)
                self._base_fwd_y.append(py)
            if self._elapsed() >= _BASELINE_S:
                self._samples_fwd_x = []; self._samples_fwd_y = []
                self.get_logger().info(
                    f'Nudging FORWARD {_NUDGE_VEL} m/s for {_NUDGE_S} s...')
                self._next_phase('NUDGE_FWD')

        # ── NUDGE_FWD ─────────────────────────────────────────────────────────
        elif self._phase == 'NUDGE_FWD':
            self._send_vel(_NUDGE_VEL, 0.0, ts)
            if b:
                self._samples_fwd_x.append(px)
                self._samples_fwd_y.append(py)
            if self._elapsed() >= _NUDGE_S:
                self.get_logger().info('Recovering...')
                self._next_phase('RECOVER_FWD')

        # ── RECOVER_FWD ───────────────────────────────────────────────────────
        elif self._phase == 'RECOVER_FWD':
            self._send_vel(0.0, 0.0, ts)
            if self._elapsed() >= _RECOVER_S:
                self._result_kpx = self._determine_kpx()
                self.get_logger().info('Press Enter to start the RIGHT test...')
                self._next_phase('CONFIRM_RIGHT')

        # ── CONFIRM_RIGHT ─────────────────────────────────────────────────────
        elif self._phase == 'CONFIRM_RIGHT':
            self._send_vel(0.0, 0.0, ts)
            if self._confirmed:
                self._base_right_x = []; self._base_right_y = []
                self.get_logger().info('Collecting right baseline...')
                self._next_phase('BASELINE_RIGHT')

        # ── BASELINE_RIGHT ────────────────────────────────────────────────────
        elif self._phase == 'BASELINE_RIGHT':
            self._send_vel(0.0, 0.0, ts)
            if b:
                self._base_right_x.append(px)
                self._base_right_y.append(py)
            if self._elapsed() >= _BASELINE_S:
                self._samples_right_x = []; self._samples_right_y = []
                self.get_logger().info(
                    f'Nudging RIGHT {_NUDGE_VEL} m/s for {_NUDGE_S} s...')
                self._next_phase('NUDGE_RIGHT')

        # ── NUDGE_RIGHT ───────────────────────────────────────────────────────
        elif self._phase == 'NUDGE_RIGHT':
            self._send_vel(0.0, _NUDGE_VEL, ts)
            if b:
                self._samples_right_x.append(px)
                self._samples_right_y.append(py)
            if self._elapsed() >= _NUDGE_S:
                self.get_logger().info('Recovering...')
                self._next_phase('RECOVER_RIGHT')

        # ── RECOVER_RIGHT ─────────────────────────────────────────────────────
        elif self._phase == 'RECOVER_RIGHT':
            self._send_vel(0.0, 0.0, ts)
            if self._elapsed() >= _RECOVER_S:
                self._result_kpy = self._determine_kpy()
                self._write_config()
                self._next_phase('DONE')

        # ── DONE ──────────────────────────────────────────────────────────────
        elif self._phase == 'DONE':
            self._send_vel(0.0, 0.0, ts)

    # ── Sign determination ────────────────────────────────────────────────────

    def _determine_kpx(self) -> float:
        n_samp = min(len(self._samples_fwd_x), len(self._samples_fwd_y))
        n_base = min(len(self._base_fwd_x),    len(self._base_fwd_y))
        if n_samp < 3 or n_base < 3:
            self.get_logger().warn('Too few samples for KP_X — keeping existing value.')
            return cfg.KP_X

        dpx = self._mean(self._samples_fwd_x) - self._mean(self._base_fwd_x)
        dpy = self._mean(self._samples_fwd_y) - self._mean(self._base_fwd_y)
        proj = self._proj_fwd(dpx, dpy)

        if abs(proj) < _MIN_PROJ_PX:
            self.get_logger().warn(
                f'Projected shift too small ({proj:+.1f} px) — nudge may be too small.\n'
                'Keeping existing KP_X.  Try increasing _NUDGE_VEL or _NUDGE_S.')
            return cfg.KP_X

        sign  = +1 if proj < 0 else -1
        value = sign * abs(cfg.KP_X)
        self.get_logger().info(
            f'Forward nudge: raw shift ({dpx:+.1f}, {dpy:+.1f}) px  '
            f'projected = {proj:+.1f} px  →  KP_X = {value:+.4f}')
        return value

    def _determine_kpy(self) -> float:
        n_samp = min(len(self._samples_right_x), len(self._samples_right_y))
        n_base = min(len(self._base_right_x),    len(self._base_right_y))
        if n_samp < 3 or n_base < 3:
            self.get_logger().warn('Too few samples for KP_Y — keeping existing value.')
            return cfg.KP_Y

        dpx = self._mean(self._samples_right_x) - self._mean(self._base_right_x)
        dpy = self._mean(self._samples_right_y) - self._mean(self._base_right_y)
        proj = self._proj_right(dpx, dpy)

        if abs(proj) < _MIN_PROJ_PX:
            self.get_logger().warn(
                f'Projected shift too small ({proj:+.1f} px) — nudge may be too small.\n'
                'Keeping existing KP_Y.  Try increasing _NUDGE_VEL or _NUDGE_S.')
            return cfg.KP_Y

        sign  = +1 if proj > 0 else -1
        value = sign * abs(cfg.KP_Y)
        self.get_logger().info(
            f'Right nudge:   raw shift ({dpx:+.1f}, {dpy:+.1f}) px  '
            f'projected = {proj:+.1f} px  →  KP_Y = {value:+.4f}')
        return value

    # ── Write results ─────────────────────────────────────────────────────────

    def _write_config(self):
        kpx = self._result_kpx if self._result_kpx is not None else cfg.KP_X
        kpy = self._result_kpy if self._result_kpy is not None else cfg.KP_Y

        cfg_path = pathlib.Path(__file__).parent / 'mission_config.py'
        text = cfg_path.read_text()
        text = re.sub(
            r'^KP_X\s*=\s*[+-]?\d*\.?\d+',
            f'KP_X = {kpx:.4f}',
            text, flags=re.MULTILINE)
        text = re.sub(
            r'^KP_Y\s*=\s*[+-]?\d*\.?\d+',
            f'KP_Y = {kpy:.4f}',
            text, flags=re.MULTILINE)
        cfg_path.write_text(text)

        self.get_logger().info(
            '\n'
            '══ Calibration complete ════════════════════════════════════════\n'
            f'  KP_X = {kpx:.4f}\n'
            f'  KP_Y = {kpy:.4f}\n'
            '  Written to mission_config.py\n'
            '  Restart visual_centering / autonomous_mission to apply.\n'
            '═══════════════════════════════════════════════════════════════'
        )


def main():
    rclpy.init()
    node = GainCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
