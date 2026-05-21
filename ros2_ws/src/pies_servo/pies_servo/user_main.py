import threading
from pies_servo.servo_driver import ServoDriver

servo = ServoDriver(gpio_pin=12)
_release_timer = None
_last_angle = None


def on_angle_command(angle: float):
    """Called when /servo/angle receives a new value (0–270 degrees)."""
    global _release_timer, _last_angle

    print(f"Moving servo to {angle:.1f}°")
    servo.set_angle(angle)

    # Hold PWM just long enough to reach position, then release to stop jitter.
    # Small moves finish fast — cut sooner. Scale 0–270° → 0.2–0.8 s.
    delta = abs(angle - _last_angle) if _last_angle is not None else 270.0
    hold_s = 0.2 + (delta / 270.0) * 0.6
    _last_angle = angle

    if _release_timer is not None:
        _release_timer.cancel()
    _release_timer = threading.Timer(hold_s, servo.release)
    _release_timer.start()


def on_shutdown():
    if _release_timer is not None:
        _release_timer.cancel()
    servo.close()
