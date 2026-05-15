import threading
from pies_servo.servo_driver import ServoDriver

servo = ServoDriver(gpio_pin=12)
_release_timer = None


def on_angle_command(angle: float):
    """Called when /servo/angle receives a new value (0–270 degrees)."""
    global _release_timer

    print(f"Moving servo to {angle:.1f}°")
    servo.set_angle(angle)

    # Stop driving after 0.5s so the servo holds position without jittering
    if _release_timer is not None:
        _release_timer.cancel()
    _release_timer = threading.Timer(0.5, servo.release)
    _release_timer.start()


def on_shutdown():
    if _release_timer is not None:
        _release_timer.cancel()
    servo.close()
