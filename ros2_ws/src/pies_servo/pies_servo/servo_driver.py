import pigpio

_MIN_PULSE = 500    # microseconds → 0°
_MAX_PULSE = 2500   # microseconds → 270°
_MAX_ANGLE = 270.0


class ServoDriver:
    def __init__(self, gpio_pin: int):
        self.gpio = gpio_pin
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpiod is not running — start it with: sudo /usr/local/bin/pigpiod")

    def set_angle(self, angle: float):
        """Move servo to angle in degrees (0–270)."""
        angle = max(0.0, min(_MAX_ANGLE, angle))
        pulse = int(_MIN_PULSE + (angle / _MAX_ANGLE) * (_MAX_PULSE - _MIN_PULSE))
        self.pi.set_servo_pulsewidth(self.gpio, pulse)

    def center(self):
        """Move servo to 135° (midpoint of 270° range)."""
        self.set_angle(135)

    def release(self):
        """Stop PWM signal — servo holds its position but stops being driven."""
        self.pi.set_servo_pulsewidth(self.gpio, 0)

    def close(self):
        self.release()
        self.pi.stop()
