"""Pan servo on the Pi's hardware PWM (jitter-free, unlike software PWM on GPIO)."""
import os
import time

import config as C

PERIOD_NS = 20_000_000  # 50 Hz


class Servo:
    def __init__(self):
        self.duty_file = None
        try:
            pwm = f"{C.PWM_CHIP}/pwm{C.PWM_CHANNEL}"
            if not os.path.isdir(pwm):
                self._write(f"{C.PWM_CHIP}/export", C.PWM_CHANNEL)
            # udev needs a moment to set permissions on a freshly exported channel
            for attempt in range(20):
                try:
                    self._write(f"{pwm}/period", PERIOD_NS)
                    break
                except PermissionError:
                    if attempt == 19:
                        raise
                    time.sleep(0.05)
            self._write(f"{pwm}/duty_cycle", self._duty_ns(0.0))
            self._write(f"{pwm}/enable", 1)
            self.duty_file = open(f"{pwm}/duty_cycle", "w")
            print("[servo] hardware PWM ready on GPIO18", flush=True)
        except OSError as e:
            print(f"[servo] PWM unavailable ({e}); is dtoverlay=pwm set?", flush=True)

    @staticmethod
    def _write(path, value):
        with open(path, "w") as f:
            f.write(str(value))

    @staticmethod
    def _duty_ns(pan_deg):
        angle = C.SERVO_CENTER_DEG + (-pan_deg if C.SERVO_INVERT else pan_deg)
        angle = min(180.0, max(0.0, angle))
        us = C.SERVO_MIN_US + (C.SERVO_MAX_US - C.SERVO_MIN_US) * angle / 180.0
        return int(us * 1000)

    @property
    def ok(self):
        return self.duty_file is not None

    def write(self, pan_deg):
        if not self.duty_file:
            return
        self.duty_file.seek(0)
        self.duty_file.write(str(self._duty_ns(pan_deg)))
        self.duty_file.flush()
