"""
Continuous servo sweep — left to right and back.
Designed for Raspberry Pi using gpiozero.

Install: pip3 install gpiozero
Default pin: GPIO 17 (change SERVO_PIN to match your wiring)

Usage:
    engine = ServoEngine()
    t = threading.Thread(target=engine.sweep)
    t.start()
    engine.change_sweep_state()  # stop
    t.join()
    engine.close()
"""

import threading
import time

try:
    from gpiozero import AngularServo
    from gpiozero.pins.pigpio import PiGPIOFactory
    _factory = PiGPIOFactory()  # hardware PWM — more precise, less jitter
except Exception:
    _factory = None  # falls back to software PWM if pigpio isn't running

SERVO_PIN = 17        # GPIO pin (BCM numbering)
MIN_ANGLE = -90       # degrees — full left
MAX_ANGLE = 90        # degrees — full right
SWEEP_STEP = 1        # degrees per step
STEP_DELAY = 0.02     # seconds between steps (~50 Hz update rate)


class ServoEngine:
    def __init__(
        self,
        pin: int = SERVO_PIN,
        min_angle: int = MIN_ANGLE,
        max_angle: int = MAX_ANGLE,
    ):
        kwargs = {"pin_factory": _factory} if _factory else {}
        self._servo = AngularServo(
            pin,
            min_angle=min_angle,
            max_angle=max_angle,
            min_pulse_width=0.001,   # 1ms — standard servo minimum
            max_pulse_width=0.002,   # 2ms — standard servo maximum
            **kwargs,
        )
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.sweep_running = False

    def change_sweep_state(self):
        """Toggle sweep_running on/off."""
        self.sweep_running = not self.sweep_running

    def sweep(self, step: int = SWEEP_STEP, step_delay: float = STEP_DELAY):
        """
        Sweep continuously from min_angle to max_angle and back until
        sweep_running is False. Intended to run in a background thread.
        """
        self.sweep_running = True
        angle = self.min_angle
        direction = 1  # 1 = moving right, -1 = moving left

        while self.sweep_running:
            self._servo.angle = angle

            angle += direction * step
            if angle >= self.max_angle:
                angle = self.max_angle
                direction = -1
            elif angle <= self.min_angle:
                angle = self.min_angle
                direction = 1

            time.sleep(step_delay)

        self._servo.mid()  # return to centre when stopped

    def close(self):
        self.sweep_running = False
        self._servo.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


if __name__ == "__main__":
    import signal

    with ServoEngine() as engine:
        print("Sweeping — Ctrl-C to stop")
        signal.signal(signal.SIGINT, lambda *_: engine.change_sweep_state())

        t = threading.Thread(target=engine.sweep)
        t.start()
        t.join()
