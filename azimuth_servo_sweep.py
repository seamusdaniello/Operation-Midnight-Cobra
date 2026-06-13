"""
Azimuth servo sweep — sine wave profile.
Designed for Raspberry Pi using gpiozero.

Install: pip3 install gpiozero
         sudo pigpiod  (optional, reduces jitter)

Sweep profile:
    angle(t) = AMPLITUDE * sin(FREQUENCY * t) + BIAS
             = 45 * sin(4.36 * t) + 45
    Range: 0° to 90°, period ≈ 1.44 s

Usage:
    engine = AzimuthServoEngine()
    t = threading.Thread(target=engine.sweep, args=(on_state,))
    t.start()
    engine.change_sweep_state()  # stop
    t.join()
    engine.close()
"""

import math
import threading
import time

try:
    import os
    from gpiozero import AngularServo
    from gpiozero.pins.pigpio import PiGPIOFactory
    _devnull  = os.open(os.devnull, os.O_WRONLY)
    _saved_fd = os.dup(2)
    os.dup2(_devnull, 2)
    os.close(_devnull)
    try:
        _factory = PiGPIOFactory()
    finally:
        os.dup2(_saved_fd, 2)
        os.close(_saved_fd)
except Exception:
    _factory = None
    class AngularServo:  # stub for non-Pi environments
        def __init__(self, *_, **settings): self.angle = settings.get('min_angle', 0.0)
        def mid(self): pass
        def close(self): pass

SERVO_PIN  = 17
AMPLITUDE  = 45.0   # degrees
BIAS       = 45.0   # degrees
FREQUENCY  = 4.36   # rad/s  →  period ≈ 1.44 s
UPDATE_HZ  = 50.0   # servo update rate


class AzimuthServoEngine:
    def __init__(self, pin: int = SERVO_PIN):
        kwargs = {"pin_factory": _factory} if _factory else {}
        self._servo = AngularServo(
            pin,
            min_angle=-90,
            max_angle=90,
            min_pulse_width=0.001,
            max_pulse_width=0.002,
            **kwargs,
        )
        self.sweep_running = False
        self.current_angle = 0.0

    def change_sweep_state(self):
        """Toggle sweep_running on/off."""
        self.sweep_running = not self.sweep_running

    def get_state(self, elapsed: float, angle: float) -> dict:
        return {
            "timestamp_s":      round(elapsed, 4),
            "angle_deg":        round(angle, 3),
            "sweep_running":    self.sweep_running,
            "amplitude_deg":    AMPLITUDE,
            "bias_deg":         BIAS,
            "frequency_rad_s":  FREQUENCY,
        }

    def sweep(self, callback=None, update_hz: float = UPDATE_HZ):
        """
        Drive the servo along a sine wave profile until sweep_running is False.
        Calls callback(state_dict) at each update if provided.
        """
        self.sweep_running = True
        interval = 1.0 / update_hz
        start = time.time()

        while self.sweep_running:
            t0 = time.time()
            elapsed = t0 - start

            angle = AMPLITUDE * math.sin(FREQUENCY * elapsed) + BIAS
            self.current_angle = angle
            self._servo.angle = angle

            if callback is not None:
                callback(self.get_state(elapsed, angle))

            remaining = interval - (time.time() - t0)
            if remaining > 0:
                time.sleep(remaining)

        self._servo.mid()

    def close(self):
        self.sweep_running = False
        self._servo.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


if __name__ == "__main__":
    import signal

    with AzimuthServoEngine() as engine:
        print("Sweeping — Ctrl-C to stop\n")

        def on_state(state: dict):
            print(state)

        signal.signal(signal.SIGINT, lambda *_: engine.change_sweep_state())

        t = threading.Thread(target=engine.sweep, args=(on_state,))
        t.start()
        t.join()
