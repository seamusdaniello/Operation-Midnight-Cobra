"""
Lightware SF20 — range collection engine.
Device: /dev/cu.usbserial-BB0_16170  |  115200 baud
Protocol: ASCII poll — send "?LD\r\n", reply is "ld,<echo>:<distance_m>\r\n"

Usage:
    engine = LidarEngine()
    measurement = engine.lidar_formatting(engine.lidar_collection())

    # streaming in a background thread:
    t = threading.Thread(target=engine.lidar_stream, args=(print,))
    t.start()
    engine.change_stream_state()  # stop
    t.join()
"""

import serial
import threading
import time
from typing import Callable, Optional

PORT = "/dev/cu.usbserial-BB0_16170"
BAUD = 115200
READ_CMD = b"?LD\r\n"


class LidarMeasurement:
    """A single range reading. range is in millimeters."""

    def __init__(self, range_mm: int):
        self.range = range_mm

    def __repr__(self) -> str:
        return f"LidarMeasurement(range={self.range}mm)"


class LidarEngine:
    def __init__(self, port: str = PORT, baud: int = BAUD):
        self._ser = serial.Serial(port, baud, timeout=0.5)
        self._ser.reset_input_buffer()
        self._lock = threading.Lock()
        self.stream_running = False

    def lidar_collection(self) -> str:
        """Send a poll command and return the raw response string."""
        with self._lock:
            self._ser.reset_input_buffer()
            self._ser.write(READ_CMD)
            return self._ser.readline().decode("ascii", errors="ignore").strip()

    def lidar_formatting(self, raw: str) -> Optional[LidarMeasurement]:
        """Parse a raw response into a LidarMeasurement. Returns None on bad data."""
        if ":" not in raw:
            return None
        try:
            meters = float(raw.split(":")[-1])
            return LidarMeasurement(int(meters * 1000))
        except ValueError:
            return None

    def change_stream_state(self):
        """Toggle stream_running on/off."""
        self.stream_running = not self.stream_running

    def lidar_stream(
        self,
        callback: Optional[Callable[[LidarMeasurement], None]] = None,
        sample_rate_hz: float = 20.0,
    ):
        """
        Collect and format readings continuously until stream_running is False.
        Intended to run in a background thread. Calls callback with each
        LidarMeasurement if provided.
        """
        self.stream_running = True
        interval = 1.0 / sample_rate_hz

        while self.stream_running:
            t0 = time.time()

            raw = self.lidar_collection()
            measurement = self.lidar_formatting(raw)

            if measurement is not None and callback is not None:
                callback(measurement)

            elapsed = time.time() - t0
            remaining = interval - elapsed
            if remaining > 0:
                time.sleep(remaining)

    def close(self):
        self.stream_running = False
        self._ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


if __name__ == "__main__":
    import signal

    with LidarEngine() as engine:
        print("Streaming at 20 Hz — Ctrl-C to stop\n")

        def on_measurement(m: LidarMeasurement):
            print(f"  {m.range:>6} mm  ({m.range / 1000:.3f} m)")

        signal.signal(signal.SIGINT, lambda *_: engine.change_stream_state())

        t = threading.Thread(target=engine.lidar_stream, args=(on_measurement, 50.0))
        t.start()
        t.join()
