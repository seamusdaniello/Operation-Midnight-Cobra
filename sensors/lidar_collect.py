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

import math
import serial
import serial.tools.list_ports
import threading
import time
from typing import Callable, Optional

BAUD = 115200
READ_CMD = b"?LD\r\n"

LIGHTWARE_VID = 0x0403  # FTDI — used on all Lightware SF series


def find_port() -> str:
    """Auto-detect the Lightware LiDAR by USB vendor ID."""
    candidates = [
        p for p in serial.tools.list_ports.comports()
        if p.vid == LIGHTWARE_VID or (p.manufacturer or "").lower() == "lightware"
    ]
    if not candidates:
        raise RuntimeError(
            "Lightware LiDAR not found. Check the USB connection and try again."
        )
    if len(candidates) > 1:
        print(f"[!] Multiple FTDI devices found, using {candidates[0].device}")
    return candidates[0].device


DETECTION_THRESHOLD_MM = 90_000  # 90 metres


class LidarMeasurement:
    """A single range reading stamped with the servo angles at collection time."""

    def __init__(self, range_mm: int, az_rad: float = 0.0, el_rad: float = 0.0):
        self.range    = range_mm
        self.az_rad   = az_rad
        self.el_rad   = el_rad
        # A valid return is a non-negative range under the threshold. The lower
        # bound rejects error sentinels (the LiDAR reports -1, but anything
        # negative — or NaN — fails this) so they never reach the Kalman filter
        # via drain_as_tracker_input, which gates on this flag.
        self.detected = 0 <= range_mm < DETECTION_THRESHOLD_MM

    def __repr__(self) -> str:
        return (
            f"LidarMeasurement(range={self.range}mm, detected={self.detected}, "
            f"az={math.degrees(self.az_rad):.2f}°, el={math.degrees(self.el_rad):.2f}°)"
        )


class LidarEngine:
    def __init__(self, port: str = None, baud: int = BAUD, az_engine=None, el_engine=None):
        port = port or find_port()
        print(f"[+] LiDAR detected on {port}")
        self._ser = serial.Serial(port, baud, timeout=0.5)
        self._ser.reset_input_buffer()
        self._lock = threading.Lock()
        self._buffer = []
        self._buffer_lock = threading.Lock()
        self.stream_running = False
        self._az_engine = az_engine   # AzimuthProxy   – read .current_angle (deg), RF Arduino feedback
        self._el_engine = el_engine   # ElevationProxy – read .current_angle (deg), RF Arduino feedback

    def drain_buffer(self) -> list:
        """Return all buffered measurements since the last drain and clear the buffer."""
        with self._buffer_lock:
            data = list(self._buffer)
            self._buffer.clear()
            return data

    def drain_as_tracker_input(self):
        """
        Drain the buffer and return a (M, 3) float64 array of
        [az_rad, el_rad, range_m] rows, one per valid detection.

        Measurements where detected=False (no return within 90 m) are rejected
        here — before they can reach the filter — rather than relying on the
        tracker's range gate as a fallback.
        """
        import numpy as np
        raw_batch = self.drain_buffer()
        rows = [
            [m.az_rad, m.el_rad, m.range / 1000.0]
            for m in raw_batch
            if m.detected
        ]
        return np.array(rows, dtype=np.float64) if rows else np.empty((0, 3), dtype=np.float64)

    def lidar_collection(self) -> str:
        """Send a poll command and return the raw response string."""
        with self._lock:
            self._ser.reset_input_buffer()
            self._ser.write(READ_CMD)
            return self._ser.readline().decode("ascii", errors="ignore").strip()

    def lidar_formatting(self, raw: str, az_rad: float = 0.0, el_rad: float = 0.0) -> Optional[LidarMeasurement]:
        """Parse a raw response into a LidarMeasurement. Returns None on bad data."""
        if ":" not in raw:
            return None
        try:
            meters = float(raw.split(":")[-1])
            return LidarMeasurement(int(meters * 1000), az_rad=az_rad, el_rad=el_rad)
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

            # Snapshot servo angles immediately after the response arrives —
            # before any other work — to minimise pointing error.
            az_rad = math.radians(self._az_engine.current_angle) if self._az_engine else 0.0
            el_rad = math.radians(self._el_engine.current_angle) if self._el_engine else 0.0

            measurement = self.lidar_formatting(raw, az_rad=az_rad, el_rad=el_rad)

            if measurement is not None:
                with self._buffer_lock:
                    self._buffer.append(measurement)
                if callback is not None:
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
