"""
Scanner — fuses LiDAR readings with azimuth/elevation servo positions.

LiDAR runs at 200 Hz; servo positions are sampled at 50 Hz.
At each 50 Hz tick, all LiDAR readings buffered since the last tick are
collected and paired with the current servo angles to form one frame.

Output vectors:
    frames     — list of frames, one per servo step:
                 [[[range_mm, detected], ...], azimuth_deg, elevation_deg]

    detections — list of booleans, one per frame:
                 True if any reading in that window had detected=True

Usage:
    scanner = Scanner()
    scanner.start()          # blocks until Ctrl-C
    print(scanner.frames)
    print(scanner.detections)
"""

import signal
import threading
import time

from lidar_collect import LidarEngine
from azimuth_servo_sweep import AzimuthServoEngine
from elevation_servo_sweep import ElevationServoEngine

LIDAR_HZ  = 200.0  # LiDAR polling rate
STEP_HZ   = 50.0   # scanner frame rate (matches servo update rate)


class Scanner:
    def __init__(self):
        self.lidar    = LidarEngine()
        self.azimuth  = AzimuthServoEngine()
        self.elevation = ElevationServoEngine()

        self.frames: list = []
        self._running = False

    def start(self):
        """Start all subsystems and block until stopped."""
        self._running = True

        lidar_thread   = threading.Thread(target=self._lidar_loop,    daemon=True)
        azimuth_thread = threading.Thread(target=self._azimuth_loop,  daemon=True)
        elev_thread    = threading.Thread(target=self._elev_loop,     daemon=True)

        lidar_thread.start()
        azimuth_thread.start()
        elev_thread.start()

        print("[*] Scanner running — Ctrl-C to stop")
        self._scanner_loop()

        self._running = False
        self.azimuth.sweep_running  = False
        self.elevation.sweep_running = False
        self.lidar.stream_running   = False

        lidar_thread.join()
        azimuth_thread.join()
        elev_thread.join()
        self.lidar.close()
        self.azimuth.close()
        self.elevation.close()

    def stop(self):
        self._running = False

    def _lidar_loop(self):
        self.lidar.lidar_stream(callback=None, sample_rate_hz=LIDAR_HZ)

    def _azimuth_loop(self):
        self.azimuth.sweep()

    def _elev_loop(self):
        self.elevation.sweep()

    def _scanner_loop(self):
        interval = 1.0 / STEP_HZ

        while self._running:
            t0 = time.time()

            readings  = self.lidar.drain_buffer()
            az        = round(self.azimuth.current_angle, 3)
            el        = round(self.elevation.current_angle, 3)

            reading_data = [[m.range, m.detected] for m in readings]
            any_detected = any(m.detected for m in readings)
            frame        = [reading_data, az, el, any_detected]

            self.frames.append(frame)

            remaining = interval - (time.time() - t0)
            if remaining > 0:
                time.sleep(remaining)


if __name__ == "__main__":
    scanner = Scanner()
    signal.signal(signal.SIGINT, lambda *_: scanner.stop())
    scanner.start()

    print(f"\n[*] Collected {len(scanner.frames)} frames")

    if scanner.frames:
        f = scanner.frames[0]
        print(f"\nFirst frame:")
        print(f"  Readings:    {f[0]}")
        print(f"  Azimuth:     {f[1]}°")
        print(f"  Elevation:   {f[2]}°")
        print(f"  Detected:    {f[3]}")
