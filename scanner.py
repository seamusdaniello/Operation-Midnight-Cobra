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
from arduino_io.arduino_servo_bridge import ArduinoServoClient, AzimuthProxy, ElevationProxy

LIDAR_HZ  = 200.0  # LiDAR polling rate
STEP_HZ   = 50.0   # scanner frame rate (matches servo update rate)


class Scanner:
    def __init__(self):
        self.arduino_client = ArduinoServoClient()
        self.azimuth        = AzimuthProxy(self.arduino_client)
        self.elevation      = ElevationProxy(self.arduino_client)
        self.lidar          = LidarEngine(az_engine=self.azimuth, el_engine=self.elevation)

        self.frames: list = []
        self._running = False

    def start(self):
        """Start all subsystems and block until stopped."""
        self._running = True

        lidar_thread   = threading.Thread(target=self._lidar_loop,   daemon=True)
        arduino_thread = threading.Thread(target=self._arduino_loop, daemon=True)

        lidar_thread.start()
        arduino_thread.start()

        self.arduino_client.send_command(start=True)

        print("[*] Scanner running — Ctrl-C to stop")
        self._scanner_loop()

        self._running = False
        self.arduino_client.send_command(start=False)
        self.arduino_client.stop()
        self.lidar.stream_running = False

        lidar_thread.join()
        arduino_thread.join()
        self.lidar.close()
        self.arduino_client.close()

    def stop(self):
        self._running = False

    def _lidar_loop(self):
        self.lidar.lidar_stream(callback=None, sample_rate_hz=LIDAR_HZ)

    def _arduino_loop(self):
        self.arduino_client.listen()

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
