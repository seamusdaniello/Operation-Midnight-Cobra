"""
lidar_track_demo.py
Live LiDAR -> Kalman demo. Streams the LiDAR, feeds each batch of valid
detections through the multi-target EKF tracker, and prints the raw
measurement next to the resulting confirmed track output every tick.

No servos required — runs the LiDAR standalone, so az/el are stamped 0 and
every return shows up dead ahead (range is the interesting axis). Negative /
error returns are already filtered out by LidarMeasurement.detected, so they
never reach the filter (see drain_as_tracker_input).

Note: a track is only reported once it's CONFIRMED (HITS_NEEDED_TO_CONFIRM
consecutive hits), so the first couple of ticks show a measurement with no
Kalman output yet — that's the tracker building confidence, not a bug.

Usage:
    python3 lidar_track_demo.py
"""

import math
import signal
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from sensors.lidar_collect import LidarEngine
from tracker.multi_rat_tracker import multi_rat_tracker, reset_tracker

LIDAR_HZ = 50.0   # LiDAR polling rate
TICK_HZ  = 10.0   # tracker update rate (one EKF cycle per tick)


def _fmt_row(az_rad: float, el_rad: float, range_m: float) -> str:
    return (f"az={math.degrees(az_rad):7.2f}°  "
            f"el={math.degrees(el_rad):7.2f}°  "
            f"range={range_m:7.3f} m")


def main():
    reset_tracker()
    engine = LidarEngine()

    lidar_thread = threading.Thread(
        target=engine.lidar_stream, args=(None, LIDAR_HZ), daemon=True
    )
    lidar_thread.start()

    signal.signal(signal.SIGINT, lambda *_: setattr(engine, "stream_running", False))

    print(f"[*] LiDAR -> Kalman demo running at {TICK_HZ:.0f} Hz — Ctrl-C to stop\n")

    interval = 1.0 / TICK_HZ
    last = time.time()
    try:
        while engine.stream_running:
            t0 = time.time()
            dt = t0 - last
            last = t0

            measurements = engine.drain_as_tracker_input()   # (M, 3) valid detections only
            ids, positions = multi_rat_tracker(measurements, sample_time=dt)

            print(f"[t=+{dt:0.3f}s]")
            if measurements.shape[0]:
                for az, el, rng in measurements:
                    print(f"  LiDAR meas:  {_fmt_row(az, el, rng)}")
            else:
                print("  LiDAR meas:  (no return)")

            if ids.size:
                for tid, (az, el, rng) in zip(ids, positions):
                    print(f"  Kalman  #{tid}: {_fmt_row(az, el, rng)}")
            else:
                print("  Kalman  out: (no confirmed track yet)")
            print()

            remaining = interval - (time.time() - t0)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        engine.stream_running = False
        lidar_thread.join(timeout=1.0)
        engine.close()


if __name__ == "__main__":
    main()
