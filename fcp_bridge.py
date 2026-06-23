"""
fcp_bridge.py
Production bridge: streams the LiDAR + live gimbal position, runs the EKF
tracker, and ships every confirmed track to the FCP as a 'positional'
message via integration/fcp_uplink.py.

Usage:
    python3 fcp_bridge.py
"""

import signal
import threading
import time

from sensors.lidar_collect import LidarEngine
from sensors.arduino_servo_bridge import ArduinoServoClient, AzimuthProxy, ElevationProxy
from tracker.multi_rat_tracker import multi_rat_tracker, confirmed_track_states, reset_tracker
from integration.fcp_uplink import build_positional_message, FCPUplink

LIDAR_HZ = 50.0   # LiDAR polling rate
TICK_HZ  = 10.0   # tracker update / uplink rate


def main():
    reset_tracker()

    arduino_client = ArduinoServoClient()
    azimuth        = AzimuthProxy(arduino_client)
    elevation      = ElevationProxy(arduino_client)
    lidar          = LidarEngine(az_engine=azimuth, el_engine=elevation)
    uplink         = FCPUplink()

    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)

    lidar_thread   = threading.Thread(target=lidar.lidar_stream, args=(None, LIDAR_HZ), daemon=True)
    arduino_thread = threading.Thread(target=arduino_client.listen, daemon=True)
    lidar_thread.start()
    arduino_thread.start()

    print(f"[*] FCP bridge running at {TICK_HZ:.0f} Hz — Ctrl-C to stop")

    interval = 1.0 / TICK_HZ
    last = time.time()
    try:
        while running:
            t0 = time.time()
            dt = t0 - last
            last = t0

            measurements = lidar.drain_as_tracker_input()
            multi_rat_tracker(measurements, sample_time=dt)

            for track_id, state in confirmed_track_states():
                az, el, rng, az_rate, el_rate, rng_rate = state
                message = build_positional_message(track_id, az, el, rng, az_rate, el_rate, rng_rate)
                if message is not None:
                    uplink.send(message)

            remaining = interval - (time.time() - t0)
            if remaining > 0:
                time.sleep(remaining)
    finally:
        lidar.stream_running = False
        arduino_client.stop()
        lidar_thread.join(timeout=1.0)
        arduino_thread.join(timeout=1.0)
        lidar.close()
        arduino_client.close()
        uplink.close()


if __name__ == "__main__":
    main()
