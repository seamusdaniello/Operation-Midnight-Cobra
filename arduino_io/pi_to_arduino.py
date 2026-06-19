"""
pi_to_arduino.py
Sends LiDAR scan readings from the Pi to the LiDAR-sink Arduino over UDP.

Packet (13 bytes, little-endian):
    [0:4]  az_millideg  int32   azimuth, degrees x1000  (0 — no servos)
    [4:8]  el_millideg  int32   elevation, degrees x1000 (0 — no servos)
    [8:12] range_mm     int32   range in millimetres
    [12]   detected     uint8   1 = valid return within 90 m, 0 = no return

IP/port are read from network/config.yaml (arduinos.lidar_sink) — edit
that file, not this one, when the address changes.
"""

import math
import socket
import struct
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from lidar_collect import LidarEngine, LidarMeasurement
from network.network_config import arduino as arduino_config

_lidar_sink   = arduino_config("lidar_sink")
ARDUINO_IP    = _lidar_sink["ip"]
ARDUINO_PORT  = _lidar_sink["port"]


def pack_measurement(m: LidarMeasurement) -> bytes:
    return struct.pack(
        "<iiiB",
        round(math.degrees(m.az_rad) * 1000),
        round(math.degrees(m.el_rad) * 1000),
        m.range,
        int(m.detected),
    )


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    lidar_engine = LidarEngine()
    threading.Thread(target=lidar_engine.lidar_stream, daemon=True).start()

    print("[+] Streaming LiDAR measurements — Ctrl-C to stop")
    try:
        while True:
            for m in lidar_engine.drain_buffer():
                sock.sendto(pack_measurement(m), (ARDUINO_IP, ARDUINO_PORT))
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_engine.close()
        sock.close()


if __name__ == "__main__":
    main()
