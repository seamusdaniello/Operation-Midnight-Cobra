"""
pi_to_arduino.py
Sends LiDAR scan readings from the Pi to Simulink over UDP.

Packet (13 bytes, little-endian):
    [0:4]  az_rad   float32   azimuth in radians  (0.0 — no servos)
    [4:8]  el_rad   float32   elevation in radians (0.0 — no servos)
    [8:12] range_m  float32   range in metres
    [12]   detected uint8     1 = valid return within 90 m, 0 = no return

Network:
    Pi      192.168.50.10   (your eth0)
    Simulink 192.168.50.20  (UDP Receive block, port 5005)
"""

import socket
import struct
import threading
import time

from lidar_collect import LidarEngine, LidarMeasurement

SIMULINK_IP   = "192.168.50.20"
SIMULINK_PORT = 5005


def pack_measurement(m: LidarMeasurement) -> bytes:
    return struct.pack(
        "<fffB",
        m.az_rad,
        m.el_rad,
        m.range / 1000.0,   # mm -> m
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
                sock.sendto(pack_measurement(m), (SIMULINK_IP, SIMULINK_PORT))
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_engine.close()
        sock.close()


if __name__ == "__main__":
    main()
