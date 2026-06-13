"""
rf_receiver.py
Receives RF detection data from the Arduino over UDP.

Packet (13 bytes, little-endian):
    [0]    rf_detection   uint8    1 = RF detected, 0 = no detection
    [1:5]  rf_strength    int32    signal strength (dBm)
    [5:9]  rf_position_az float32  azimuth in radians
    [9:13] rf_position_el float32  elevation in radians

Network:
    Arduino  192.168.50.20  (sender)
    Pi       192.168.50.10  (listener, port 5006)
"""

import socket
import struct
from dataclasses import dataclass

LISTEN_IP   = "0.0.0.0"
LISTEN_PORT = 5006
PACKET_SIZE = 13


@dataclass
class rf_reading:
    rf_detection:   bool
    rf_strength:    int
    rf_position_az: float
    rf_position_el: float


def unpack_rf_reading(raw_bytes: bytes) -> rf_reading:
    rf_detection_raw, rf_strength, rf_position_az, rf_position_el = struct.unpack(
        "<Biff", raw_bytes
    )
    return rf_reading(
        rf_detection   = bool(rf_detection_raw),
        rf_strength    = rf_strength,
        rf_position_az = rf_position_az,
        rf_position_el = rf_position_el,
    )


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    print(f"[+] Listening for RF data on port {LISTEN_PORT} — Ctrl-C to stop")

    try:
        while True:
            raw_bytes, sender_addr = sock.recvfrom(1024)
            if len(raw_bytes) < PACKET_SIZE:
                continue
            reading = unpack_rf_reading(raw_bytes)
            print(reading)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()


if __name__ == "__main__":
    main()
