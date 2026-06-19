"""
test_arduino_connection.py
Listens for the RF Arduino's UDP feeds and decodes them.

One Arduino, one int32 per port (little-endian, 4 bytes). Ports and field
names are read straight from network/config.yaml (arduinos.rf.ports) —
edit that file, not this one, when anything changes. Currently:
    rf_range, rf_azimuth, rf_elevation, lidar_azimuth, lidar_elevation,
    alive (0/1 heartbeat, not a bool).

Usage:
    python3 test_arduino_connection.py
"""

import socket
import struct
import sys
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from network.network_config import arduino as arduino_config

PORTS       = arduino_config("rf")["ports"]
PACKET_SIZE = 4   # one int32


def listen_on_port(field: str, port: int) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    print(f"[+] Listening for {field} on UDP {port}")

    while True:
        data, addr = sock.recvfrom(1024)
        if len(data) < PACKET_SIZE:
            print(f"[{field}:{port}] short packet ({len(data)} bytes): {list(data)}")
            continue

        value, = struct.unpack("<i", data[:PACKET_SIZE])
        print(f"[{addr[0]}] {field:<15} = {value}")


def main():
    print(f"[+] Listening on {len(PORTS)} ports — Ctrl-C to stop")
    threads = [
        threading.Thread(target=listen_on_port, args=(field, port), daemon=True)
        for field, port in PORTS.items()
    ]
    for t in threads:
        t.start()

    try:
        for t in threads:
            t.join()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
