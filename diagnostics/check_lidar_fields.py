"""
check_lidar_fields.py
Listen to the LiDAR-gimbal field ports the RF Arduino drives (lidar_azimuth,
lidar_elevation) defined in network/config.yaml and print each value as it
arrives, labelled by field.

These ports live in the RF Arduino's block (arduinos.rf.ports), not a board of
their own — this script just filters that block to the "lidar_" fields. Edit
config.yaml, not this file, when anything changes.

A port that's silent simply never appears; one that can't be bound (e.g.
already in use) is skipped with a warning rather than crashing the script.

RF data is int32, little-endian (4 bytes).

Usage:
    python3 check_lidar_fields.py
"""

import select
import socket
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from network.network_config import arduino as arduino_config

PACKET_SIZE = 4   # one int32


def lidar_ports() -> dict:
    """Map each LiDAR-gimbal field port -> field_name."""
    return {
        port: field
        for field, port in arduino_config("rf").get("ports", {}).items()
        if field.startswith("lidar_")
    }


def main():
    by_sock = {}
    socks   = []
    for port, field in sorted(lidar_ports().items()):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("0.0.0.0", port))
        except OSError as e:
            print(f"[!] skipping port {port} ({field}): {e}")
            s.close()
            continue
        socks.append(s)
        by_sock[s] = field

    print(f"[+] Listening on {len(socks)} LiDAR field ports — Ctrl-C to stop")
    print("    (fields that aren't sending just won't appear)\n")

    try:
        while True:
            ready, _, _ = select.select(socks, [], [], 0.5)
            for s in ready:
                field = by_sock[s]
                data, addr = s.recvfrom(1024)
                if len(data) >= PACKET_SIZE:
                    value, = struct.unpack("<i", data[:PACKET_SIZE])
                    print(f"from {addr[0]}  {field:<16} = {value}")
                else:
                    print(f"from {addr[0]}  {field:<16}  {len(data)} bytes: {list(data)}")
    except KeyboardInterrupt:
        pass
    finally:
        for s in socks:
            s.close()


if __name__ == "__main__":
    main()
