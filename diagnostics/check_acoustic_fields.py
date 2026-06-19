"""
check_acoustic_fields.py
Listen to every acoustic board's field ports (az, range, detect, mic_id,
board_id, zone) defined in network/config.yaml and print each value as it
arrives, labelled by board and field.

Works with any subset of boards powered on — ports for boards that aren't
sending are still bound, they just stay silent and never appear. A port
that can't be bound (e.g. already in use) is skipped with a warning rather
than crashing the whole script.

Acoustic data is int16, little-endian.

Usage:
    python3 check_acoustic_fields.py
"""

import select
import socket
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from network.network_config import load_config


def acoustic_ports() -> dict:
    """Map each acoustic field port -> (board_name, field_name)."""
    mapping = {}
    for name, info in load_config()["arduinos"].items():
        if not name.startswith("acoustic"):
            continue
        for field, port in info.get("ports", {}).items():
            mapping[port] = (name, field)
    return mapping


def main():
    by_sock = {}
    socks   = []
    for port, (board, field) in sorted(acoustic_ports().items()):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("0.0.0.0", port))
        except OSError as e:
            print(f"[!] skipping port {port} ({board} {field}): {e}")
            s.close()
            continue
        socks.append(s)
        by_sock[s] = (board, field)

    print(f"[+] Listening on {len(socks)} acoustic ports — Ctrl-C to stop")
    print("    (boards that aren't sending just won't appear)\n")

    try:
        while True:
            ready, _, _ = select.select(socks, [], [], 0.5)
            for s in ready:
                board, field = by_sock[s]
                data, addr = s.recvfrom(1024)
                if len(data) >= 2:
                    value, = struct.unpack("<h", data[:2])
                    print(f"[{board}] from {addr[0]}  {field:<9} = {value}")
                else:
                    print(f"[{board}] from {addr[0]}  {field:<9}  {len(data)} bytes: {list(data)}")
    except KeyboardInterrupt:
        pass
    finally:
        for s in socks:
            s.close()


if __name__ == "__main__":
    main()
