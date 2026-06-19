"""
test_arduino_connection.py
Listens for RF contact reports from the Arduino over UDP and decodes them.

One Arduino, 4 separate UDP ports — each port carries a single int32,
little-endian, 4 bytes per packet:
    range      -> network/config.yaml: arduinos.rf.test_ports.range
    azimuth    -> ...test_ports.azimuth
    elevation  -> ...test_ports.elevation
    alive      -> ...test_ports.alive   (0 or 1 heartbeat, not a bool)

Ports are read from network/config.yaml — edit that file, not this one,
when an address changes.

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

TEST_PORTS  = arduino_config("rf")["test_ports"]
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
        print(f"[{addr[0]}] {field:<9} = {value}")


def main():
    print("[+] Listening on 4 ports — Ctrl-C to stop")
    threads = [
        threading.Thread(target=listen_on_port, args=(field, port), daemon=True)
        for field, port in TEST_PORTS.items()
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
