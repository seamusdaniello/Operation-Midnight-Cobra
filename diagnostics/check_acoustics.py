"""
check_acoustics.py
Dump whatever the two acoustic Arduinos send to the Pi. Binds each board's
port and prints every packet exactly as it arrives — source, byte count,
raw bytes, and an int16 (little-endian) decode when the payload is big
enough.

Edit TARGETS to match your wiring, then:
    python3 check_acoustics.py
"""

import select
import socket
import struct

# (label, port)
TARGETS = [
    ("acoustic-1", 55020),
    ("acoustic-2", 55030),
]


def main():
    by_sock = {}
    socks   = []
    for label, port in TARGETS:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", port))
        socks.append(s)
        by_sock[s] = (label, port)

    ports = ", ".join(str(p) for _, p in TARGETS)
    print(f"[+] Listening on {ports} — Ctrl-C to stop\n")

    try:
        while True:
            ready, _, _ = select.select(socks, [], [], 0.5)
            for s in ready:
                label, port = by_sock[s]
                data, addr = s.recvfrom(1024)
                line = f"[{label} :{port}] from {addr[0]}  {len(data)} bytes: {list(data)}"
                if len(data) >= 2:
                    value, = struct.unpack("<h", data[:2])
                    line += f"  int16={value}"
                print(line)
    except KeyboardInterrupt:
        pass
    finally:
        for s in socks:
            s.close()


if __name__ == "__main__":
    main()
