"""
raw_listen.py
Dumb raw UDP listener — binds a port and prints whatever arrives.

Prints the raw bytes, and also decodes a signed little-endian value when the
payload is 2 bytes (int16) or 4 bytes (int32) so sentinels like -1 are
readable instead of showing up as [255, 255].

Usage:
    python3 raw_listen.py 55020
"""

import socket
import struct
import sys

port = int(sys.argv[1]) if len(sys.argv) > 1 else 55020

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", port))
print(f"[+] Listening on UDP {port} — Ctrl-C to stop")

while True:
    data, addr = sock.recvfrom(1024)
    decoded = ""
    if len(data) == 2:
        print("GOT AN INT 16")
        decoded = f"  int16={struct.unpack('<h', data)[0]}"
    elif len(data) == 4:
        print("GOT AN INT 32!!!!!")
        decoded = f"  int32={struct.unpack('<i', data)[0]}"
    print(f"[{addr[0]}] {len(data)} bytes: {list(data)}{decoded}")
