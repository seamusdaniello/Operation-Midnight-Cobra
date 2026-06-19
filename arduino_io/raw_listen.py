"""
raw_listen.py
Dumb raw UDP listener — binds a port and prints whatever arrives.

Usage:
    python3 raw_listen.py 55020
"""

import socket
import sys

port = int(sys.argv[1]) if len(sys.argv) > 1 else 55020

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", port))
print(f"[+] Listening on UDP {port} — Ctrl-C to stop")

while True:
    data, addr = sock.recvfrom(1024)
    print(f"[{addr[0]}] {len(data)} bytes: {list(data)}")
