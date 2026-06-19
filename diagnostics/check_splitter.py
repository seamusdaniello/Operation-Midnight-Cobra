"""
check_splitter.py
Test that the Pi hears from multiple Arduinos through the Ethernet splitter.

Listens on every Arduino port defined in network/config.yaml and reports,
live, which source IPs are sending. Wire the Pi + 2 Arduinos into the
splitter, run this, and you should see two distinct senders appear.

Usage:
    python3 check_splitter.py
"""

import select
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from network.network_config import load_config


def all_ports() -> set:
    """Every integer port found under any arduino in config.yaml."""
    ports = set()
    for board in load_config()["arduinos"].values():
        for value in board.values():
            if isinstance(value, int):
                ports.add(value)
            elif isinstance(value, dict):            # e.g. the 'ports' sub-map
                ports.update(v for v in value.values() if isinstance(v, int))
    return ports


def main():
    ports = sorted(all_ports())
    socks = []
    for port in ports:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", port))
        socks.append(s)

    print(f"[+] Listening on {len(ports)} ports — Ctrl-C to stop")
    print(f"    {ports}\n")

    counts = {}          # source_ip -> packet count
    next_report = time.time() + 2.0

    try:
        while True:
            ready, _, _ = select.select(socks, [], [], 0.5)
            for s in ready:
                _, addr = s.recvfrom(1024)
                ip = addr[0]
                if ip not in counts:
                    print(f"[+] NEW sender: {ip}")
                counts[ip] = counts.get(ip, 0) + 1

            if time.time() >= next_report:
                senders = ", ".join(f"{ip} ({n})" for ip, n in sorted(counts.items()))
                print(f"    {len(counts)} sender(s): {senders or 'none yet'}")
                next_report = time.time() + 2.0
    except KeyboardInterrupt:
        print(f"\n[=] Done. Heard from {len(counts)} distinct sender(s):")
        for ip, n in sorted(counts.items()):
            print(f"      {ip}  —  {n} packets")
    finally:
        for s in socks:
            s.close()


if __name__ == "__main__":
    main()
