"""
fcp_uplink.py
Formats confirmed-track output from the EKF tracker into the FCP's
'positional' message schema and ships it out. Serial (/dev/ttyACM0) is
primary; UDP is a fallback used only when the serial link isn't available.

Message schema (per the FCP's _process_dnn_message, msg_type="positional"):
    {
      "msg_type": "positional",
      "rat_id": "<track id, string>",
      "zone": 1|2|3,            # 1=outer/85m  2=middle/60m  3=central/30m
      "values": {"az_value", "el_value", "range_value"},
      "rates":  {"az_rate", "el_rate", "range_rate"},
      "current_time": <unix epoch seconds>,
    }

ASSUMPTIONS (unverified against the FCP repo — fix here if wrong):
  - az/el/range and their rates are passed through in the tracker's native
    units (radians, metres, rad/s, m/s), not degrees.
  - current_time is a Unix epoch float.
  - UDP fallback target (network/config.yaml: fcp.udp_fallback) defaults to
    loopback; point it at the real FCP host if that's a separate machine.

Tracks beyond the outermost zone (85 m) aren't reported — the FCP doesn't
model anything past zone 1.
"""

import json
import socket
import sys
import time
from pathlib import Path
from typing import Optional

import serial

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from network.network_config import fcp as fcp_config

ZONE_3_CENTRAL_M = 30.0   # engage-eligible
ZONE_2_MIDDLE_M  = 60.0
ZONE_1_OUTER_M   = 85.0


def zone_for_range(range_m: float) -> Optional[int]:
    """Classify a range into the FCP's engagement zones; None if beyond all of them."""
    if range_m <= ZONE_3_CENTRAL_M:
        return 3
    if range_m <= ZONE_2_MIDDLE_M:
        return 2
    if range_m <= ZONE_1_OUTER_M:
        return 1
    return None


def build_positional_message(
    track_id:   int,
    az_rad:     float, el_rad:     float, range_m:    float,
    az_rate:    float, el_rate:    float, range_rate: float,
) -> Optional[dict]:
    """Build one 'positional' message for a confirmed track; None if it's outside every zone."""
    zone = zone_for_range(range_m)
    if zone is None:
        return None

    return {
        "msg_type": "positional",
        "rat_id":   str(track_id),
        "zone":     zone,
        "values": {
            "az_value":    az_rad,
            "el_value":    el_rad,
            "range_value": range_m,
        },
        "rates": {
            "az_rate":    az_rate,
            "el_rate":    el_rate,
            "range_rate": range_rate,
        },
        "current_time": time.time(),
    }


class FCPUplink:
    """
    Sends positional/health messages to the FCP. Serial is primary; if the
    serial link can't be opened, or a write to it fails, falls back to UDP
    for that send (and every send thereafter, until reopened).
    """

    def __init__(self, config: Optional[dict] = None):
        config = config or fcp_config()
        self._serial_port = config["serial"]["port"]
        self._baud        = config["serial"]["baud"]
        self._udp_addr    = (config["udp_fallback"]["ip"], config["udp_fallback"]["port"])

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._ser: Optional[serial.Serial] = self._try_open_serial()

    def _try_open_serial(self) -> Optional[serial.Serial]:
        try:
            return serial.Serial(self._serial_port, self._baud, timeout=0.5)
        except serial.SerialException as e:
            print(f"[!] FCP serial link unavailable ({e}) — falling back to UDP {self._udp_addr}")
            return None

    def send(self, message: dict) -> None:
        payload = (json.dumps(message) + "\n").encode("utf-8")

        if self._ser is not None:
            try:
                self._ser.write(payload)
                return
            except serial.SerialException as e:
                print(f"[!] FCP serial write failed ({e}) — falling back to UDP {self._udp_addr}")
                self._ser.close()
                self._ser = None

        self._sock.sendto(payload, self._udp_addr)

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()
        self._sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
