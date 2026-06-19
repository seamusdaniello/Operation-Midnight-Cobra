"""
arduino_servo_bridge.py
Pi-side interface for receiving servo position feedback from the Arduino
over Ethernet (UDP) and exposing az/el current_angle proxies compatible
with the existing LidarEngine and scanner pipeline.

Arduino → Pi  (position feedback):
    UDP packet, 8 bytes, little-endian:
        [0:4]  az_deg   float32   azimuth angle in degrees
        [4:8]  el_deg   float32   elevation angle in degrees
    Arduino sends to PI_IP:FEEDBACK_PORT at ~100 Hz.

Pi → Arduino  (sweep commands):
    UDP packet, 1 byte:
        0x01 = start sweep
        0x00 = stop sweep
    Pi sends to ARDUINO_IP:COMMAND_PORT.

IP/ports are read from network/config.yaml (arduinos.servo_bridge) — edit
that file, not this one, when an address changes.
"""

import socket
import struct
import sys
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from network.network_config import arduino as arduino_config

_servo_bridge = arduino_config("servo_bridge")
ARDUINO_IP    = _servo_bridge["ip"]
FEEDBACK_PORT = _servo_bridge["feedback_port"]   # Pi listens here for position packets
COMMAND_PORT  = _servo_bridge["command_port"]    # Pi sends start/stop here
RECV_TIMEOUT  = 1.0    # seconds — allows clean shutdown between retries


class ArduinoServoClient:
    """
    Receives az/el position packets from the Arduino and exposes them
    as thread-safe properties. Call listen() in a dedicated thread.
    """

    def __init__(
        self,
        arduino_ip: str = ARDUINO_IP,
        feedback_port: int = FEEDBACK_PORT,
        command_port: int = COMMAND_PORT,
    ):
        self._arduino_ip   = arduino_ip
        self._command_port = command_port
        self._running      = False
        self._lock         = threading.Lock()
        self._az           = 0.0
        self._el           = 0.0

        self._recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._recv_sock.bind(("", feedback_port))
        self._recv_sock.settimeout(RECV_TIMEOUT)

        self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    @property
    def az_current_angle(self) -> float:
        with self._lock:
            return self._az

    @property
    def el_current_angle(self) -> float:
        with self._lock:
            return self._el

    def send_command(self, start: bool) -> None:
        """Send start (True) or stop (False) sweep command to the Arduino."""
        self._cmd_sock.sendto(
            b"\x01" if start else b"\x00",
            (self._arduino_ip, self._command_port),
        )

    def listen(self) -> None:
        """Block and receive position packets until stop() is called."""
        self._running = True
        while self._running:
            try:
                data, _ = self._recv_sock.recvfrom(8)
                if len(data) >= 8:
                    az, el = struct.unpack_from("<ff", data)
                    with self._lock:
                        self._az = az
                        self._el = el
            except socket.timeout:
                continue

    def stop(self) -> None:
        self._running = False

    def close(self) -> None:
        self.stop()
        self._recv_sock.close()
        self._cmd_sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


class AzimuthProxy:
    """
    Drop-in replacement for AzimuthServoEngine.
    current_angle is sourced from live Arduino feedback.
    """

    def __init__(self, client: ArduinoServoClient):
        self._client = client

    @property
    def current_angle(self) -> float:
        return self._client.az_current_angle

    @property
    def sweep_running(self) -> bool:
        return self._client._running


class ElevationProxy:
    """
    Drop-in replacement for ElevationServoEngine.
    current_angle is sourced from live Arduino feedback.
    """

    def __init__(self, client: ArduinoServoClient):
        self._client = client

    @property
    def current_angle(self) -> float:
        return self._client.el_current_angle

    @property
    def sweep_running(self) -> bool:
        return self._client._running
