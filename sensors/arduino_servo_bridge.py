"""
arduino_servo_bridge.py
Pi-side interface for receiving LiDAR-gimbal position from the RF Arduino
(which drives the gimbal autonomously) over Ethernet (UDP), exposing az/el
current_angle proxies compatible with the LidarEngine and scanner pipeline.

Arduino → Pi  (gimbal position feedback):
    One int32 per UDP packet, little-endian, one port per axis:
        lidar_azimuth    -> arduinos.rf.ports.lidar_azimuth
        lidar_elevation  -> arduinos.rf.ports.lidar_elevation

    ASSUMPTION: the int32 is millidegrees (degrees x1000), so 45.300° ->
    45300. If the Arduino actually sends whole integer degrees, set
    ANGLE_SCALE = 1 below. (The old feedback packet was float32 degrees;
    millidegrees preserves that sub-degree precision in an int.)

There is no Pi → Arduino command channel — the gimbal sweeps on its own.

Ports are read from network/config.yaml (arduinos.rf.ports) — edit that
file, not this one, when an address changes.
"""

import select
import socket
import struct
import sys
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from network.network_config import arduino as arduino_config

_rf          = arduino_config("rf")
AZ_PORT      = _rf["ports"]["lidar_azimuth"]
EL_PORT      = _rf["ports"]["lidar_elevation"]
RECV_TIMEOUT = 1.0      # seconds — allows clean shutdown between retries
ANGLE_SCALE  = 1000.0   # int32 millidegrees -> degrees (see module docstring)


class ArduinoServoClient:
    """
    Receives gimbal az/el position (one int32 per port) and exposes the
    angles in degrees as thread-safe properties. Call listen() in a
    dedicated thread.
    """

    def __init__(self, az_port: int = AZ_PORT, el_port: int = EL_PORT):
        self._running = False
        self._lock    = threading.Lock()
        self._az      = 0.0
        self._el      = 0.0

        self._az_sock = self._bind(az_port)
        self._el_sock = self._bind(el_port)

    @staticmethod
    def _bind(port: int) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", port))
        return sock

    @property
    def az_current_angle(self) -> float:
        with self._lock:
            return self._az

    @property
    def el_current_angle(self) -> float:
        with self._lock:
            return self._el

    def listen(self) -> None:
        """Block and receive position packets until stop() is called."""
        self._running = True
        socks = [self._az_sock, self._el_sock]
        while self._running:
            ready, _, _ = select.select(socks, [], [], RECV_TIMEOUT)
            for sock in ready:
                data, _ = sock.recvfrom(4)
                if len(data) < 4:
                    continue
                value,  = struct.unpack("<i", data[:4])
                angle   = value / ANGLE_SCALE
                with self._lock:
                    if sock is self._az_sock:
                        self._az = angle
                    else:
                        self._el = angle

    def stop(self) -> None:
        self._running = False

    def close(self) -> None:
        self.stop()
        self._az_sock.close()
        self._el_sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


class AzimuthProxy:
    """
    Azimuth angle proxy. current_angle is sourced from live RF Arduino
    feedback (arduinos.rf.ports.lidar_azimuth) — the Pi no longer drives
    the gimbal over GPIO.
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
    Elevation angle proxy. current_angle is sourced from live RF Arduino
    feedback (arduinos.rf.ports.lidar_elevation) — the Pi no longer drives
    the gimbal over GPIO.
    """

    def __init__(self, client: ArduinoServoClient):
        self._client = client

    @property
    def current_angle(self) -> float:
        return self._client.el_current_angle

    @property
    def sweep_running(self) -> bool:
        return self._client._running
