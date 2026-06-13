# Operation Midnight Cobra

Multi-target drone tracking system. A Lightware SF20 LiDAR mounted on az/el servos scans a 90 m airspace, streams detections to a Simulink UDP Receive block, and receives RF contact reports back from an Arduino sensor node. An EKF tracker with GNN data association runs on the Pi and maintains confirmed track estimates in spherical coordinates.

---

## Hardware

| Component | Role |
|---|---|
| Raspberry Pi (eth0 → 192.168.50.10) | Runs tracker, LiDAR driver, UDP sender/receiver |
| Lightware SF20 LiDAR | Range sensor on `/dev/ttyUSB0` |
| Azimuth servo (GPIO 17) | 0–90° sine sweep, ~1.44 s period |
| Elevation servo (GPIO 27) | ±60° sine sweep, ~62.8 s period |
| Arduino + Ethernet shield (192.168.50.20) | RF detection node |
| Cisco switch | Connects all nodes on 192.168.50.0/24 |
| Simulink host (192.168.50.20, port 5005) | UDP Receive block for LiDAR stream |

---

## File Overview

| File | What it does |
|---|---|
| `multi_rat_tracker.py` | EKF tracker — state in spherical coords, GNN assignment |
| `lidar_collect.py` | Lightware SF20 driver — polls, parses, buffers measurements |
| `pi_to_arduino.py` | Drains LiDAR buffer, packs 13-byte UDP payload, sends to Simulink |
| `rf_receiver.py` | Listens on port 5006 for RF contact reports from the Arduino |
| `azimuth_servo_sweep.py` | Sine-wave azimuth sweep engine |
| `elevation_servo_sweep.py` | Sine-wave elevation sweep engine |
| `arduino_receiver.ino` | Arduino sketch — receives LiDAR stream, prints to Serial |
| `test_suite.py` | 73-test pytest suite covering all Python modules |

---

## Dependencies

```bash
pip3 install pyserial numpy scipy pytest
```

On the Pi, also install gpiozero and start pigpiod for servo control:

```bash
pip3 install gpiozero
sudo pigpiod
```

---

## Running

### Stream LiDAR to Simulink

```bash
python3 pi_to_arduino.py
```

Detects the SF20 automatically by USB vendor ID and starts streaming 13-byte UDP packets to `192.168.50.20:5005`.

### Receive RF data from Arduino

```bash
python3 rf_receiver.py
```

Listens on `0.0.0.0:5006` and prints each `rf_reading` as it arrives.

### Run the tracker standalone

```python
from multi_rat_tracker import multi_rat_tracker, reset_tracker
import numpy as np

reset_tracker()
measurements = np.array([[0.3, 0.1, 35.0]])   # [az_rad, el_rad, range_m]
track_ids, track_positions = multi_rat_tracker(measurements, sample_time=0.5)
```

---

## UDP Packet Formats

### Pi → Simulink (port 5005) — 13 bytes, little-endian

| Bytes | Field | Type | Notes |
|---|---|---|---|
| 0–3 | az_rad | float32 | Azimuth in radians |
| 4–7 | el_rad | float32 | Elevation in radians |
| 8–11 | range_m | float32 | Range in metres |
| 12 | detected | uint8 | 1 = valid return within 90 m |

### Arduino → Pi (port 5006) — 13 bytes, little-endian

| Bytes | Field | Type | Notes |
|---|---|---|---|
| 0 | rf_detection | uint8 | 1 = RF signal detected |
| 1–4 | rf_strength | int32 | Signal strength in dBm |
| 5–8 | rf_position_az | float32 | Azimuth in radians |
| 9–12 | rf_position_el | float32 | Elevation in radians |

---

## Running the Tests

```bash
pytest tests/ -v
```

Expected output: **73 passed**.

### What the tests cover

| Module | Tests |
|---|---|
| `multi_rat_tracker` | Filter, angle wrapping, F/Q matrix properties, GNN assignment, track lifecycle (confirm, delete, reset, multi-target) |
| `lidar_collect` | `LidarMeasurement` detection boundary, `lidar_formatting` parse/fail cases, buffer drain, `drain_as_tracker_input` filtering and unit conversion |
| `rf_receiver` | `rf_reading` fields, `unpack_rf_reading` roundtrip for all four fields, packet size constant |
| `pi_to_arduino` | `pack_measurement` byte layout, mm→m conversion, detected flag encoding |
| Servo engines | Stub instantiation without hardware, `get_state` keys, sweep angle bounds, toggle behaviour |

No hardware is required to run the tests. Serial ports and GPIO are mocked.

---

## Tracker Tuning

All constants are at the top of `multi_rat_tracker.py`:

| Constant | Default | Effect |
|---|---|---|
| `MAX_DRONE_SPEED_MPS` | 5.0 m/s | Bounds initial velocity uncertainty |
| `MAX_AIRSPACE_RANGE_M` | 90.0 m | Hard gate — detections beyond this dropped |
| `ANGULAR_ACCEL_NOISE_RADPS2` | 0.08 rad/s² | Process noise — increase for more agile targets |
| `RANGE_ACCEL_NOISE_MPS2` | 1.5 m/s² | Radial process noise |
| `GATE_CHI2_99PCT_3DOF` | 11.34 | Gating threshold — lower = tighter gate |
| `HITS_NEEDED_TO_CONFIRM` | 3 | Consecutive hits before a track is confirmed |
| `MISSES_BEFORE_DELETION` | 4 | Consecutive misses before a track is deleted |
