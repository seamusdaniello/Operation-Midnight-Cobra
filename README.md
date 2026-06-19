# Operation Midnight Cobra

Multi-target drone tracking system. A Lightware SF20 LiDAR mounted on az/el servos scans a 90 m airspace and streams detections onward over UDP, while a team of Arduino sensor/actuator nodes report back RF contacts and servo position. An EKF tracker with GNN data association runs on the Pi and maintains confirmed track estimates in spherical coordinates.

---

## Hardware

| Component | Role |
|---|---|
| Raspberry Pi (eth0 → `192.168.50.10`, static) | Runs tracker, LiDAR driver, UDP sender/receiver |
| Lightware SF20 LiDAR | Range sensor on `/dev/ttyUSB0` |
| Cisco switch + Ethernet splitter | Connects Pi + Arduino team on `192.168.50.0/24` — static IPs only, no DHCP |
| Arduino team (6 boards, Ethernet shields) | 1 RF (also drives the LiDAR gimbal) + 5 acoustic — see `network/config.yaml` |

All Arduino IPs/ports are defined in one place: [`network/config.yaml`](network/config.yaml). Edit that file when hardware changes — the Python scripts read from it instead of hardcoding addresses. Run [`network/check_network.sh`](network/check_network.sh) to ping every configured Arduino and check the Pi's own interface.

---

## Layout

```
scanner.py, lidar_collect.py   — core pipeline (root)
tracker/        — EKF tracker + its smoke test
  multi_rat_tracker.py
  test_tracker.py
arduino_io/     — everything that talks to an Arduino over UDP
  rf_receiver.py               — RF arduino -> Pi (production listener)
  arduino_servo_bridge.py      — RF arduino -> Pi (LiDAR-gimbal az/el position)
  test_arduino_connection.py   — RF arduino -> Pi (decode/debug listener)
network/        — network config + ops tooling
  config.yaml          — single source of truth for every static IP/port
  network_config.py    — loader used by the arduino_io scripts
  check_network.sh      — ping health check
tests/
  test_suite.py   — pytest suite covering all Python modules
```

`scanner.py` stays at the repo root and fuses LiDAR readings with servo position into per-frame output.

---

## Dependencies

```bash
pip3 install pyserial numpy scipy pytest pyyaml
```

`pyyaml` backs the `network/config.yaml` loader; prebuilt wheels exist for Raspberry Pi OS (ARM), no compiling needed.

---

## Running

### Receive RF data from the Arduino

```bash
python3 arduino_io/rf_receiver.py
```

Listens on `arduinos.rf.data_port` (from config) and prints each `rf_reading` as it arrives.

### Decode/debug the RF connection directly

```bash
python3 arduino_io/test_arduino_connection.py
```

Binds every port under `arduinos.rf.ports` (one thread each) and prints each int32 as it arrives, labelled by field — useful when bringing up the RF Arduino link without going through the full tracker pipeline.

### Run the tracker standalone

```python
from tracker.multi_rat_tracker import multi_rat_tracker, reset_tracker
import numpy as np

reset_tracker()
measurements = np.array([[0.3, 0.1, 35.0]])   # [az_rad, el_rad, range_m]
track_ids, track_positions = multi_rat_tracker(measurements, sample_time=0.5)
```

---

## UDP Packet Formats

### RF Arduino → Pi, production (`arduinos.rf.data_port`, 5006) — 13 bytes, little-endian

| Bytes | Field | Type | Notes |
|---|---|---|---|
| 0 | rf_detection | uint8 | 1 = RF signal detected |
| 1–4 | rf_strength | int32 | Signal strength in dBm |
| 5–8 | rf_position_az | float32 | Azimuth in radians |
| 9–12 | rf_position_el | float32 | Elevation in radians |

### RF Arduino → Pi (`arduinos.rf.ports`) — one int32 per port, 4 bytes, little-endian

One UDP port per field; `test_arduino_connection.py` listens on all of them.

| Port | Field |
|---|---|
| 55001 | rf_range |
| 55002 | rf_azimuth |
| 55003 | rf_elevation |
| 55004 | lidar_azimuth |
| 55005 | lidar_elevation |
| 55006 | alive (0/1 heartbeat) |

### RF Arduino → Pi, LiDAR-gimbal position (`arduinos.rf.ports`) — one int32 per port

The RF Arduino drives the gimbal autonomously and reports its angle. One
int32 per port, little-endian, assumed millidegrees (degrees × 1000);
`arduino_servo_bridge.py` divides by 1000 to expose degrees. There is no
Pi → Arduino command channel.

| Port | Field |
|---|---|
| 55004 | lidar_azimuth |
| 55005 | lidar_elevation |

---

## Running the Tests

```bash
pytest tests/ -v
```

Expected output: **65 passed**.

### What the tests cover

| Module | Tests |
|---|---|
| `multi_rat_tracker` | Filter, angle wrapping, F/Q matrix properties, GNN assignment, track lifecycle (confirm, delete, reset, multi-target) |
| `lidar_collect` | `LidarMeasurement` detection boundary, `lidar_formatting` parse/fail cases, buffer drain, `drain_as_tracker_input` filtering and unit conversion |
| `rf_receiver` | `rf_reading` fields, `unpack_rf_reading` roundtrip for all four fields, packet size constant |

No hardware is required to run the tests. Serial ports and GPIO are mocked.

---

## Tracker Tuning

All constants are at the top of `tracker/multi_rat_tracker.py`:

| Constant | Default | Effect |
|---|---|---|
| `MAX_DRONE_SPEED_MPS` | 5.0 m/s | Bounds initial velocity uncertainty |
| `MAX_AIRSPACE_RANGE_M` | 90.0 m | Hard gate — detections beyond this dropped |
| `ANGULAR_ACCEL_NOISE_RADPS2` | 0.08 rad/s² | Process noise — increase for more agile targets |
| `RANGE_ACCEL_NOISE_MPS2` | 1.5 m/s² | Radial process noise |
| `GATE_CHI2_99PCT_3DOF` | 11.34 | Gating threshold — lower = tighter gate |
| `HITS_NEEDED_TO_CONFIRM` | 3 | Consecutive hits before a track is confirmed |
| `MISSES_BEFORE_DELETION` | 4 | Consecutive misses before a track is deleted |
