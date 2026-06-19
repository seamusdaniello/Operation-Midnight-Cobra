# Operation Midnight Cobra

Multi-target drone tracking system. A Lightware SF20 LiDAR mounted on az/el servos scans a 90 m airspace and streams detections onward over UDP, while a team of Arduino sensor/actuator nodes report back RF contacts and servo position. An EKF tracker with GNN data association runs on the Pi and maintains confirmed track estimates in spherical coordinates.

---

## Hardware

| Component | Role |
|---|---|
| Raspberry Pi (eth0 → `192.168.50.10`, static) | Runs tracker, LiDAR driver, UDP sender/receiver |
| Lightware SF20 LiDAR | Range sensor on `/dev/ttyUSB0` |
| Cisco switch + Ethernet splitter | Connects Pi + Arduino team on `192.168.50.0/24` — static IPs only, no DHCP |
| Arduino team (6 boards, Ethernet shields) | RF detection, az/el servo bridge, LiDAR sink, acoustic, FCP, + 1 TBD — see `network/config.yaml` |

All Arduino IPs/ports are defined in one place: [`network/config.yaml`](network/config.yaml). Edit that file when hardware changes — the Python scripts read from it instead of hardcoding addresses. Run [`network/check_network.sh`](network/check_network.sh) to ping every configured Arduino and check the Pi's own interface.

---

## Layout

```
scanner.py, lidar_collect.py   — core pipeline (root)
tracker/        — EKF tracker + its smoke test
  multi_rat_tracker.py
  test_tracker.py
arduino_io/     — everything that talks to an Arduino over UDP
  pi_to_arduino.py            — Pi -> LiDAR-sink Arduino
  rf_receiver.py               — RF arduino -> Pi (production listener)
  arduino_servo_bridge.py      — Pi <-> servo bridge Arduino (az/el feedback + sweep commands)
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

### Stream LiDAR to the Arduino sink

```bash
python3 arduino_io/pi_to_arduino.py
```

Detects the SF20 automatically by USB vendor ID and streams 13-byte UDP packets to whatever IP/port `network/config.yaml` has under `arduinos.lidar_sink`.

### Receive RF data from the Arduino

```bash
python3 arduino_io/rf_receiver.py
```

Listens on `arduinos.rf.data_port` (from config) and prints each `rf_reading` as it arrives.

### Decode/debug the RF connection directly

```bash
python3 arduino_io/test_arduino_connection.py
```

Listens on `arduinos.rf.test_port` and prints decoded `range_m / az_deg / el_deg / alive` per packet — useful when bringing up the RF Arduino link without going through the full tracker pipeline.

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

### Pi → LiDAR-sink Arduino (`arduinos.lidar_sink`, port 5005) — 13 bytes, little-endian

| Bytes | Field | Type | Notes |
|---|---|---|---|
| 0–3 | az_millideg | int32 | Azimuth, degrees x1000 |
| 4–7 | el_millideg | int32 | Elevation, degrees x1000 |
| 8–11 | range_mm | int32 | Range in millimetres |
| 12 | detected | uint8 | 1 = valid return within 90 m |

### RF Arduino → Pi, production (`arduinos.rf.data_port`, 5006) — 13 bytes, little-endian

| Bytes | Field | Type | Notes |
|---|---|---|---|
| 0 | rf_detection | uint8 | 1 = RF signal detected |
| 1–4 | rf_strength | int32 | Signal strength in dBm |
| 5–8 | rf_position_az | float32 | Azimuth in radians |
| 9–12 | rf_position_el | float32 | Elevation in radians |

### RF Arduino → Pi, debug (`arduinos.rf.test_port`, 55001) — 13 bytes, little-endian

| Bytes | Field | Type | Notes |
|---|---|---|---|
| 0–3 | range_m | float32 | Range in metres |
| 4–7 | az_deg | float32 | Azimuth in degrees |
| 8–11 | el_deg | float32 | Elevation in degrees |
| 12 | alive | uint8 | 1 = RF arduino alive, 0 = not alive (not a Python bool) |

### Servo bridge Arduino → Pi (`arduinos.servo_bridge`, feedback port 5010) — 8 bytes, little-endian

| Bytes | Field | Type | Notes |
|---|---|---|---|
| 0–3 | az_deg | float32 | Azimuth angle |
| 4–7 | el_deg | float32 | Elevation angle |

### Pi → Servo bridge Arduino (`arduinos.servo_bridge`, command port 5011) — 1 byte

`0x01` = start sweep, `0x00` = stop sweep.

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
| `pi_to_arduino` | `pack_measurement` byte layout, az/el millideg encoding, range in millimetres, detected flag encoding |

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
