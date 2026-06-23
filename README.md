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

All Arduino IPs/ports are defined in one place: [`network/config.yaml`](network/config.yaml). Edit that file when hardware changes — the Python scripts read from it instead of hardcoding addresses. Run [`diagnostics/check_network.sh`](diagnostics/check_network.sh) to ping every configured Arduino and check the Pi's own interface.

---

## Layout

```
scanner.py      — orchestrator (root): fuses LiDAR + servo position into frames
fcp_bridge.py   — orchestrator (root): tracker -> FCP uplink, live
tracker/        — EKF tracker + its smoke test
  multi_rat_tracker.py
  test_tracker.py
sensors/        — imported hardware input drivers
  lidar_collect.py             — Lightware SF20 serial driver
  rf_receiver.py               — RF arduino -> Pi (production listener)
  arduino_servo_bridge.py      — RF arduino -> Pi (LiDAR-gimbal az/el position)
integration/    — outbound drivers to other systems
  fcp_uplink.py        — confirmed tracks -> FCP 'positional' messages (serial, UDP fallback)
network/        — network config
  config.yaml          — single source of truth for every static IP/port
  network_config.py    — loader
diagnostics/    — standalone hand-run tools (not imported by anything)
  check_rf.py          — RF arduino port listener
  check_acoustics.py   — dump two acoustic boards' packets
  check_splitter.py    — confirm multiple boards reach the Pi through the splitter
  check_network.sh     — ping health check
  raw_listen.py        — dumb single-port UDP dumper
tests/
  test_suite.py   — pytest suite covering all Python modules
```

`scanner.py` stays at the repo root and fuses LiDAR readings with servo position into per-frame output.
`fcp_bridge.py` is the live version of that pipeline that also runs the tracker and pushes confirmed tracks to the FCP.

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
python3 sensors/rf_receiver.py
```

Listens on `arduinos.rf.data_port` (from config) and prints each `rf_reading` as it arrives.

### Decode/debug the RF connection directly

```bash
python3 diagnostics/check_rf.py
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

### Bridge confirmed tracks to the FCP

```bash
python3 fcp_bridge.py
```

Streams the LiDAR + live gimbal position, runs the tracker, and sends every
confirmed track to the FCP as a `positional` message — see
[FCP Uplink](#fcp-uplink) below. Requires the LiDAR and the RF Arduino's
gimbal-feedback link; no FCP connection is required for it to start (it logs
and falls back to UDP if the serial link isn't there).

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

One UDP port per field; `diagnostics/check_rf.py` listens on all of them.

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

## FCP Uplink

`fcp_bridge.py` reports each confirmed track to the FCP as a `positional`
message — see `integration/fcp_uplink.py`. Sent over serial
(`network/config.yaml: fcp.serial`, `/dev/ttyACM0`) when available; falls
back to UDP (`fcp.udp_fallback`) for the rest of the process's life once a
serial write fails. One JSON object per line:

```json
{
  "msg_type": "positional",
  "rat_id": "<tracker's track_id, as a string>",
  "zone": 3,
  "values":  {"az_value": 0.21, "el_value": 0.05, "range_value": 27.4},
  "rates":   {"az_rate": 0.01, "el_rate": -0.02, "range_rate": -0.6},
  "current_time": 1750000000.123
}
```

| Field | Notes |
|---|---|
| `zone` | 1 = outer (≤85 m), 2 = middle (≤60 m), 3 = central (≤30 m, engage-eligible). Tracks beyond 85 m aren't sent. |
| `values.*`, `rates.*` | Radians / metres / (rad or m)/s — the tracker's native units, not degrees. |
| `current_time` | Unix epoch seconds. |

These two assumptions (units, timestamp format) haven't been confirmed
against the FCP's `rat_model.py` — adjust `build_positional_message()` if
the FCP expects degrees or an ISO timestamp instead.

---

## Running the Tests

```bash
pytest tests/ -v
```

Expected output: **84 passed**.

### What the tests cover

| Module | Tests |
|---|---|
| `multi_rat_tracker` | Filter, angle wrapping, F/Q matrix properties, GNN assignment, track lifecycle (confirm, delete, reset, multi-target), `confirmed_track_states` |
| `lidar_collect` | `LidarMeasurement` detection boundary, `lidar_formatting` parse/fail cases, buffer drain, `drain_as_tracker_input` filtering and unit conversion |
| `rf_receiver` | `rf_reading` fields, `unpack_rf_reading` roundtrip for all four fields, packet size constant |
| `fcp_uplink` | Zone classification boundaries, `positional` message shape, serial-primary send, UDP fallback (on open failure and on write failure), close |

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
