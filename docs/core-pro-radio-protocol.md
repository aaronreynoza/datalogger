# core module FSK Radio Protocol

**Version:** 1
**Status:** Draft
**Last updated:** February 2026

---

## Overview

The core module uses the SX1262 radio in **FSK mode** (not LoRa) to transmit telemetry from cars to a Race Control Gateway. FSK provides ~300 kbps throughput — enough for rich telemetry data from 35+ cars at 1 Hz updates.

This document specifies the radio packet format, TDMA scheduling, encryption, and multi-team architecture.

---

## 1. SX1262 FSK Configuration

| Parameter | Value | Notes |
|-----------|-------|-------|
| Modulation | FSK (Frequency Shift Keying) | Not LoRa |
| Frequency | 915 MHz (Americas) / 868 MHz (EU) | ISM band, region-configurable |
| Bitrate | 300 kbps | SX1262 maximum in FSK mode |
| Frequency deviation | ±75 kHz | Standard for 300 kbps FSK |
| Bandwidth | 467 kHz (RX) | Auto-calculated by SX1262 |
| Preamble | 5 bytes | For receiver synchronization |
| Sync word | 4 bytes (`0xAD1EC700`) | "ApexDirector" signature |
| CRC | CRC-16 CCITT | Hardware-computed by SX1262 |
| TX power | +14 dBm (25 mW) | Adjustable up to +22 dBm |
| Expected range | 1-3 km (line of sight) | Sufficient for any race circuit |

### Why FSK Over LoRa?

| | LoRa | FSK |
|---|---|---|
| Throughput | ~5-11 kbps | ~300 kbps |
| Per-car budget | ~50 bytes/s | ~1,000 bytes/s |
| Range | 5-15 km | 1-3 km |
| Interference resilience | Excellent | Good |
| Latency | High (long air time) | Low |

For race circuits (typically < 5 km perimeter), FSK's 1-3 km range is sufficient and the 60× throughput increase allows rich telemetry instead of position-only data.

---

## 2. TDMA Slot Structure

All cars share a single frequency. Time-Division Multiple Access (TDMA) prevents collisions.

### Time Frame (1 second)

```
|←──────────────────── 1000 ms ────────────────────→|
|  Slot 1  |  Slot 2  |  Slot 3  | ... |  Slot 35  | Guard |
|  28 ms   |  28 ms   |  28 ms   |     |  28 ms    | 20 ms |
|  Car 1   |  Car 2   |  Car 3   |     |  Car 35   |       |
```

| Parameter | Value |
|-----------|-------|
| Frame duration | 1000 ms |
| Slot duration | 28 ms |
| Max cars per frame | 35 |
| Total airtime | 35 × 28 ms = 980 ms |
| Guard time | 20 ms |
| Update rate | 1 Hz per car |

### Slot Duration Calculation

At 300 kbps:
- 250 bytes payload = 6.67 ms air time
- Plus preamble (5B), sync (4B), length (1B), CRC (2B) = 12 bytes overhead
- Total on-air: (262 × 8) / 300,000 = ~7 ms
- Plus TX/RX turnaround: ~1 ms
- Plus guard margin: ~20 ms
- **Slot = 28 ms** (conservative, allows for clock drift)

### Time Synchronization

The Race Control Gateway is the TDMA master clock. Synchronization methods:

1. **GPS PPS (Pulse Per Second)** — all T-Beam devices have GPS. The PPS signal provides microsecond-accurate time sync without any radio coordination.
2. **Beacon frame** — Gateway broadcasts a sync beacon at the start of each frame. Cars adjust their slot timing.

GPS PPS is preferred — it requires no extra radio traffic and works even before the gateway is online.

### Slot Assignment

Each car receives its slot number during check-in (via USB, NFC, or QR code along with the `race_key`). The slot number determines when the car transmits:

```
tx_time = frame_start + (slot_number × slot_duration)
```

---

## 3. Packet Format

Each car transmits one packet per TDMA slot.

### Packet Structure

```
┌──────────────────────────────────────────────┐
│  Radio Header (handled by SX1262 hardware)   │
│  Preamble (5B) + Sync (4B) + Length (1B)     │
├──────────────────────────────────────────────┤
│  Packet Header (8 bytes)                     │
│  ├─ version:     uint8                       │
│  ├─ car_id:      uint8                       │
│  ├─ slot:        uint8                       │
│  ├─ seq:         uint8  (sequence counter)   │
│  ├─ timestamp:   uint32 (ms since race start)│
├──────────────────────────────────────────────┤
│  Telemetry Payload (variable, ~60-200 bytes) │
│  ├─ Core fields (always present)             │
│  └─ Extended fields (if bandwidth allows)    │
├──────────────────────────────────────────────┤
│  AES-128 Tag (16 bytes)                      │
├──────────────────────────────────────────────┤
│  CRC-16 (2 bytes, handled by SX1262)         │
└──────────────────────────────────────────────┘
```

### Core Telemetry Fields (always present — 60 bytes)

| Offset | Type | Size | Field | Canonical ID | Unit |
|--------|------|------|-------|-------------|------|
| 0 | float32 | 4 | `gps_lat` | `position.gps.lat` | degrees |
| 4 | float32 | 4 | `gps_lon` | `position.gps.lon` | degrees |
| 8 | int16 | 2 | `gps_alt` | `position.gps.alt` | meters (offset from track alt) |
| 10 | uint16 | 2 | `speed` | `position.gps.speed` | km/h × 10 |
| 12 | uint16 | 2 | `heading` | `position.gps.course` | degrees × 10 |
| 14 | int16 | 2 | `accel_lat` | `dynamics.accel.lat` | G × 1000 |
| 16 | int16 | 2 | `accel_lon` | `dynamics.accel.lon` | G × 1000 |
| 18 | int16 | 2 | `accel_vert` | `dynamics.accel.vert` | G × 1000 |
| 20 | int16 | 2 | `yaw_rate` | `dynamics.yaw.rate` | °/s × 100 |
| 22 | uint16 | 2 | `lap_number` | `event.lap` | |
| 24 | uint32 | 4 | `lap_time_ms` | — | ms |
| 28 | uint32 | 4 | `last_lap_ms` | `event.lap.time` | ms |
| 32 | uint32 | 4 | `best_lap_ms` | — | ms |
| 36 | float32 | 4 | `lap_distance` | `position.lap.dist` | meters |
| 40 | uint8 | 1 | `sector` | — | current sector (0-based) |
| 41 | uint8 | 1 | `gps_sats` | — | satellite count |
| 42 | uint16 | 2 | `event_flags` | — | bitmask |
| 44 | uint8 | 1 | `battery_pct` | — | 0-100 |
| 45 | uint8 | 1 | `status_flags` | — | bitmask (in_pit, radio_ok, gps_ok) |
| 46 | uint16 | 2 | `reserved` | — | |
| **Total** | | **48** | | | |

### Extended Fields (optional — when CAN/ECU is connected)

| Offset | Type | Size | Field | Canonical ID |
|--------|------|------|-------|-------------|
| 48 | uint16 | 2 | `rpm` | `engine.rpm` |
| 50 | int8 | 1 | `gear` | `event.gear` |
| 51 | uint8 | 1 | `throttle` | `input.throttle` |
| 52 | uint8 | 1 | `brake` | `input.brake` |
| 53 | uint8 | 1 | `tc_active` | `event.tc` |
| 54 | uint8 | 1 | `abs_active` | `event.abs` |
| 55 | int8 | 1 | `water_temp` | `engine.water.temp` |
| 56 | int8 | 1 | `oil_temp` | `engine.oil.temp` |
| 57 | uint8 | 1 | `ext_field_mask` | — (which extra fields follow) |

More fields can follow based on `ext_field_mask` bits. Total packet stays within the 250-byte radio payload limit.

---

## 4. Encryption

### Algorithm

**AES-128-CTR** (Counter mode) with **CMAC** authentication tag.

- Encrypt: telemetry payload (core + extended fields)
- Authenticate: header + encrypted payload
- Tag: 16 bytes appended after payload
- The packet header (version, car_id, slot, seq, timestamp) is sent in cleartext for TDMA coordination

### Key Material

| Key | Purpose | Distribution | Lifetime |
|-----|---------|-------------|----------|
| `race_key` (16 bytes) | Encrypt car → gateway packets | At check-in (USB/NFC/QR) | One event |
| `team_key` (16 bytes) | Encrypt gateway → team feeds | To team manager at check-in | One event |
| `device_key` (16 bytes) | Device attestation | Burned at manufacturing | Permanent |

### Nonce Construction (CTR mode)

```
Nonce (12 bytes):
├─ car_id:    uint8   (1 byte)
├─ slot:      uint8   (1 byte)
├─ reserved:  uint16  (2 bytes, 0x0000)
├─ timestamp: uint32  (4 bytes, ms since race start)
└─ seq:       uint32  (4 bytes, packet sequence number)
```

The combination of `car_id + timestamp + seq` guarantees nonce uniqueness. Even if two cars have the same timestamp (different slots), `car_id` differentiates them.

### What an Eavesdropper Sees

Without the `race_key`:
- Packet header: car_id, slot, timestamp (cleartext) — they know a car is transmitting
- Payload: random bytes (AES-128 encrypted)
- They know how many cars are on track (from slot usage patterns)
- They do NOT know: position, speed, lap times, or any telemetry

---

## 5. Multi-Team Architecture

### Data Flow

```
Car 1 (Team A) ──FSK──┐
Car 2 (Team A) ──FSK──┤
Car 3 (Team B) ──FSK──┼──→ Race Control Gateway ──┬──→ Race Control Dashboard
Car 4 (Team B) ──FSK──┤                           │    (all cars, basic data)
Car 5 (privateer)──FSK─┤                           │
...                    │                           ├──→ Team A Feed (WebSocket)
Car 35 ────────FSK─────┘                           │    (cars 1-2, full data)
                                                   │
                                                   ├──→ Team B Feed (WebSocket)
                                                   │    (cars 3-4, full data)
                                                   │
                                                   └──→ No feed for privateers
                                                        (download post-session)
```

### Race Control Gateway

Hardware: T-Beam Supreme (or Raspberry Pi + SX1262 HAT)

Functions:
1. **FSK Receiver** — listens to all TDMA slots, decrypts with `race_key`
2. **Race Control Dashboard** — web UI showing all cars on a track map with live timing
3. **Team Data Server** — WebSocket endpoints per team, encrypted with `team_key`
4. **Timing System** — official lap times, sector times, race standings

### Team Feed Protocol

Teams connect to the gateway via WiFi (the gateway runs a WiFi AP or joins the venue network):

```
WebSocket: ws://{gateway_ip}/api/v1/teams/{team_id}/stream

Authentication: Bearer {team_token}
```

The gateway:
1. Receives all car packets via FSK
2. Decrypts with `race_key`
3. Filters to only the requesting team's cars
4. Re-encrypts with `team_key` (or sends over HTTPS/WSS)
5. Streams to the team's WebSocket connection

### What Each Party Sees

| Party | Cars visible | Data depth | Transport |
|-------|-------------|-----------|-----------|
| Race Control | All cars | Position, speed, lap times, flags | Direct FSK decode |
| Team A | Team A cars only | Full telemetry (IMU, ECU, everything) | WebSocket from gateway |
| Team B | Team B cars only | Full telemetry | WebSocket from gateway |
| Other teams | Nothing | N/A | N/A |
| Eavesdropper | Nothing meaningful | Encrypted packets | N/A |

---

## 6. Check-In Flow

### Pre-Race Setup

1. **Event organizer** generates `race_key` and per-team `team_key`s using the ApexDirector Race Control software
2. **Gateway** is configured with `race_key` and team roster

### Per-Car Check-In

```
1. Car arrives at check-in
2. Race official connects to car's WiFi (or uses USB/NFC)
3. Sends check-in packet:
   {
     "race_key": "base64...",
     "slot_number": 7,
     "car_number": 42,
     "team_id": "team_a",    // or null for privateers
     "event_name": "Round 3 - Calder Park"
   }
4. Device stores race_key, slot, car number
5. Device responds with attestation:
   {
     "serial": "CP-00001",
     "firmware": "1.0.0",
     "attestation": "HMAC-SHA256(nonce, device_key)"
   }
6. Gateway verifies attestation against known device database
7. Car is cleared to race
```

### Check-In Endpoint (on the core module)

```
POST /api/v1/radio/configure
```

**Request:**
```json
{
  "race_key": "aGVsbG8gd29ybGQgdGhpcyBpcyBhIGtleQ==",
  "slot_number": 7,
  "car_number": 42,
  "team_id": "team_a",
  "event_name": "Round 3 - Calder Park",
  "nonce": "random_challenge_bytes_base64"
}
```

**Response:**
```json
{
  "status": "configured",
  "serial": "CP-00001",
  "attestation": "hmac_response_base64",
  "radio_test": "ok"
}
```

---

## 7. Failure Modes

### Lost Packets

FSK is not guaranteed delivery. Packets may be lost due to:
- Interference (other ISM band devices)
- Physical obstruction (car behind a building)
- Collision (clock drift causing slot overlap)

**Mitigation:**
- The gateway tracks packet sequence numbers per car
- Missing packets are interpolated (linear interpolation between last known and next received)
- Race control UI shows a "signal quality" indicator per car
- After 5 seconds of no packets, the car is marked as "signal lost"

### Device Dropout

If a car's radio stops transmitting (battery dead, hardware failure):
- Gateway marks car as "offline" after 10 seconds
- Last known position is displayed with an "offline" indicator
- The car's TDMA slot remains reserved (not reassigned mid-race)

### Clock Drift

GPS PPS provides sub-microsecond timing. Without GPS fix:
- ESP32-S3 crystal accuracy: ±40 ppm → ±0.04 ms per second
- Over 1 hour: ±144 ms drift — well within the 28ms slot + 20ms guard window
- If drift exceeds tolerance, the beacon sync mechanism corrects it

### Interference

Sub-GHz ISM bands have less congestion than 2.4 GHz WiFi but are shared with:
- Garage door openers, weather stations, alarm systems
- Other LoRa/FSK devices in the area

**Mitigation:**
- The 4-byte sync word (`0xAD1EC700`) rejects non-ApexDirector packets
- CRC-16 catches corrupted packets
- AES CMAC detects tampered packets
- If a frequency is congested, the system can be reconfigured to a different channel within the ISM band

---

## 8. Scaling

| Cars | Slots | Airtime | Update Rate | Notes |
|------|-------|---------|-------------|-------|
| 10 | 10 | 280 ms | 1 Hz | Typical club race |
| 20 | 20 | 560 ms | 1 Hz | Regional event |
| 35 | 35 | 980 ms | 1 Hz | Full grid |
| 50 | 50 | 1400 ms | ~0.7 Hz | Tight, but workable |
| 70 | 35+35 | 2 × 980 ms | 1 Hz | Dual frequency (2 gateways) |

For events with 50+ cars, the system supports **dual-frequency operation**: cars 1-35 on frequency A, cars 36-70 on frequency B. Each frequency has its own TDMA frame. The gateway listens on both (using two SX1262 modules or two gateways).
