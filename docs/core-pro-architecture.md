# Core Pro — Architecture & Design Discussions

**Device:** ApexDirector Core Pro (LilyGo T-Beam Supreme)
**Last updated:** February 2026

---

## Table of Contents

1. [Hardware Overview](#1-hardware-overview)
2. [Communication Layers](#2-communication-layers)
3. [Race Control Architecture](#3-race-control-architecture)
4. [Security Model](#4-security-model)
5. [Anti-Cheat / Firmware Verification](#5-anti-cheat--firmware-verification)
6. [Business Scenarios](#6-business-scenarios)
7. [Bandwidth & Capacity](#7-bandwidth--capacity)

---

## 1. Hardware Overview

The Core Pro is built on the **LilyGo T-Beam Supreme** development board.

| Component | Detail |
|-----------|--------|
| MCU | ESP32-S3 (dual-core Xtensa LX7, 240MHz, 512KB SRAM, 8MB PSRAM) |
| GPS | u-blox GNSS, 1 Hz update rate, UART |
| IMU | QMI8658 (6-axis: 3-axis accelerometer + 3-axis gyroscope), 100 Hz, SPI |
| Radio | Semtech SX1262 (sub-GHz, supports LoRa and FSK modulation) |
| Storage | SD card (FAT32), SPI |
| WiFi | ESP32-S3 built-in 802.11 b/g/n, 2.4 GHz |
| Bluetooth | ESP32-S3 built-in BLE 5.0 |
| Power | 18650 Li-ion battery + USB-C charging |

### Sensor Specifications

| Sensor | Range | Resolution | Rate |
|--------|-------|------------|------|
| Accelerometer | ±4g (configurable) | 4.0/32768 g per LSB | 100 Hz |
| Gyroscope | ±64 °/s (configurable) | 64.0/32768 °/s per LSB | 100 Hz |
| GPS Position | Global (WGS84) | ~2.5m CEP (open sky) | 1 Hz |
| GPS Speed | 0-500 km/h | 0.1 km/h | 1 Hz |

### Future Sensor Expansion

The Core Pro will support additional sensors via CAN bus, analog inputs, and digital inputs:

| Sensor Type | Connection | Channels | Typical Rate |
|-------------|-----------|----------|-------------|
| Suspension potentiometers | Analog (4x) | FL, FR, RL, RR travel | 100 Hz |
| Brake pressure sensors | Analog (2-4x) | Front, rear (or per-corner) | 50 Hz |
| ECU data (RPM, gear, throttle) | CAN bus | Multiple | 10-100 Hz |
| Traction control state | CAN bus | TC active, TC level | Event-based |
| ABS state | CAN bus | ABS active, ABS level | Event-based |
| Wheel speed sensors | Digital pulse | FL, FR, RL, RR | 50 Hz |
| Exhaust gas temperature | Analog/CAN | 1-4 cylinders | 10 Hz |
| Lambda sensor | CAN | 1-2 banks | 10 Hz |
| Oil pressure / temperature | CAN/Analog | Pressure, temp | 7-10 Hz |
| Water temperature | CAN/Analog | Temp | 7 Hz |

These channels are recorded alongside IMU and GPS data in the same `.atp` file. The channel table in the file header declares what channels are present — no format change needed when sensors are added.

### IMU Axis Note

The QMI8658 chip axes are fixed to the PCB. The physical mapping to vehicle axes (longitudinal, lateral, vertical) depends on how the device is mounted. The ApexDirector desktop app provides axis mapping configuration — the user specifies which IMU axis corresponds to which vehicle axis and sign convention.

---

## 2. Communication Layers

The Core Pro uses three communication channels, each optimized for a different purpose:

| Layer | Technology | Bandwidth | Range | Purpose |
|-------|-----------|-----------|-------|---------|
| **WiFi** | ESP32-S3 802.11n | ~10 Mbps | ~50m | Full telemetry download, live analysis, device management |
| **FSK Radio** | SX1262 sub-GHz FSK | ~300 kbps | 1-3 km | Race control telemetry, team feeds |
| **SD Card** | SPI FAT32 | ~2 MB/s write | N/A | Canonical storage, post-session download |

### WiFi

The device acts as a WiFi Access Point (like the AIM Solo2DL) or connects to an existing network:

- **AP Mode (default):** SSID = device serial number, WPA2-PSK
- **Station Mode:** Connects to a known WiFi network (configured via BLE or USB)

WiFi exposes an **HTTP + WebSocket API** (documented in `core-pro-wifi-api.md`):
- REST/JSON for device management, session listing, track management
- Binary `.atp` file download for session data
- WebSocket for real-time live telemetry streaming

### FSK Radio

The SX1262 runs in **FSK mode** (not LoRa) for higher throughput:

- ~300 kbps raw bitrate (vs ~5-11 kbps for LoRa)
- Shorter range (~1-3 km vs ~5-15 km for LoRa), but sufficient for any race circuit
- Same antenna and hardware as LoRa — just a different modulation mode
- Each car gets ~1 KB per time slot at 1 Hz update rate
- Enough bandwidth for rich telemetry (IMU + GPS + lap data + ECU subset), not just position/speed

The FSK protocol is documented in `core-pro-radio-protocol.md`.

### SD Card

The SD card stores `.atp` files — the same binary format served over WiFi. The device writes continuously during a session. Post-session, data is downloaded via WiFi or by physically removing the SD card.

---

## 3. Race Control Architecture

### The Problem

A race event has:
- **Race control** — needs position, speed, lap times for all cars (safety, timing, flags)
- **Teams** — need detailed telemetry for their own cars (strategy, driver coaching)
- **Privateers** — only care about their own data, post-session

These groups have **different data needs** and **must not see each other's detailed data**.

### The Solution: Gateway Architecture

```
                    ┌────────────────────────────────────────┐
                    │          Race Control Gateway           │
                    │   (T-Beam or Raspberry Pi + SX1262)    │
                    │                                        │
                    │   FSK RX → Decrypt → Validate →        │
                    │     ├→ Race Control Dashboard           │
                    │     ├→ Team A feed (WebSocket)          │
                    │     └→ Team B feed (WebSocket)          │
                    └───────────────┬────────────────────────┘
                                    │ FSK radio (encrypted)
                    ┌───────────────┼────────────────────────┐
                    ▼               ▼                        ▼
               ┌──────────┐   ┌──────────┐           ┌──────────┐
               │  Car 1   │   │  Car 2   │   ...     │  Car 35  │
               │ Core Pro │   │ Core Pro │           │ Core Pro │
               │          │   │          │           │          │
               │ FSK TX   │   │ FSK TX   │           │ FSK TX   │
               │ WiFi AP  │   │ WiFi AP  │           │ WiFi AP  │
               │ SD card  │   │ SD card  │           │ SD card  │
               └──────────┘   └──────────┘           └──────────┘
                    │               │                        │
                  WiFi            WiFi                     WiFi
                    │               │                        │
               Team A pit     Team A pit              Privateer
               laptop         laptop                  phone
```

**Data flow:**

1. Each car's Core Pro broadcasts telemetry via FSK radio, encrypted with the event's `race_key`
2. The Race Control Gateway receives all car packets
3. Race control dashboard shows position, speed, lap times for all cars
4. Gateway filters and re-encrypts data for each team with their `team_key`, forwarded over WiFi/Ethernet
5. Teams receive only their own cars' data via WebSocket
6. Privateers don't receive FSK data — they download from their own device's WiFi after the session

**Key principle:** Cars send ONE packet to race control. Race control redistributes. This halves FSK airtime compared to cars sending separate packets to race control and their team.

### What Race Control Sees vs What Teams See

| Data | Race Control | Team |
|------|-------------|------|
| GPS position | ✅ All cars | ✅ Own cars only |
| Speed | ✅ All cars | ✅ Own cars only |
| Lap times / sectors | ✅ All cars | ✅ Own cars only |
| Current lap | ✅ All cars | ✅ Own cars only |
| IMU (accel, gyro) | ❌ | ✅ Own cars only |
| ECU (RPM, gear, throttle) | ❌ | ✅ Own cars only |
| Tyre/brake data | ❌ | ✅ Own cars only |
| Flags (yellow, SC) | ✅ Issues to all | ✅ Receives |

Race control gets only what it needs for safety and timing. Teams get full telemetry for their cars. No team ever sees another team's data.

---

## 4. Security Model

### Encryption

All FSK radio communication is encrypted with **AES-128** in CTR mode.

**Key hierarchy:**

```
Manufacturing Key (burned into device eFuses, never leaves the chip)
    │
    ├── Device Key (derived from manufacturing key + device serial)
    │   └── Used for: device attestation, firmware verification
    │
    ├── Race Key (distributed at event check-in)
    │   └── Used for: encrypting car → race control radio packets
    │
    └── Team Key (distributed to team at event check-in)
        └── Used for: encrypting race control → team data feeds
```

**Key distribution workflow:**

1. Event organizer generates a unique `race_key` for the event
2. At check-in, each device receives the `race_key` via:
   - USB connection (most secure)
   - NFC tap (convenient)
   - QR code scan (fallback)
3. Teams receive their `team_key` separately
4. After the event, all keys expire — they cannot be reused

### Why AES-128 and Not AES-256?

The ESP32-S3 has **hardware AES-128 acceleration**. AES-128 has zero performance cost. AES-256 would require software implementation (slower, more power). For our threat model (preventing casual eavesdropping at a race track, not defending against nation-state adversaries), AES-128 is more than sufficient.

---

## 5. Anti-Cheat / Firmware Verification

### The Problem

If a pilot buys their own Core Pro, how does race control trust that the device is sending real data and not spoofed telemetry?

### Defense Layers

**Layer 1: Secure Boot (prevents casual firmware tampering)**

The ESP32-S3 supports Secure Boot v2:
- Only firmware signed with ApexDirector's private key will boot
- The signing key hash is burned into the chip's eFuses (one-time, irreversible)
- Reflashing with unsigned firmware bricks the device
- This stops 99% of tampering attempts

**Layer 2: Device Attestation (proves the device is genuine at check-in)**

At check-in, race control challenges the device:

```
Race Control → Device:  "nonce: 0xA7B3F19D..."
Device → Race Control:  HMAC-SHA256(nonce, device_secret)
Race Control:           verifies against known device_secret in database
```

This proves:
- The device has the correct `device_secret` (burned at manufacturing)
- The device is running firmware that implements the attestation protocol
- Custom firmware would need to extract the device_secret (protected by eFuses)

**Layer 3: Plausibility Checks (catches spoofed data)**

Race control software validates telemetry against physics:
- GPS position delta vs reported speed (do they match?)
- Acceleration magnitude vs speed change (physically possible?)
- Lap time vs sector times (do they add up?)
- Cross-reference between nearby cars (consistent positions?)
- Signal quality metrics (HDOP, satellite count — hard to fake)

**Layer 4: Physical Inspection + Rental (high-stakes events)**

| Race Level | Trust Model |
|-----------|-------------|
| Club/hobby | Secure Boot + attestation (nobody hires a hardware hacker for a kart race) |
| Regional | Add plausibility checks + random spot-checks |
| National/pro | Rental devices only — race control owns the hardware |

---

## 6. Business Scenarios

### Scenario 1: Pilot with own Core Pro

> "I bought a Core Pro. I normally race alone and don't use radio. Now I'm at an organized event and race direction wants to track me."

1. Pilot arrives with their Core Pro at check-in
2. Race control sends `race_key` to the device (NFC tap or USB)
3. Device starts FSK radio transmission alongside normal SD + WiFi logging
4. Race control receives the pilot's position, speed, lap times
5. After the event, `race_key` expires — device returns to standalone mode

**No rental needed.** The pilot's own device joins the race network temporarily.

### Scenario 2: Team with 5 cars

> "We have 5 drivers on track. We want live metrics for our own analysis. They also need to be connected to race direction."

1. All 5 devices get the `race_key` at check-in (same as every other car)
2. Team manager receives the `team_key`
3. Cars broadcast FSK telemetry → Race Control Gateway receives
4. Gateway forwards Team's 5 cars' detailed data to Team via WebSocket (encrypted with `team_key`)
5. Team laptop shows live dashboard of all 5 drivers — full IMU, GPS, ECU, lap times
6. Race control sees only basic data (position, speed, laps)

### Scenario 3: 35-car grid, 9 in teams

> "35 pilots total. 9 are spread across 3 teams of 3. 26 are privateers."

```
35 cars on track
├── Team A (3 cars) → Team A laptop sees 3 cars, full telemetry
├── Team B (4 cars) → Team B laptop sees 4 cars, full telemetry
├── Team C (2 cars) → Team C laptop sees 2 cars, full telemetry
└── 26 privateers   → No live feed; download post-session via WiFi
```

- Race control sees all 35 cars (basic data)
- Each team sees only their own cars (full data)
- Teams cannot sniff each other (different `team_key` per team, AES-128)
- Privateers get their data from their own device WiFi after the session

---

## 7. Bandwidth & Capacity

### FSK Radio Budget

**SX1262 FSK at typical settings:**
- Bitrate: 300 kbps
- Packet overhead: ~20 bytes (preamble, sync, CRC, AES tag)
- Usable payload per packet: ~230 bytes

**TDMA slotting (1 Hz update, 35 cars):**

| Parameter | Value |
|-----------|-------|
| Slot duration | ~28 ms (230 bytes at 300 kbps) |
| Cars | 35 |
| Total airtime | 35 × 28ms = 980ms |
| Guard time | 20ms |
| Update rate | 1 Hz per car |

Each car gets **~230 bytes of usable payload per second** — enough for:

| Field | Bytes | Notes |
|-------|-------|-------|
| Car ID + sequence | 3 | |
| Timestamp | 4 | ms since session start |
| GPS lat/lon | 8 | float32 pair (sufficient precision) |
| GPS alt | 2 | int16, meters above track |
| Speed | 2 | uint16, km/h × 10 |
| RPM | 2 | uint16 |
| Gear | 1 | int8 |
| Throttle | 1 | uint8 (0-255) |
| Brake | 1 | uint8 (0-255) |
| Accel lat/lon | 4 | 2 × int16, fixed-point G |
| Yaw rate | 2 | int16, °/s |
| Lap number | 2 | uint16 |
| Lap time ms | 4 | uint32 |
| Last lap ms | 4 | uint32 |
| Sector | 1 | uint8 |
| Event flags | 2 | uint16 bitmask |
| Status | 1 | uint8 (in pit, flag state, etc.) |
| AES tag | 16 | Authentication tag |
| **Total** | **60** | Well within 230-byte budget |

Remaining ~170 bytes available for additional data (tyre temps, suspension, team-specific channels).

### Scaling Beyond 35 Cars

| Cars | Update Rate | Feasible? |
|------|------------|-----------|
| 35 | 1 Hz | Yes (980ms of 1000ms used) |
| 50 | ~0.7 Hz | Acceptable for most events |
| 70 | 0.5 Hz | Marginal (every 2 seconds) |
| 100+ | Dual frequency | Two SX1262 channels, cars split between them |

### WiFi Budget

WiFi bandwidth is not a concern:
- Session download: ~10 MB .atp file over WiFi at ~2 MB/s = 5 seconds
- Live stream: ~10 KB/s per WebSocket connection = trivial
- Multiple simultaneous connections (phone + laptop) supported
