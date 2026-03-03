# Race Datalogger - Project Guide

## Overview

ESP32-S3 race datalogger targeting the **LilyGo T-Beam Supreme** board. Comparable to an AIM Solo2DL — records high-rate GPS+IMU telemetry, detects tracks, times laps, and streams live data over LoRa. Logs are downloadable via WiFi AP.

## Hardware

| Component | IC/Type | Interface | Key Pins |
|-----------|---------|-----------|----------|
| MCU | ESP32-S3 | — | 8MB Flash, 8MB PSRAM |
| GPS/GNSS | u-blox | UART1 (9600) | RX=9, TX=8 |
| IMU | QMI8658 | SPI (HSPI) | CS=34, SCK=36, MOSI=35, MISO=37 |
| Display | SH1106 OLED 128x64 | I2C (0x3C) | SDA=17, SCL=18 |
| LoRa | SX1262 | SPI (FSPI) | CS=10, DIO1=1, RST=5, BUSY=4 |
| SD Card | — | SPI (HSPI) | CS=47 (shared bus with IMU) |
| PMU | AXP2101 | I2C (0x34) | SDA=42, SCL=41 |
| Button | Track control | GPIO 0 | Pull-up, debounced |

## Architecture

```
main.cpp  — orchestrates all modules, runs the main loop
├── gps.cpp/h         — UART GNSS polling via TinyGPSPlus, 10 Hz via UBX config
├── imu.cpp/h         — QMI8658 accel/gyro @ 100 Hz over SPI
├── track.cpp/h       — track detection, recording, lap timing state machine
├── race_logger.cpp/h — high-rate race CSV logging with GPS/IMU sync
├── logging.cpp/h     — SD card file management and buffered writes
├── display.cpp/h     — SH1106 OLED multi-screen UI (U8g2)
├── lora.cpp/h        — SX1262 telemetry TX @ 1 Hz (RadioLib)
├── pmu.cpp/h         — AXP2101 power rail control and battery telemetry
├── can_bus.cpp/h     — CAN bus module stub (ready for MCP2515/ESP32-C3)
└── wifi_server.cpp/h — WiFi AP + HTTP server for log download
```

## Key Timing

- **IMU polling:** 10 ms (100 Hz)
- **Track update:** 10 ms (100 Hz)
- **Serial telemetry:** 200 ms
- **Display refresh:** 500 ms
- **LoRa TX:** 1000 ms
- **Race log flush:** 500 ms or 512 B buffer full

## Track & Lap Logic (track.cpp)

State machine: `IDLE → RECORDING → READY → RACING`

- Tracks auto-detected within 500 m of saved start points
- Start-line crossing: speed > 5 km/h, within 30 m, heading ±45°, min 5 s between crossings
- Must leave 50 m radius before re-crossing counts
- Track IDs via FNV1a hash of (lat, lon, alt)
- Saved to `/tracks.csv` on SD

## Race Logger (race_logger.cpp)

- 600-sample IMU ring buffer (~6 s backlog)
- GPS at 10 Hz with IMU interpolation between fixes
- ENU coordinate projection from track start reference point
- Velocity decomposition (north/east) from speed + course
- Euclidean lap distance integration
- Output: ATP binary format at `/race-YYYYMMDD-HHMM.atp`

## SD Card Files

| Pattern | Contents |
|---------|----------|
| `track_XXXXXXXX.csv` | GPS snapshots during track recording (10 Hz) |
| `race-YYYYMMDD-HHMM.atp` | Binary race telemetry (ATP format) |
| `tracks.csv` | Saved track metadata (start points) |

## Build & Flash

```bash
# Build
pio run

# Upload
pio run -t upload

# Serial monitor
pio device monitor -b 115200
```

## Libraries (platformio.ini)

- `TinyGPSPlus` — NMEA parsing
- `lewisxhe/SensorLib` — QMI8658 IMU driver
- `lewisxhe/XPowersLib` — AXP2101 PMU driver
- `olikraus/U8g2` — SH1106 OLED
- `jgromes/RadioLib` — SX1262 LoRa

## WiFi Access

- SSID: `tbeam-telemetry` / Password: `tbeam123`
- `GET /` — root page
- `GET /logs` — JSON list of log files
- `GET /log?file=<name>` — download CSV

## Current Status

All core modules are implemented and working:

- [x] GPS acquisition and NMEA parsing
- [x] IMU data collection (accel + gyro + temp) @ 100 Hz
- [x] Track auto-detection and recording
- [x] Lap timing with start-line crossing detection
- [x] High-rate race data logging with GPS/IMU synchronization
- [x] OLED display with multi-mode UI (boot, recording, racing, idle)
- [x] LoRa telemetry TX @ 1 Hz
- [x] SD card storage with buffered writes
- [x] WiFi AP with HTTP log download
- [x] Power management and battery telemetry

## Future Work / Ideas

- [ ] RX-side LoRa receiver / pit display
- [ ] Predictive lap timing (delta to best lap)
- [ ] Sector timing
- [x] Higher GPS update rate (10 Hz via UBX-CFG-RATE)
- [ ] IMU-aided dead reckoning (complementary filter)
- [ ] CAN bus integration (quickshifter, ECU)
- [ ] BLE connectivity for mobile app
- [ ] OTA firmware updates via WiFi
