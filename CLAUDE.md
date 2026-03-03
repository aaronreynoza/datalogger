# Race Datalogger - Project Guide

## Overview

ESP32-S3 race datalogger targeting the **LilyGo T-Beam Supreme** board. Comparable to an AIM Solo2DL — records high-rate GPS+IMU telemetry, detects tracks, times laps. Logs downloadable via WiFi AP.

## Hardware

| Component | IC/Type | Interface | Key Pins |
|-----------|---------|-----------|----------|
| MCU | ESP32-S3 | — | 8MB Flash, 8MB PSRAM |
| GPS/GNSS | u-blox | UART1 (38400) | RX=9, TX=8 |
| IMU | QMI8658 | SPI (HSPI) | CS=34, SCK=36, MOSI=35, MISO=37 |
| Display | SH1106 OLED 128x64 | I2C (0x3C) | SDA=17, SCL=18 |
| SD Card | — | SPI (HSPI) | CS=47 (shared bus with IMU) |
| PMU | AXP2101 | I2C (0x34) | SDA=42, SCL=41 |
| Button | Track control | GPIO 0 | Pull-up, debounced |

**Removed:** LoRa (SX1262) — antenna reserved for future receiver use.

## Architecture

```
main.cpp  — orchestrates all modules, runs the main loop
├── gps.cpp/h         — UART GNSS polling via TinyGPSPlus, 10 Hz via UBX config
├── imu.cpp/h         — QMI8658 accel/gyro @ 100 Hz over SPI
├── track.cpp/h       — track detection, recording, lap timing state machine
├── race_logger.cpp/h — high-rate race logging with GPS/IMU sync (ATP binary format)
├── logging.cpp/h     — SD card file management, buffered writes, diagnostic log
├── display.cpp/h     — SH1106 OLED multi-screen UI (U8g2)
├── pmu.cpp/h         — AXP2101 power rail control and battery telemetry
├── can_bus.cpp/h     — CAN bus module stub (ready for MCP2515/ESP32-C3)
├── device_config.cpp/h — Device serial number and configuration
└── wifi_server.cpp/h — WiFi AP + HTTP REST API (auto-disabled during recording/racing)
```

## Key Timing

- **IMU polling:** 10 ms (100 Hz)
- **Track update:** 10 ms (100 Hz)
- **Serial telemetry:** 200 ms
- **Display refresh:** 500 ms (all states)
- **Track CSV flush:** 2000 ms
- **Diag log flush:** 5000 ms
- **Housekeeping:** 1000 ms (WiFi management)
- **Watchdog:** 5 s (reboot on hang)
- **Race log flush:** 500 ms or 512 B buffer full

## Track & Lap Logic (track.cpp)

State machine: `IDLE → RECORDING → READY → RACING`

- Tracks auto-detected within 1000 m of saved start points
- **RECORDING:** Circuit detection via proximity (12m) + cumulative distance (50m) + heading tolerance (90°). Heading updated at circuit completion for gate accuracy.
- **READY/RACING:** Line-segment crossing — 30m gate perpendicular to start heading, cross-product straddle test with direction validation.
- Out-lap handling: first crossing starts timer but doesn't count as Lap 1
- Track IDs via FNV1a hash of quantized (lat, lon) — altitude excluded
- Saved to `/tracks.csv` on SD
- TODO: increase MIN_SPEED_KMH from 5 to 25 for production

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

## Build & Flash

```bash
pio run -e t-beams3-supreme          # Build (use this env, not default)
pio run -t upload -e t-beams3-supreme # Flash
pio device monitor -b 115200          # Serial monitor
```

**Note:** The `native` test env fails with `Arduino.h not found` — this is expected (it's for unit tests only).

## Libraries (platformio.ini)

- `TinyGPSPlus` — NMEA parsing
- `lewisxhe/SensorLib` — QMI8658 IMU driver
- `lewisxhe/XPowersLib` — AXP2101 PMU driver
- `olikraus/U8g2` — SH1106 OLED
- `ArduinoJson` — REST API responses
- `ESPAsyncWebServer` — Non-blocking HTTP server

## WiFi Access

- SSID: `tbeam-telemetry` / Password: `tbeam123`
- **Auto-disabled during RECORDING/RACING** (SD bus contention)
- `GET /api/v1/status` — device status JSON
- `GET /api/v1/tracks` — saved tracks list
- `GET /api/v1/sessions` — race session list
- `GET /log?file=<name>` — download file

## Hardware Constraints (Critical)

- **Never use `delay()` in main loop** — watchdog reboots after 5s
- **SD + IMU share HSPI bus** — WiFi handlers must never touch SD during recording
- **UART buffer = 2048 bytes** — 10Hz GPS at 38400 baud fills fast, poll frequently
- **WiFi API cannot be reached from dev machine** — device is AP, connecting loses internet
- **Serial port:** `/dev/cu.usbmodem2101`

## Future Work

- [ ] FreeRTOS dual-core (GPS+SD on Core 0, IMU+track on Core 1)
- [ ] Line-segment crossing time interpolation (sub-GPS-interval accuracy)
- [ ] Predictive lap timing (delta to best lap)
- [ ] Sector timing
- [ ] CAN bus integration (quickshifter, ECU)
- [ ] GPS PPS-disciplined IMU timestamps
- [ ] LoRa receiver / pit display
