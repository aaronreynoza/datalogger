# T-Beam Datalogger — File Format Specification

## Overview

The T-Beam Supreme race datalogger produces three types of CSV files on its SD card. All files are UTF-8 encoded, comma-delimited, with `\r\n` line endings (the header may have `\r\n`, data rows use `\n`). The first row is always a header. All 34 columns are always present in every row.

**File relationships:** The `track_id` field links everything together. A track defined in `tracks.csv` has a decimal ID (e.g., `2495971338`). The corresponding track recording file uses the hex form (`track_94C5800A.csv`). Race telemetry files reference the track by decimal ID in column 34.

---

## 1. Track Definition File: `tracks.csv`

**Purpose:** Stores the start/finish line definition for all saved tracks.

**Location:** `/tracks.csv` (single file, always overwritten on update)

### Columns

| Column | Type | Unit | Description |
|--------|------|------|-------------|
| `start_lat` | float64 | degrees | Start/finish line latitude (WGS84) |
| `start_lon` | float64 | degrees | Start/finish line longitude (WGS84) |
| `start_alt` | float64 | meters | Altitude at start/finish (MSL) |
| `start_course_deg` | float64 | degrees | Heading when crossing start/finish (0-360, true north) |
| `track_id` | uint32 | — | Unique track ID (FNV1a hash of lat/lon/alt, decimal) |

### Example

```csv
start_lat,start_lon,start_alt,start_course_deg,track_id
13.778436,-89.191724,492.00,105.26,2495971338
13.778619,-89.188273,479.70,105.38,3043393695
```

### Notes
- `track_id` is computed as FNV1a hash of `(round(lat*1e6), round(lon*1e6), round(alt*100))` — this means the same physical location always produces the same ID.
- `start_course_deg` defines the expected heading when crossing the line. The datalogger uses a ±45° tolerance for crossing detection.
- The hex form of `track_id` is used for track recording filenames (e.g., `2495971338` decimal = `94C5800A` hex → `track_94C5800A.csv`).

---

## 2. Track Recording File: `track_XXXXXXXX.csv`

**Purpose:** Raw GPS breadcrumb trail recorded while the user drives around a circuit to define a new track. The `XXXXXXXX` is the hex track ID (zero-padded to 8 characters).

**Location:** `/track_<TRACK_ID_HEX>.csv` (one file per track recording)

### Columns

| Column | Type | Unit | Description |
|--------|------|------|-------------|
| `epoch_s` | uint32 | seconds | Unix timestamp (UTC) |
| `ms_since_boot` | uint32 | ms | Milliseconds since device boot |
| `lat` | float64 | degrees | Latitude (WGS84) |
| `lon` | float64 | degrees | Longitude (WGS84) |
| `alt_m` | float64 | meters | Altitude (MSL) |
| `spd_kmph` | float64 | km/h | Ground speed |
| `course_deg` | float64 | degrees | Course over ground (0-360, true north) |
| `hdop` | float64 | — | Horizontal dilution of precision |
| `sats` | uint32 | — | Number of satellites used |

### Example

```csv
epoch_s,ms_since_boot,lat,lon,alt_m,spd_kmph,course_deg,hdop,sats
1772238046,471328,13.778436,-89.191724,492.00,7.09,105.26,1.01,12
1772238047,472331,13.778440,-89.191680,491.80,8.23,104.50,1.01,12
```

### Notes
- Sample rate: ~1 Hz (one row per GPS fix)
- Recording starts when the user presses the button and begins moving (>5 km/h)
- Recording ends automatically when the user returns to the start point (circuit completion)
- These files can be used to visualize the track layout

---

## 3. Race Telemetry File: `race-YYYYMMDD-HHMM.csv`

**Purpose:** High-rate telemetry log combining 100 Hz IMU data with interpolated GPS positions and lap timing. This is the primary data file for analysis.

**Location:** `/race-<YYYYMMDD>-<HHMM>.csv` (timestamp is UTC)

### Columns

| # | Column | Type | Unit | Description |
|---|--------|------|------|-------------|
| 1 | `epoch_s` | uint32 | seconds | Unix timestamp (UTC), interpolated between GPS fixes |
| 2 | `ms_since_boot` | uint32 | ms | Monotonic time since device boot (primary time reference) |
| 3 | `imu_ts` | uint32 | — | IMU hardware timestamp (internal counter) |
| 4 | `ax_g` | float32 | g | Accelerometer X (1g = 9.80665 m/s²) |
| 5 | `ay_g` | float32 | g | Accelerometer Y |
| 6 | `az_g` | float32 | g | Accelerometer Z (~1.0 when device is level and stationary) |
| 7 | `gx_dps` | float32 | °/s | Gyroscope X (roll rate) |
| 8 | `gy_dps` | float32 | °/s | Gyroscope Y (pitch rate) |
| 9 | `gz_dps` | float32 | °/s | Gyroscope Z (yaw rate) |
| 10 | `imu_temp_C` | float32 | °C | IMU die temperature |
| 11 | `gps_lat` | float64 | degrees | Latitude (WGS84) — empty if no GPS for this sample |
| 12 | `gps_lon` | float64 | degrees | Longitude (WGS84) — empty if no GPS |
| 13 | `gps_alt_m` | float64 | meters | Altitude (MSL) |
| 14 | `gps_spd_kmh` | float64 | km/h | Ground speed |
| 15 | `gps_course_deg` | float64 | degrees | Course over ground (0-360, true north) |
| 16 | `gps_hdop` | float64 | — | Horizontal dilution of precision |
| 17 | `gps_sats` | uint32 | — | Number of satellites |
| 18 | `gps_x_m` | float64 | meters | East-North-Up X (east, relative to track start) |
| 19 | `gps_y_m` | float64 | meters | East-North-Up Y (north, relative to track start) |
| 20 | `gps_z_m` | float64 | meters | East-North-Up Z (up, relative to track start) |
| 21 | `gps_vn_mps` | float64 | m/s | Velocity north component |
| 22 | `gps_ve_mps` | float64 | m/s | Velocity east component |
| 23 | `gps_fix_epoch_s` | uint32 | seconds | Epoch of the anchor GPS fix used |
| 24 | `gps_fix_ms_since_boot` | uint32 | ms | Boot-relative time of anchor GPS fix |
| 25 | `gps_age_ms` | uint32 | ms | Age of GPS data relative to this IMU sample |
| 26 | `gps_source` | uint8 | — | 0=no GPS, 1=interpolated, 2=real fix |
| 27 | `lap_count` | uint32 | — | Completed laps (0 = no laps completed yet) |
| 28 | `lap_ms` | uint32 | ms | Time elapsed in current lap |
| 29 | `last_lap_ms` | uint32 | ms | Duration of the last completed lap (0 if none) |
| 30 | `lap_distance_m` | float32 | meters | Distance traveled in current lap (Euclidean sum of GPS segments in ENU) |
| 31 | `track_state` | uint8 | — | State machine: 0=IDLE, 1=RECORDING, 2=READY, 3=RACING |
| 32 | `lap_active` | uint8 | — | 1 if currently timing a lap, 0 otherwise |
| 33 | `event_flags` | uint16 | bitmask | Event flags (see below) |
| 34 | `track_id` | uint32 | — | Track ID (matches `tracks.csv`) |

### Parsing Notes

- **Empty fields:** ~99.9% of rows are IMU-only. When there is no GPS data for a row, columns 11-22 (`gps_lat` through `gps_ve_mps`) are empty (consecutive commas). Parsers should treat empty fields as null/NaN, not zero.
- **Use a proper CSV parser.** Do not split on commas naively — column count must always be 34. If a row has fewer columns, it's from a firmware bug (fixed in latest version) and should be skipped.
- **Primary time axis:** Use `ms_since_boot` (column 2) as the monotonic time reference. `epoch_s` can have duplicates or small jumps due to GPS time interpolation.
- **To convert `ax_g` to m/s²:** multiply by 9.80665.
- **IMU axis orientation** depends on how the device is physically mounted. The QMI8658 chip axes are fixed relative to the PCB — they are NOT automatically aligned to vehicle lateral/longitudinal. apexDirector should provide axis mapping configuration.

### Event Flags (column 33, bitmask)

| Bit | Value | Name | Description |
|-----|-------|------|-------------|
| 0 | 1 | `LAP_START` | Start/finish line crossed, new lap started |
| 1 | 2 | `LAP_END` | Lap completed (always paired with LAP_START) |
| 2 | 4 | `RECOGNIZED` | Track recognized, RACING mode entered |
| 3 | 8 | `GPS_GAP` | GPS interpolation not available (gap between fixes) |
| 4 | 16 | `GPS_DEGRADED` | GPS fix has poor quality (HDOP > 2.5 or sats < 6) |
| 5 | 32 | `GPS_INVALID` | No valid GPS fix available |

### Track State Machine (column 31)

```
IDLE (0) ──button──► RECORDING (1) ──circuit complete──► READY (2)
                                                            │
IDLE (0) ◄──auto-detect nearest saved track────────────► READY (2)
                                                            │
                                                    cross start line
                                                            │
                                                        RACING (3)
                                                            │
                                              (laps counted until power-off)
```

**Important:** A race file may contain rows with multiple states. The race logger starts when a track is first recognized (READY → RACING transition). If the user records a new track during the same session, RECORDING (1) rows will also appear in the file. **For lap analysis, filter to `track_state == 3` (RACING) rows only.**

### GPS Source Values (column 26)

| Value | Meaning | GPS columns (11-22) |
|-------|---------|---------------------|
| 0 | No GPS available | All empty |
| 1 | Interpolated between two good fixes | Filled (computed) |
| 2 | Real GPS fix (within 10ms of actual fix time) | Filled (measured) |

GPS is received at ~1 Hz. Between fixes, positions are linearly interpolated in ENU space when both the previous and next fixes are "good" (HDOP ≤ 2.5, sats ≥ 6). At 100 Hz IMU rate, roughly 1 in 100 rows will have `gps_source=2`.

### ENU Coordinate System (columns 18-20)

Positions are projected to a local East-North-Up coordinate system with the origin at the track's start/finish line:
- **x_m**: East displacement (positive = east)
- **y_m**: North displacement (positive = north)
- **z_m**: Altitude displacement (positive = up)

This makes it easy to plot the track layout in meters without geodetic math.

### Lap Timing

- `lap_count` increments each time the car crosses the start/finish line while in RACING state
- `lap_ms` is a running timer for the current lap (resets on each crossing)
- `last_lap_ms` holds the duration of the most recently completed lap
- `lap_distance_m` accumulates distance during the current lap (haversine sum of GPS positions)
- A lap crossing is detected when: distance to start < 30m AND speed > 5 km/h AND heading within ±45° of start heading AND previously left > 50m radius AND at least 5 seconds since last crossing

### How to Extract Lap Splits

1. Filter rows to `track_state == 3` (RACING)
2. Look for rows where `event_flags & 2` (LAP_END bit set)
3. On those rows, `last_lap_ms` contains the completed lap duration in milliseconds
4. `lap_count` on those rows gives the lap number

### Example

```csv
epoch_s,ms_since_boot,imu_ts,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,imu_temp_C,gps_lat,gps_lon,gps_alt_m,gps_spd_kmh,gps_course_deg,gps_hdop,gps_sats,gps_x_m,gps_y_m,gps_z_m,gps_vn_mps,gps_ve_mps,gps_fix_epoch_s,gps_fix_ms_since_boot,gps_age_ms,gps_source,lap_count,lap_ms,last_lap_ms,lap_distance_m,track_state,lap_active,event_flags,track_id
1772244856,67230,59684,-0.163,0.489,0.744,9.928,10.686,5.146,31.15,,,,,,,,,,,,,1772244856,69202,,0,0,0,0,0.00,3,1,4,2495971338
1772244856,67240,59751,-0.116,0.511,0.786,3.918,4.686,-2.506,31.16,,,,,,,,,,,,,1772244856,69202,,0,0,10,0,0.00,3,1,0,2495971338
```

- First row: `event_flags=4` → `RECOGNIZED` bit set, RACING mode just entered
- Second row: `event_flags=0` → normal RACING row, `lap_ms=10` (10ms into first lap)
- GPS columns (11-22) are empty because `gps_source=0` (IMU-only row)

---

## Hardware Reference

| Component | Details |
|-----------|---------|
| MCU | ESP32-S3 (LilyGo T-Beam Supreme) |
| GPS | u-blox GNSS, 1 Hz update rate, UART |
| IMU | QMI8658, 100 Hz accel+gyro, SPI |
| Storage | SD card (FAT32), SPI |
| Accel range | ±4g (configured) |
| Accel resolution | 4.0 / 32768 g per LSB |
| Gyro range | ±64 °/s (configured) |
| Gyro resolution | 64.0 / 32768 °/s per LSB |
| GPS datum | WGS84 |

### IMU Axis Orientation

The QMI8658 axes are fixed relative to the T-Beam PCB. The physical mapping to vehicle axes depends on how the device is mounted. The IMU is NOT automatically aligned to lateral/longitudinal — the consuming application should allow the user to configure axis mapping and sign conventions.

---

## Comparison with AIM Solo2

| Feature | T-Beam Datalogger | AIM Solo2 DL |
|---------|-------------------|--------------|
| GPS rate | 1 Hz | 10 Hz |
| IMU rate | 100 Hz | 100 Hz |
| Accel axes | 3 (X/Y/Z) | 3 (X/Y/Z) |
| Gyro axes | 3 (X/Y/Z) | 3 (X/Y/Z) |
| Accel unit | g | g |
| Lap timing | GPS-based (30m zone) | GPS-based |
| Output format | CSV (this spec) | AIM .drk / RaceStudio CSV |
| Coordinate system | ENU from start + lat/lon | Lat/Lon |
