# ATP File Format Specification

**ApexDirector Telemetry Protocol — Binary File Format**
**Version:** 1
**Extension:** `.atp`
**Status:** Draft
**Last updated:** February 2026

---

## Overview

ATP is a self-describing binary file format for motorsport telemetry. It stores multi-rate time-series data (IMU, GPS, ECU, sensors) organized in 1-second chunks with index tables for random access by time or lap.

**Design goals:**
- **Self-describing** — header contains all channel definitions, units, and rates
- **Multi-rate** — each channel runs at its own frequency (1-200 Hz)
- **Seekable** — chunk index and lap index enable random access without full scan
- **Streamable** — can be written incrementally and partially read (power-loss safe)
- **Compact** — ~3-4x smaller than equivalent CSV
- **Open** — this spec is public; anyone can write a reader or writer
- **Extensible** — add new channels or record types without breaking existing readers

**Non-goals:**
- Compression (can be applied externally; the format is already compact)
- Encryption (handled at the transport layer, not the file layer)

---

## File Structure

```
┌──────────────────────────────┐
│  File Header (16 bytes)      │  Magic, version, flags
├──────────────────────────────┤
│  Session Metadata (variable) │  Driver, track, vehicle, timestamps
├──────────────────────────────┤
│  Channel Table (variable)    │  Channel definitions
├──────────────────────────────┤
│  Track Definition (variable) │  Start/finish GPS, track outline
├──────────────────────────────┤
│  Data Section                │
│  ┌────────────────────────┐  │
│  │  Chunk 0 (second 0)   │  │  IMU samples + GPS + events
│  ├────────────────────────┤  │
│  │  Chunk 1 (second 1)   │  │
│  ├────────────────────────┤  │
│  │  ...                   │  │
│  ├────────────────────────┤  │
│  │  Chunk N (second N)    │  │
│  └────────────────────────┘  │
├──────────────────────────────┤
│  Chunk Index (variable)      │  Offset table for time-based access
├──────────────────────────────┤
│  Lap Index (variable)        │  Lap → chunk mapping
├──────────────────────────────┤
│  Footer (32 bytes)           │  Offsets, totals, integrity
└──────────────────────────────┘
```

All multi-byte integers are **little-endian**. All floating-point values use IEEE 754.

---

## 1. File Header

**Size:** 16 bytes, fixed.

| Offset | Type | Size | Field | Description |
|--------|------|------|-------|-------------|
| 0 | char[4] | 4 | `magic` | `"ATP\0"` (0x41, 0x54, 0x50, 0x00) |
| 4 | uint16 | 2 | `version` | Format version (currently 1) |
| 6 | uint16 | 2 | `flags` | Bitfield (see below) |
| 8 | uint32 | 4 | `header_size` | Total bytes from start of file to first data chunk |
| 12 | uint32 | 4 | `data_offset` | Byte offset where data section begins (same as header_size) |

### Flags (uint16 bitfield)

| Bit | Name | Description |
|-----|------|-------------|
| 0 | `HAS_TRACK_RECORDING` | Track outline points are included |
| 1 | `HAS_CAN_DATA` | CAN/ECU channels present |
| 2 | `HAS_ANALOG_SENSORS` | Analog sensor channels present (potentiometers, pressure) |
| 3 | `SESSION_COMPLETE` | Set when session is cleanly closed (footer is valid) |
| 4-15 | Reserved | Must be 0 |

If `SESSION_COMPLETE` is not set, the file was not cleanly closed (power loss, crash). Readers should reconstruct the chunk index by scanning the data section.

---

## 2. Session Metadata

Immediately follows the file header. Variable length.

| Offset | Type | Size | Field | Description |
|--------|------|------|-------|-------------|
| 0 | uint16 | 2 | `metadata_size` | Total size of this section in bytes |
| 2 | uint8 | 1 | `source` | Data source (see enum below) |
| 3 | uint8 | 1 | `session_type` | Session type (see enum below) |
| 4 | uint64 | 8 | `start_time_ms` | Session start time (Unix milliseconds, UTC) |
| 12 | uint64 | 8 | `end_time_ms` | Session end time (0 if not yet closed) |
| 20 | uint32 | 4 | `duration_ms` | Session duration in milliseconds (0 if not yet closed) |
| 24 | uint32 | 4 | `track_id` | Track ID (FNV1a hash, same as T-Beam track system) |
| 28 | lstring | var | `driver_name` | Driver name |
| var | lstring | var | `track_name` | Track name |
| var | lstring | var | `vehicle_name` | Vehicle name |
| var | lstring | var | `session_notes` | Free-form notes (empty string if none) |
| var | uint16 | 2 | `extra_count` | Number of extra key-value pairs |
| var | lstring pairs | var | Extra metadata (key, value) × extra_count |

### Length-Prefixed String (`lstring`)

| Offset | Type | Size | Field |
|--------|------|------|-------|
| 0 | uint16 | 2 | Length in bytes (N) |
| 2 | char[N] | N | UTF-8 string content (NOT null-terminated) |

An empty string is encoded as `00 00` (length = 0, no content bytes).

### Source Enum (uint8)

| Value | Name | Description |
|-------|------|-------------|
| 1 | `CORE_PRO` | ApexDirector Core Pro hardware |
| 2 | `AIM` | Converted from AIM Solo2DL .xrk |
| 3 | `LMU` | Converted from Le Mans Ultimate .duckdb |
| 4 | `ACC` | Converted from Assetto Corsa Competizione |
| 5 | `AC1` | Converted from Assetto Corsa (original) |
| 6 | `IRACING` | Converted from iRacing .ibt |
| 7-254 | Reserved | Future sources |
| 255 | `OTHER` | Unknown or third-party source |

### Session Type Enum (uint8)

| Value | Name |
|-------|------|
| 0 | `UNKNOWN` |
| 1 | `PRACTICE` |
| 2 | `QUALIFYING` |
| 3 | `RACE` |
| 4 | `TEST` |

---

## 3. Channel Table

Declares all channels present in this file. The reader uses this to know what data to expect in each chunk.

| Offset | Type | Size | Field | Description |
|--------|------|------|-------|-------------|
| 0 | uint16 | 2 | `table_size` | Total size of this section in bytes |
| 2 | uint16 | 2 | `channel_count` | Number of channel definitions |
| 4 | Channel[N] | var | channels | Channel definitions |

### Channel Definition

| Offset | Type | Size | Field | Description |
|--------|------|------|-------|-------------|
| 0 | uint16 | 2 | `channel_id` | Unique numeric ID (used in data chunks) |
| 2 | lstring | var | `canonical_name` | Canonical name (e.g., `"dynamics.accel.lat"`) |
| var | uint8 | 1 | `unit` | Unit enum (see below) |
| var | uint8 | 1 | `data_type` | Data type enum (see below) |
| var | uint16 | 2 | `sample_rate_hz` | Sample rate in Hz (0 = event-based) |
| var | uint8 | 1 | `flags` | Channel flags (see below) |

### Unit Enum (uint8)

| Value | Name | Symbol | Description |
|-------|------|--------|-------------|
| 0 | `NONE` | — | Dimensionless |
| 1 | `G` | g | G-force (1g = 9.80665 m/s2) |
| 2 | `DEG_PER_S` | deg/s | Angular velocity |
| 3 | `DEG_C` | degC | Temperature |
| 4 | `DEG` | deg | Angle (heading, steering) |
| 5 | `M_PER_S` | m/s | Velocity |
| 6 | `KM_PER_H` | km/h | Velocity |
| 7 | `RPM` | rpm | Revolutions per minute |
| 8 | `PERCENT` | % | Percentage (0-100) |
| 9 | `BAR` | bar | Pressure |
| 10 | `METERS` | m | Distance |
| 11 | `MM` | mm | Small distance |
| 12 | `SECONDS` | s | Time |
| 13 | `MS` | ms | Time (milliseconds) |
| 14 | `NEWTONS` | N | Force |
| 15 | `VOLTS` | V | Voltage |
| 16 | `AMPS` | A | Current |
| 17 | `LITERS` | L | Volume |
| 18 | `LAMBDA` | lambda | Air-fuel ratio |
| 19 | `GEAR` | — | Gear number (-1=R, 0=N, 1-8) |
| 20 | `BOOLEAN` | — | 0 or 1 |
| 21-254 | Reserved | | |
| 255 | `CUSTOM` | — | Unit specified in channel name |

### Data Type Enum (uint8)

| Value | Name | Size | Range |
|-------|------|------|-------|
| 1 | `FLOAT32` | 4 | IEEE 754 single-precision |
| 2 | `FLOAT64` | 8 | IEEE 754 double-precision |
| 3 | `INT8` | 1 | -128 to 127 |
| 4 | `UINT8` | 1 | 0 to 255 |
| 5 | `INT16` | 2 | -32768 to 32767 |
| 6 | `UINT16` | 2 | 0 to 65535 |
| 7 | `INT32` | 4 | -2^31 to 2^31-1 |
| 8 | `UINT32` | 4 | 0 to 2^32-1 |

### Channel Flags (uint8 bitfield)

| Bit | Name | Description |
|-----|------|-------------|
| 0 | `IS_PER_CORNER` | 4 values per sample: FL, FR, RL, RR |
| 1 | `IS_GPS` | GPS-rate channel (present only in GPS records) |
| 2 | `IS_EVENT` | Event-based (present only when state changes) |
| 3-7 | Reserved | Must be 0 |

---

## 4. Track Definition

| Offset | Type | Size | Field | Description |
|--------|------|------|-------|-------------|
| 0 | uint16 | 2 | `section_size` | Total size of this section |
| 2 | uint32 | 4 | `track_id` | Track ID (matches session metadata) |
| 6 | float64 | 8 | `start_lat` | Start/finish line latitude (WGS84) |
| 14 | float64 | 8 | `start_lon` | Start/finish line longitude |
| 22 | float64 | 8 | `start_alt` | Altitude at start/finish (MSL, meters) |
| 30 | float64 | 8 | `start_heading` | Expected heading when crossing (degrees, true north) |
| 38 | uint16 | 2 | `recording_points` | Number of track outline GPS points (0 if none) |
| 40 | TrackPoint[N] | var | Track outline points |

### Track Point (24 bytes each)

| Offset | Type | Size | Field |
|--------|------|------|-------|
| 0 | float64 | 8 | `lat` |
| 8 | float64 | 8 | `lon` |
| 16 | float32 | 4 | `alt` |
| 20 | float32 | 4 | `speed_kmh` |

If `HAS_TRACK_RECORDING` flag is not set, `recording_points` is 0 and no TrackPoint data follows.

---

## 5. Data Section — Chunks

The data section consists of sequential **1-second chunks**. Each chunk contains all samples recorded during that 1-second window, at their respective rates.

### Chunk Header

| Offset | Type | Size | Field | Description |
|--------|------|------|-------|-------------|
| 0 | char[4] | 4 | `chunk_magic` | `"CHK\0"` (0x43, 0x48, 0x4B, 0x00) |
| 4 | uint32 | 4 | `chunk_size` | Total chunk size in bytes (including this header) |
| 8 | uint32 | 4 | `timestamp_ms` | Milliseconds since session start |
| 12 | uint8 | 1 | `record_count` | Number of records in this chunk |
| 13 | uint8 | 1 | `flags` | Chunk flags (see below) |
| 14 | uint16 | 2 | `lap_number` | Current lap at start of this chunk |

### Chunk Flags (uint8 bitfield)

| Bit | Name | Description |
|-----|------|-------------|
| 0 | `HAS_GPS` | This chunk contains at least one GPS record |
| 1 | `HAS_LAP_EVENT` | A lap crossing occurred during this chunk |
| 2 | `HAS_STATE_CHANGE` | Track state changed during this chunk |
| 3-7 | Reserved | |

### Records Within a Chunk

After the chunk header, records are written sequentially. Each record starts with a **Record Header**:

| Offset | Type | Size | Field | Description |
|--------|------|------|-------|-------------|
| 0 | uint8 | 1 | `record_type` | Record type (see enum below) |
| 1 | uint16 | 2 | `record_size` | Total record size including this header |
| 3 | uint16 | 2 | `time_offset_ms` | Offset within this chunk (0-999 ms) |

### Record Type Enum (uint8)

| Value | Name | Description |
|-------|------|-------------|
| 0x01 | `IMU_BATCH` | Batch of IMU samples (typically 100 per chunk) |
| 0x02 | `GPS_FIX` | Single GPS fix (typically 1 per chunk) |
| 0x03 | `LAP_EVENT` | Lap crossing event |
| 0x04 | `STATE_CHANGE` | Track state change |
| 0x05 | `CAN_BATCH` | Batch of CAN/ECU samples |
| 0x06 | `ANALOG_BATCH` | Batch of analog sensor samples |
| 0x07-0xFE | Reserved | Future record types |
| 0xFF | `CUSTOM` | Vendor-specific data |

---

### Record Type 0x01: IMU Batch

A batch of IMU samples for this chunk. Samples are evenly spaced within the chunk at the declared IMU sample rate (typically 100 Hz = 100 samples per chunk).

| Offset | Type | Size | Field | Description |
|--------|------|------|-------|-------------|
| 0 | uint8 | 1 | `record_type` | 0x01 |
| 1 | uint16 | 2 | `record_size` | 5 + (sample_count × sample_size) |
| 3 | uint16 | 2 | `time_offset_ms` | Time of first sample in this batch |
| 5 | uint8 | 1 | `sample_count` | Number of IMU samples |
| 6 | uint8 | 1 | `sample_size` | Bytes per sample (see below) |

Followed by `sample_count` IMU samples. Each sample (14 bytes with default channels):

| Offset | Type | Size | Field | Canonical ID | Unit |
|--------|------|------|-------|-------------|------|
| 0 | int16 | 2 | `ax` | `dynamics.accel.lon` | ÷8192 → G |
| 2 | int16 | 2 | `ay` | `dynamics.accel.lat` | ÷8192 → G |
| 4 | int16 | 2 | `az` | `dynamics.accel.vert` | ÷8192 → G |
| 6 | int16 | 2 | `gx` | `dynamics.roll.rate` | ÷512 → °/s |
| 8 | int16 | 2 | `gy` | `dynamics.pitch.rate` | ÷512 → °/s |
| 10 | int16 | 2 | `gz` | `dynamics.yaw.rate` | ÷512 → °/s |
| 12 | int16 | 2 | `temp` | `environment.imu.temp` | ÷100 → °C |

**Fixed-point encoding:**
- Accelerometer: int16 with ±4g range → divide by 8192 to get G (resolution: 0.000122 G)
- Gyroscope: int16 with ±64°/s range → divide by 512 to get °/s (resolution: 0.00195 °/s)
- Temperature: int16 → divide by 100 to get °C (resolution: 0.01 °C)

This is more compact than float32 (14 bytes vs 28 bytes per sample) with no meaningful loss of precision for motorsport telemetry.

**100 IMU samples per chunk = 100 × 14 = 1,400 bytes.**

---

### Record Type 0x02: GPS Fix

A single GPS fix (typically 1 per chunk at 1 Hz).

| Offset | Type | Size | Field | Canonical ID | Unit |
|--------|------|------|-------|-------------|------|
| 0 | uint8 | 1 | `record_type` | 0x02 | |
| 1 | uint16 | 2 | `record_size` | | |
| 3 | uint16 | 2 | `time_offset_ms` | | ms within chunk |
| 5 | float64 | 8 | `lat` | `position.gps.lat` | degrees |
| 13 | float64 | 8 | `lon` | `position.gps.lon` | degrees |
| 21 | float32 | 4 | `alt` | `position.gps.alt` | meters MSL |
| 25 | float32 | 4 | `speed` | `position.gps.speed` | m/s |
| 29 | float32 | 4 | `course` | `position.gps.course` | degrees |
| 33 | uint16 | 2 | `hdop` | — | ×100 (e.g., 125 = 1.25) |
| 35 | uint8 | 1 | `sats` | — | satellite count |
| 36 | float32 | 4 | `enu_x` | `position.local.x` | meters east |
| 40 | float32 | 4 | `enu_y` | `position.local.y` | meters north |
| 44 | float32 | 4 | `enu_z` | `position.local.z` | meters up |
| 48 | float32 | 4 | `vel_north` | `dynamics.vel.north` | m/s |
| 52 | float32 | 4 | `vel_east` | `dynamics.vel.east` | m/s |
| 56 | float32 | 4 | `lap_distance` | `position.lap.dist` | meters |

**Total:** 60 bytes per GPS record.

GPS speed is stored in m/s (SI). The reader converts to km/h for display. This matches the canonical schema unit (`m/s`).

---

### Record Type 0x03: Lap Event

Written when the car crosses the start/finish line.

| Offset | Type | Size | Field | Canonical ID |
|--------|------|------|-------|-------------|
| 0 | uint8 | 1 | `record_type` | 0x03 |
| 1 | uint16 | 2 | `record_size` | 19 |
| 3 | uint16 | 2 | `time_offset_ms` | ms within chunk |
| 5 | uint16 | 2 | `lap_number` | `event.lap` |
| 7 | uint32 | 4 | `lap_time_ms` | `event.lap.time` |
| 11 | float32 | 4 | `lap_distance_m` | — |
| 15 | uint16 | 2 | `event_flags` | — (bitmask) |
| 17 | uint16 | 2 | `reserved` | — |

---

### Record Type 0x04: State Change

Written when the track state machine transitions.

| Offset | Type | Size | Field |
|--------|------|------|-------|
| 0 | uint8 | 1 | `record_type` | 0x04 |
| 1 | uint16 | 2 | `record_size` | 8 |
| 3 | uint16 | 2 | `time_offset_ms` |
| 5 | uint8 | 1 | `old_state` |
| 6 | uint8 | 1 | `new_state` |
| 7 | uint8 | 1 | `reserved` |

State values: 0=IDLE, 1=RECORDING, 2=READY, 3=RACING.

---

### Record Type 0x05: CAN Batch

Batch of CAN/ECU samples. Used when the Core Pro is connected to the vehicle's CAN bus.

| Offset | Type | Size | Field |
|--------|------|------|-------|
| 0 | uint8 | 1 | `record_type` | 0x05 |
| 1 | uint16 | 2 | `record_size` | 5 + (channel_count × sample_count × value_size) |
| 3 | uint16 | 2 | `time_offset_ms` |
| 5 | uint8 | 1 | `channel_count` | How many CAN channels in this batch |
| 6 | uint8 | 1 | `sample_count` | Samples per channel |

Followed by `channel_count` channel descriptors:

| Type | Size | Field |
|------|------|-------|
| uint16 | 2 | `channel_id` | References channel table |

Followed by interleaved sample data: `sample_count` rounds of `channel_count` values, each value sized according to the channel's declared `data_type` in the channel table.

**Example:** If CAN provides RPM (uint16), gear (int8), throttle (uint8) at 50 Hz:
- `channel_count` = 3
- `sample_count` = 50
- Data: 50 × (2 + 1 + 1) = 200 bytes

---

### Record Type 0x06: Analog Batch

Batch of analog sensor samples (potentiometers, pressure sensors).

Same structure as CAN Batch (0x05) — `channel_count`, `sample_count`, channel IDs, then interleaved values.

**Example:** 4 suspension potentiometers (per-corner, uint16) at 100 Hz:
- `channel_count` = 4 (or 1 per-corner channel with IS_PER_CORNER flag)
- `sample_count` = 100
- Data: 100 × 4 × 2 = 800 bytes

For per-corner channels (IS_PER_CORNER flag), each sample contains 4 values in order: FL, FR, RL, RR.

---

## 6. Chunk Index

Written after all data chunks. Enables random access by time.

| Offset | Type | Size | Field |
|--------|------|------|-------|
| 0 | char[4] | 4 | `"IDX\0"` magic |
| 4 | uint32 | 4 | `entry_count` | Number of chunks |
| 8 | IndexEntry[N] | var | One per chunk |

### Index Entry (12 bytes each)

| Offset | Type | Size | Field |
|--------|------|------|-------|
| 0 | uint32 | 4 | `timestamp_ms` | Chunk start time (ms since session start) |
| 4 | uint32 | 4 | `file_offset` | Byte offset from start of file |
| 8 | uint32 | 4 | `chunk_size` | Chunk size in bytes |

**Random access by time:** Binary search the index by `timestamp_ms`, then seek to `file_offset`.

---

## 7. Lap Index

Written after the chunk index. Enables random access by lap.

| Offset | Type | Size | Field |
|--------|------|------|-------|
| 0 | char[4] | 4 | `"LAP\0"` magic |
| 4 | uint16 | 2 | `lap_count` | Number of completed laps |
| 6 | LapEntry[N] | var | One per completed lap |

### Lap Entry (16 bytes each)

| Offset | Type | Size | Field |
|--------|------|------|-------|
| 0 | uint16 | 2 | `lap_number` |
| 2 | uint32 | 4 | `start_chunk` | Index of chunk where this lap starts |
| 6 | uint32 | 4 | `end_chunk` | Index of chunk where this lap ends |
| 10 | uint32 | 4 | `lap_time_ms` | Lap duration in milliseconds |
| 14 | uint16 | 2 | `flags` | Lap flags (in_lap, out_lap, invalid, etc.) |

**Random access by lap:** Look up `start_chunk` in the chunk index, seek to that file offset.

---

## 8. Footer

Written last when the session is cleanly closed. Fixed size: 32 bytes.

| Offset | Type | Size | Field | Description |
|--------|------|------|-------|-------------|
| 0 | char[4] | 4 | `magic` | `"ATP\0"` |
| 4 | uint32 | 4 | `chunk_index_offset` | File offset of chunk index |
| 8 | uint32 | 4 | `lap_index_offset` | File offset of lap index |
| 12 | uint32 | 4 | `total_chunks` | Number of data chunks |
| 16 | uint16 | 2 | `total_laps` | Number of completed laps |
| 18 | uint16 | 2 | `reserved` | 0 |
| 20 | uint32 | 4 | `total_duration_ms` | Session duration |
| 24 | uint32 | 4 | `total_samples` | Total IMU samples written |
| 28 | uint32 | 4 | `checksum` | CRC32 of all data chunks |

---

## 9. Power Loss Recovery

If the `SESSION_COMPLETE` flag is not set in the file header, the footer and indices are missing or incomplete. Recovery procedure:

1. Read file header, metadata, channel table, and track definition (always written first)
2. Seek to `data_offset`
3. Scan for chunk magic (`"CHK\0"`) sequentially
4. Rebuild the chunk index from discovered chunks
5. Rebuild the lap index from LAP_EVENT records within chunks
6. All complete chunks are recoverable — only the last partial chunk is lost

---

## 10. T-Beam CSV → ATP Channel Mapping

Maps current T-Beam CSV columns (from `datalogger-files/FILE_FORMAT_SPEC.md`) to ATP canonical IDs.

| CSV Column | ATP Canonical ID | Record Type | Unit | Notes |
|-----------|-----------------|-------------|------|-------|
| `epoch_s` | (timestamp) | — | — | Used to derive `start_time_ms` |
| `ms_since_boot` | (timestamp) | — | — | Primary time reference → `timestamp_ms` |
| `imu_ts` | — | — | — | Internal counter, not stored in ATP |
| `ax_mps2` | `dynamics.accel.lon` | IMU_BATCH | G | Firmware outputs m/s2; convert ÷9.80665 |
| `ay_mps2` | `dynamics.accel.lat` | IMU_BATCH | G | Same conversion |
| `az_mps2` | `dynamics.accel.vert` | IMU_BATCH | G | Same conversion |
| `gx_dps` | `dynamics.roll.rate` | IMU_BATCH | °/s | Direct |
| `gy_dps` | `dynamics.pitch.rate` | IMU_BATCH | °/s | Direct |
| `gz_dps` | `dynamics.yaw.rate` | IMU_BATCH | °/s | Direct |
| `imu_temp_C` | `environment.imu.temp` | IMU_BATCH | °C | Direct |
| `gps_lat` | `position.gps.lat` | GPS_FIX | degrees | Direct |
| `gps_lon` | `position.gps.lon` | GPS_FIX | degrees | Direct |
| `gps_alt_m` | `position.gps.alt` | GPS_FIX | meters | Direct |
| `gps_spd_kmh` | `position.gps.speed` | GPS_FIX | m/s | Convert ÷3.6 |
| `gps_course_deg` | `position.gps.course` | GPS_FIX | degrees | Direct |
| `gps_hdop` | (quality metadata) | GPS_FIX | — | Stored as uint16 ×100 |
| `gps_sats` | (quality metadata) | GPS_FIX | — | Stored as uint8 |
| `gps_x_m` | `position.local.x` | GPS_FIX | meters | ENU east |
| `gps_y_m` | `position.local.y` | GPS_FIX | meters | ENU north |
| `gps_z_m` | `position.local.z` | GPS_FIX | meters | ENU up |
| `gps_vn_mps` | `dynamics.vel.north` | GPS_FIX | m/s | Direct |
| `gps_ve_mps` | `dynamics.vel.east` | GPS_FIX | m/s | Direct |
| `gps_fix_epoch_s` | — | — | — | Internal, not stored |
| `gps_fix_ms_since_boot` | — | — | — | Internal, not stored |
| `gps_age_ms` | — | — | — | Internal, not stored |
| `gps_source` | — | — | — | Implicit: GPS_FIX presence = has GPS |
| `lap_count` | `event.lap` | LAP_EVENT | — | Via lap events |
| `lap_ms` | (derived) | — | — | Calculated from chunk timestamps |
| `last_lap_ms` | `event.lap.time` | LAP_EVENT | ms | Via lap events |
| `lap_distance_m` | `position.lap.dist` | GPS_FIX | meters | Direct |
| `track_state` | — | STATE_CHANGE | — | Via state change events |
| `lap_active` | — | — | — | Derived from track_state == RACING |
| `event_flags` | — | LAP_EVENT | — | Bitmask in lap events |
| `track_id` | (track definition) | — | — | In session metadata |

---

## 11. Comparison: ATP vs Other Formats

| Feature | ATP (.atp) | AIM (.xrk) | LMU (.duckdb) | T-Beam CSV |
|---------|-----------|------------|---------------|------------|
| **Type** | Binary, chunked | Binary, message-based | SQL database | Text |
| **Self-describing** | Yes (channel table) | Partially (needs XRK parser) | Yes (SQL schema) | Yes (CSV header) |
| **Multi-rate** | Yes (per-channel rate) | Yes (per-channel rate) | Yes (per-table rate) | No (single rate, empty fields) |
| **Random access** | Yes (chunk + lap index) | Limited | Yes (SQL queries) | No (sequential scan) |
| **Streamable** | Yes (chunk-by-chunk) | No | No | Yes (line-by-line) |
| **Open spec** | Yes (this document) | No (reverse-engineered) | Yes (DuckDB + LMU schema) | Yes |
| **Power-loss safe** | Yes (per-chunk write) | Unknown | Unknown | Yes (per-line write) |
| **Size (1 hr session)** | ~9 MB (IMU+GPS) | ~15-25 MB | ~75 MB | ~30 MB (CSV) |
| **Extensible** | Yes (new record types) | No (fixed format) | Yes (new tables) | Limited (new columns) |
| **Compression** | Optional (external) | zlib (in .xrz) | DuckDB internal | None |
| **Per-corner data** | Flag in channel def | Separate channels | 4-value columns | N/A (no corner data yet) |

### Size Analysis — Current T-Beam Data

Using the actual race files in `datalogger-files/`:

**race-20260228-0214.csv** (39,500 rows = ~395 seconds at 100 Hz):

| Format | Size | Notes |
|--------|------|-------|
| Current CSV | 4.0 MB | 34 columns, mostly empty GPS fields |
| ATP (projected) | ~0.6 MB | 100 IMU × 14B × 395s + 395 GPS × 60B + overhead |
| Ratio | **6.7× smaller** | |

**race-20260228-0209.csv** (17,756 rows = ~178 seconds):

| Format | Size | Notes |
|--------|------|-------|
| Current CSV | 1.8 MB | |
| ATP (projected) | ~0.3 MB | |
| Ratio | **6× smaller** | |

When CAN/ECU data is added (RPM, gear, throttle, brake, suspension × 4), the ATP advantage grows because binary packing of these channels is far more efficient than CSV text representation.

---

## 12. Reference: Writing an ATP File (Pseudocode)

### Writer (ESP32 firmware / converter)

```
OPEN file
WRITE file_header(magic="ATP\0", version=1, flags=0)
WRITE session_metadata(driver, track, vehicle, start_time)
WRITE channel_table(channels[])
WRITE track_definition(start_lat, start_lon, ...)
SAVE data_offset = current position
UPDATE file_header.data_offset = data_offset

chunk_offsets = []

WHILE recording:
    COLLECT 1 second of samples into buffer

    chunk = new Chunk(
        timestamp = current_time - session_start,
        records = []
    )

    chunk.add(IMU_BATCH(samples=imu_buffer))

    IF gps_fix_available:
        chunk.add(GPS_FIX(lat, lon, alt, speed, ...))

    IF lap_crossing:
        chunk.add(LAP_EVENT(lap_number, lap_time, distance))

    IF state_changed:
        chunk.add(STATE_CHANGE(old_state, new_state))

    IF can_data_available:
        chunk.add(CAN_BATCH(channels, samples))

    WRITE chunk to file
    APPEND chunk_offset to chunk_offsets[]

WRITE chunk_index(chunk_offsets)
WRITE lap_index(completed_laps)
WRITE footer(index_offsets, totals, crc32)
SET flag SESSION_COMPLETE in file header
CLOSE file
```

### Reader (Go desktop app)

```
OPEN file
READ file_header → verify magic "ATP\0", check version
READ session_metadata → populate Session struct
READ channel_table → discover available channels
READ track_definition → get track info

IF SESSION_COMPLETE flag set:
    SEEK to footer
    READ footer → get chunk_index_offset, lap_index_offset
    SEEK to chunk_index → read all chunk offsets
    SEEK to lap_index → read lap table

TO READ ALL DATA:
    FOR each chunk in chunk_index:
        SEEK to chunk.file_offset
        READ chunk_header
        FOR each record in chunk:
            SWITCH record_type:
                IMU_BATCH: decode int16 samples → float G/°/s
                GPS_FIX: read float64 lat/lon, float32 fields
                LAP_EVENT: record lap crossing
                CAN_BATCH: decode per channel_table data types
                ANALOG_BATCH: decode per channel_table data types

TO READ SPECIFIC LAP:
    FIND lap in lap_index → get start_chunk, end_chunk
    FOR chunk_index[start_chunk] to chunk_index[end_chunk]:
        READ chunk and decode (same as above)
```

---

## 13. Versioning & Forward Compatibility

- **Version field** in file header allows readers to detect newer formats
- **Unknown record types** (0x07-0xFE) must be skippable — readers use `record_size` to skip unknown records without failing
- **Unknown channel flags** must be ignored by older readers
- **Extra metadata key-value pairs** allow adding session metadata without version bump
- **Channel table** is the extensibility mechanism — new sensors add new channels, not new record types (unless the data structure is fundamentally different)

**Compatibility rule:** A version-1 reader must be able to read any version-1 file, even if it contains channels or flags the reader doesn't recognize. Unknown data is skipped, not rejected.
