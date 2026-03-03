# Core Pro WiFi API Specification

**Version:** 1
**Status:** Draft
**Last updated:** March 2026

---

## Overview

The Core Pro exposes an HTTP + WebSocket API over WiFi for session management, data download, and live telemetry streaming. The API is designed to be consumed by the ApexDirector desktop app but is open — any HTTP client can interact with it.

**Design principles:**
- **JSON for control** — human-readable, debuggable with `curl`, standard tooling
- **Binary for data** — `.atp` files served directly, no transcoding overhead
- **mDNS discovery** — no hardcoded IPs (unlike AIM's `10.0.0.1`)
- **Stateless** — no sessions, no cookies, each request is self-contained
- **Versioned** — all endpoints under `/api/v1/` for future evolution

---

## 1. Discovery

### mDNS Service Advertisement

The Core Pro advertises itself on the local network using mDNS (Bonjour/Avahi):

```
Service type: _apexdirector._tcp.local
Service name: CorePro-{SERIAL}._apexdirector._tcp.local
Port: 80
```

**TXT records:**
```
version=1
firmware=1.0.0
hardware=tbeam-supreme
serial=CP-00001
```

### Discovery from Desktop App (Go)

```go
import "github.com/grandcat/zeroconf"

resolver, _ := zeroconf.NewResolver(nil)
entries := make(chan *zeroconf.ServiceEntry)
resolver.Browse(ctx, "_apexdirector._tcp", "local.", entries)

for entry := range entries {
    // entry.HostName = "CorePro-CP00001.local"
    // entry.AddrIPv4 = [192.168.4.1]
    // entry.Port = 80
    // entry.Text = ["version=1", "firmware=1.0.0", ...]
}
```

### WiFi Modes

**AP Mode (default):**
- SSID: `ApexDirector-{SERIAL}` (e.g., `ApexDirector-CP00001`)
- Security: WPA2-PSK
- Device IP: `192.168.4.1`
- DHCP: built-in, range `192.168.4.2–192.168.4.14`

**Station Mode (configured):**
- Device joins an existing WiFi network
- IP assigned by network DHCP
- Discovered via mDNS

---

## 2. Control Endpoints (REST/JSON)

Base URL: `http://{device_ip}/api/v1`

All responses use `Content-Type: application/json` unless otherwise noted.

### Error Format

All errors return an HTTP error status code with a JSON body:

```json
{
  "error": "not_found",
  "message": "Session abc123 not found",
  "code": 404
}
```

---

### GET /api/v1/device

Returns device information.

**Response:**
```json
{
  "serial": "CP-00001",
  "firmware": "1.0.0",
  "hardware": "tbeam-supreme",
  "protocol_version": 1,
  "capabilities": ["imu", "gps", "fsk_radio", "sd_card"],
  "storage": {
    "total_mb": 3800,
    "used_mb": 245,
    "free_mb": 3555
  },
  "battery": {
    "percent": 78,
    "charging": false,
    "voltage_mv": 3850
  },
  "imu": {
    "accel_range_g": 4,
    "gyro_range_dps": 64,
    "sample_rate_hz": 100
  },
  "gps": {
    "fix": true,
    "sats": 12,
    "hdop": 0.95
  },
  "state": "IDLE"
}
```

---

### GET /api/v1/sessions

Returns a list of all sessions on the device.

**Query parameters:**
- `limit` (int, optional) — max sessions to return (default: all)
- `offset` (int, optional) — skip N sessions (for pagination)

**Response:**
```json
{
  "sessions": [
    {
      "id": "race-20260228-0214",
      "filename": "race-20260228-0214.atp",
      "driver": "Aaron Valdez",
      "track": "Calder Park",
      "vehicle": "Go Kart",
      "session_type": "race",
      "start_time": "2026-02-28T02:14:16Z",
      "duration_ms": 395000,
      "laps": 5,
      "best_lap_ms": 78400,
      "size_bytes": 612000,
      "channels": ["imu", "gps"]
    },
    {
      "id": "race-20260228-0209",
      "filename": "race-20260228-0209.atp",
      "driver": "Aaron Valdez",
      "track": "Street Circuit",
      "vehicle": "Go Kart",
      "session_type": "practice",
      "start_time": "2026-02-28T02:09:19Z",
      "duration_ms": 178000,
      "laps": 2,
      "best_lap_ms": 85200,
      "size_bytes": 284000,
      "channels": ["imu", "gps"]
    }
  ],
  "total": 2
}
```

---

### GET /api/v1/sessions/{id}

Returns detailed session information including the lap table.

**Response:**
```json
{
  "id": "race-20260228-0214",
  "filename": "race-20260228-0214.atp",
  "driver": "Aaron Valdez",
  "track": "Calder Park",
  "vehicle": "Go Kart",
  "session_type": "race",
  "start_time": "2026-02-28T02:14:16Z",
  "duration_ms": 395000,
  "size_bytes": 612000,
  "track_id": 2495971338,
  "channels": [
    {
      "id": 1,
      "canonical_name": "dynamics.accel.lon",
      "unit": "G",
      "rate_hz": 100
    },
    {
      "id": 2,
      "canonical_name": "dynamics.accel.lat",
      "unit": "G",
      "rate_hz": 100
    },
    {
      "id": 7,
      "canonical_name": "position.gps.lat",
      "unit": "deg",
      "rate_hz": 1
    }
  ],
  "laps": [
    {
      "number": 1,
      "time_ms": 85200,
      "distance_m": 1230.5,
      "flags": 0
    },
    {
      "number": 2,
      "time_ms": 82100,
      "distance_m": 1228.3,
      "flags": 0
    },
    {
      "number": 3,
      "time_ms": 78400,
      "distance_m": 1225.1,
      "flags": 0
    }
  ],
  "best_lap": 3,
  "best_lap_ms": 78400
}
```

---

### GET /api/v1/tracks

Returns all saved track definitions.

**Response:**
```json
{
  "tracks": [
    {
      "track_id": 2495971338,
      "track_id_hex": "94C5800A",
      "name": "Calder Park",
      "start_lat": 13.778436,
      "start_lon": -89.191724,
      "start_alt": 492.0,
      "start_heading_deg": 105.26,
      "has_recording": true,
      "recording_points": 148
    },
    {
      "track_id": 3043393695,
      "track_id_hex": "B566809F",
      "name": "Street Circuit",
      "start_lat": 13.778619,
      "start_lon": -89.188273,
      "start_alt": 479.7,
      "start_heading_deg": 105.38,
      "has_recording": true,
      "recording_points": 148
    }
  ]
}
```

---

### POST /api/v1/tracks

Upload a track definition to the device. Allows preloading tracks from the desktop app so the device recognizes them immediately.

**Request body:**
```json
{
  "name": "Imola",
  "start_lat": 44.344236,
  "start_lon": 11.713606,
  "start_alt": 47.0,
  "start_heading_deg": 135.0
}
```

The device computes the `track_id` from the GPS coordinates using the FNV1a hash algorithm.

**Response:**
```json
{
  "track_id": 1847293456,
  "track_id_hex": "6E1BA410",
  "name": "Imola",
  "status": "saved"
}
```

---

### GET /api/v1/config

Returns the current session configuration stored on the device. The desktop app calls this on connect to pre-populate the configuration form with whatever was last pushed.

**When to call:** Immediately after device discovery, before showing the config form to the user. This lets the user see what's already configured and only change what's needed.

**Response:**
```json
{
  "driver_name": "Aaron Valdez",
  "vehicle_name": "Yamaha R3",
  "session_type": "practice",
  "session_notes": "",
  "weather": {
    "ambient_temp_c": 28.5,
    "track_temp_c": 42.0,
    "conditions": "dry"
  }
}
```

**Field details:**

| Field | Type | Description |
|-------|------|-------------|
| `driver_name` | string | Driver name. Empty string if never set. |
| `vehicle_name` | string | Vehicle identifier. Empty string if never set. |
| `session_type` | string | One of: `"practice"`, `"qualifying"`, `"race"`, `"test"`. Defaults to `"practice"`. |
| `session_notes` | string | Free-text notes. Empty string if none. |
| `weather` | object or null | Weather conditions block. `null` if never set. |
| `weather.ambient_temp_c` | float | Ambient air temperature in Celsius. `0` if not set. |
| `weather.track_temp_c` | float | Track surface temperature in Celsius. `0` if not set. |
| `weather.conditions` | string | One of: `"dry"`, `"damp"`, `"wet"`, `"mixed"`. Defaults to `"dry"`. |

**If no config has ever been pushed**, return defaults:
```json
{
  "driver_name": "",
  "vehicle_name": "",
  "session_type": "practice",
  "session_notes": "",
  "weather": null
}
```

**Error responses:**
- `500` — internal error reading config from NVS/flash

---

### POST /api/v1/config

Pushes session configuration to the device. This metadata is embedded into the `.atp` file header when recording starts, so the session file carries driver name, vehicle, weather, etc.

**When to call:** Before the driver goes on track. The desktop app shows a config form; when the user clicks "Push to Device", this endpoint is called.

**Important:** This endpoint should **only work when the device is IDLE** (not recording). If the device is currently recording (`state == "RACING"`), reject with `409 Conflict` — you can't change the config mid-session.

**Request body:**
```json
{
  "driver_name": "Aaron Valdez",
  "vehicle_name": "Yamaha R3",
  "session_type": "practice",
  "session_notes": "Testing new tire compound",
  "weather": {
    "ambient_temp_c": 28.5,
    "track_temp_c": 42.0,
    "conditions": "dry"
  }
}
```

**Field validation:**

| Field | Type | Required | Validation |
|-------|------|----------|------------|
| `driver_name` | string | No | Max 64 chars. Empty string is valid. |
| `vehicle_name` | string | No | Max 64 chars. Empty string is valid. |
| `session_type` | string | No | Must be one of: `"practice"`, `"qualifying"`, `"race"`, `"test"`. Defaults to `"practice"` if omitted or invalid. |
| `session_notes` | string | No | Max 256 chars. Empty string is valid. |
| `weather` | object | No | Can be `null` or omitted entirely. |
| `weather.ambient_temp_c` | float | No | Reasonable range: -40 to 60. No strict enforcement needed. |
| `weather.track_temp_c` | float | No | Reasonable range: -40 to 80. No strict enforcement needed. |
| `weather.conditions` | string | No | Must be one of: `"dry"`, `"damp"`, `"wet"`, `"mixed"`. Defaults to `"dry"` if omitted or invalid. |

**Success response (200):**
```json
{
  "status": "ok",
  "config": {
    "driver_name": "Aaron Valdez",
    "vehicle_name": "Yamaha R3",
    "session_type": "practice",
    "session_notes": "Testing new tire compound",
    "weather": {
      "ambient_temp_c": 28.5,
      "track_temp_c": 42.0,
      "conditions": "dry"
    }
  }
}
```

The `config` field in the response echoes back exactly what was stored, confirming the device received it correctly.

**Error responses:**

- `409 Conflict` — device is currently recording, config cannot be changed:
```json
{
  "error": "recording_active",
  "message": "Cannot update config while recording. Stop the session first.",
  "code": 409
}
```

- `400 Bad Request` — malformed JSON or invalid content-type:
```json
{
  "error": "bad_request",
  "message": "Invalid JSON body",
  "code": 400
}
```

- `500 Internal Server Error` — failed to persist config to NVS/flash:
```json
{
  "error": "storage_error",
  "message": "Failed to save config to NVS",
  "code": 500
}
```

**Storage implementation notes:**

The config should be persisted in NVS (Non-Volatile Storage) so it survives reboots. Suggested NVS keys:
- `cfg_driver` — driver_name (string)
- `cfg_vehicle` — vehicle_name (string)
- `cfg_sesstype` — session_type (string)
- `cfg_notes` — session_notes (string)
- `cfg_amb_temp` — weather.ambient_temp_c (float)
- `cfg_trk_temp` — weather.track_temp_c (float)
- `cfg_conditions` — weather.conditions (string)

When recording starts, read these values from NVS and write them into the `.atp` file header metadata (the `Extra` map in the ATP format).

**curl examples:**

```bash
# Push config
curl -X POST http://192.168.4.1/api/v1/config \
  -H "Content-Type: application/json" \
  -d '{
    "driver_name": "Aaron Valdez",
    "vehicle_name": "Yamaha R3",
    "session_type": "practice",
    "weather": {
      "ambient_temp_c": 28.5,
      "track_temp_c": 42.0,
      "conditions": "dry"
    }
  }'

# Read current config
curl http://192.168.4.1/api/v1/config
```

---

### GET /api/v1/live/status

Returns the current device state.

**Response (when idle):**
```json
{
  "state": "IDLE",
  "recording": false,
  "gps_fix": true,
  "sats": 11,
  "battery_percent": 78
}
```

**Response (when racing):**
```json
{
  "state": "RACING",
  "recording": true,
  "session_id": "race-20260228-0214",
  "track": "Calder Park",
  "track_id": 2495971338,
  "current_lap": 3,
  "lap_time_ms": 42300,
  "last_lap_ms": 82100,
  "best_lap_ms": 78400,
  "speed_kmh": 67.2,
  "gps_fix": true,
  "sats": 11,
  "battery_percent": 72,
  "sd_free_mb": 3540
}
```

---

## 3. Data Endpoints

### GET /api/v1/sessions/{id}/data

Downloads the raw `.atp` binary file.

**Response:**
- `Content-Type: application/octet-stream`
- `Content-Disposition: attachment; filename="race-20260228-0214.atp"`
- `Content-Length: 612000`
- Body: raw `.atp` file bytes

**Progress:** The `Content-Length` header enables the client to show a download progress bar. At ~2 MB/s WiFi throughput, a 10 MB file takes ~5 seconds.

**Example with curl:**
```bash
curl -o session.atp http://192.168.4.1/api/v1/sessions/race-20260228-0214/data
```

---

### WS /api/v1/live/stream

WebSocket endpoint for real-time telemetry streaming. Sends one ATP chunk per second while the device is recording.

**Connection:**
```javascript
const ws = new WebSocket('ws://192.168.4.1/api/v1/live/stream')

ws.onmessage = (event) => {
    // event.data is a binary ATP chunk (same format as in .atp files)
    const chunk = parseATPChunk(event.data)
    updateDashboard(chunk)
}
```

**Message format:** Each WebSocket message is a single binary ATP chunk (chunk header + records). The format is identical to the chunks in an `.atp` file — the same parser handles both.

**First message:** When a client connects, the server sends a JSON handshake message:
```json
{
  "type": "handshake",
  "session_id": "race-20260228-0214",
  "track": "Calder Park",
  "channels": [
    {"id": 1, "name": "dynamics.accel.lon", "unit": "G", "rate_hz": 100},
    {"id": 2, "name": "dynamics.accel.lat", "unit": "G", "rate_hz": 100}
  ]
}
```

**Subsequent messages:** Binary ATP chunks (every 1 second).

**Disconnection:** Client simply closes the WebSocket. No cleanup needed.

---

## 4. Comparison with AIM Solo2DL WiFi

| Feature | Core Pro (ATP) | AIM Solo2DL (STCP) |
|---------|---------------|---------------------|
| **Discovery** | mDNS (automatic) | Hardcoded IP 10.0.0.1 |
| **Protocol** | HTTP/JSON + WebSocket | Custom binary (STCP/STNC) |
| **Documentation** | This document (public) | None (reverse-engineered) |
| **Session list** | `GET /sessions` (JSON) | Download dwnsm file, decompress, parse CSV |
| **File download** | `GET /sessions/{id}/data` (HTTP) | Custom flow control (65KB chunks + ACK) |
| **Live streaming** | WebSocket (1 chunk/sec) | Not available |
| **Track management** | `POST /tracks` (upload tracks) | Not available via WiFi |
| **Session config** | `GET/POST /config` (driver, vehicle, weather) | Not available via WiFi |
| **Authentication** | Planned (API key header) | None |
| **Debuggability** | `curl` + any browser | Hex dump + custom parser |
| **Ports** | 80 (HTTP) | 2000 (TCP) + 36002 (UDP) |
| **Connection overhead** | Single HTTP request | 3-round STNC negotiate + date filter |

---

## 5. Implementation Notes for Desktop App

### Go Client

The desktop app (`reader/core_pro/`) should implement:

```go
type CoreProClient struct {
    baseURL string  // e.g., "http://192.168.4.1"
}

// Discover finds Core Pro devices on the network via mDNS
func Discover(ctx context.Context) ([]DeviceInfo, error)

// GetDevice returns device info
func (c *CoreProClient) GetDevice() (*DeviceInfo, error)

// ListSessions returns all sessions
func (c *CoreProClient) ListSessions() ([]SessionSummary, error)

// GetSession returns session detail with laps
func (c *CoreProClient) GetSession(id string) (*SessionDetail, error)

// DownloadSession downloads the .atp file
func (c *CoreProClient) DownloadSession(id string, w io.Writer, onProgress func(int64, int64)) error

// ListTracks returns saved tracks
func (c *CoreProClient) ListTracks() ([]TrackInfo, error)

// UploadTrack uploads a track definition
func (c *CoreProClient) UploadTrack(track TrackDefinition) (*TrackInfo, error)

// SetSessionConfig pushes session config to the device
func (c *CoreProClient) SetSessionConfig(config *SessionConfig) (*ConfigResponse, error)

// GetSessionConfig reads the current config from the device
func (c *CoreProClient) GetSessionConfig() (*SessionConfig, error)

// LiveStatus returns current device state
func (c *CoreProClient) LiveStatus() (*LiveStatus, error)

// LiveStream opens a WebSocket for real-time chunks
func (c *CoreProClient) LiveStream(ctx context.Context) (<-chan []byte, error)
```

All methods use standard `net/http` — no custom protocol parsing needed (unlike AIM's WiFi client).

### Error Handling

- **Device not found:** mDNS discovery timeout (5 seconds)
- **Connection lost:** HTTP request timeout (10 seconds)
- **Session not found:** 404 response
- **Device busy:** 503 response (e.g., firmware update in progress)
- **SD card error:** 500 response with error detail
