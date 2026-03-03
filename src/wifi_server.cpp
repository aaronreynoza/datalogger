#include "wifi_server.h"

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <ESPmDNS.h>
#include <SD.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>

#include <atomic>
#include "atp_writer.h"
#include "device_config.h"
#include "logging.h"
#include "pmu.h"
#include "race_logger.h"
#include "track.h"

static AsyncWebServer server(80);

// ── Selective DNS server ──
// Only resolves captive-portal check domains to our AP IP.
// Everything else gets NXDOMAIN so macOS background services fail cleanly
// at DNS instead of getting fake IPs (which causes broken HTTPS → macOS
// marks the network as broken and switches to another WiFi).
static WiFiUDP dnsUdp;
static bool dnsRunning = false;
static IPAddress dnsResolveIP;

static bool isCaptiveCheckDomain(const char *name) {
  return (strcasecmp(name, "captive.apple.com") == 0 ||
          strcasecmp(name, "www.apple.com") == 0 ||
          strcasecmp(name, "connectivitycheck.gstatic.com") == 0 ||
          strcasecmp(name, "clients3.google.com") == 0 ||
          strcasecmp(name, "www.msftconnecttest.com") == 0 ||
          strcasecmp(name, "www.msftncsi.com") == 0 ||
          strcasecmp(name, "detectportal.firefox.com") == 0);
}

// Parse DNS wire-format name (length-prefixed labels) into dotted string.
// Returns bytes consumed, 0 on error.
static int dnsReadName(const uint8_t *buf, int bufLen, int offset,
                       char *out, int outMax) {
  int pos = offset, written = 0;
  while (pos < bufLen) {
    uint8_t len = buf[pos];
    if (len == 0) { pos++; break; }
    if ((len & 0xC0) == 0xC0) { pos += 2; break; } // compression pointer
    if (len > 63 || pos + 1 + len > bufLen) return 0;
    if (written > 0 && written < outMax - 1) out[written++] = '.';
    for (int i = 0; i < len && written < outMax - 1; i++)
      out[written++] = (char)buf[pos + 1 + i];
    pos += 1 + len;
  }
  out[written] = '\0';
  return pos - offset;
}

static void dnsProcessPacket() {
  int pktLen = dnsUdp.parsePacket();
  if (pktLen < 12) return;

  uint8_t pkt[512];
  int len = dnsUdp.read(pkt, sizeof(pkt));
  if (len < 12) return;

  // Must be a standard query (QR=0, Opcode=0)
  if (pkt[2] & 0x80) return;
  if ((pkt[2] >> 3) & 0x0F) return;
  uint16_t qdcount = (pkt[4] << 8) | pkt[5];
  if (qdcount < 1) return;

  // Parse first question name
  char name[128];
  int nameBytes = dnsReadName(pkt, len, 12, name, sizeof(name));
  if (nameBytes == 0) return;
  int qEnd = 12 + nameBytes + 4; // name + qtype(2) + qclass(2)
  if (qEnd > len) return;

  bool resolve = isCaptiveCheckDomain(name);

  // Build response: copy header + question, set flags
  uint8_t resp[512];
  memcpy(resp, pkt, qEnd);
  resp[2] = 0x81;  // QR=1, RD=1
  resp[3] = resolve ? 0x80 : 0x83;  // RA=1; rcode=0 or NXDOMAIN(3)
  resp[6] = 0; resp[7] = resolve ? 1 : 0;  // ANCOUNT
  resp[8] = 0; resp[9] = 0;   // NSCOUNT
  resp[10] = 0; resp[11] = 0; // ARCOUNT

  int respLen = qEnd;

  if (resolve) {
    // A record answer: name-pointer, type A, class IN, TTL 60s, 4-byte IP
    resp[respLen++] = 0xC0; resp[respLen++] = 0x0C;  // pointer to name
    resp[respLen++] = 0x00; resp[respLen++] = 0x01;  // TYPE A
    resp[respLen++] = 0x00; resp[respLen++] = 0x01;  // CLASS IN
    resp[respLen++] = 0x00; resp[respLen++] = 0x00;
    resp[respLen++] = 0x00; resp[respLen++] = 0x3C;  // TTL = 60
    resp[respLen++] = 0x00; resp[respLen++] = 0x04;  // RDLENGTH = 4
    resp[respLen++] = dnsResolveIP[0];
    resp[respLen++] = dnsResolveIP[1];
    resp[respLen++] = dnsResolveIP[2];
    resp[respLen++] = dnsResolveIP[3];
  }

  dnsUdp.beginPacket(dnsUdp.remoteIP(), dnsUdp.remotePort());
  dnsUdp.write(resp, respLen);
  dnsUdp.endPacket();
}
static WifiGpsStatus gpsStatus = {false, 0, 99.0f, 0.0f};

// SPI bus guard — prevents main loop IMU reads while HTTP handlers use SD.
// Both IMU and SD share the HSPI bus; concurrent access blocks the main loop
// on the SPI driver's internal mutex, triggering the 5-second watchdog.
static std::atomic<bool> _sdBusy{false};

struct SdBusyGuard {
  SdBusyGuard()  { _sdBusy.store(true, std::memory_order_release); }
  ~SdBusyGuard() { _sdBusy.store(false, std::memory_order_release); }
};

bool isWifiSdBusy() { return _sdBusy.load(std::memory_order_acquire); }

// ── Cached storage stats (SD.totalBytes/usedBytes are very slow on SPI) ──
static uint32_t cachedTotalMB = 0;
static uint32_t cachedUsedMB = 0;
static uint32_t cachedFreeMB = 0;
static void refreshStorageCache() {
  if (cachedTotalMB > 0) return;  // Computed once at boot; stable while WiFi is active
  uint64_t totalBytes = SD.totalBytes();
  uint64_t usedBytes = SD.usedBytes();
  cachedTotalMB = (uint32_t)(totalBytes / (1024 * 1024));
  cachedUsedMB  = (uint32_t)(usedBytes / (1024 * 1024));
  cachedFreeMB  = (uint32_t)((totalBytes - usedBytes) / (1024 * 1024));
}

void updateWifiGpsStatus(bool fix, uint32_t sats, float hdop, float speedKmh) {
  gpsStatus.fix = fix;
  gpsStatus.sats = sats;
  gpsStatus.hdop = hdop;
  gpsStatus.speedKmh = speedKmh;
}

// ===================== Helpers =====================

static void sendJson(AsyncWebServerRequest *request, int code, JsonDocument &doc) {
  String output;
  serializeJson(doc, output);
  request->send(code, "application/json", output);
}

static void sendError(AsyncWebServerRequest *request, int code, const char *error, const char *message) {
  JsonDocument doc;
  doc["error"] = error;
  doc["message"] = message;
  doc["code"] = code;
  sendJson(request, code, doc);
}

static const char *trackStateName(uint8_t state) {
  switch (state) {
    case TRACK_STATE_IDLE:      return "IDLE";
    case TRACK_STATE_RECORDING: return "RECORDING";
    case TRACK_STATE_READY:     return "READY";
    case TRACK_STATE_RACING:    return "RACING";
    default:                    return "UNKNOWN";
  }
}

static const char *sessionTypeName(uint8_t type) {
  switch (type) {
    case 1:  return "practice";
    case 2:  return "qualifying";
    case 3:  return "race";
    case 4:  return "test";
    default: return "unknown";
  }
}

static int batteryPercentFromVoltage(uint16_t mv) {
  if (mv >= 4150) return 100;
  if (mv <= 3300) return 0;
  return (int)((mv - 3300) * 100L / 850);
}

// ===================== GET /api/v1/device =====================

static void handleDevice(AsyncWebServerRequest *request) {
  SdBusyGuard guard;
  const DeviceConfig &cfg = getConfig();
  TrackUiState ts = getTrackUiState();

  JsonDocument doc;
  doc["serial"] = cfg.serial;
  doc["firmware"] = FIRMWARE_VERSION;
  doc["hardware"] = HARDWARE_ID;
  doc["protocol_version"] = 1;

  JsonArray caps = doc["capabilities"].to<JsonArray>();
  caps.add("imu");
  caps.add("gps");
  caps.add("sd_card");

  JsonObject storage = doc["storage"].to<JsonObject>();
  refreshStorageCache();
  storage["total_mb"] = cachedTotalMB;
  storage["used_mb"] = cachedUsedMB;
  storage["free_mb"] = cachedFreeMB;

  JsonObject battery = doc["battery"].to<JsonObject>();
  uint16_t battMv = pmuBattVoltageMv();
  battery["voltage_mv"] = battMv;
  battery["percent"] = batteryPercentFromVoltage(battMv);
  battery["charging"] = pmuIsCharging();

  JsonObject imu = doc["imu"].to<JsonObject>();
  imu["accel_range_g"] = 4;
  imu["gyro_range_dps"] = 64;
  imu["sample_rate_hz"] = 100;

  JsonObject gps = doc["gps"].to<JsonObject>();
  gps["fix"] = gpsStatus.fix;
  gps["sats"] = gpsStatus.sats;
  gps["hdop"] = serialized(String(gpsStatus.hdop, 2));

  doc["state"] = trackStateName(ts.trackState);

  sendJson(request, 200, doc);
}

// ===================== GET /api/v1/sessions =====================

// Format epoch milliseconds to ISO 8601 string. Returns false if startTimeMs==0.
static bool epochToISO(uint64_t startTimeMs, char *buf, size_t bufLen) {
  if (startTimeMs == 0) return false;
  uint32_t epochS = (uint32_t)(startTimeMs / 1000);
  uint32_t sec = epochS;
  int ss = sec % 60; sec /= 60;
  int mm = sec % 60; sec /= 60;
  int hh = sec % 24;
  uint32_t days = sec / 24;
  int year = 1970;
  while (true) {
    int diy = ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0) ? 366 : 365;
    if (days < (uint32_t)diy) break;
    days -= diy;
    year++;
  }
  static const uint8_t dim[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  int mon = 1;
  for (int i = 0; i < 12; i++) {
    int d = dim[i];
    if (i == 1 && ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0)) d++;
    if (days < (uint32_t)d) { mon = i + 1; break; }
    days -= d;
  }
  snprintf(buf, bufLen, "%04d-%02d-%02dT%02d:%02d:%02dZ",
           year, mon, (int)days + 1, hh, mm, ss);
  return true;
}

static void handleSessions(AsyncWebServerRequest *request) {
  SdBusyGuard guard;
  JsonDocument doc;
  JsonArray sessions = doc["sessions"].to<JsonArray>();
  int total = 0;

  File root = SD.open("/");
  if (!root) {
    sendError(request, 500, "sd_error", "Failed to open SD root");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    String name = file.name();
    if (name.endsWith(".atp")) {
      size_t sz = file.size();
      file.close();

      // Strip leading slash for display
      String dispName = name;
      if (dispName.startsWith("/")) dispName.remove(0, 1);

      // Derive session ID from filename (remove leading / and .atp extension)
      String sessionId = dispName;
      sessionId.replace(".atp", "");

      // Ensure path has leading slash for SD.open
      String path = name;
      if (!path.startsWith("/")) path = "/" + path;

      AtpFileMeta meta;
      bool hasMeta = atpReadMeta(path.c_str(), meta);

      JsonObject s = sessions.add<JsonObject>();
      s["id"] = sessionId;
      s["filename"] = dispName;
      s["size_bytes"] = sz;

      if (hasMeta) {
        s["driver"] = meta.driverName;
        s["vehicle"] = meta.vehicleName;
        s["track"] = meta.trackName;
        s["track_id"] = meta.trackId;
        s["session_type"] = sessionTypeName(meta.sessionType);
        s["complete"] = meta.complete;

        if (meta.complete) {
          s["duration_ms"] = meta.durationMs;
          s["laps"] = meta.totalLaps;
        }
        // Incomplete sessions: skip chunk scan (too slow for listing).
        // Desktop reads the full ATP file anyway — duration extracted on import.

        // Channels (static for Core Pro)
        JsonArray ch = s["channels"].to<JsonArray>();
        ch.add("imu");
        ch.add("gps");

        char timeBuf[32];
        if (epochToISO(meta.startTimeMs, timeBuf, sizeof(timeBuf))) {
          s["start_time"] = timeBuf;
        }
      }

      total++;
      yield();  // Let WiFi stack + watchdog breathe between files
      file = root.openNextFile();
      continue;
    }
    file = root.openNextFile();
  }
  root.close();

  doc["total"] = total;
  sendJson(request, 200, doc);
}

// ===================== GET /api/v1/sessions/{id}/data =====================

// Holds file data in PSRAM for async streaming (avoids SPI bus contention with IMU).
struct PsramFileData {
  uint8_t *buf;
  size_t   len;
};

static void handleSessionDownload(AsyncWebServerRequest *request) {
  SdBusyGuard guard;
  // Extract session ID from the URI: /api/v1/sessions/{id}/data
  String uri = request->url();
  int sessStart = strlen("/api/v1/sessions/");
  int dataPos = uri.lastIndexOf("/data");
  if (dataPos < 0 || dataPos <= sessStart) {
    sendError(request, 400, "bad_request", "Invalid session URL");
    return;
  }
  String sessionId = uri.substring(sessStart, dataPos);
  String filename = "/" + sessionId + ".atp";

  // Read entire file into PSRAM synchronously (SPI works fine in handler context).
  // Streaming directly from SD via beginResponse(SD, ...) hangs because the async
  // TCP task's SD reads conflict with the main loop's IMU SPI reads on the same bus.
  File f = SD.open(filename, "r");
  if (!f) {
    sendError(request, 404, "not_found", "Session not found");
    return;
  }
  size_t sz = f.size();
  uint8_t *data = (uint8_t *)ps_malloc(sz);
  if (!data) {
    f.close();
    sendError(request, 500, "memory_error", "Out of PSRAM");
    return;
  }
  size_t bytesRead = f.read(data, sz);
  f.close();

  Serial.printf("[WiFi] Streaming %s (%u bytes from PSRAM)\n", filename.c_str(), bytesRead);

  // Stream from PSRAM buffer — no more SD/SPI access needed.
  // Cleanup via onDisconnect to avoid use-after-free in the streaming callback.
  PsramFileData *fd = new PsramFileData{data, bytesRead};
  request->onDisconnect([fd]() {
    if (fd->buf) free(fd->buf);
    delete fd;
  });

  AsyncWebServerResponse *response = request->beginResponse(
    "application/octet-stream", bytesRead,
    [fd](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
      if (index >= fd->len) return 0;
      size_t remaining = fd->len - index;
      size_t toSend = (remaining < maxLen) ? remaining : maxLen;
      memcpy(buffer, fd->buf + index, toSend);
      return toSend;
    }
  );
  String dispName = sessionId + ".atp";
  response->addHeader("Content-Disposition", "attachment; filename=\"" + dispName + "\"");
  request->send(response);
}

// ===================== GET /api/v1/sessions/{id} =====================

// Channel info struct for static channel table
struct ChannelInfo {
  uint16_t id;
  const char *name;
  const char *unit;
  uint16_t rateHz;
};

static const ChannelInfo CHANNEL_TABLE[] = {
  {1,  "dynamics.gforce.lon",   "G",     100},
  {2,  "dynamics.gforce.lat",   "G",     100},
  {3,  "dynamics.gforce.vert",  "G",     100},
  {4,  "dynamics.roll.rate",    "deg/s", 100},
  {5,  "dynamics.pitch.rate",   "deg/s", 100},
  {6,  "dynamics.yaw.rate",     "deg/s", 100},
  {7,  "environment.imu.temp",  "C",     100},
  {8,  "position.gps.lat",      "deg",   10},
  {9,  "position.gps.lon",      "deg",   10},
  {10, "position.gps.alt",      "m",     10},
  {11, "position.gps.speed",    "m/s",   10},
  {12, "position.gps.course",   "deg",   10},
  {13, "position.local.x",      "m",     10},
  {14, "position.local.y",      "m",     10},
  {15, "position.local.z",      "m",     10},
  {16, "dynamics.vel.north",    "m/s",   10},
  {17, "dynamics.vel.east",     "m/s",   10},
  {18, "position.lap.dist",     "m",     10},
  {19, "dynamics.speed",        "m/s",   10},
};
static constexpr uint16_t NUM_API_CHANNELS = sizeof(CHANNEL_TABLE) / sizeof(CHANNEL_TABLE[0]);

static void handleSessionDetail(AsyncWebServerRequest *request, const String &sessionId) {
  SdBusyGuard guard;
  String filename = "/" + sessionId + ".atp";

  AtpFileMeta meta;
  if (!atpReadMeta(filename.c_str(), meta)) {
    sendError(request, 404, "not_found", "Session not found or unreadable");
    return;
  }

  JsonDocument doc;
  doc["id"] = sessionId;
  doc["filename"] = sessionId + ".atp";
  doc["size_bytes"] = meta.fileSize;
  doc["driver"] = meta.driverName;
  doc["vehicle"] = meta.vehicleName;
  doc["track"] = meta.trackName;
  doc["track_id"] = meta.trackId;
  doc["session_type"] = sessionTypeName(meta.sessionType);
  doc["complete"] = meta.complete;

  if (meta.complete) {
    doc["duration_ms"] = meta.durationMs;
  }
  // Incomplete sessions: skip chunk scan (too slow over SPI).
  // Desktop reads the full ATP file — duration extracted on import.

  char timeBuf[32];
  if (epochToISO(meta.startTimeMs, timeBuf, sizeof(timeBuf))) {
    doc["start_time"] = timeBuf;
  }

  // Channel table (static, no SD access needed)
  JsonArray channels = doc["channels"].to<JsonArray>();
  for (uint16_t i = 0; i < NUM_API_CHANNELS; ++i) {
    JsonObject ch = channels.add<JsonObject>();
    ch["id"] = CHANNEL_TABLE[i].id;
    ch["canonical_name"] = CHANNEL_TABLE[i].name;
    ch["unit"] = CHANNEL_TABLE[i].unit;
    ch["rate_hz"] = CHANNEL_TABLE[i].rateHz;
  }

  // Lap table (only for complete sessions — requires one more SD read)
  JsonArray lapsArr = doc["laps"].to<JsonArray>();
  if (meta.complete) {
    AtpLapInfo lapBuf[64];
    uint16_t lapCount = atpReadLaps(filename.c_str(), lapBuf, 64);
    uint32_t bestLapMs = UINT32_MAX;
    uint16_t bestLapNum = 0;
    for (uint16_t i = 0; i < lapCount; ++i) {
      JsonObject lap = lapsArr.add<JsonObject>();
      lap["number"] = lapBuf[i].number;
      lap["time_ms"] = lapBuf[i].timeMs;
      lap["distance_m"] = serialized(String(lapBuf[i].distanceM, 1));
      lap["flags"] = lapBuf[i].flags;
      if (lapBuf[i].timeMs > 0 && lapBuf[i].timeMs < bestLapMs) {
        bestLapMs = lapBuf[i].timeMs;
        bestLapNum = lapBuf[i].number;
      }
    }
    if (bestLapMs < UINT32_MAX) {
      doc["best_lap"] = bestLapNum;
      doc["best_lap_ms"] = bestLapMs;
    }
  }

  sendJson(request, 200, doc);
}

// ===================== GET /api/v1/tracks =====================

static void handleGetTracks(AsyncWebServerRequest *request) {
  SdBusyGuard guard;
  JsonDocument doc;
  JsonArray tracks = doc["tracks"].to<JsonArray>();

  int count = getSavedTrackCount();
  for (int i = 0; i < count; i++) {
    SavedTrackInfo info;
    if (!getSavedTrack(i, info)) continue;

    JsonObject t = tracks.add<JsonObject>();
    t["track_id"] = info.id;
    char hexBuf[12];
    snprintf(hexBuf, sizeof(hexBuf), "%08lX", (unsigned long)info.id);
    t["track_id_hex"] = hexBuf;
    t["name"] = info.name;
    t["start_lat"] = serialized(String(info.lat, 6));
    t["start_lon"] = serialized(String(info.lon, 6));
    t["start_alt"] = serialized(String(info.alt, 1));
    t["start_heading_deg"] = serialized(String(info.courseDeg, 2));

    // Check if a track recording file exists
    char recPath[32];
    snprintf(recPath, sizeof(recPath), "/track_%08lX.csv", (unsigned long)info.id);
    bool hasRec = SD.exists(recPath);
    t["has_recording"] = hasRec;
    if (hasRec) {
      t["recording_points"] = countTrackRecordingPoints(info.id);
    }
  }

  sendJson(request, 200, doc);
}

// ===================== POST /api/v1/tracks =====================

static void handlePostTrack(AsyncWebServerRequest *request, JsonVariant &json) {
  JsonObject reqDoc = json.as<JsonObject>();

  if (!reqDoc["start_lat"].is<double>() || !reqDoc["start_lon"].is<double>()) {
    sendError(request, 400, "bad_request", "start_lat and start_lon are required");
    return;
  }

  double lat = reqDoc["start_lat"];
  double lon = reqDoc["start_lon"];
  double alt = reqDoc["start_alt"] | 0.0;
  double heading = reqDoc["start_heading_deg"] | 0.0;
  const char *name = reqDoc["name"] | "";

  uint32_t trackId = 0;
  if (!addTrackFromApi(lat, lon, alt, heading, name, trackId)) {
    sendError(request, 500, "save_failed", "Failed to save track");
    return;
  }

  JsonDocument doc;
  doc["track_id"] = trackId;
  char hexBuf[12];
  snprintf(hexBuf, sizeof(hexBuf), "%08lX", (unsigned long)trackId);
  doc["track_id_hex"] = hexBuf;
  doc["name"] = name;
  doc["status"] = "saved";
  sendJson(request, 200, doc);
}

// ===================== DELETE /api/v1/tracks?id=XXXX =====================

static void handleDeleteTrack(AsyncWebServerRequest *request) {
  if (!request->hasParam("id")) {
    sendError(request, 400, "bad_request", "Missing id parameter");
    return;
  }
  uint32_t tid = strtoul(request->getParam("id")->value().c_str(), nullptr, 10);
  if (tid == 0) {
    sendError(request, 400, "bad_request", "Invalid track id");
    return;
  }
  if (!deleteTrack(tid)) {
    sendError(request, 404, "not_found", "Track not found");
    return;
  }
  JsonDocument doc;
  doc["status"] = "deleted";
  doc["track_id"] = tid;
  sendJson(request, 200, doc);
}

// ===================== GET /api/v1/config =====================

static void handleGetConfig(AsyncWebServerRequest *request) {
  const DeviceConfig &cfg = getConfig();
  JsonDocument doc;
  doc["driver_name"] = cfg.driverName;
  doc["vehicle_name"] = cfg.vehicleName;
  doc["session_type"] = sessionTypeName(cfg.sessionType);
  doc["session_notes"] = cfg.sessionNotes;
  if (cfg.weather.set) {
    JsonObject w = doc["weather"].to<JsonObject>();
    w["ambient_temp_c"] = cfg.weather.ambientTempC;
    w["track_temp_c"] = cfg.weather.trackTempC;
    w["conditions"] = cfg.weather.conditions;
  } else {
    doc["weather"] = (const char *)nullptr;  // JSON null
  }
  sendJson(request, 200, doc);
}

// ===================== POST /api/v1/config =====================

static void handleSetConfig(AsyncWebServerRequest *request, JsonVariant &json) {
  // Reject if currently recording
  TrackUiState ts = getTrackUiState();
  if (ts.trackState == TRACK_STATE_RACING) {
    sendError(request, 409, "recording_active",
              "Cannot update config while recording. Stop the session first.");
    return;
  }

  JsonObject reqDoc = json.as<JsonObject>();

  // Apply fields (only update what's provided)
  if (reqDoc["driver_name"].is<const char *>()) {
    setDriverName(reqDoc["driver_name"]);
  }
  if (reqDoc["vehicle_name"].is<const char *>()) {
    setVehicleName(reqDoc["vehicle_name"]);
  }
  if (reqDoc["session_type"].is<const char *>()) {
    setSessionType(sessionTypeFromString(reqDoc["session_type"]));
  }
  if (reqDoc["session_notes"].is<const char *>()) {
    setSessionNotes(reqDoc["session_notes"]);
  }
  // Weather: null clears, object sets, absent leaves unchanged
  if (reqDoc["weather"].is<JsonVariantConst>()) {
    if (reqDoc["weather"].isNull()) {
      clearWeather();
    } else if (reqDoc["weather"].is<JsonObject>()) {
      JsonObject w = reqDoc["weather"];
      float ambC = w["ambient_temp_c"] | 0.0f;
      float trkC = w["track_temp_c"] | 0.0f;
      const char *cond = w["conditions"] | "dry";
      setWeather(ambC, trkC, cond);
    }
  }

  // Echo back stored config
  const DeviceConfig &cfg = getConfig();
  JsonDocument doc;
  doc["status"] = "ok";
  JsonObject cfgObj = doc["config"].to<JsonObject>();
  cfgObj["driver_name"] = cfg.driverName;
  cfgObj["vehicle_name"] = cfg.vehicleName;
  cfgObj["session_type"] = sessionTypeName(cfg.sessionType);
  cfgObj["session_notes"] = cfg.sessionNotes;
  if (cfg.weather.set) {
    JsonObject w = cfgObj["weather"].to<JsonObject>();
    w["ambient_temp_c"] = cfg.weather.ambientTempC;
    w["track_temp_c"] = cfg.weather.trackTempC;
    w["conditions"] = cfg.weather.conditions;
  } else {
    cfgObj["weather"] = (const char *)nullptr;
  }
  sendJson(request, 200, doc);
}

// ===================== GET /api/v1/live/status =====================

static void handleLiveStatus(AsyncWebServerRequest *request) {
  SdBusyGuard guard;
  TrackUiState ts = getTrackUiState();

  JsonDocument doc;
  doc["state"] = trackStateName(ts.trackState);
  doc["recording"] = (ts.trackState == TRACK_STATE_RACING);

  if (ts.trackState == TRACK_STATE_RACING || ts.trackState == TRACK_STATE_READY) {
    if (ts.trackId) {
      doc["track_id"] = ts.trackId;
      if (ts.trackName[0]) doc["track"] = ts.trackName;
    }
  }

  if (ts.trackState == TRACK_STATE_RACING) {
    doc["current_lap"] = ts.lapCount;
    doc["lap_time_ms"] = ts.currentLapMs;
    if (ts.lastLapMs > 0) doc["last_lap_ms"] = ts.lastLapMs;
    if (ts.bestLapMs > 0 && ts.bestLapMs != UINT32_MAX) doc["best_lap_ms"] = ts.bestLapMs;
    doc["speed_kmh"] = serialized(String(gpsStatus.speedKmh, 1));

    // Session ID from active ATP filename
    const char *fn = getRaceSessionFilename();
    if (fn) {
      // Strip leading "/" and ".atp" → session ID
      String sid = fn;
      if (sid.startsWith("/")) sid.remove(0, 1);
      sid.replace(".atp", "");
      doc["session_id"] = sid;
    }
  }

  doc["gps_fix"] = gpsStatus.fix;
  doc["sats"] = gpsStatus.sats;

  uint16_t battMv = pmuBattVoltageMv();
  doc["battery_percent"] = batteryPercentFromVoltage(battMv);

  refreshStorageCache();
  doc["sd_free_mb"] = cachedFreeMB;

  sendJson(request, 200, doc);
}

// ===================== GET /api/v1/debug =====================

static void handleDebug(AsyncWebServerRequest *request) {
  String log = readDiagLog();
  request->send(200, "text/plain", log);
}

// ===================== GET / (legacy root page) =====================

static void handleRoot(AsyncWebServerRequest *request) {
  SdBusyGuard guard;
  const DeviceConfig &cfg = getConfig();
  TrackUiState ts = getTrackUiState();

  String html = "<html><head><style>"
    "body{font-family:sans-serif;margin:20px;background:#1a1a2e;color:#e0e0e0;}"
    "h1{color:#00d4ff;}h2{color:#aaa;}"
    "a{display:block;margin:6px 0;font-size:16px;color:#00d4ff;}"
    ".sz{color:#888;font-size:13px;}"
    ".info{background:#16213e;padding:12px;border-radius:8px;margin:10px 0;}"
    "</style></head><body>"
    "<h1>ApexDirector Core Pro</h1>"
    "<div class='info'>"
    "<b>Serial:</b> " + String(cfg.serial) + "<br>"
    "<b>Firmware:</b> " + String(FIRMWARE_VERSION) + "<br>"
    "<b>State:</b> " + String(trackStateName(ts.trackState)) + "<br>"
    "<b>Driver:</b> " + String(cfg.driverName) + "<br>"
    "<b>Vehicle:</b> " + String(cfg.vehicleName) +
    "</div>"
    "<h2>API Endpoints</h2>"
    "<a href='/api/v1/device'>GET /api/v1/device</a>"
    "<a href='/api/v1/sessions'>GET /api/v1/sessions</a>"
    "<a href='/api/v1/tracks'>GET /api/v1/tracks</a>"
    "<a href='/api/v1/config'>GET /api/v1/config</a>"
    "<a href='/api/v1/live/status'>GET /api/v1/live/status</a>"
    "<h2>Files</h2>";

  File root = SD.open("/");
  if (root) {
    int count = 0;
    File file = root.openNextFile();
    while (file) {
      String name = file.name();
      bool isLog = name.endsWith(".atp") || name.endsWith(".csv");
      if (isLog) {
        if (name.startsWith("/")) name.remove(0, 1);
        size_t sz = file.size();
        if (name.endsWith(".atp")) {
          // Link to session data endpoint
          String sessId = name;
          sessId.replace(".atp", "");
          html += "<a href='/api/v1/sessions/" + sessId + "/data'>" +
                  name + " <span class='sz'>(" + String(sz / 1024) + " KB)</span></a>";
        } else {
          html += "<a href='/log?file=" + name + "'>" +
                  name + " <span class='sz'>(" + String(sz / 1024) + " KB)</span></a>";
        }
        count++;
      }
      file = root.openNextFile();
    }
    root.close();
    if (count == 0) html += "<p>No files found.</p>";
  }
  html += "</body></html>";
  request->send(200, "text/html", html);
}

// ===================== Legacy CSV download (for track recordings) =====================

static void handleLogDownload(AsyncWebServerRequest *request) {
  SdBusyGuard guard;
  if (!request->hasParam("file")) {
    request->send(400, "text/plain", "Missing ?file=");
    return;
  }
  String fname = request->getParam("file")->value();
  if (!fname.startsWith("/")) fname = "/" + fname;

  File f = SD.open(fname, "r");
  if (!f) {
    request->send(404, "text/plain", "File not found");
    return;
  }
  size_t sz = f.size();
  uint8_t *data = (uint8_t *)ps_malloc(sz);
  if (!data) {
    f.close();
    request->send(500, "text/plain", "Out of memory");
    return;
  }
  size_t bytesRead = f.read(data, sz);
  f.close();

  String dispName = fname;
  if (dispName.startsWith("/")) dispName.remove(0, 1);

  PsramFileData *fd = new PsramFileData{data, bytesRead};
  request->onDisconnect([fd]() {
    if (fd->buf) free(fd->buf);
    delete fd;
  });

  AsyncWebServerResponse *response = request->beginResponse(
    "text/csv", bytesRead,
    [fd](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
      if (index >= fd->len) return 0;
      size_t remaining = fd->len - index;
      size_t toSend = (remaining < maxLen) ? remaining : maxLen;
      memcpy(buffer, fd->buf + index, toSend);
      return toSend;
    }
  );
  response->addHeader("Content-Disposition", "attachment; filename=\"" + dispName + "\"");
  request->send(response);
}

// ===================== Captive portal (keeps macOS/iOS/Android connected) =====================

static const char CAPTIVE_SUCCESS[] PROGMEM =
  "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>";

static bool handleCaptivePortal(AsyncWebServerRequest *request) {
  String uri = request->url();
  String host = request->host();

  // Apple connectivity check
  if (uri == "/hotspot-detect.html" ||
      uri == "/library/test/success.html" ||
      host == "captive.apple.com" ||
      host == "www.apple.com") {
    request->send(200, "text/html", CAPTIVE_SUCCESS);
    return true;
  }

  // Android / Chrome connectivity check
  if (uri == "/generate_204" || uri == "/gen_204") {
    request->send(204);
    return true;
  }

  // Windows connectivity check
  if (uri == "/connecttest.txt") {
    request->send(200, "text/plain", "Microsoft Connect Test");
    return true;
  }
  if (uri == "/redirect") {
    request->send(200, "text/html", CAPTIVE_SUCCESS);
    return true;
  }

  // Firefox connectivity check
  if (uri == "/success.txt") {
    request->send(200, "text/plain", "success");
    return true;
  }

  return false;
}

// ===================== Route not found =====================

static void handleNotFound(AsyncWebServerRequest *request) {
  // Captive portal detection (macOS, iOS, Android, Windows, Firefox)
  if (handleCaptivePortal(request)) return;

  String uri = request->url();

  // CORS preflight for any API route
  if (request->method() == HTTP_OPTIONS) {
    request->send(204);
    return;
  }

  // Session routes: listing, detail, and download (all under /api/v1/sessions)
  if (uri == "/api/v1/sessions" && request->method() == HTTP_GET) {
    handleSessions(request);
    return;
  }
  if (uri.startsWith("/api/v1/sessions/") && request->method() == HTTP_GET) {
    int sessStart = strlen("/api/v1/sessions/");
    // GET /api/v1/sessions/{id}/data — binary download
    if (uri.endsWith("/data")) {
      handleSessionDownload(request);
      return;
    }
    // GET /api/v1/sessions/{id} — session detail with laps + channels
    String sessionId = uri.substring(sessStart);
    if (sessionId.length() > 0 && sessionId.indexOf('/') < 0) {
      handleSessionDetail(request, sessionId);
      return;
    }
  }
  sendError(request, 404, "not_found", "Endpoint not found");
}

// ===================== Setup =====================

void setupWiFi() {
  const DeviceConfig &cfg = getConfig();

  // Build SSID from serial: ApexDirector-CPXXYYZZ
  String ssid = "ApexDirector-" + String(cfg.serial);
  const char *password = "apex1234";

  WiFi.mode(WIFI_AP);
  // Use a clean, memorable IP instead of the default 192.168.4.1
  IPAddress apIP(10, 0, 1, 1);
  IPAddress gateway(10, 0, 1, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(apIP, gateway, subnet);
  if (!WiFi.softAP(ssid.c_str(), password)) {
    Serial.println("[WiFi] AP start FAILED");
    return;
  }
  WiFi.setSleep(false);  // Disable WiFi power saving for reliable HTTP responses
  IPAddress ip = WiFi.softAPIP();
  Serial.print("[WiFi] AP: "); Serial.print(ssid);
  Serial.print("  pass="); Serial.println(password);
  Serial.print("[WiFi] http://"); Serial.println(ip.toString());

  // mDNS
  String hostname = "corepro-" + String(cfg.serial);
  hostname.toLowerCase();
  if (MDNS.begin(hostname.c_str())) {
    MDNS.addService("_apexdirector", "_tcp", 80);
    MDNS.addServiceTxt("_apexdirector", "_tcp", "version", "1");
    MDNS.addServiceTxt("_apexdirector", "_tcp", "firmware", FIRMWARE_VERSION);
    MDNS.addServiceTxt("_apexdirector", "_tcp", "hardware", HARDWARE_ID);
    MDNS.addServiceTxt("_apexdirector", "_tcp", "serial", cfg.serial);
    Serial.print("[mDNS] "); Serial.print(hostname);
    Serial.println(".local");
  } else {
    Serial.println("[mDNS] start FAILED");
  }

  // Selective captive portal DNS — only resolves portal-check domains.
  // Non-portal domains get NXDOMAIN (clean failure) instead of fake IPs
  // (which cause broken HTTPS and trigger macOS WiFi switching).
  dnsResolveIP = apIP;
  dnsUdp.begin(53);
  dnsRunning = true;
  Serial.println("[WiFi] Selective DNS started (captive portal domains only)");

  // Pre-warm storage cache so first request is fast
  refreshStorageCache();

  // CORS headers on all responses
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, DELETE, OPTIONS");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");

  // API v1 GET endpoints
  server.on("/api/v1/device",      HTTP_GET, handleDevice);
  // NOTE: /api/v1/sessions is handled in handleNotFound because ESPAsyncWebServer
  // does prefix matching — registering it here would intercept /sessions/{id} and
  // /sessions/{id}/data requests too.
  server.on("/api/v1/tracks",      HTTP_GET, handleGetTracks);
  server.on("/api/v1/config",      HTTP_GET, handleGetConfig);
  server.on("/api/v1/live/status", HTTP_GET, handleLiveStatus);
  server.on("/api/v1/debug",       HTTP_GET, handleDebug);

  // API v1 DELETE endpoint
  server.on("/api/v1/tracks", HTTP_DELETE, handleDeleteTrack);

  // API v1 POST endpoints (JSON body via AsyncCallbackJsonWebHandler)
  AsyncCallbackJsonWebHandler *trackPostHandler = new AsyncCallbackJsonWebHandler(
      "/api/v1/tracks",
      [](AsyncWebServerRequest *request, JsonVariant &json) { handlePostTrack(request, json); }
  );
  server.addHandler(trackPostHandler);

  AsyncCallbackJsonWebHandler *configPostHandler = new AsyncCallbackJsonWebHandler(
      "/api/v1/config",
      [](AsyncWebServerRequest *request, JsonVariant &json) { handleSetConfig(request, json); }
  );
  server.addHandler(configPostHandler);

  // Legacy endpoints
  server.on("/", HTTP_GET, handleRoot);
  server.on("/log", HTTP_GET, handleLogDownload);

  // Catch-all for dynamic session routes + OPTIONS preflight
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("[WiFi] Async HTTP server started on port 80");
}

void stopWiFi() {
  if (dnsRunning) {
    dnsUdp.stop();
    dnsRunning = false;
  }
  server.end();
  WiFi.mode(WIFI_OFF);
  Serial.println("[WiFi] OFF — SD bus freed for recording");
}

void tickWiFiDNS() {
  if (dnsRunning) dnsProcessPacket();
}

void restartWiFi() {
  // Re-setup WiFi AP + server (setupWiFi handles everything)
  setupWiFi();
}
