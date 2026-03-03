#include "wifi_server.h"

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <ESPmDNS.h>
#include <SD.h>
#include <ArduinoJson.h>

#include "atp_writer.h"
#include "device_config.h"
#include "logging.h"
#include "pmu.h"
#include "race_logger.h"
#include "track.h"

static AsyncWebServer server(80);
static WifiGpsStatus gpsStatus = {false, 0, 99.0f, 0.0f};

// ── Cached storage stats (SD.totalBytes/usedBytes are very slow on SPI) ──
static uint32_t cachedTotalMB = 0;
static uint32_t cachedUsedMB = 0;
static uint32_t cachedFreeMB = 0;
static uint32_t lastStorageCacheMs = 0;
static const uint32_t STORAGE_CACHE_TTL_MS = 30000; // refresh every 30s

static void refreshStorageCache() {
  uint32_t now = millis();
  if (cachedTotalMB > 0 && (now - lastStorageCacheMs) < STORAGE_CACHE_TTL_MS) return;
  uint64_t totalBytes = SD.totalBytes();
  uint64_t usedBytes = SD.usedBytes();
  cachedTotalMB = (uint32_t)(totalBytes / (1024 * 1024));
  cachedUsedMB  = (uint32_t)(usedBytes / (1024 * 1024));
  cachedFreeMB  = (uint32_t)((totalBytes - usedBytes) / (1024 * 1024));
  lastStorageCacheMs = now;
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

static void handleSessions(AsyncWebServerRequest *request) {
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

          // Find best lap time
          AtpLapInfo lapBuf[64];
          uint16_t lapCount = atpReadLaps(path.c_str(), lapBuf, 64);
          uint32_t bestMs = UINT32_MAX;
          for (uint16_t li = 0; li < lapCount; ++li) {
            if (lapBuf[li].timeMs > 0 && lapBuf[li].timeMs < bestMs)
              bestMs = lapBuf[li].timeMs;
          }
          if (bestMs < UINT32_MAX) s["best_lap_ms"] = bestMs;
        } else {
          // Incomplete session — scan chunks for estimated duration
          AtpScanResult scan;
          if (atpScanChunks(path.c_str(), scan)) {
            s["duration_ms"] = scan.lastTimestampMs;
            s["chunks"] = scan.chunkCount;
          }
        }

        // Channels (static for Core Pro)
        JsonArray ch = s["channels"].to<JsonArray>();
        ch.add("imu");
        ch.add("gps");

        if (meta.startTimeMs > 0) {
          // Format ISO 8601 timestamp
          uint32_t epochS = (uint32_t)(meta.startTimeMs / 1000);
          char timeBuf[32];
          // Simple epoch to ISO conversion
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
          snprintf(timeBuf, sizeof(timeBuf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
                   year, mon, (int)days + 1, hh, mm, ss);
          s["start_time"] = timeBuf;
        }
      }

      total++;
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

static void handleSessionDownload(AsyncWebServerRequest *request) {
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

  if (!SD.exists(filename)) {
    sendError(request, 404, "not_found", "Session not found");
    return;
  }

  String dispName = sessionId + ".atp";
  AsyncWebServerResponse *response = request->beginResponse(SD, filename, "application/octet-stream");
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
  String filename = "/" + sessionId + ".atp";

  if (!SD.exists(filename)) {
    sendError(request, 404, "not_found", "Session not found");
    return;
  }

  AtpFileMeta meta;
  if (!atpReadMeta(filename.c_str(), meta)) {
    sendError(request, 500, "read_error", "Failed to read session metadata");
    return;
  }

  File f = SD.open(filename, "r");
  size_t fileSize = f ? f.size() : 0;
  if (f) f.close();

  JsonDocument doc;
  doc["id"] = sessionId;
  doc["filename"] = sessionId + ".atp";
  doc["size_bytes"] = fileSize;
  doc["driver"] = meta.driverName;
  doc["vehicle"] = meta.vehicleName;
  doc["track"] = meta.trackName;
  doc["track_id"] = meta.trackId;
  doc["session_type"] = sessionTypeName(meta.sessionType);
  doc["complete"] = meta.complete;

  if (meta.complete) {
    doc["duration_ms"] = meta.durationMs;
  } else {
    AtpScanResult scan;
    if (atpScanChunks(filename.c_str(), scan)) {
      doc["duration_ms"] = scan.lastTimestampMs;
      doc["chunks"] = scan.chunkCount;
    }
  }

  if (meta.startTimeMs > 0) {
    uint32_t epochS = (uint32_t)(meta.startTimeMs / 1000);
    char timeBuf[32];
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
    snprintf(timeBuf, sizeof(timeBuf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
             year, mon, (int)days + 1, hh, mm, ss);
    doc["start_time"] = timeBuf;
  }

  // Channel table
  JsonArray channels = doc["channels"].to<JsonArray>();
  for (uint16_t i = 0; i < NUM_API_CHANNELS; ++i) {
    JsonObject ch = channels.add<JsonObject>();
    ch["id"] = CHANNEL_TABLE[i].id;
    ch["canonical_name"] = CHANNEL_TABLE[i].name;
    ch["unit"] = CHANNEL_TABLE[i].unit;
    ch["rate_hz"] = CHANNEL_TABLE[i].rateHz;
  }

  // Lap table (only available for complete sessions with footer)
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
  if (!request->hasParam("file")) {
    request->send(400, "text/plain", "Missing ?file=");
    return;
  }
  String fname = request->getParam("file")->value();
  if (!fname.startsWith("/")) fname = "/" + fname;
  if (!SD.exists(fname)) {
    request->send(404, "text/plain", "File not found");
    return;
  }
  String dispName = fname;
  if (dispName.startsWith("/")) dispName.remove(0, 1);
  AsyncWebServerResponse *response = request->beginResponse(SD, fname, "text/csv");
  response->addHeader("Content-Disposition", "attachment; filename=\"" + dispName + "\"");
  request->send(response);
}

// ===================== Route not found =====================

static void handleNotFound(AsyncWebServerRequest *request) {
  String uri = request->url();

  // CORS preflight for any API route
  if (request->method() == HTTP_OPTIONS) {
    request->send(204);
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

  // Pre-warm storage cache so first request is fast
  refreshStorageCache();

  // CORS headers on all responses
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, DELETE, OPTIONS");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");

  // API v1 GET endpoints
  server.on("/api/v1/device",      HTTP_GET, handleDevice);
  server.on("/api/v1/sessions",    HTTP_GET, handleSessions);
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
