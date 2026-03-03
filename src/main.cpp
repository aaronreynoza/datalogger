#include <Arduino.h>
#include <SD.h>

#include "gps.h"
#include "imu.h"
#include "display.h"
#include "device_config.h"
#include "logging.h"
#include "pmu.h"
#include "wifi_server.h"
#include "lora.h"
#include "track.h"
#include "race_logger.h"

static constexpr uint32_t PRINT_INTERVAL_MS = 200;
static constexpr uint32_t IMU_INTERVAL_MS   = 10;

// ===================== Latest GPS snapshot =====================

struct LatestGps {
  bool     locValid;
  bool     altValid;
  bool     spdValid;
  bool     hdopValid;
  bool     satsValid;
  bool     courseValid;
  double   lat;
  double   lon;
  double   alt_m;
  double   spd_kmh;
  double   hdop;
  double   course_deg;
  uint32_t sats;
  uint32_t epoch;
  uint32_t fixMs;
  uint32_t ageMs;
  bool     updated;
};

static LatestGps latestGps = {
  false, false, false, false, false, false,
  NAN, NAN, NAN, NAN, NAN, NAN,
  0, 0, 0, 0, false
};

static void updateLatestGps(uint32_t nowMs) {
  if (!gps.location.isUpdated()) return;

  latestGps.locValid    = gps.location.isValid();
  latestGps.altValid    = gps.altitude.isValid();
  latestGps.spdValid    = gps.speed.isValid();
  latestGps.hdopValid   = gps.hdop.isValid();
  latestGps.satsValid   = gps.satellites.isValid();
  latestGps.courseValid  = gps.course.isValid();

  latestGps.lat        = latestGps.locValid    ? gps.location.lat()    : NAN;
  latestGps.lon        = latestGps.locValid    ? gps.location.lng()    : NAN;
  latestGps.alt_m      = latestGps.altValid    ? gps.altitude.meters() : NAN;
  latestGps.spd_kmh    = latestGps.spdValid    ? gps.speed.kmph()     : NAN;
  latestGps.hdop       = latestGps.hdopValid   ? gps.hdop.hdop()      : NAN;
  latestGps.course_deg = latestGps.courseValid  ? gps.course.deg()     : NAN;
  latestGps.sats       = latestGps.satsValid   ? gps.satellites.value(): 0;
  latestGps.epoch      = gpsUnixTime();

  latestGps.ageMs   = gps.location.age();
  latestGps.fixMs   = nowMs - latestGps.ageMs;
  latestGps.updated = true;
}

// ===================== Serial commands =====================

static void handleSerialCommands() {
  if (!Serial.available()) return;
  char cmd = Serial.read();
  switch (cmd) {
    case 'D':
      if (!storageReady) {
        Serial.println("Storage not ready");
        break;
      }
      loggingEnabled = !loggingEnabled;
      if (!loggingEnabled) {
        stopRaceLogger();
        stopTrackLog();
      }
      Serial.print("Logging ");
      Serial.println(loggingEnabled ? "ENABLED" : "DISABLED");
      break;
    case 'd':
      if (!trackLogPath.isEmpty()) dumpFile(trackLogPath);
      else Serial.println("No current log file");
      break;
    case 'c':
    case 'C':
      clearLogs();
      resetTrack();
      stopRaceLogger();
      Serial.println("Logs cleared");
      break;
    case 'l':
    case 'L':
      Serial.println("Log files:");
      listLogsTo(Serial);
      break;
  }
}

// ===================== Serial telemetry =====================

static void printTelemetry() {
  uint32_t epoch = gpsUnixTime();

  Serial.print("t=");
  Serial.print(epoch ? String(epoch) : "?");

  Serial.print(" | GPS: ");
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(",");
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print("NO FIX");
  }
  Serial.print(" sat:");
  Serial.print(gps.satellites.isValid() ? String(gps.satellites.value()) : "?");

  Serial.print(" | IMU: ");
  if (imuData.hasData) {
    Serial.print(imuData.acc.x, 2); Serial.print(",");
    Serial.print(imuData.acc.y, 2); Serial.print(",");
    Serial.print(imuData.acc.z, 2);
  } else {
    Serial.print("N/A");
  }

  Serial.print(" | BAT:");
  uint16_t battMv = pmuBattVoltageMv();
  Serial.print(String(battMv / 1000) + "." + String((battMv % 1000) / 10) + "V");

  Serial.print(" | LOG:");
  Serial.println(loggingEnabled ? "ON" : "OFF");
}

// ===================== Setup =====================

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println();
  Serial.println("==== ApexDirector Core Pro ====");
  Serial.println("Commands: D=toggle log, d=dump, c=clear, l=list");

  // PMU must be first — it powers all peripherals
  bool pmuOk = initPmu();
  delay(200);

  // Display next — so we can show boot progress
  initDisplay();
  displayBootStatus("PMU", pmuOk);

  // IMU first — sets up shared HSPI bus and pulls IMU CS high,
  // which is needed before SD can use the same bus without contention
  bool imuOk = initImu();
  displayBootStatus("IMU", imuOk);

  // Storage (shares HSPI bus with IMU)
  storageReady = initStorage();
  displayBootStatus("SD", storageReady);
  if (!storageReady) {
    Serial.println("[SD] Storage error: logging disabled");
  }

  // Device config (requires SD)
  if (storageReady) {
    loadConfig();
    Serial.print("[Config] Serial: ");
    Serial.println(getConfig().serial);
  }

  // GPS (always succeeds — fix comes later)
  initGps();
  displayBootStatus("GPS", true);

  // WiFi AP + REST API + mDNS
  setupWiFi();
  displayBootStatus("API", true);

  // LoRa
  bool loraOk = initLoRa();
  displayBootStatus("LoRa", loraOk);
  if (!loraOk) {
    Serial.println("[LoRa] Telemetry TX disabled");
  }

  // Track + race logger (no display status — these are logic, not hardware)
  initTrack();
  initRaceLogger();

  // Summary to serial so we always see it
  Serial.println("---- Boot summary ----");
  Serial.print("  PMU:  "); Serial.println(pmuOk  ? "OK" : "FAIL");
  Serial.print("  IMU:  "); Serial.println(imuOk  ? "OK" : "FAIL");
  Serial.print("  SD:   "); Serial.println(storageReady ? "OK" : "FAIL");
  Serial.print("  LoRa: "); Serial.println(loraOk ? "OK" : "FAIL");
  Serial.println("----------------------");

  // List SD files for debugging
  if (storageReady) {
    Serial.println("---- SD files ----");
    File root = SD.open("/");
    if (root) {
      File f = root.openNextFile();
      while (f) {
        Serial.print("  ");
        Serial.print(f.name());
        Serial.print("  (");
        Serial.print(f.size());
        Serial.println(" B)");
        f = root.openNextFile();
      }
      root.close();
    }
    Serial.println("------------------");
  }

  displayBootDone();
  diagLog("=== BOOT ===");
  diagLogf("SD=%s IMU=%s LoRa=%s LOG=%s",
           storageReady ? "OK" : "FAIL",
           imuOk ? "OK" : "FAIL",
           loraOk ? "OK" : "FAIL",
           loggingEnabled ? "ON" : "OFF");
}

// ===================== Main loop =====================

void loop() {
  pollGps();

  uint32_t now = millis();

  // Snapshot GPS when new fix arrives
  updateLatestGps(now);
  static bool gpsFixPulse = false;
  if (latestGps.updated) {
    GpsFix fix = {};
    fix.valid       = latestGps.locValid;
    fix.courseValid  = latestGps.courseValid;
    fix.ms          = latestGps.fixMs;
    fix.epoch       = latestGps.epoch;
    fix.lat         = latestGps.lat;
    fix.lon         = latestGps.lon;
    fix.alt_m       = latestGps.alt_m;
    fix.spd_kmh     = latestGps.spd_kmh;
    fix.course_deg  = latestGps.course_deg;
    fix.hdop        = latestGps.hdop;
    fix.sats        = latestGps.sats;
    fix.good        = fix.valid && latestGps.hdopValid && latestGps.satsValid &&
                      latestGps.hdop <= 2.5 && latestGps.sats >= 6;
    fix.degraded    = fix.valid && !fix.good;
    onGpsFix(fix);
    updateWifiGpsStatus(fix.valid, fix.sats,
                        latestGps.hdopValid ? (float)latestGps.hdop : 99.0f,
                        latestGps.spdValid ? (float)latestGps.spd_kmh : 0.0f);
    latestGps.updated = false;
    gpsFixPulse = true;
  }

  handleSerialCommands();
  pollTrackButton(now);

  // IMU + track update at 100 Hz
  static uint32_t lastImuMs = 0;
  if (now - lastImuMs >= IMU_INTERVAL_MS) {
    while (now - lastImuMs >= IMU_INTERVAL_MS) {
      lastImuMs += IMU_INTERVAL_MS;
      updateImu();

      updateTrack(lastImuMs,
                  gpsFixPulse,
                  latestGps.locValid,
                  latestGps.courseValid,
                  latestGps.lat,
                  latestGps.lon,
                  latestGps.alt_m,
                  latestGps.spd_kmh,
                  latestGps.course_deg,
                  latestGps.hdop,
                  latestGps.sats,
                  latestGps.epoch);
      gpsFixPulse = false;

      uint8_t trackEvents = getTrackEventFlags();
      TrackUiState trackState = getTrackUiState();

      // Set race logger reference frame when track is loaded
      if ((trackState.trackState == TRACK_STATE_READY ||
           trackState.trackState == TRACK_STATE_RACING) &&
          (!hasRaceReference() || trackState.trackId != getRaceTrackId())) {
        double refLat, refLon, refAlt, refCourse;
        uint32_t refId;
        if (getTrackReference(refLat, refLon, refAlt, refCourse, refId)) {
          setRaceReference(refLat, refLon, refAlt, refId);
        }
      }

      // Start race logger on first track recognition
      if ((trackEvents & TRACK_EVENT_RECOGNIZED) && !isRaceLoggerActive()) {
        diagLogf("RACING entered — epoch=%lu ref=%s sd=%s log=%s",
                 (unsigned long)latestGps.epoch,
                 hasRaceReference() ? "OK" : "NONE",
                 storageReady ? "OK" : "FAIL",
                 loggingEnabled ? "ON" : "OFF");
        bool started = startRaceLogger(latestGps.epoch, lastImuMs, trackState.trackName);
        if (started) {
          Serial.println("[RACE] ATP recording started");
          diagLog("ATP recording STARTED");
        } else {
          Serial.println("[RACE] ATP recording FAILED to start!");
          Serial.print("  refFrame="); Serial.print(hasRaceReference() ? "OK" : "NONE");
          Serial.print("  sd="); Serial.print(storageReady ? "OK" : "FAIL");
          Serial.print("  log="); Serial.println(loggingEnabled ? "ON" : "OFF");
          diagLogf("ATP recording FAILED ref=%s sd=%s log=%s",
                   hasRaceReference() ? "OK" : "NONE",
                   storageReady ? "OK" : "FAIL",
                   loggingEnabled ? "ON" : "OFF");
        }
      }

      // Log lap events
      if (trackEvents & TRACK_EVENT_LAP_END) {
        diagLogf("LAP %lu — %lums", (unsigned long)trackState.lapCount,
                 (unsigned long)trackState.lastLapMs);
      }

      // Stop race logger if we leave RACING state (button reset, etc.)
      if (isRaceLoggerActive() && trackState.trackState != TRACK_STATE_RACING) {
        Serial.println("[RACE] ATP recording stopped (left RACING)");
        diagLog("ATP recording STOPPED (left RACING)");
        stopRaceLogger();
      }

      // Push IMU sample to race logger
      ImuSample sample = {};
      sample.ms         = lastImuMs;
      sample.imu        = imuData;
      sample.lapCount   = trackState.lapCount;
      sample.lapMs      = trackState.currentLapMs;
      sample.lastLapMs  = trackState.lastLapMs;
      sample.trackState = trackState.trackState;
      sample.lapActive  = trackState.lapActive;
      sample.eventFlags = trackEvents;
      pushImuSample(sample);
    }
  }

  flushRaceLogger(now);

  // Telemetry + LoRa at PRINT_INTERVAL_MS
  static uint32_t lastPrintMs = 0;
  if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;
    printTelemetry();

    TrackUiState ts = getTrackUiState();
    sendLoRaTelemetry(latestGps.epoch,
                      latestGps.lat,
                      latestGps.lon,
                      latestGps.alt_m,
                      latestGps.spd_kmh,
                      latestGps.hdop,
                      latestGps.sats,
                      imuData,
                      ts.trackState,
                      ts.lapCount,
                      ts.currentLapMs,
                      ts.lastLapMs,
                      ts.bestLapMs);
  }

  updateDisplay(now);
}
