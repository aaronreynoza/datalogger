
#include <Arduino.h>

#include "gps.h"
#include "imu.h"
#include "led.h"
#include "logging.h"
#include "pmu.h"
#include "wifi_server.h"

// sample rate for print/log (ms)
static const uint32_t SAMPLE_INTERVAL_MS = 200;

// ================== Serial commands ==================
// D -> toggle logging ON/OFF (new file each time you turn it ON)
// d -> dump current file
// c -> clear all telemetry_XYZ files
// l -> list files
void handleSerialCommands() {
  if (!Serial.available()) return;
  char cmd = Serial.read();
  switch (cmd) {
    case 'D':
      if (!storageReady) {
        Serial.println("Storage not ready; can't log");
        break;
      }
      if (!loggingEnabled) {
        if (!createSessionLogs()) {
          loggingEnabled = false;
          break;
        }
        loggingEnabled = true;
        Serial.print("Logging ENABLED to ");
        Serial.print(gpsLogPath);
        Serial.print(" and ");
        Serial.println(imuLogPath);
      } else {
        stopLogging();
        Serial.print("Logging DISABLED. Last file: ");
        Serial.print(gpsLogPath);
        Serial.print(" / ");
        Serial.println(imuLogPath);
      }
      break;
    case 'd':
      if (!gpsLogPath.isEmpty()) dumpFile(gpsLogPath);
      if (!imuLogPath.isEmpty()) dumpFile(imuLogPath);
      else Serial.println("No current log file.");
      break;
    case 'c':
    case 'C':
      clearLogs();
      Serial.println("All telemetry files removed.");
      break;
    case 'l':
    case 'L':
      Serial.println("Telemetry files:");
      listLogsTo(Serial);
      break;
    default:
      break;
  }
}

// ================== Printing to serial ==================
void printTelemetry() {
  uint32_t epoch = gpsUnixTime();

  Serial.print("t=");
  if (epoch) Serial.print(epoch);
  else       Serial.print("?");

  // GPS
  Serial.print(" | Lat/Lon: ");
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(", ");
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print("INVALID");
  }

  Serial.print(" | Sats: ");
  if (gps.satellites.isValid()) Serial.print(gps.satellites.value());
  else                          Serial.print("?");

  Serial.print(" | HDOP: ");
  if (gps.hdop.isValid()) Serial.print(gps.hdop.hdop());
  else                    Serial.print("?");

  Serial.print(" | Alt: ");
  if (gps.altitude.isValid()) {
    Serial.print(gps.altitude.meters());
    Serial.print(" m");
  } else {
    Serial.print("?");
  }

  Serial.print(" | Speed: ");
  if (gps.speed.isValid()) {
    Serial.print(gps.speed.kmph());
    Serial.print(" km/h");
  } else {
    Serial.print("?");
  }

  // IMU
  Serial.print(" | IMU ACC: ");
  if (imuData.hasData) {
    Serial.print(imuData.acc.x, 3); Serial.print(",");
    Serial.print(imuData.acc.y, 3); Serial.print(",");
    Serial.print(imuData.acc.z, 3);
  } else {
    Serial.print("N/A");
  }

  Serial.print(" | IMU GYR: ");
  if (imuData.hasData) {
    Serial.print(imuData.gyr.x, 3); Serial.print(",");
    Serial.print(imuData.gyr.y, 3); Serial.print(",");
    Serial.print(imuData.gyr.z, 3);
  } else {
    Serial.print("N/A");
  }

  Serial.print(" | IMU T: ");
  if (!isnan(imuData.tempC)) {
    Serial.print(imuData.tempC, 2);
    Serial.print(" C");
  } else {
    Serial.print("N/A");
  }

  Serial.print(" | logging=");
  Serial.println(loggingEnabled ? "ON" : "OFF");
}

// ================== Setup & Loop ==================
void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for USB */ }

  Serial.println();
  Serial.println("==== T-Beam Supreme GPS + IMU + WiFi logger ====");
  Serial.println("Commands: D=toggle logging, d=dump current file, c=clear, l=list files");

  initPmu();
  delay(200);
  initLed();

  // Storage
  storageReady = initStorage();
  if (!storageReady) {
    Serial.println("Storage error: logging disabled");
  }

  // GNSS
  initGps();

  // IMU
  initImu();

  // Wi-Fi AP + HTTP
  setupWiFi();
}

void loop() {
  // ----- Pump GPS -----
  pollGps();

  // ----- Read IMU -----
  updateImu();

  // ----- HTTP -----
  handleServer();

  // ----- Serial commands -----
  handleSerialCommands();

  // ----- Periodic sample → print + log -----
  static uint32_t lastSampleMs = 0;
  uint32_t now = millis();
  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;

    bool locValid  = gps.location.isValid();
    bool altValid  = gps.altitude.isValid();
    bool spdValid  = gps.speed.isValid();
    bool hdopValid = gps.hdop.isValid();
    bool satsValid = gps.satellites.isValid();

    double lat     = locValid  ? gps.location.lat()    : NAN;
    double lon     = locValid  ? gps.location.lng()    : NAN;
    double alt_m   = altValid  ? gps.altitude.meters() : NAN;
    double spd_kmh = spdValid  ? gps.speed.kmph()      : NAN;
    double hdop    = hdopValid ? gps.hdop.hdop()       : NAN;
    uint32_t sats  = satsValid ? gps.satellites.value(): 0;
    uint32_t epoch = gpsUnixTime();

    printTelemetry();
    appendGpsLog(epoch, lat, lon, alt_m, spd_kmh, hdop, sats);
    appendImuLog(epoch);
  }

  updateLed(now);
}
