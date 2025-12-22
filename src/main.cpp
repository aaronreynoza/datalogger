
#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>
#include <TinyGPSPlus.h>

#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include "SensorQMI8658.hpp"   // from SensorLib

// ================== GNSS ==================
HardwareSerial GNSS(1);
static const int GNSS_RX_PIN = 9;   // ESP32S3 receives on this
static const int GNSS_TX_PIN = 8;   // ESP32S3 transmits on this
TinyGPSPlus gps;

// ================ IMU (QMI8658 over SPI) ================
// Pins from Meshtastic T-Beam S3 Supreme docs
static const int IMU_MOSI_PIN = 35;
static const int IMU_MISO_PIN = 37;
static const int IMU_SCK_PIN  = 36;
static const int IMU_CS_PIN   = 34;

SPIClass imuSPI(HSPI);
SensorQMI8658 qmi;
IMUdata imuAcc;
IMUdata imuGyr;
bool     imuHasData   = false;
float    imuTempC     = NAN;
uint32_t imuTimestamp = 0;

// ================== Logging / FS ==================
static const char *LOG_PREFIX = "/telemetry_";
static const char *LOG_EXT    = ".csv";

bool   storageReady   = false;
bool   loggingEnabled = false;
int    sessionId      = 1;
String currentLogPath;

// sample rate for print/log (ms)
static const uint32_t SAMPLE_INTERVAL_MS = 200;

// ================== Wi-Fi / HTTP ==================
const char *WIFI_AP_SSID = "tbeam-telemetry";
const char *WIFI_AP_PASS = "tbeam123";   // change if you want
WebServer server(80);

// ================== Time helper (UNIX from GPS) ==================
uint32_t gpsUnixTime() {
  if (!gps.date.isValid() || !gps.time.isValid()) {
    return 0;
  }

  const int year  = gps.date.year();
  const int month = gps.date.month();
  const int day   = gps.date.day();
  const int hour  = gps.time.hour();
  const int min   = gps.time.minute();
  const int sec   = gps.time.second();

  if (year < 1970 || month < 1 || month > 12 || day < 1 || day > 31) {
    return 0;
  }

  auto isLeap = [](int y) {
    return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
  };

  static const uint8_t daysInMonth[] = {
      31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

  uint32_t days = 0;
  for (int y = 1970; y < year; ++y) {
    days += isLeap(y) ? 366 : 365;
  }
  for (int m = 1; m < month; ++m) {
    days += daysInMonth[m - 1];
    if (m == 2 && isLeap(year)) {
      days += 1;
    }
  }
  days += static_cast<uint32_t>(day - 1);

  return days * 86400UL + hour * 3600UL + min * 60UL + sec;
}

// ================== LittleFS helpers ==================
bool initStorage() {
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
    return false;
  }
  Serial.println("LittleFS mounted OK");
  return true;
}

String makeLogPath(int id) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%s%03d%s", LOG_PREFIX, id, LOG_EXT);
  return String(buf);
}

bool createNewLogFile(const String &path) {
  File f = LittleFS.open(path, "w");
  if (!f) return false;

  // CSV header
  f.println(
    "epoch_s,ms_since_boot,"
    "lat,lon,alt_m,spd_kmph,hdop,sats,"
    "ax_mps2,ay_mps2,az_mps2,"
    "gx_dps,gy_dps,gz_dps,imu_temp_C"
  );
  f.close();
  Serial.print("Created log file: ");
  Serial.println(path);
  return true;
}

void appendLog(uint32_t epoch,
               double lat, double lon,
               double alt_m, double spd_kmph,
               double hdop, uint32_t sats) {
  if (!storageReady || !loggingEnabled || currentLogPath.isEmpty()) return;

  File f = LittleFS.open(currentLogPath, "a");
  if (!f) {
    Serial.println("Failed to open log file for append");
    return;
  }

  String line;
  line.reserve(200);

  // epoch_s
  if (epoch) line += String(epoch);
  line += ",";

  // ms_since_boot
  line += String(millis());
  line += ",";

  // GPS
  if (!isnan(lat)) line += String(lat, 6);
  line += ",";
  if (!isnan(lon)) line += String(lon, 6);
  line += ",";
  if (!isnan(alt_m)) line += String(alt_m, 2);
  line += ",";
  if (!isnan(spd_kmph)) line += String(spd_kmph, 2);
  line += ",";
  if (!isnan(hdop)) line += String(hdop, 2);
  line += ",";
  if (sats) line += String(sats);
  line += ",";

  // IMU accel
  if (imuHasData) {
    line += String(imuAcc.x, 3);  line += ",";
    line += String(imuAcc.y, 3);  line += ",";
    line += String(imuAcc.z, 3);  line += ",";
  } else {
    line += ",,,";
  }

  // IMU gyro
  if (imuHasData) {
    line += String(imuGyr.x, 3);  line += ",";
    line += String(imuGyr.y, 3);  line += ",";
    line += String(imuGyr.z, 3);  line += ",";
  } else {
    line += ",,,";
  }

  // IMU temperature
  if (!isnan(imuTempC)) {
    line += String(imuTempC, 2);
  }

  f.println(line);
  f.close();
}

void dumpFile(const String &path) {
  File f = LittleFS.open(path, "r");
  if (!f) {
    Serial.println("No log file");
    return;
  }
  Serial.print("--- DUMP ");
  Serial.print(path);
  Serial.println(" ---");
  while (f.available()) Serial.write(f.read());
  Serial.println("--- END DUMP ---");
  f.close();
}

void clearLogs() {
  File root = LittleFS.open("/");
  if (!root) return;
  File file = root.openNextFile();
  while (file) {
    String name = file.name();
    if (name.startsWith(LOG_PREFIX)) {
      Serial.print("Removing "); Serial.println(name);
      LittleFS.remove(name);
    }
    file = root.openNextFile();
  }
  root.close();
  currentLogPath = "";
  loggingEnabled = false;
  sessionId = 1;
}

// ================== Wi-Fi / HTTP ==================
void listLogsTo(Print &out) {
  File root = LittleFS.open("/");
  if (!root) {
    out.println("Failed to open root");
    return;
  }
  File file = root.openNextFile();
  while (file) {
    String name = file.name();
    if (name.startsWith(LOG_PREFIX) && name.endsWith(LOG_EXT)) {
      if (name.startsWith("/")) name.remove(0,1);
      out.println(name);
    }
    file = root.openNextFile();
  }
  root.close();
}

void handleRoot() {
  String html =
    "<html><body><h1>T-Beam Telemetry</h1>"
    "<p><a href=\"/logs\">List logs (JSON)</a></p>"
    "<p>Download: /log?file=telemetry_001.csv</p>"
    "</body></html>";
  server.send(200, "text/html", html);
}

void handleLogs() {
  File root = LittleFS.open("/");
  if (!root) {
    server.send(500, "text/plain", "Failed to open root");
    return;
  }
  String json = "[";
  bool first = true;
  File file = root.openNextFile();
  while (file) {
    String name = file.name();
    if (name.startsWith(LOG_PREFIX) && name.endsWith(LOG_EXT)) {
      if (!first) json += ",";
      if (name.startsWith("/")) name.remove(0,1);
      json += "\"" + name + "\"";
      first = false;
    }
    file = root.openNextFile();
  }
  root.close();
  json += "]";
  server.send(200, "application/json", json);
}

void handleLogDownload() {
  if (!server.hasArg("file")) {
    server.send(400, "text/plain", "Missing ?file=");
    return;
  }
  String fname = server.arg("file");
  if (!fname.startsWith("/")) fname = "/" + fname;
  if (!LittleFS.exists(fname)) {
    server.send(404, "text/plain", "File not found");
    return;
  }
  File f = LittleFS.open(fname, "r");
  server.streamFile(f, "text/csv");
  f.close();
}

void setupWiFi() {
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS)) {
    Serial.println("WiFi AP start FAILED");
    return;
  }
  IPAddress ip = WiFi.softAPIP();
  Serial.print("WiFi AP: "); Serial.print(WIFI_AP_SSID);
  Serial.print("  pass="); Serial.println(WIFI_AP_PASS);
  Serial.print("Open: http://"); Serial.println(ip.toString());

  server.on("/", handleRoot);
  server.on("/logs", HTTP_GET, handleLogs);
  server.on("/log",  HTTP_GET, handleLogDownload);
  server.begin();
  Serial.println("HTTP server started on port 80");
}

// ================== IMU helpers (SPI, API-FIXED) ==================
void initImu() {
  Serial.println("Init QMI8658 over SPI (SensorLib 0.3.x API)");

  // SPI setup
  imuSPI.begin(IMU_SCK_PIN, IMU_MISO_PIN, IMU_MOSI_PIN, IMU_CS_PIN);
  pinMode(IMU_CS_PIN, OUTPUT);
  digitalWrite(IMU_CS_PIN, HIGH);

  // NOTE: new SensorLib API wants SPIClass& + CS pin
  if (!qmi.begin(imuSPI, IMU_CS_PIN)) {
    Serial.println("QMI8658 init FAILED");
    return;
  }

  Serial.print("QMI8658 ID: 0x");
  Serial.println(qmi.getChipID(), HEX);

  // New API: config* only take 3 args, "enable" is separate
  qmi.configAccelerometer(
      SensorQMI8658::ACC_RANGE_4G,
      SensorQMI8658::ACC_ODR_1000Hz,
      SensorQMI8658::LPF_MODE_0);

  qmi.configGyroscope(
      SensorQMI8658::GYR_RANGE_64DPS,
      SensorQMI8658::GYR_ODR_896_8Hz,
      SensorQMI8658::LPF_MODE_3);

  qmi.enableAccelerometer();
  qmi.enableGyroscope();

  Serial.println("QMI8658 configured");
}

void updateImu() {
  // Same pattern as your working version, but using new API
  if (qmi.getDataReady()) {
    if (qmi.getAccelerometer(imuAcc.x, imuAcc.y, imuAcc.z) &&
        qmi.getGyroscope(imuGyr.x, imuGyr.y, imuGyr.z)) {
      imuTempC    = qmi.getTemperature_C();
      imuTimestamp = qmi.getTimestamp();
      imuHasData   = true;
    } else {
      imuHasData = false;
    }
  } else {
    imuHasData = false;
  }
}

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
        currentLogPath = makeLogPath(sessionId++);
        if (!createNewLogFile(currentLogPath)) {
          currentLogPath = "";
          loggingEnabled = false;
          break;
        }
        loggingEnabled = true;
        Serial.print("Logging ENABLED to ");
        Serial.println(currentLogPath);
      } else {
        loggingEnabled = false;
        Serial.print("Logging DISABLED. Last file: ");
        Serial.println(currentLogPath);
      }
      break;
    case 'd':
      if (!currentLogPath.isEmpty()) dumpFile(currentLogPath);
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
  if (imuHasData) {
    Serial.print(imuAcc.x, 3); Serial.print(",");
    Serial.print(imuAcc.y, 3); Serial.print(",");
    Serial.print(imuAcc.z, 3);
  } else {
    Serial.print("N/A");
  }

  Serial.print(" | IMU GYR: ");
  if (imuHasData) {
    Serial.print(imuGyr.x, 3); Serial.print(",");
    Serial.print(imuGyr.y, 3); Serial.print(",");
    Serial.print(imuGyr.z, 3);
  } else {
    Serial.print("N/A");
  }

  Serial.print(" | IMU T: ");
  if (!isnan(imuTempC)) {
    Serial.print(imuTempC, 2);
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

  // Storage
  storageReady = initStorage();
  if (!storageReady) {
    Serial.println("Storage error: logging disabled");
  }

  // GNSS
  GNSS.begin(9600, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
  Serial.println("GNSS serial started, waiting for fix...");

  // IMU
  initImu();

  // Wi-Fi AP + HTTP
  setupWiFi();
}

void loop() {
  // ----- Pump GPS -----
  while (GNSS.available() > 0) {
    gps.encode(GNSS.read());
  }

  // ----- Read IMU -----
  updateImu();

  // ----- HTTP -----
  server.handleClient();

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
    appendLog(epoch, lat, lon, alt_m, spd_kmh, hdop, sats);
  }
}
