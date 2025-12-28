#include "logging.h"

#include "imu.h"

static const char *LOG_GPS_PREFIX = "/gps_";
static const char *LOG_IMU_PREFIX = "/imu_";
static const char *LOG_STATUS_PREFIX = "/status_";
static const char *LOG_EXT        = ".csv";

bool storageReady = false;
bool loggingEnabled = false;
String gpsLogPath;
String imuLogPath;
String statusLogPath;

static int sessionId = 1;

static String makeLogPath(int id) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%s%03d%s", LOG_GPS_PREFIX, id, LOG_EXT);
  return String(buf);
}

static String makeImuLogPath(int id) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%s%03d%s", LOG_IMU_PREFIX, id, LOG_EXT);
  return String(buf);
}

static String makeStatusLogPath(int id) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%s%03d%s", LOG_STATUS_PREFIX, id, LOG_EXT);
  return String(buf);
}

static bool createNewLogFile(const String &path, const char *header) {
  File f = LittleFS.open(path, "w");
  if (!f) return false;

  f.println(header);
  f.close();
  Serial.print("Created log file: ");
  Serial.println(path);
  return true;
}

bool initStorage() {
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
    return false;
  }
  Serial.println("LittleFS mounted OK");
  return true;
}

bool createSessionLogs() {
  int newId = sessionId++;
  gpsLogPath = makeLogPath(newId);
  imuLogPath = makeImuLogPath(newId);
  statusLogPath = makeStatusLogPath(newId);

  static const char *gpsHeader =
    "epoch_s,ms_since_boot,lat,lon,alt_m,spd_kmph,hdop,sats";
  static const char *imuHeader =
    "epoch_s,ms_since_boot,ax_mps2,ay_mps2,az_mps2,"
    "gx_dps,gy_dps,gz_dps,imu_temp_C";
  static const char *statusHeader =
    "ms_since_boot,gps_ok,imu_ok,lora_ok,storage_ok";

#if defined(LOG_STATUS_ONLY)
  if (!createNewLogFile(statusLogPath, statusHeader)) {
    statusLogPath = "";
    return false;
  }
  gpsLogPath = "";
  imuLogPath = "";
  return true;
#else
  if (!createNewLogFile(gpsLogPath, gpsHeader) ||
      !createNewLogFile(imuLogPath, imuHeader)) {
    gpsLogPath = "";
    imuLogPath = "";
    return false;
  }
  statusLogPath = "";
  return true;
#endif
}

void stopLogging() {
  loggingEnabled = false;
}

void appendGpsLog(uint32_t epoch,
                  double lat, double lon,
                  double alt_m, double spd_kmph,
                  double hdop, uint32_t sats) {
#if defined(LOG_STATUS_ONLY)
  (void)epoch;
  (void)lat;
  (void)lon;
  (void)alt_m;
  (void)spd_kmph;
  (void)hdop;
  (void)sats;
  return;
#endif
  if (!storageReady || !loggingEnabled || gpsLogPath.isEmpty()) return;

  File f = LittleFS.open(gpsLogPath, "a");
  if (!f) {
    Serial.println("Failed to open GPS log file for append");
    return;
  }

  String line;
  line.reserve(120);

  if (epoch) line += String(epoch);
  line += ",";

  line += String(millis());
  line += ",";

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

  f.println(line);
  f.close();
}

void appendImuLog(uint32_t epoch) {
#if defined(LOG_STATUS_ONLY)
  (void)epoch;
  return;
#endif
  if (!storageReady || !loggingEnabled || imuLogPath.isEmpty()) return;

  File f = LittleFS.open(imuLogPath, "a");
  if (!f) {
    Serial.println("Failed to open IMU log file for append");
    return;
  }

  String line;
  line.reserve(120);

  if (epoch) line += String(epoch);
  line += ",";
  line += String(millis());
  line += ",";

  if (imuData.hasData) {
    line += String(imuData.acc.x, 3); line += ",";
    line += String(imuData.acc.y, 3); line += ",";
    line += String(imuData.acc.z, 3); line += ",";
    line += String(imuData.gyr.x, 3); line += ",";
    line += String(imuData.gyr.y, 3); line += ",";
    line += String(imuData.gyr.z, 3); line += ",";
  } else {
    line += ",,,,,,";
  }

  if (!isnan(imuData.tempC)) line += String(imuData.tempC, 2);
  f.println(line);
  f.close();
}

void appendStatusLog(bool gpsOk, bool imuOk, bool loraOk, bool storageOk) {
#if !defined(LOG_STATUS_ONLY)
  (void)gpsOk;
  (void)imuOk;
  (void)loraOk;
  (void)storageOk;
  return;
#endif
  if (!storageReady || !loggingEnabled || statusLogPath.isEmpty()) return;

  File f = LittleFS.open(statusLogPath, "a");
  if (!f) {
    Serial.println("Failed to open status log file for append");
    return;
  }

  String line;
  line.reserve(64);
  line += String(millis());
  line += ",";
  line += (gpsOk ? "1" : "0");
  line += ",";
  line += (imuOk ? "1" : "0");
  line += ",";
  line += (loraOk ? "1" : "0");
  line += ",";
  line += (storageOk ? "1" : "0");
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
    if (name.startsWith(LOG_GPS_PREFIX) ||
        name.startsWith(LOG_IMU_PREFIX) ||
        name.startsWith(LOG_STATUS_PREFIX)) {
      Serial.print("Removing "); Serial.println(name);
      LittleFS.remove(name);
    }
    file = root.openNextFile();
  }
  root.close();
  gpsLogPath = "";
  imuLogPath = "";
  statusLogPath = "";
  loggingEnabled = false;
  sessionId = 1;
}

void listLogsTo(Print &out) {
  File root = LittleFS.open("/");
  if (!root) {
    out.println("Failed to open root");
    return;
  }
  File file = root.openNextFile();
  while (file) {
    String name = file.name();
    if ((name.startsWith(LOG_GPS_PREFIX) ||
         name.startsWith(LOG_IMU_PREFIX) ||
         name.startsWith(LOG_STATUS_PREFIX)) &&
        name.endsWith(LOG_EXT)) {
      if (name.startsWith("/")) name.remove(0,1);
      out.println(name);
    }
    file = root.openNextFile();
  }
  root.close();
}
