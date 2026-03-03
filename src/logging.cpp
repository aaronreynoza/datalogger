#include "logging.h"

#include <SD.h>
#include <SPI.h>
#include <cstring>

#include "imu.h"  // for sharedHSPI()

static constexpr int SD_CS_PIN = 47;

static const char *TRACKS_FILE = "/tracks.csv";

bool storageReady = false;
bool loggingEnabled = false;
String trackLogPath;

static File trackFile;
static bool trackLogOpen = false;

static String makeTrackLogPath(uint32_t trackId) {
  char buf[32];
  snprintf(buf, sizeof(buf), "/track_%08lX.csv", static_cast<unsigned long>(trackId));
  return String(buf);
}

bool initStorage() {
  Serial.print("[SD] CS=");
  Serial.print(SD_CS_PIN);
  Serial.println(" — attempting mount...");
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  delay(50);
  if (!SD.begin(SD_CS_PIN, sharedHSPI())) {
    Serial.println("[SD] mount FAILED — check card is FAT32 and seated properly");
    storageReady = false;
    loggingEnabled = false;
    return false;
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.print("[SD] mounted OK — ");
  Serial.print((uint32_t)cardSize);
  Serial.println(" MB");
  storageReady = true;
  loggingEnabled = true;
  return true;
}

bool startTrackLog(uint32_t trackId) {
  if (!storageReady || !loggingEnabled) return false;
  if (trackLogOpen) {
    trackFile.close();
    trackLogOpen = false;
  }
  trackLogPath = makeTrackLogPath(trackId);
  trackFile = SD.open(trackLogPath, "w");
  if (!trackFile) {
    trackLogPath = "";
    return false;
  }
  trackFile.println("epoch_s,ms_since_boot,lat,lon,alt_m,spd_kmph,course_deg,hdop,sats");
  trackFile.flush();
  trackLogOpen = true;
  return true;
}

void stopTrackLog() {
  if (trackLogOpen) {
    trackFile.close();
    trackLogOpen = false;
  }
}

void appendTrackLog(uint32_t epoch,
                    double lat, double lon, double alt_m,
                    double spd_kmph, double course_deg,
                    double hdop, uint32_t sats) {
  if (!storageReady || !loggingEnabled || !trackLogOpen) return;

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
  if (!isnan(course_deg)) line += String(course_deg, 2);
  line += ",";
  if (!isnan(hdop)) line += String(hdop, 2);
  line += ",";
  if (sats) line += String(sats);
  trackFile.println(line);
  trackFile.flush();
}

void dumpFile(const String &path) {
  File f = SD.open(path, "r");
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
  if (trackLogOpen) {
    trackFile.close();
    trackLogOpen = false;
  }

  // Remove race logs (.atp), track recording logs, but NOT tracks.csv or config.json
  File root = SD.open("/");
  if (!root) return;
  File file = root.openNextFile();
  while (file) {
    String name = file.name();
    if (name.startsWith("/track_") || name.startsWith("/race-") ||
        name.endsWith(".atp")) {
      Serial.print("Removing "); Serial.println(name);
      SD.remove(name);
    }
    file = root.openNextFile();
  }
  root.close();
  trackLogPath = "";
}

// ===================== Persistent diagnostic log =====================

static const char *DIAG_LOG_FILE = "/diag.log";
static constexpr size_t DIAG_MAX_SIZE = 8192;  // 8 KB max, truncate old entries

void diagLog(const char *msg) {
  if (!storageReady) return;

  // Truncate if too large (keep last half)
  File check = SD.open(DIAG_LOG_FILE, "r");
  if (check) {
    size_t sz = check.size();
    check.close();
    if (sz > DIAG_MAX_SIZE) {
      // Read last half, rewrite
      File r = SD.open(DIAG_LOG_FILE, "r");
      r.seek(sz - DIAG_MAX_SIZE / 2);
      String keep;
      keep.reserve(DIAG_MAX_SIZE / 2 + 64);
      keep = "--- truncated ---\n";
      while (r.available()) {
        keep += (char)r.read();
      }
      r.close();
      File w = SD.open(DIAG_LOG_FILE, "w");
      if (w) { w.print(keep); w.close(); }
    }
  }

  File f = SD.open(DIAG_LOG_FILE, "a");
  if (!f) return;

  // Write boot-relative timestamp
  uint32_t sec = millis() / 1000;
  char timeBuf[16];
  snprintf(timeBuf, sizeof(timeBuf), "[%lu.%lus] ",
           (unsigned long)(sec / 60), (unsigned long)(sec % 60));
  f.print(timeBuf);
  f.println(msg);
  f.close();
}

void diagLogf(const char *fmt, ...) {
  char buf[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  diagLog(buf);
}

String readDiagLog() {
  if (!storageReady) return "SD not ready";
  File f = SD.open(DIAG_LOG_FILE, "r");
  if (!f) return "No diagnostic log";
  String content;
  content.reserve(f.size() + 1);
  while (f.available()) content += (char)f.read();
  f.close();
  return content;
}

void listLogsTo(Print &out) {
  File root = SD.open("/");
  if (!root) {
    out.println("Failed to open root");
    return;
  }
  File file = root.openNextFile();
  while (file) {
    String name = file.name();
    if (name.startsWith("/track_") ||
        name.startsWith("/race-") ||
        name.endsWith(".atp") ||
        name == TRACKS_FILE) {
      if (name.startsWith("/")) name.remove(0, 1);
      out.println(name);
    }
    file = root.openNextFile();
  }
  root.close();
}
