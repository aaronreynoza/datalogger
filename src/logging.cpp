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

  char buf[160];
  char *p = buf;
  char *end = buf + sizeof(buf);

  if (epoch) p += snprintf(p, end - p, "%lu", (unsigned long)epoch);
  p += snprintf(p, end - p, ",%lu,", (unsigned long)millis());
  if (!isnan(lat)) p += snprintf(p, end - p, "%.6f", lat);
  *p++ = ',';
  if (!isnan(lon)) p += snprintf(p, end - p, "%.6f", lon);
  *p++ = ',';
  if (!isnan(alt_m)) p += snprintf(p, end - p, "%.2f", alt_m);
  *p++ = ',';
  if (!isnan(spd_kmph)) p += snprintf(p, end - p, "%.2f", spd_kmph);
  *p++ = ',';
  if (!isnan(course_deg)) p += snprintf(p, end - p, "%.2f", course_deg);
  *p++ = ',';
  if (!isnan(hdop)) p += snprintf(p, end - p, "%.2f", hdop);
  *p++ = ',';
  if (sats) p += snprintf(p, end - p, "%lu", (unsigned long)sats);
  *p = '\0';

  trackFile.println(buf);
}

void flushTrackLog() {
  if (trackLogOpen) trackFile.flush();
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

static char diagBuf[2048];
static size_t diagBufPos = 0;
static uint32_t lastDiagFlushMs = 0;
static constexpr uint32_t DIAG_FLUSH_INTERVAL_MS = 5000;

void flushDiagLog() {
  if (!storageReady || diagBufPos == 0) return;

  // Truncate if file too large (keep last half)
  File check = SD.open(DIAG_LOG_FILE, "r");
  if (check) {
    size_t sz = check.size();
    check.close();
    if (sz > DIAG_MAX_SIZE) {
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
  f.write((const uint8_t *)diagBuf, diagBufPos);
  f.close();
  diagBufPos = 0;
  lastDiagFlushMs = millis();
}

void tickDiagLog() {
  if (diagBufPos > 0 && millis() - lastDiagFlushMs >= DIAG_FLUSH_INTERVAL_MS) {
    flushDiagLog();
  }
}

void diagLog(const char *msg) {
  if (!storageReady) return;

  uint32_t sec = millis() / 1000;
  char timeBuf[16];
  int tLen = snprintf(timeBuf, sizeof(timeBuf), "[%lu.%lus] ",
                      (unsigned long)(sec / 60), (unsigned long)(sec % 60));

  size_t msgLen = strlen(msg);
  size_t needed = tLen + msgLen + 1;  // +1 for newline

  if (diagBufPos + needed >= sizeof(diagBuf)) {
    flushDiagLog();  // buffer full, force flush
  }

  memcpy(diagBuf + diagBufPos, timeBuf, tLen);
  diagBufPos += tLen;
  memcpy(diagBuf + diagBufPos, msg, msgLen);
  diagBufPos += msgLen;
  diagBuf[diagBufPos++] = '\n';
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
  flushDiagLog();  // write pending messages first
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
