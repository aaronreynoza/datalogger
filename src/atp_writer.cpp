#include "atp_writer.h"

#include <cstring>
#include <cmath>

#include "logging.h"

static constexpr uint32_t INITIAL_CHUNK_CAPACITY = 7200;  // ~2 hours

// ===================== Little-endian write helpers =====================

static void writeU8(File &f, uint8_t v) { f.write(v); }

static void writeU16(File &f, uint16_t v) {
  uint8_t buf[2] = {(uint8_t)(v & 0xFF), (uint8_t)(v >> 8)};
  f.write(buf, 2);
}

static void writeU32(File &f, uint32_t v) {
  uint8_t buf[4];
  buf[0] = v & 0xFF; buf[1] = (v >> 8) & 0xFF;
  buf[2] = (v >> 16) & 0xFF; buf[3] = (v >> 24) & 0xFF;
  f.write(buf, 4);
}

static void writeU64(File &f, uint64_t v) {
  writeU32(f, (uint32_t)(v & 0xFFFFFFFF));
  writeU32(f, (uint32_t)(v >> 32));
}

static void writeF32(File &f, float v) {
  uint32_t bits;
  memcpy(&bits, &v, 4);
  writeU32(f, bits);
}

static void writeF64(File &f, double v) {
  uint64_t bits;
  memcpy(&bits, &v, 8);
  writeU64(f, bits);
}

static void writeI16(File &f, int16_t v) {
  writeU16(f, static_cast<uint16_t>(v));
}

static void writeLString(File &f, const char *str) {
  uint16_t len = str ? (uint16_t)strlen(str) : 0;
  writeU16(f, len);
  if (len > 0) f.write(reinterpret_cast<const uint8_t*>(str), len);
}

// ===================== Buffer write helpers =====================

static void bufWriteU8(uint8_t *buf, size_t &pos, uint8_t v) {
  buf[pos++] = v;
}

static void bufWriteU16(uint8_t *buf, size_t &pos, uint16_t v) {
  buf[pos++] = v & 0xFF;
  buf[pos++] = (v >> 8) & 0xFF;
}

static void bufWriteU32(uint8_t *buf, size_t &pos, uint32_t v) {
  buf[pos++] = v & 0xFF;
  buf[pos++] = (v >> 8) & 0xFF;
  buf[pos++] = (v >> 16) & 0xFF;
  buf[pos++] = (v >> 24) & 0xFF;
}

static void bufWriteI16(uint8_t *buf, size_t &pos, int16_t v) {
  bufWriteU16(buf, pos, static_cast<uint16_t>(v));
}

static void bufWriteF32(uint8_t *buf, size_t &pos, float v) {
  uint32_t bits;
  memcpy(&bits, &v, 4);
  bufWriteU32(buf, pos, bits);
}

static void bufWriteF64(uint8_t *buf, size_t &pos, double v) {
  uint64_t bits;
  memcpy(&bits, &v, 8);
  bufWriteU32(buf, pos, (uint32_t)(bits & 0xFFFFFFFF));
  bufWriteU32(buf, pos, (uint32_t)(bits >> 32));
}

// ===================== Fixed-point encoding =====================

static int16_t encodeAccel(float g) {
  float v = g * ACCEL_SCALE;
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)roundf(v);
}

static int16_t encodeGyro(float dps) {
  float v = dps * GYRO_SCALE;
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)roundf(v);
}

static int16_t encodeTemp(float c) {
  float v = c * TEMP_SCALE;
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)roundf(v);
}

// ===================== Read helpers (for atpReadMeta) =====================

static uint16_t readU16(File &f) {
  uint8_t buf[2] = {0};
  f.read(buf, 2);
  return buf[0] | (buf[1] << 8);
}

static uint32_t readU32(File &f) {
  uint8_t buf[4] = {0};
  f.read(buf, 4);
  return buf[0] | (buf[1] << 8) | (buf[2] << 16) | ((uint32_t)buf[3] << 24);
}

static uint64_t readU64(File &f) {
  uint32_t lo = readU32(f);
  uint32_t hi = readU32(f);
  return (uint64_t)lo | ((uint64_t)hi << 32);
}

static void readLString(File &f, char *out, size_t maxLen) {
  uint16_t len = readU16(f);
  if (len == 0) { out[0] = '\0'; return; }
  size_t toRead = (len < maxLen - 1) ? len : maxLen - 1;
  f.read(reinterpret_cast<uint8_t*>(out), toRead);
  out[toRead] = '\0';
  // Skip remaining if string was longer than buffer
  if (len > toRead) {
    for (uint16_t i = 0; i < len - toRead; ++i) f.read();
  }
}

// ===================== Channel table =====================

struct ChannelDef {
  uint16_t id;
  const char *name;
  uint8_t unit;
  uint8_t dataType;
  uint16_t rateHz;
  uint8_t flags;
};

// Unit enum values (from ATP spec)
static constexpr uint8_t UNIT_NONE      = 0;
static constexpr uint8_t UNIT_G         = 1;
static constexpr uint8_t UNIT_DEG_PER_S = 2;
static constexpr uint8_t UNIT_DEG_C     = 3;
static constexpr uint8_t UNIT_DEG       = 4;
static constexpr uint8_t UNIT_M_PER_S   = 5;
static constexpr uint8_t UNIT_METERS    = 10;
static constexpr uint8_t UNIT_MS        = 13;

// Data type enum
static constexpr uint8_t DT_FLOAT32 = 1;
static constexpr uint8_t DT_FLOAT64 = 2;
static constexpr uint8_t DT_INT16   = 5;
static constexpr uint8_t DT_UINT16  = 6;
static constexpr uint8_t DT_UINT8   = 4;

// Channel flags
static constexpr uint8_t CF_IS_GPS = 1 << 1;

static const ChannelDef CHANNELS[] = {
  {1,  "dynamics.gforce.lon",   UNIT_G,         DT_INT16,   100, 0},
  {2,  "dynamics.gforce.lat",   UNIT_G,         DT_INT16,   100, 0},
  {3,  "dynamics.gforce.vert",  UNIT_G,         DT_INT16,   100, 0},
  {4,  "dynamics.roll.rate",    UNIT_DEG_PER_S, DT_INT16,   100, 0},
  {5,  "dynamics.pitch.rate",   UNIT_DEG_PER_S, DT_INT16,   100, 0},
  {6,  "dynamics.yaw.rate",     UNIT_DEG_PER_S, DT_INT16,   100, 0},
  {7,  "environment.imu.temp",  UNIT_DEG_C,     DT_INT16,   100, 0},
  {8,  "position.gps.lat",      UNIT_DEG,       DT_FLOAT64, 10,  CF_IS_GPS},
  {9,  "position.gps.lon",      UNIT_DEG,       DT_FLOAT64, 10,  CF_IS_GPS},
  {10, "position.gps.alt",      UNIT_METERS,    DT_FLOAT32, 10,  CF_IS_GPS},
  {11, "position.gps.speed",    UNIT_M_PER_S,   DT_FLOAT32, 10,  CF_IS_GPS},
  {12, "position.gps.course",   UNIT_DEG,       DT_FLOAT32, 10,  CF_IS_GPS},
  {13, "position.local.x",      UNIT_METERS,    DT_FLOAT32, 10,  CF_IS_GPS},
  {14, "position.local.y",      UNIT_METERS,    DT_FLOAT32, 10,  CF_IS_GPS},
  {15, "position.local.z",      UNIT_METERS,    DT_FLOAT32, 10,  CF_IS_GPS},
  {16, "dynamics.vel.north",    UNIT_M_PER_S,   DT_FLOAT32, 10,  CF_IS_GPS},
  {17, "dynamics.vel.east",     UNIT_M_PER_S,   DT_FLOAT32, 10,  CF_IS_GPS},
  {18, "position.lap.dist",     UNIT_METERS,    DT_FLOAT32, 10,  CF_IS_GPS},
  {19, "dynamics.speed",         UNIT_M_PER_S,   DT_FLOAT32, 10,  CF_IS_GPS},
};

static constexpr uint16_t NUM_CHANNELS = sizeof(CHANNELS) / sizeof(CHANNELS[0]);

// ===================== Write file sections =====================

static void writeFileHeader(File &f, uint16_t flags, uint32_t headerSize, uint32_t dataOffset) {
  f.write(ATP_MAGIC, 4);
  writeU16(f, ATP_VERSION);
  writeU16(f, flags);
  writeU32(f, headerSize);
  writeU32(f, dataOffset);
}

static uint32_t computeMetadataSize(const DeviceConfig &config, const char *trackName) {
  uint32_t size = 2 + 1 + 1 + 8 + 8 + 4 + 4;  // fixed fields
  size += 2 + strlen(config.driverName);   // driver lstring
  size += 2 + strlen(trackName);           // track lstring
  size += 2 + strlen(config.vehicleName);  // vehicle lstring
  size += 2;                               // session_notes (empty)
  size += 2;                               // extra_count
  return size;
}

static void writeSessionMetadata(File &f, const DeviceConfig &config,
                                  uint64_t startTimeMs, uint32_t trackId,
                                  const char *trackName) {
  uint16_t metaSize = (uint16_t)computeMetadataSize(config, trackName);
  writeU16(f, metaSize);
  writeU8(f, ATP_SOURCE_CORE_PRO);     // source
  writeU8(f, config.sessionType);       // session_type
  writeU64(f, startTimeMs);             // start_time_ms
  writeU64(f, 0);                       // end_time_ms (0 = not closed)
  writeU32(f, 0);                       // duration_ms (0 = not closed)
  writeU32(f, trackId);                 // track_id
  writeLString(f, config.driverName);
  writeLString(f, trackName);
  writeLString(f, config.vehicleName);
  writeLString(f, "");                  // session_notes
  writeU16(f, 0);                       // extra_count
}

static uint32_t computeChannelTableSize() {
  uint32_t size = 2 + 2;  // table_size + channel_count
  for (uint16_t i = 0; i < NUM_CHANNELS; ++i) {
    size += 2;                          // channel_id
    size += 2 + strlen(CHANNELS[i].name);  // canonical_name lstring
    size += 1 + 1 + 2 + 1;             // unit + data_type + sample_rate + flags
  }
  return size;
}

static void writeChannelTable(File &f) {
  uint16_t tableSize = (uint16_t)computeChannelTableSize();
  writeU16(f, tableSize);
  writeU16(f, NUM_CHANNELS);
  for (uint16_t i = 0; i < NUM_CHANNELS; ++i) {
    const ChannelDef &ch = CHANNELS[i];
    writeU16(f, ch.id);
    writeLString(f, ch.name);
    writeU8(f, ch.unit);
    writeU8(f, ch.dataType);
    writeU16(f, ch.rateHz);
    writeU8(f, ch.flags);
  }
}

static uint32_t computeTrackDefSize(uint16_t numPoints) {
  return 2 + 4 + 8 + 8 + 8 + 8 + 2 + (numPoints * 24);
}

static void writeTrackDefinition(File &f, uint32_t trackId,
                                  double startLat, double startLon,
                                  double startAlt, double startHeading,
                                  const AtpTrackPoint *points, uint16_t numPoints) {
  uint16_t sectionSize = (uint16_t)computeTrackDefSize(numPoints);
  writeU16(f, sectionSize);
  writeU32(f, trackId);
  writeF64(f, startLat);
  writeF64(f, startLon);
  writeF64(f, startAlt);
  writeF64(f, startHeading);
  writeU16(f, numPoints);
  for (uint16_t i = 0; i < numPoints; ++i) {
    writeF64(f, points[i].lat);
    writeF64(f, points[i].lon);
    writeF32(f, points[i].alt);
    writeF32(f, points[i].speedKmh);
  }
}

// ===================== Chunk writing =====================

static void resetChunk(AtpSession &s, uint32_t timestampMs) {
  s.imuCount = 0;
  s.hasGps = false;
  s.gpsTimeOffsetMs = 0;
  s.lapEventCount = 0;
  s.stateChangeCount = 0;
  s.chunkFlags = 0;
  s.chunkStartMs = timestampMs;
}

static void writeChunkToFile(AtpSession &s) {
  if (s.imuCount == 0 && !s.hasGps && s.lapEventCount == 0 && s.stateChangeCount == 0) {
    return;  // empty chunk, skip
  }

  uint32_t fileOffset = s.file.position();

  // Count records
  uint8_t recordCount = 0;
  if (s.imuCount > 0) recordCount++;
  if (s.hasGps) recordCount++;
  recordCount += s.lapEventCount;
  recordCount += s.stateChangeCount;

  // Calculate chunk size
  // Chunk header: 4(magic) + 4(size) + 4(timestamp) + 1(record_count) + 1(flags) + 2(lap_number) = 16
  uint32_t chunkSize = 16;

  // IMU batch: 3(record header) + 2(time_offset) + 1(sample_count) + 1(sample_size) + data
  if (s.imuCount > 0) {
    chunkSize += 5 + 2 + (uint32_t)s.imuCount * IMU_SAMPLE_SIZE;
  }
  // GPS fix: 5(record header) + 55(data) = 60
  if (s.hasGps) {
    chunkSize += 5 + GPS_RECORD_SIZE;
  }
  // Lap events
  chunkSize += s.lapEventCount * (5u + LAP_EVENT_SIZE);
  // State changes
  chunkSize += s.stateChangeCount * (5u + STATE_CHANGE_SIZE);

  // Write chunk header
  s.file.write(CHK_MAGIC, 4);
  writeU32(s.file, chunkSize);
  writeU32(s.file, s.chunkStartMs);
  writeU8(s.file, recordCount);
  writeU8(s.file, s.chunkFlags);
  writeU16(s.file, s.currentLapNumber);

  // Write IMU batch record
  if (s.imuCount > 0) {
    uint16_t recSize = 5 + 2 + (uint16_t)s.imuCount * IMU_SAMPLE_SIZE;
    writeU8(s.file, REC_IMU_BATCH);
    writeU16(s.file, recSize);
    writeU16(s.file, 0);  // time_offset_ms (start of chunk)
    writeU8(s.file, s.imuCount);
    writeU8(s.file, IMU_SAMPLE_SIZE);
    for (uint8_t i = 0; i < s.imuCount; ++i) {
      s.file.write(s.imuBuf[i], IMU_SAMPLE_SIZE);
    }
    s.totalImuSamples += s.imuCount;
  }

  // Write GPS fix record
  if (s.hasGps) {
    uint16_t recSize = 5 + GPS_RECORD_SIZE;
    writeU8(s.file, REC_GPS_FIX);
    writeU16(s.file, recSize);
    writeU16(s.file, s.gpsTimeOffsetMs);
    s.file.write(s.gpsBuf, GPS_RECORD_SIZE);
  }

  // Write lap events
  for (uint8_t i = 0; i < s.lapEventCount; ++i) {
    uint16_t recSize = 5 + LAP_EVENT_SIZE;
    writeU8(s.file, REC_LAP_EVENT);
    writeU16(s.file, recSize);
    // time_offset_ms is embedded in the lap event buffer's first 2 bytes
    s.file.write(s.lapBuf[i], LAP_EVENT_SIZE);
  }

  // Write state changes
  for (uint8_t i = 0; i < s.stateChangeCount; ++i) {
    uint16_t recSize = 5 + STATE_CHANGE_SIZE;
    writeU8(s.file, REC_STATE_CHANGE);
    writeU16(s.file, recSize);
    s.file.write(s.stateBuf[i], STATE_CHANGE_SIZE);
  }

  s.file.flush();

  // Record in index
  if (s.chunkCount < s.chunkCapacity) {
    s.chunkFileOffsets[s.chunkCount] = fileOffset;
    s.chunkTimestamps[s.chunkCount] = s.chunkStartMs;
    s.chunkSizes[s.chunkCount] = (uint16_t)chunkSize;
  }
  s.chunkCount++;
}

// ===================== Index and footer writing =====================

static uint32_t computeCrc32(AtpSession &s) {
  // Simple CRC32 of data section — we approximate by using chunk count + total samples
  // A proper implementation would re-read the data section, but that's slow on SD
  // For v1, we use a fast hash of the metadata
  uint32_t crc = 0;
  crc ^= s.chunkCount;
  crc ^= s.totalImuSamples;
  crc ^= s.lapCount;
  return crc;
}

static void writeChunkIndex(AtpSession &s) {
  s.file.write(IDX_MAGIC, 4);
  writeU32(s.file, s.chunkCount);
  uint32_t count = (s.chunkCount <= s.chunkCapacity) ? s.chunkCount : s.chunkCapacity;
  for (uint32_t i = 0; i < count; ++i) {
    writeU32(s.file, s.chunkTimestamps[i]);
    writeU32(s.file, s.chunkFileOffsets[i]);
    writeU32(s.file, s.chunkSizes[i]);
  }
}

static void writeLapIndex(AtpSession &s) {
  s.file.write(LAP_MAGIC, 4);
  writeU16(s.file, s.lapCount);
  for (uint16_t i = 0; i < s.lapCount; ++i) {
    const AtpSession::LapEntry &lap = s.laps[i];
    writeU16(s.file, lap.number);
    writeU32(s.file, lap.startChunk);
    writeU32(s.file, lap.endChunk);
    writeU32(s.file, lap.timeMs);
    writeU16(s.file, lap.flags);
    writeF32(s.file, lap.distanceM);
  }
}

static void writeFooter(AtpSession &s, uint32_t chunkIndexOffset, uint32_t lapIndexOffset,
                         uint32_t durationMs) {
  s.file.write(ATP_MAGIC, 4);
  writeU32(s.file, chunkIndexOffset);
  writeU32(s.file, lapIndexOffset);
  writeU32(s.file, s.chunkCount);
  writeU16(s.file, s.lapCount);
  writeU16(s.file, 0);  // reserved
  writeU32(s.file, durationMs);
  writeU32(s.file, s.totalImuSamples);
  writeU32(s.file, computeCrc32(s));
}

// ===================== Public API =====================

bool atpOpen(AtpSession &s, uint32_t epochS, uint32_t bootMs,
             uint32_t trackId, const DeviceConfig &config,
             double startLat, double startLon, double startAlt,
             double startHeading, const char *trackName,
             const AtpTrackPoint *trackPoints, uint16_t numTrackPoints) {
  Serial.println("[ATP] atpOpen() entered");
  diagLog("atpOpen entered");

  // Close any previously open file properly before reinitializing
  if (s.file) s.file.close();

  // Zero POD members without touching the File object (memset on File is UB)
  s.active = false;
  s.filename[0] = '\0';
  s.sessionStartMs = 0;
  s.sessionStartEpochMs = 0;
  s.dataOffset = 0;
  s.headerFlags = 0;
  s.imuCount = 0;
  s.hasGps = false;
  s.gpsTimeOffsetMs = 0;
  s.lapEventCount = 0;
  s.stateChangeCount = 0;
  s.currentLapNumber = 0;
  s.chunkFlags = 0;
  s.chunkStartMs = 0;
  s.chunkFileOffsets = nullptr;
  s.chunkTimestamps = nullptr;
  s.chunkSizes = nullptr;
  s.chunkCount = 0;
  s.chunkCapacity = 0;
  s.lapCount = 0;
  s.currentLapStartChunk = 0;
  s.totalImuSamples = 0;

  Serial.println("[ATP] Allocating chunk index...");
  Serial.printf("[ATP] Free heap: %u, PSRAM: %u\n", ESP.getFreeHeap(), ESP.getFreePsram());
  diagLogf("alloc heap=%u psram=%u", ESP.getFreeHeap(), ESP.getFreePsram());

  // Allocate chunk index — try PSRAM first, fall back to heap
  s.chunkCapacity = INITIAL_CHUNK_CAPACITY;
  s.chunkFileOffsets = (uint32_t *)ps_malloc(s.chunkCapacity * sizeof(uint32_t));
  s.chunkTimestamps  = (uint32_t *)ps_malloc(s.chunkCapacity * sizeof(uint32_t));
  s.chunkSizes       = (uint16_t *)ps_malloc(s.chunkCapacity * sizeof(uint16_t));
  if (!s.chunkFileOffsets || !s.chunkTimestamps || !s.chunkSizes) {
    // PSRAM unavailable — fall back to heap with smaller capacity
    free(s.chunkFileOffsets); free(s.chunkTimestamps); free(s.chunkSizes);
    s.chunkCapacity = 1800;  // ~30 min
    s.chunkFileOffsets = (uint32_t *)malloc(s.chunkCapacity * sizeof(uint32_t));
    s.chunkTimestamps  = (uint32_t *)malloc(s.chunkCapacity * sizeof(uint32_t));
    s.chunkSizes       = (uint16_t *)malloc(s.chunkCapacity * sizeof(uint16_t));
    if (!s.chunkFileOffsets || !s.chunkTimestamps || !s.chunkSizes) {
      Serial.println("[ATP] Chunk index allocation failed");
      free(s.chunkFileOffsets); free(s.chunkTimestamps); free(s.chunkSizes);
      return false;
    }
    Serial.println("[ATP] Using heap fallback (1800 chunks)");
    diagLog("alloc: heap fallback 1800");
  } else {
    diagLog("alloc: PSRAM OK 7200");
  }

  // Build filename
  char filename[32];
  int year = 1970, month = 1, day = 1, hour = 0, minute = 0;
  if (epochS) {
    // Simple epoch to date (reuse logging.cpp approach)
    uint32_t seconds = epochS;
    uint32_t days = seconds / 86400;
    uint32_t timeOfDay = seconds % 86400;
    hour = timeOfDay / 3600;
    minute = (timeOfDay % 3600) / 60;

    year = 1970;
    auto isLeap = [](int y) { return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0); };
    while (true) {
      uint16_t diy = isLeap(year) ? 366 : 365;
      if (days < diy) break;
      days -= diy;
      year++;
    }
    static const uint8_t daysInMonth[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    month = 1;
    for (int i = 0; i < 12; ++i) {
      uint8_t dim = daysInMonth[i];
      if (i == 1 && isLeap(year)) dim++;
      if (days < dim) { month = i + 1; day = days + 1; break; }
      days -= dim;
    }
  }
  snprintf(filename, sizeof(filename), "/race-%04d%02d%02d-%02d%02d.atp",
           year, month, day, hour, minute);

  strlcpy(s.filename, filename, sizeof(s.filename));
  diagLogf("creating %s", filename);
  s.file = SD.open(filename, "w");
  if (!s.file) {
    Serial.print("[ATP] Failed to create ");
    Serial.println(filename);
    diagLog("file create FAILED");
    free(s.chunkFileOffsets); free(s.chunkTimestamps); free(s.chunkSizes);
    return false;
  }
  diagLog("file created OK");

  s.sessionStartMs = bootMs;
  s.sessionStartEpochMs = epochS ? (uint64_t)epochS * 1000ULL : 0;

  // Calculate header sizes
  uint16_t headerFlags = numTrackPoints > 0 ? ATP_FLAG_HAS_TRACK_RECORDING : 0;
  s.headerFlags = headerFlags;

  uint32_t metaSize = computeMetadataSize(config, trackName);
  uint32_t chanTableSize = computeChannelTableSize();
  uint32_t trackDefSize = computeTrackDefSize(numTrackPoints);
  uint32_t dataOffset = 16 + metaSize + chanTableSize + trackDefSize;

  // Write all header sections
  diagLog("writing file header");
  writeFileHeader(s.file, headerFlags, dataOffset, dataOffset);
  diagLog("writing session metadata");
  writeSessionMetadata(s.file, config, s.sessionStartEpochMs, trackId, trackName);
  diagLog("writing channel table");
  writeChannelTable(s.file);
  diagLog("writing track definition");
  writeTrackDefinition(s.file, trackId, startLat, startLon, startAlt, startHeading,
                       trackPoints, numTrackPoints);

  s.dataOffset = s.file.position();
  s.file.flush();

  s.active = true;
  resetChunk(s, 0);

  diagLog("atpOpen SUCCESS");
  Serial.print("[ATP] Session started: ");
  Serial.println(filename);
  return true;
}

void atpPushImu(AtpSession &s, uint32_t bootMs,
                float ax_g, float ay_g, float az_g,
                float gx_dps, float gy_dps, float gz_dps,
                float tempC) {
  if (!s.active) return;

  // Auto-flush when chunk is full (100 samples = 1 second at 100 Hz)
  if (s.imuCount >= MAX_IMU_PER_CHUNK) {
    writeChunkToFile(s);
    uint32_t elapsed = bootMs - s.sessionStartMs;
    resetChunk(s, elapsed);
  }

  // Initialize chunk start time from first sample
  if (s.imuCount == 0 && s.chunkStartMs == 0 && s.chunkCount == 0) {
    s.chunkStartMs = bootMs - s.sessionStartMs;
  }

  // Encode IMU sample as 14 bytes of int16 fixed-point
  uint8_t *sample = s.imuBuf[s.imuCount];
  size_t pos = 0;
  bufWriteI16(sample, pos, encodeAccel(ax_g));
  bufWriteI16(sample, pos, encodeAccel(ay_g));
  bufWriteI16(sample, pos, encodeAccel(az_g));
  bufWriteI16(sample, pos, encodeGyro(gx_dps));
  bufWriteI16(sample, pos, encodeGyro(gy_dps));
  bufWriteI16(sample, pos, encodeGyro(gz_dps));
  bufWriteI16(sample, pos, encodeTemp(tempC));
  s.imuCount++;
}

void atpPushGps(AtpSession &s, uint32_t bootMs,
                double lat, double lon, float alt_m,
                float speed_mps, float course_deg,
                float hdop, uint8_t sats,
                float enu_x, float enu_y, float enu_z,
                float vel_n, float vel_e, float lapDist_m) {
  if (!s.active) return;

  uint32_t elapsed = bootMs - s.sessionStartMs;
  uint16_t offsetInChunk = (uint16_t)(elapsed - s.chunkStartMs);
  if (offsetInChunk > 999) offsetInChunk = 999;

  size_t pos = 0;
  bufWriteF64(s.gpsBuf, pos, lat);                              // 8
  bufWriteF64(s.gpsBuf, pos, lon);                              // 8
  bufWriteF32(s.gpsBuf, pos, alt_m);                            // 4
  bufWriteF32(s.gpsBuf, pos, speed_mps);                        // 4
  bufWriteF32(s.gpsBuf, pos, course_deg);                       // 4
  bufWriteU16(s.gpsBuf, pos, (uint16_t)(hdop * 100.0f));        // 2
  bufWriteU8(s.gpsBuf, pos, sats);                              // 1
  bufWriteF32(s.gpsBuf, pos, enu_x);                            // 4
  bufWriteF32(s.gpsBuf, pos, enu_y);                            // 4
  bufWriteF32(s.gpsBuf, pos, enu_z);                            // 4
  bufWriteF32(s.gpsBuf, pos, vel_n);                            // 4
  bufWriteF32(s.gpsBuf, pos, vel_e);                            // 4
  bufWriteF32(s.gpsBuf, pos, lapDist_m);                        // 4
  // Total: 55 bytes of payload, pad to 60 for alignment
  while (pos < GPS_RECORD_SIZE) s.gpsBuf[pos++] = 0;

  s.hasGps = true;
  s.gpsTimeOffsetMs = offsetInChunk;
  s.chunkFlags |= CHUNK_HAS_GPS;
}

void atpPushLapEvent(AtpSession &s, uint32_t bootMs,
                     uint16_t lapNumber, uint32_t lapTimeMs,
                     float lapDistanceM, uint16_t eventFlags) {
  if (!s.active) return;
  if (s.lapEventCount >= MAX_EVENTS_PER_CHUNK) return;

  uint32_t elapsed = bootMs - s.sessionStartMs;
  uint16_t offsetInChunk = (uint16_t)(elapsed - s.chunkStartMs);
  if (offsetInChunk > 999) offsetInChunk = 999;

  uint8_t *buf = s.lapBuf[s.lapEventCount];
  size_t pos = 0;
  bufWriteU16(buf, pos, offsetInChunk);               // time_offset_ms
  bufWriteU16(buf, pos, lapNumber);                    // lap_number
  bufWriteU32(buf, pos, lapTimeMs);                    // lap_time_ms
  bufWriteF32(buf, pos, lapDistanceM);                 // lap_distance_m
  bufWriteU16(buf, pos, eventFlags);                   // event_flags
  bufWriteU16(buf, pos, 0);                            // reserved
  // pos should be LAP_EVENT_SIZE (19) — pad remaining
  while (pos < LAP_EVENT_SIZE) buf[pos++] = 0;

  s.lapEventCount++;
  s.chunkFlags |= CHUNK_HAS_LAP_EVENT;
  s.currentLapNumber = lapNumber;

  // Update lap index
  if (lapTimeMs > 0 && s.lapCount < 256) {
    // Close current lap
    AtpSession::LapEntry &lap = s.laps[s.lapCount];
    lap.number = lapNumber;
    lap.startChunk = s.currentLapStartChunk;
    lap.endChunk = s.chunkCount;
    lap.timeMs = lapTimeMs;
    lap.flags = 0;
    lap.distanceM = lapDistanceM;
    s.lapCount++;
    s.currentLapStartChunk = s.chunkCount;
  }
}

void atpPushStateChange(AtpSession &s, uint32_t bootMs,
                        uint8_t oldState, uint8_t newState) {
  if (!s.active) return;
  if (s.stateChangeCount >= MAX_EVENTS_PER_CHUNK) return;

  uint32_t elapsed = bootMs - s.sessionStartMs;
  uint16_t offsetInChunk = (uint16_t)(elapsed - s.chunkStartMs);
  if (offsetInChunk > 999) offsetInChunk = 999;

  uint8_t *buf = s.stateBuf[s.stateChangeCount];
  size_t pos = 0;
  bufWriteU16(buf, pos, offsetInChunk);
  bufWriteU8(buf, pos, oldState);
  bufWriteU8(buf, pos, newState);
  bufWriteU8(buf, pos, 0);  // reserved
  while (pos < STATE_CHANGE_SIZE) buf[pos++] = 0;

  s.stateChangeCount++;
  s.chunkFlags |= CHUNK_HAS_STATE_CHANGE;
}

void atpFlushChunk(AtpSession &s, bool force) {
  if (!s.active) return;
  if (!force && s.imuCount < MAX_IMU_PER_CHUNK) return;
  if (s.imuCount == 0 && !s.hasGps && s.lapEventCount == 0 && s.stateChangeCount == 0) return;

  writeChunkToFile(s);
  uint32_t nextChunkMs = s.chunkStartMs + 1000;
  resetChunk(s, nextChunkMs);
}

void atpClose(AtpSession &s) {
  if (!s.active) return;

  // Flush any remaining data
  if (s.imuCount > 0 || s.hasGps || s.lapEventCount > 0 || s.stateChangeCount > 0) {
    writeChunkToFile(s);
  }

  uint32_t durationMs = millis() - s.sessionStartMs;

  // Write chunk index
  uint32_t chunkIndexOffset = s.file.position();
  writeChunkIndex(s);

  // Write lap index
  uint32_t lapIndexOffset = s.file.position();
  writeLapIndex(s);

  // Write footer
  writeFooter(s, chunkIndexOffset, lapIndexOffset, durationMs);

  // Update header: set SESSION_COMPLETE flag
  s.file.seek(6);  // flags field offset in file header
  writeU16(s.file, s.headerFlags | ATP_FLAG_SESSION_COMPLETE);

  s.file.close();
  s.active = false;

  // Free PSRAM
  free(s.chunkFileOffsets);
  free(s.chunkTimestamps);
  free(s.chunkSizes);
  s.chunkFileOffsets = nullptr;
  s.chunkTimestamps = nullptr;
  s.chunkSizes = nullptr;

  Serial.print("[ATP] Session closed — ");
  Serial.print(s.chunkCount);
  Serial.print(" chunks, ");
  Serial.print(s.lapCount);
  Serial.print(" laps, ");
  Serial.print(durationMs / 1000);
  Serial.println("s");
}

bool atpIsActive(const AtpSession &s) {
  return s.active;
}

bool atpReadMeta(const char *path, AtpFileMeta &meta) {
  memset(&meta, 0, sizeof(meta));
  File f = SD.open(path, "r");
  if (!f) return false;

  // Read and verify magic
  uint8_t magic[4];
  if (f.read(magic, 4) != 4 || memcmp(magic, ATP_MAGIC, 4) != 0) {
    f.close();
    return false;
  }

  uint16_t version = readU16(f);
  if (version != ATP_VERSION) { f.close(); return false; }

  uint16_t flags = readU16(f);
  /* uint32_t headerSize = */ readU32(f);
  /* uint32_t dataOffset = */ readU32(f);

  // Session metadata
  /* uint16_t metaSize = */ readU16(f);
  meta.source = f.read();
  meta.sessionType = f.read();
  meta.startTimeMs = readU64(f);
  /* uint64_t endTimeMs = */ readU64(f);
  /* uint32_t durationField = */ readU32(f);
  meta.trackId = readU32(f);
  readLString(f, meta.driverName, sizeof(meta.driverName));
  readLString(f, meta.trackName, sizeof(meta.trackName));
  readLString(f, meta.vehicleName, sizeof(meta.vehicleName));

  meta.complete = (flags & ATP_FLAG_SESSION_COMPLETE) != 0;

  // If session is complete, read footer for duration/laps
  if (flags & ATP_FLAG_SESSION_COMPLETE) {
    uint32_t fileSize = f.size();
    if (fileSize >= 32) {
      f.seek(fileSize - 32);
      uint8_t footerMagic[4];
      f.read(footerMagic, 4);
      if (memcmp(footerMagic, ATP_MAGIC, 4) == 0) {
        /* uint32_t chunkIdxOff = */ readU32(f);
        /* uint32_t lapIdxOff  = */ readU32(f);
        /* uint32_t totalChunks= */ readU32(f);
        meta.totalLaps = readU16(f);
        /* uint16_t reserved   = */ readU16(f);
        meta.durationMs = readU32(f);
      }
    }
  }

  f.close();
  meta.valid = true;
  return true;
}

bool atpScanChunks(const char *path, AtpScanResult &result) {
  memset(&result, 0, sizeof(result));
  File f = SD.open(path, "r");
  if (!f) return false;

  // Verify magic + version
  uint8_t magic[4];
  if (f.read(magic, 4) != 4 || memcmp(magic, ATP_MAGIC, 4) != 0) {
    f.close();
    return false;
  }
  uint16_t version = readU16(f);
  if (version != ATP_VERSION) { f.close(); return false; }

  /* uint16_t flags = */ readU16(f);
  /* uint32_t headerSize = */ readU32(f);
  uint32_t dataOffset = readU32(f);

  uint32_t fileSize = f.size();
  if (dataOffset >= fileSize) { f.close(); return false; }

  // Walk chunks from dataOffset
  f.seek(dataOffset);
  uint32_t pos = dataOffset;

  while (pos + 12 <= fileSize) {
    uint8_t chkMagic[4];
    if (f.read(chkMagic, 4) != 4) break;
    if (memcmp(chkMagic, CHK_MAGIC, 4) != 0) break;

    uint32_t chunkSize = readU32(f);
    uint32_t timestampMs = readU32(f);

    // Sanity check chunk size
    if (chunkSize < 16 || pos + chunkSize > fileSize) break;

    result.chunkCount++;
    result.lastTimestampMs = timestampMs;

    // Seek to next chunk
    pos += chunkSize;
    f.seek(pos);
  }

  f.close();
  return result.chunkCount > 0;
}

uint16_t atpReadLaps(const char *path, AtpLapInfo *laps, uint16_t maxLaps) {
  File f = SD.open(path, "r");
  if (!f) return 0;

  // Verify magic
  uint8_t magic[4];
  if (f.read(magic, 4) != 4 || memcmp(magic, ATP_MAGIC, 4) != 0) {
    f.close();
    return 0;
  }

  /* uint16_t version = */ readU16(f);
  uint16_t flags = readU16(f);

  if (!(flags & ATP_FLAG_SESSION_COMPLETE)) {
    f.close();
    return 0;
  }

  // Read footer (last 32 bytes) to find lap index offset
  uint32_t fileSize = f.size();
  if (fileSize < 32) { f.close(); return 0; }
  f.seek(fileSize - 32);

  uint8_t footerMagic[4];
  f.read(footerMagic, 4);
  if (memcmp(footerMagic, ATP_MAGIC, 4) != 0) { f.close(); return 0; }

  /* uint32_t chunkIdxOff = */ readU32(f);
  uint32_t lapIdxOff = readU32(f);

  // Seek to lap index
  f.seek(lapIdxOff);
  uint8_t lapMagic[4];
  f.read(lapMagic, 4);
  if (memcmp(lapMagic, LAP_MAGIC, 4) != 0) { f.close(); return 0; }

  uint16_t lapCount = readU16(f);
  uint16_t toRead = (lapCount < maxLaps) ? lapCount : maxLaps;

  // Detect old (16 bytes/lap) vs new (20 bytes/lap with distance) format
  // Lap data region: from current position to footer start (fileSize - 32)
  uint32_t lapDataStart = lapIdxOff + 6;  // after LAP magic(4) + count(2)
  uint32_t lapDataBytes = (fileSize - 32) - lapDataStart;
  bool hasDistance = (lapCount > 0) && (lapDataBytes / lapCount >= 20);

  for (uint16_t i = 0; i < toRead; ++i) {
    laps[i].number     = readU16(f);
    laps[i].startChunk = readU32(f);
    laps[i].endChunk   = readU32(f);
    laps[i].timeMs     = readU32(f);
    laps[i].flags      = readU16(f);
    if (hasDistance) {
      uint32_t distBits = readU32(f);
      memcpy(&laps[i].distanceM, &distBits, 4);
    } else {
      laps[i].distanceM = 0.0f;
    }
  }

  f.close();
  return toRead;
}
