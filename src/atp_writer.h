#pragma once

#include <Arduino.h>
#include <SD.h>

#include "device_config.h"

// --- ATP format constants ---
static constexpr uint8_t ATP_MAGIC[4] = {'A', 'T', 'P', '\0'};
static constexpr uint8_t CHK_MAGIC[4] = {'C', 'H', 'K', '\0'};
static constexpr uint8_t IDX_MAGIC[4] = {'I', 'D', 'X', '\0'};
static constexpr uint8_t LAP_MAGIC[4] = {'L', 'A', 'P', '\0'};

static constexpr uint16_t ATP_VERSION = 1;

// File header flags
static constexpr uint16_t ATP_FLAG_HAS_TRACK_RECORDING = 1 << 0;
static constexpr uint16_t ATP_FLAG_SESSION_COMPLETE     = 1 << 3;

// Record types
static constexpr uint8_t REC_IMU_BATCH    = 0x01;
static constexpr uint8_t REC_GPS_FIX      = 0x02;
static constexpr uint8_t REC_LAP_EVENT    = 0x03;
static constexpr uint8_t REC_STATE_CHANGE = 0x04;
static constexpr uint8_t REC_CAN_FRAME    = 0x05;

// Chunk flags
static constexpr uint8_t CHUNK_HAS_GPS          = 1 << 0;
static constexpr uint8_t CHUNK_HAS_LAP_EVENT    = 1 << 1;
static constexpr uint8_t CHUNK_HAS_STATE_CHANGE = 1 << 2;

// Fixed-point scales
static constexpr float ACCEL_SCALE = 8192.0f;   // ±4g → int16
static constexpr float GYRO_SCALE  = 512.0f;    // ±64°/s → int16
static constexpr float TEMP_SCALE  = 100.0f;    // °C → int16

static constexpr uint8_t IMU_SAMPLE_SIZE = 14;  // 7 × int16
static constexpr uint8_t MAX_IMU_PER_CHUNK = 100;
static constexpr uint8_t GPS_RECORD_SIZE = 60;
static constexpr uint8_t LAP_EVENT_SIZE = 19;
static constexpr uint8_t STATE_CHANGE_SIZE = 8;
static constexpr uint8_t MAX_EVENTS_PER_CHUNK = 4;

// Track outline point for embedding in ATP
struct AtpTrackPoint {
  double lat, lon;
  float alt, speedKmh;
};

// --- ATP session handle ---
struct AtpSession {
  File     file;
  bool     active;
  char     filename[32];  // e.g. "/race-20260301-1838.atp"
  uint32_t sessionStartMs;
  uint64_t sessionStartEpochMs;
  uint32_t dataOffset;
  uint16_t headerFlags;

  // Current chunk being accumulated
  uint8_t  imuBuf[MAX_IMU_PER_CHUNK][IMU_SAMPLE_SIZE];
  uint8_t  imuCount;
  uint8_t  gpsBuf[GPS_RECORD_SIZE];
  bool     hasGps;
  uint16_t gpsTimeOffsetMs;
  uint8_t  lapBuf[MAX_EVENTS_PER_CHUNK][LAP_EVENT_SIZE];
  uint8_t  lapEventCount;
  uint8_t  stateBuf[MAX_EVENTS_PER_CHUNK][STATE_CHANGE_SIZE];
  uint8_t  stateChangeCount;
  uint16_t currentLapNumber;
  uint8_t  chunkFlags;
  uint32_t chunkStartMs;  // ms since session start for current chunk

  // Index tables (PSRAM-allocated)
  uint32_t *chunkFileOffsets;
  uint32_t *chunkTimestamps;
  uint16_t *chunkSizes;
  uint32_t  chunkCount;
  uint32_t  chunkCapacity;

  // Lap index
  struct LapEntry {
    uint16_t number;
    uint32_t startChunk;
    uint32_t endChunk;
    uint32_t timeMs;
    uint16_t flags;
    float    distanceM;
  };
  LapEntry laps[256];
  uint16_t lapCount;
  uint32_t currentLapStartChunk;

  uint32_t totalImuSamples;
};

// --- Public API ---
bool atpOpen(AtpSession &s, uint32_t epochS, uint32_t bootMs,
             uint32_t trackId, const DeviceConfig &config,
             double startLat, double startLon, double startAlt,
             double startHeading, const char *trackName,
             const AtpTrackPoint *trackPoints, uint16_t numTrackPoints);

void atpPushImu(AtpSession &s, uint32_t bootMs,
                float ax_g, float ay_g, float az_g,
                float gx_dps, float gy_dps, float gz_dps,
                float tempC);

void atpPushGps(AtpSession &s, uint32_t bootMs,
                double lat, double lon, float alt_m,
                float speed_mps, float course_deg,
                float hdop, uint8_t sats,
                float enu_x, float enu_y, float enu_z,
                float vel_n, float vel_e, float lapDist_m);

void atpPushLapEvent(AtpSession &s, uint32_t bootMs,
                     uint16_t lapNumber, uint32_t lapTimeMs,
                     float lapDistanceM, uint16_t eventFlags);

void atpPushStateChange(AtpSession &s, uint32_t bootMs,
                        uint8_t oldState, uint8_t newState);

void atpFlushChunk(AtpSession &s, bool force);

void atpClose(AtpSession &s);

bool atpIsActive(const AtpSession &s);

// Read metadata from an existing .atp file (for WiFi API session listing)
struct AtpFileMeta {
  bool     valid;
  bool     complete;  // SESSION_COMPLETE flag set (cleanly closed)
  uint32_t trackId;
  uint32_t durationMs;
  uint16_t totalLaps;
  uint8_t  source;
  uint8_t  sessionType;
  char     driverName[64];
  char     vehicleName[64];
  char     trackName[32];
  uint64_t startTimeMs;
  uint32_t fileSize;   // total file size in bytes
};
bool atpReadMeta(const char *path, AtpFileMeta &meta);

// Scan data section of incomplete (or complete) sessions
// Returns estimated duration and chunk count by walking chunk headers
struct AtpScanResult {
  uint32_t chunkCount;
  uint32_t lastTimestampMs;  // last chunk's timestamp = estimated duration
};
bool atpScanChunks(const char *path, AtpScanResult &result);

// Lap entry returned by atpReadLaps
struct AtpLapInfo {
  uint16_t number;
  uint32_t startChunk;
  uint32_t endChunk;
  uint32_t timeMs;
  uint16_t flags;
  float    distanceM;
};

// Read lap table from a completed .atp file (max maxLaps entries)
// Returns number of laps read, 0 on error
uint16_t atpReadLaps(const char *path, AtpLapInfo *laps, uint16_t maxLaps);
