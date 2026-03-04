#pragma once

#include <Arduino.h>

#include "imu.h"

struct GpsFix {
  bool valid;
  bool good;
  bool degraded;
  bool courseValid;
  uint32_t ms;
  uint32_t epoch;
  double lat;
  double lon;
  double alt_m;
  double spd_kmh;
  double course_deg;
  double hdop;
  uint32_t sats;
  double x_m;
  double y_m;
  double z_m;
  double vn_mps;
  double ve_mps;
};

struct ImuSample {
  uint32_t ms;
  ImuData imu;
  uint32_t lapCount;
  uint32_t lapMs;
  uint32_t lastLapMs;
  uint8_t trackState;
  bool lapActive;
  uint16_t eventFlags;
};

static constexpr uint16_t RACE_EVENT_GPS_GAP = 1 << 3;
static constexpr uint16_t RACE_EVENT_GPS_DEGRADED = 1 << 4;
static constexpr uint16_t RACE_EVENT_GPS_INVALID = 1 << 5;

void initRaceLogger();
void setRaceReference(double lat, double lon, double alt, uint32_t trackId);
bool hasRaceReference();
uint32_t getRaceTrackId();
bool startRaceLogger(uint32_t epoch, uint32_t nowMs,
                     const char *trackName);
bool isRaceLoggerActive();
void stopRaceLogger();
void pushImuSample(const ImuSample &sample);
void onGpsFix(const GpsFix &fix);
void flushRaceLogger(uint32_t nowMs);
const char *getRaceSessionFilename();
void getRaceGpsStats(uint32_t &received, uint32_t &written);
