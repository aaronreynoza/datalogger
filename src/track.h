#pragma once

#include <Arduino.h>

static constexpr uint8_t TRACK_STATE_IDLE      = 0;
static constexpr uint8_t TRACK_STATE_RECORDING  = 1;
static constexpr uint8_t TRACK_STATE_READY      = 2;
static constexpr uint8_t TRACK_STATE_RACING     = 3;

static constexpr uint8_t TRACK_EVENT_LAP_START   = 1 << 0;
static constexpr uint8_t TRACK_EVENT_LAP_END     = 1 << 1;
static constexpr uint8_t TRACK_EVENT_RECOGNIZED  = 1 << 2;

struct TrackUiState {
  uint8_t trackState;
  bool    lapActive;
  bool    startPending;
  uint32_t lapCount;
  uint32_t currentLapMs;
  uint32_t lastLapMs;
  uint32_t bestLapMs;
  uint32_t trackId;
  char     trackName[32];
};

struct SavedTrackInfo {
  double   lat, lon, alt, courseDeg;
  uint32_t id;
  char     name[32];
};

void initTrack();
void pollTrackButton(uint32_t nowMs);
void updateTrack(uint32_t nowMs,
                 bool gpsUpdated,
                 bool gpsValid,
                 bool courseValid,
                 double lat,
                 double lon,
                 double alt_m,
                 double spd_kmph,
                 double course_deg,
                 double hdop,
                 uint32_t sats,
                 uint32_t epoch);
TrackUiState getTrackUiState();
uint8_t getTrackEventFlags();
bool getTrackReference(double &lat, double &lon, double &alt,
                       double &courseDeg, uint32_t &trackId);
void resetTrack();
int getSavedTrackCount();
bool getSavedTrack(int index, SavedTrackInfo &info);
bool setTrackName(uint32_t trackId, const char *name);
bool addTrackFromApi(double lat, double lon, double alt,
                     double courseDeg, const char *name,
                     uint32_t &outTrackId);
bool deleteTrack(uint32_t trackId);
int countTrackRecordingPoints(uint32_t trackId);
