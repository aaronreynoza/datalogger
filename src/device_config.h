#pragma once

#include <Arduino.h>

static constexpr const char *FIRMWARE_VERSION = "2.0.0";
static constexpr const char *HARDWARE_ID = "tbeam-supreme";
static constexpr uint8_t ATP_SOURCE_CORE_PRO = 1;

struct WeatherConfig {
  float ambientTempC;    // 0 if not set
  float trackTempC;      // 0 if not set
  char conditions[8];    // "dry", "damp", "wet", "mixed"
  bool set;              // true if weather was ever pushed
};

struct DeviceConfig {
  char serial[16];
  char driverName[64];
  char vehicleName[64];
  uint8_t sessionType;     // 0=unknown, 1=practice, 2=qualifying, 3=race, 4=test
  char sessionNotes[256];  // free-text notes
  WeatherConfig weather;   // weather conditions (set=false if never pushed)
};

bool loadConfig();
void saveConfig();
const DeviceConfig &getConfig();
void setDriverName(const char *name);
void setVehicleName(const char *name);
void setSessionType(uint8_t type);
void setSessionNotes(const char *notes);
void setWeather(float ambientC, float trackC, const char *conditions);
void clearWeather();
uint8_t sessionTypeFromString(const char *str);
