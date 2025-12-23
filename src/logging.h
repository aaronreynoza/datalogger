#pragma once

#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>

extern bool storageReady;
extern bool loggingEnabled;
extern String gpsLogPath;
extern String imuLogPath;

bool initStorage();
bool createSessionLogs();
void stopLogging();
void appendGpsLog(uint32_t epoch,
                  double lat, double lon,
                  double alt_m, double spd_kmph,
                  double hdop, uint32_t sats);
void appendImuLog(uint32_t epoch);
void dumpFile(const String &path);
void clearLogs();
void listLogsTo(Print &out);
