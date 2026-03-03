#pragma once

#include <Arduino.h>
#include <FS.h>

extern bool storageReady;
extern bool loggingEnabled;
extern String trackLogPath;

bool initStorage();

bool startTrackLog(uint32_t trackId);
void stopTrackLog();
void appendTrackLog(uint32_t epoch,
                    double lat, double lon, double alt_m,
                    double spd_kmph, double course_deg,
                    double hdop, uint32_t sats);
void flushTrackLog();

void dumpFile(const String &path);
void clearLogs();
void listLogsTo(Print &out);

// Persistent diagnostic log (survives power cycles)
void diagLog(const char *msg);
void diagLogf(const char *fmt, ...);
void flushDiagLog();
void tickDiagLog();
String readDiagLog();
