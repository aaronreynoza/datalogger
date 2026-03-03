#pragma once

#include <stdint.h>

struct WifiGpsStatus {
  bool  fix;
  uint32_t sats;
  float hdop;
  float speedKmh;
};

void updateWifiGpsStatus(bool fix, uint32_t sats, float hdop, float speedKmh);

void setupWiFi();
