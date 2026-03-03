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
void stopWiFi();
void restartWiFi();
void tickWiFiDNS();  // call from loop() to process captive portal DNS
bool isWifiSdBusy(); // true while HTTP handler is accessing SD over SPI
