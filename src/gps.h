#pragma once

#include <Arduino.h>
#include <TinyGPSPlus.h>

extern TinyGPSPlus gps;
extern HardwareSerial GNSS;

void initGps();
void pollGps();
uint32_t gpsUnixTime();
