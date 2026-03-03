#pragma once

#include <Arduino.h>
#include <SPI.h>
#include "SensorQMI8658.hpp"

// Shared HSPI bus for IMU + SD card (same physical pins)
SPIClass &sharedHSPI();

struct ImuData {
  IMUdata acc;
  IMUdata gyr;
  bool hasData = false;
  float tempC = NAN;
  uint32_t timestamp = 0;
};

extern ImuData imuData;

bool initImu();
bool isImuReady();
void updateImu();
