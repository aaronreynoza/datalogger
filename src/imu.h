#pragma once

#include <Arduino.h>
#include "SensorQMI8658.hpp"

struct ImuData {
  IMUdata acc;
  IMUdata gyr;
  bool hasData = false;
  float tempC = NAN;
  uint32_t timestamp = 0;
};

extern ImuData imuData;

void initImu();
void updateImu();
