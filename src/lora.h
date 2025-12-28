#pragma once

#include <Arduino.h>

#include "imu.h"

bool initLoRa();
bool isLoRaReady();
void sendLoRaTelemetry(uint32_t epoch,
                       double lat, double lon,
                       double alt_m, double spd_kmph,
                       double hdop, uint32_t sats,
                       const ImuData &imu);
