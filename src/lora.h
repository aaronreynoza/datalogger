#pragma once

#include <Arduino.h>

void initLoRa();
void sendLoRaStatus(uint32_t nowMs,
                    uint32_t epoch,
                    double lat,
                    double lon,
                    uint32_t sats);
bool loraIsReady();
