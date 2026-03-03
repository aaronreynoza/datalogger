#pragma once

#include <stdint.h>

bool initPmu();
bool isPmuReady();
uint16_t pmuBattVoltageMv();
uint16_t pmuVbusVoltageMv();
uint16_t pmuSystemVoltageMv();
int pmuBatteryPercent();
bool pmuBatteryConnected();
bool pmuVbusPresent();
bool pmuIsCharging();
