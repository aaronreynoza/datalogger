#pragma once

#include <Arduino.h>

void initDisplay();
void displayBootStatus(const char *module, bool ok);
void displayBootDone();
void updateDisplay(uint32_t nowMs);
