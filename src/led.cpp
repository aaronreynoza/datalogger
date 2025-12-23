#include "led.h"

#ifndef LED_BUILTIN
static constexpr int LED_BUILTIN_PIN = 21;
#else
static constexpr int LED_BUILTIN_PIN = LED_BUILTIN;
#endif

void initLed() {
  pinMode(LED_BUILTIN_PIN, OUTPUT);
}

void updateLed(uint32_t nowMs) {
  static uint32_t lastLedToggleMs = 0;
  static bool ledOn = false;
  if (nowMs - lastLedToggleMs >= 500) {
    lastLedToggleMs = nowMs;
    ledOn = !ledOn;
    digitalWrite(LED_BUILTIN_PIN, ledOn ? HIGH : LOW);
  }
}
