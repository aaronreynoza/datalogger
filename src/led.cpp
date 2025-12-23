#include "led.h"

#ifdef LED_PIN
static constexpr int LED_BUILTIN_PIN = LED_PIN;
static constexpr bool LED_ACTIVE_LOW = false;
#elif defined(LED_BUILTIN)
static constexpr int LED_BUILTIN_PIN = LED_BUILTIN;
static constexpr bool LED_ACTIVE_LOW = false;
#else
static constexpr int LED_BUILTIN_PIN = 21;
static constexpr bool LED_ACTIVE_LOW = true;
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
    if (LED_ACTIVE_LOW) {
      digitalWrite(LED_BUILTIN_PIN, ledOn ? LOW : HIGH);
    } else {
      digitalWrite(LED_BUILTIN_PIN, ledOn ? HIGH : LOW);
    }
  }
}
