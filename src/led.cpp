#include "led.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <soc/soc_caps.h>
extern "C" void neopixelWrite(uint8_t pin, uint8_t r, uint8_t g, uint8_t b);
#endif

#if defined(LED_PIN)
static constexpr int LED_BUILTIN_PIN = LED_PIN;
#elif defined(LED_BUILTIN)
static constexpr int LED_BUILTIN_PIN = LED_BUILTIN;
#else
static constexpr int LED_BUILTIN_PIN = 21;
#endif

#if defined(LED_ACTIVE_LOW)
static constexpr bool LED_IS_ACTIVE_LOW = true;
#elif defined(LED_ACTIVE_HIGH)
static constexpr bool LED_IS_ACTIVE_LOW = false;
#else
static constexpr bool LED_IS_ACTIVE_LOW = false;
#endif

#if defined(LED_NEOPIXEL_GPIO)
static constexpr bool LED_IS_NEOPIXEL = true;
static constexpr int LED_NEOPIXEL_PIN = LED_NEOPIXEL_GPIO;
#elif defined(PIN_NEOPIXEL)
static constexpr bool LED_IS_NEOPIXEL = true;
static constexpr int LED_NEOPIXEL_PIN = PIN_NEOPIXEL;
#elif defined(SOC_GPIO_PIN_COUNT) && defined(LED_BUILTIN)
static constexpr bool LED_IS_NEOPIXEL = (LED_BUILTIN >= SOC_GPIO_PIN_COUNT);
static constexpr int LED_NEOPIXEL_PIN = LED_BUILTIN - SOC_GPIO_PIN_COUNT;
#else
static constexpr bool LED_IS_NEOPIXEL = false;
static constexpr int LED_NEOPIXEL_PIN = -1;
#endif

static constexpr uint8_t LED_NEOPIXEL_ON_R = 0;
static constexpr uint8_t LED_NEOPIXEL_ON_G = 0;
static constexpr uint8_t LED_NEOPIXEL_ON_B = 32;

void initLed() {
  if (LED_IS_NEOPIXEL) {
    neopixelWrite(LED_NEOPIXEL_PIN, 0, 0, 0);
    return;
  }
  pinMode(LED_BUILTIN_PIN, OUTPUT);
}

void updateLed(uint32_t nowMs) {
  static uint32_t lastLedToggleMs = 0;
  static bool ledOn = false;
  if (nowMs - lastLedToggleMs >= 500) {
    lastLedToggleMs = nowMs;
    ledOn = !ledOn;
    if (LED_IS_NEOPIXEL) {
      if (ledOn) {
        neopixelWrite(
            LED_NEOPIXEL_PIN,
            LED_NEOPIXEL_ON_R,
            LED_NEOPIXEL_ON_G,
            LED_NEOPIXEL_ON_B);
      } else {
        neopixelWrite(LED_NEOPIXEL_PIN, 0, 0, 0);
      }
      return;
    }
    if (LED_IS_ACTIVE_LOW) {
      digitalWrite(LED_BUILTIN_PIN, ledOn ? LOW : HIGH);
    } else {
      digitalWrite(LED_BUILTIN_PIN, ledOn ? HIGH : LOW);
    }
  }
}
