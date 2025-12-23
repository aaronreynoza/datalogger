#include "lora.h"

#if defined(LORA_ENABLE)
#include <RadioLib.h>

#if !defined(LORA_CS) || !defined(LORA_DIO1) || !defined(LORA_RST) || \
    !defined(LORA_BUSY) || !defined(LORA_SCK) || !defined(LORA_MISO) || \
    !defined(LORA_MOSI)
#error "LORA_* pins must be defined when LORA_ENABLE is set"
#endif

#if !defined(LORA_FREQUENCY_MHZ)
#define LORA_FREQUENCY_MHZ 915.0
#endif

#if !defined(LORA_TX_INTERVAL_MS)
#define LORA_TX_INTERVAL_MS 5000
#endif

static SPIClass loraSpi(HSPI);
static Module loraModule(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSpi);
static SX1262 lora(&loraModule);
static bool loraReady = false;
static uint32_t lastTxMs = 0;

void initLoRa() {
  Serial.println("[LORA] Initializing SX1262");
  loraSpi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  int state = lora.begin(LORA_FREQUENCY_MHZ);
  if (state == RADIOLIB_ERR_NONE) {
    loraReady = true;
    Serial.println("[LORA] Radio initialized");
  } else {
    loraReady = false;
    Serial.print("[LORA] Init failed, code=");
    Serial.println(state);
  }
}

bool loraIsReady() {
  return loraReady;
}

void sendLoRaStatus(uint32_t nowMs,
                    uint32_t epoch,
                    double lat,
                    double lon,
                    uint32_t sats) {
  if (!loraReady) {
    return;
  }
  if (nowMs - lastTxMs < LORA_TX_INTERVAL_MS) {
    return;
  }
  lastTxMs = nowMs;

  String payload;
  payload.reserve(96);
  payload += "t=";
  payload += String(epoch);
  payload += " lat=";
  if (isnan(lat) || isnan(lon)) {
    payload += "INVALID";
  } else {
    payload += String(lat, 6);
    payload += ",";
    payload += String(lon, 6);
  }
  payload += " sats=";
  payload += String(sats);

  int state = lora.transmit(payload);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("[LORA] TX failed, code=");
    Serial.println(state);
  } else {
    Serial.println("[LORA] TX ok");
  }
}

#else

void initLoRa() {}

bool loraIsReady() {
  return false;
}

void sendLoRaStatus(uint32_t, uint32_t, double, double, uint32_t) {}

#endif
