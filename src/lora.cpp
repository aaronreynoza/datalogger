#include "lora.h"

#include <RadioLib.h>
#include <SPI.h>

#include "pmu.h"

// SX1262 pins for T-Beam S3 Supreme
static constexpr int LORA_SCK_PIN  = 12;
static constexpr int LORA_MISO_PIN = 13;
static constexpr int LORA_MOSI_PIN = 11;
static constexpr int LORA_CS_PIN   = 10;
static constexpr int LORA_RST_PIN  = 5;
static constexpr int LORA_BUSY_PIN = 4;
static constexpr int LORA_DIO1_PIN = 1;

static constexpr float    LORA_FREQ_MHZ         = 915.0f;
static constexpr uint16_t LORA_BW_KHZ           = 125;
static constexpr uint8_t  LORA_SF               = 9;
static constexpr uint8_t  LORA_CR               = 5;
static constexpr int8_t   LORA_TX_POWER_DBM     = 17;
static constexpr uint32_t LORA_SEND_INTERVAL_MS = 1000;
static const char        *LORA_DEVICE_ID        = "tbeam01";

static SPIClass loraSPI(FSPI);
static Module   loraModule(LORA_CS_PIN, LORA_DIO1_PIN, LORA_RST_PIN,
                           LORA_BUSY_PIN, loraSPI);
static SX1262   lora(&loraModule);

static bool     loraReady       = false;
static uint32_t lastLoRaSendMs  = 0;
static int      loraFailCounter = 0;

bool initLoRa() {
  Serial.println("[LoRa] Initializing SX1262");

  loraSPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_CS_PIN);

  int state = lora.begin(LORA_FREQ_MHZ);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("[LoRa] begin failed, code ");
    Serial.println(state);
    return false;
  }

  lora.setSpreadingFactor(LORA_SF);
  lora.setBandwidth(LORA_BW_KHZ);
  lora.setCodingRate(LORA_CR);
  lora.setOutputPower(LORA_TX_POWER_DBM);
  lora.setDio2AsRfSwitch(true);

  loraReady = true;
  Serial.print("[LoRa] Ready @ ");
  Serial.print(LORA_FREQ_MHZ);
  Serial.println(" MHz");
  return true;
}

bool isLoRaReady() {
  return loraReady;
}

void sendLoRaTelemetry(uint32_t epoch,
                       double lat, double lon,
                       double alt_m, double spd_kmph,
                       double hdop, uint32_t sats,
                       const ImuData &imu,
                       uint8_t trackState,
                       uint32_t lapCount,
                       uint32_t currentLapMs,
                       uint32_t lastLapMs,
                       uint32_t bestLapMs) {
  if (!loraReady) return;

  uint32_t now = millis();
  if (now - lastLoRaSendMs < LORA_SEND_INTERVAL_MS) return;

  String payload;
  payload.reserve(256);

  // Device ID + timestamps
  payload += LORA_DEVICE_ID;      payload += ",";
  if (epoch) payload += String(epoch);
  payload += ",";
  payload += String(now);         payload += ",";

  // GPS
  if (!isnan(lat))      payload += String(lat, 6);      payload += ",";
  if (!isnan(lon))      payload += String(lon, 6);      payload += ",";
  if (!isnan(alt_m))    payload += String(alt_m, 2);    payload += ",";
  if (!isnan(spd_kmph)) payload += String(spd_kmph, 2); payload += ",";
  if (!isnan(hdop))     payload += String(hdop, 2);     payload += ",";
  if (sats) payload += String(sats);
  payload += ",";

  // IMU
  if (imu.hasData) {
    payload += String(imu.acc.x, 3); payload += ",";
    payload += String(imu.acc.y, 3); payload += ",";
    payload += String(imu.acc.z, 3); payload += ",";
    payload += String(imu.gyr.x, 3); payload += ",";
    payload += String(imu.gyr.y, 3); payload += ",";
    payload += String(imu.gyr.z, 3); payload += ",";
  } else {
    payload += ",,,,,,";
  }
  if (!isnan(imu.tempC)) payload += String(imu.tempC, 2);
  payload += ",";

  // PMU
  uint16_t vbatMv = pmuBattVoltageMv();
  if (vbatMv) payload += String(vbatMv);
  payload += ",";
  uint16_t vbusMv = pmuVbusVoltageMv();
  if (vbusMv) payload += String(vbusMv);
  payload += ",";
  uint16_t vsysMv = pmuSystemVoltageMv();
  if (vsysMv) payload += String(vsysMv);
  payload += ",";
  int battPct = pmuBatteryPercent();
  if (battPct >= 0) payload += String(battPct);
  payload += ",";
  payload += (pmuBatteryConnected() ? "1" : "0"); payload += ",";
  payload += (pmuVbusPresent()      ? "1" : "0"); payload += ",";
  payload += (pmuIsCharging()       ? "1" : "0"); payload += ",";

  // Track / lap data (new fields, appended for backward compatibility)
  payload += String(trackState);      payload += ",";
  payload += String(lapCount);        payload += ",";
  payload += String(currentLapMs);    payload += ",";
  payload += String(lastLapMs);       payload += ",";
  payload += String(bestLapMs);

  int state = lora.transmit(payload);
  if (state != RADIOLIB_ERR_NONE) {
    if (loraFailCounter % 10 == 0) {
      Serial.print("[LoRa] TX failed, code ");
      Serial.println(state);
    }
    loraFailCounter++;
    return;
  }

  lastLoRaSendMs = now;
  loraFailCounter = 0;
}
