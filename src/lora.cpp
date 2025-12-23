#include "lora.h"

#include <RadioLib.h>
#include <SPI.h>

// SX1262 pins for T-Beam S3 Supreme.
// NOTE: Double-check these pins with your board silkscreen/datasheet.
static const int LORA_SCK_PIN  = 5;
static const int LORA_MISO_PIN = 6;
static const int LORA_MOSI_PIN = 4;
static const int LORA_CS_PIN   = 7;  // NSS
static const int LORA_RST_PIN  = 8;
static const int LORA_BUSY_PIN = 9;
static const int LORA_DIO1_PIN = 3;

static const float    LORA_FREQ_MHZ         = 915.0f;  // set to 868.0 for EU, etc.
static const uint16_t LORA_BW_KHZ           = 125;
static const uint8_t  LORA_SF               = 9;
static const uint8_t  LORA_CR               = 5;       // 4/5
static const int8_t   LORA_TX_POWER_DBM     = 17;
static const uint32_t LORA_SEND_INTERVAL_MS = 1000;    // throttle RF duty cycle

static SPIClass loraSPI(FSPI);
static Module   loraModule(LORA_CS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN, loraSPI);
static SX1262   lora(&loraModule);

static bool     loraReady = false;
static uint32_t lastLoRaSendMs = 0;
static int      loraFailCounter = 0;

bool initLoRa() {
  Serial.println("Init SX1262 LoRa");

  loraSPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_CS_PIN);

  int state = lora.begin(LORA_FREQ_MHZ);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("LoRa begin failed, code ");
    Serial.println(state);
    return false;
  }

  lora.setSpreadingFactor(LORA_SF);
  lora.setBandwidth(LORA_BW_KHZ);
  lora.setCodingRate(LORA_CR);
  lora.setOutputPower(LORA_TX_POWER_DBM);
  lora.setDio2AsRfSwitch(true);

  Serial.print("LoRa ready @ ");
  Serial.print(LORA_FREQ_MHZ);
  Serial.println(" MHz");
  loraReady = true;
  return true;
}

void sendLoRaTelemetry(uint32_t epoch,
                       double lat, double lon,
                       double alt_m, double spd_kmph,
                       double hdop, uint32_t sats,
                       const ImuData &imu) {
  if (!loraReady) return;

  uint32_t now = millis();
  if (now - lastLoRaSendMs < LORA_SEND_INTERVAL_MS) return;

  String payload;
  payload.reserve(180);

  if (epoch) payload += String(epoch);
  payload += ",";
  payload += String(now);
  payload += ",";

  if (!isnan(lat)) payload += String(lat, 6);
  payload += ",";
  if (!isnan(lon)) payload += String(lon, 6);
  payload += ",";
  if (!isnan(alt_m)) payload += String(alt_m, 2);
  payload += ",";
  if (!isnan(spd_kmph)) payload += String(spd_kmph, 2);
  payload += ",";
  if (!isnan(hdop)) payload += String(hdop, 2);
  payload += ",";
  if (sats) payload += String(sats);
  payload += ",";

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

  int state = lora.transmit(payload);
  if (state != RADIOLIB_ERR_NONE) {
    if (loraFailCounter % 10 == 0) {
      Serial.print("LoRa TX failed, code ");
      Serial.println(state);
    }
    loraFailCounter++;
    return;
  }

  lastLoRaSendMs = now;
  loraFailCounter = 0;
}
