#include "gps.h"

static const int GNSS_RX_PIN = 9;   // ESP32S3 receives on this
static const int GNSS_TX_PIN = 8;   // ESP32S3 transmits on this

TinyGPSPlus gps;
HardwareSerial GNSS(1);

static uint32_t lastGpsDataMs = 0;

void initGps() {
  GNSS.begin(9600, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
  Serial.println("GNSS serial started, waiting for fix...");
  lastGpsDataMs = millis();
}

void pollGps() {
  bool gotData = false;
  while (GNSS.available() > 0) {
    gps.encode(GNSS.read());
    gotData = true;
  }
  if (gotData) {
    lastGpsDataMs = millis();
  } else if (millis() - lastGpsDataMs > 5000) {
    Serial.println("GNSS idle, restarting serial...");
    GNSS.end();
    delay(20);
    GNSS.begin(9600, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
    lastGpsDataMs = millis();
  }
}

uint32_t gpsUnixTime() {
  if (!gps.date.isValid() || !gps.time.isValid()) {
    return 0;
  }

  const int year  = gps.date.year();
  const int month = gps.date.month();
  const int day   = gps.date.day();
  const int hour  = gps.time.hour();
  const int min   = gps.time.minute();
  const int sec   = gps.time.second();

  if (year < 1970 || month < 1 || month > 12 || day < 1 || day > 31) {
    return 0;
  }

  auto isLeap = [](int y) {
    return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
  };

  static const uint8_t daysInMonth[] = {
      31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

  uint32_t days = 0;
  for (int y = 1970; y < year; ++y) {
    days += isLeap(y) ? 366 : 365;
  }
  for (int m = 1; m < month; ++m) {
    days += daysInMonth[m - 1];
    if (m == 2 && isLeap(year)) {
      days += 1;
    }
  }
  days += static_cast<uint32_t>(day - 1);

  return days * 86400UL + hour * 3600UL + min * 60UL + sec;
}
