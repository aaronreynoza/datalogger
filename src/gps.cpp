#include "gps.h"

static const int GNSS_RX_PIN = 9;   // ESP32S3 receives on this
static const int GNSS_TX_PIN = 8;   // ESP32S3 transmits on this
static const uint32_t GNSS_BAUD_DEFAULT = 9600;
static const uint32_t GNSS_BAUD_FAST = 38400;

TinyGPSPlus gps;
HardwareSerial GNSS(1);

static uint32_t lastGpsDataMs = 0;
#if defined(GNSS_DEBUG)
static uint32_t lastGnssReportMs = 0;
static uint32_t gnssBytesSinceReport = 0;
#endif

// UBX checksum: CK_A/CK_B over class+id+length+payload
static void ubxChecksum(const uint8_t *buf, size_t len, uint8_t &ckA, uint8_t &ckB) {
  ckA = 0; ckB = 0;
  for (size_t i = 0; i < len; i++) {
    ckA += buf[i];
    ckB += ckA;
  }
}

static void sendUbx(const uint8_t *payload, size_t len) {
  uint8_t ckA, ckB;
  ubxChecksum(payload, len, ckA, ckB);
  GNSS.write(0xB5);  // sync 1
  GNSS.write(0x62);  // sync 2
  GNSS.write(payload, len);
  GNSS.write(ckA);
  GNSS.write(ckB);
  GNSS.flush();
}

void configureGps10Hz() {
  // Step 1: Set baud to 38400 via UBX-CFG-PRT (port 1 = UART1)
  // Class=0x06, ID=0x00, Len=20
  const uint8_t cfgPrt[] = {
    0x06, 0x00,             // class, id
    0x14, 0x00,             // length = 20
    0x01,                   // portID = 1 (UART1)
    0x00,                   // reserved
    0x00, 0x00,             // txReady
    0xD0, 0x08, 0x00, 0x00, // mode: 8N1
    0x00, 0x96, 0x00, 0x00, // baudRate: 38400
    0x07, 0x00,             // inProtoMask: UBX+NMEA
    0x03, 0x00,             // outProtoMask: UBX+NMEA
    0x00, 0x00,             // flags
    0x00, 0x00,             // reserved
  };
  sendUbx(cfgPrt, sizeof(cfgPrt));
  delay(100);

  // Step 2: Reinitialize UART at new baud
  GNSS.end();
  delay(50);
  GNSS.begin(GNSS_BAUD_FAST, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
  delay(100);

  // Step 3: Set measurement rate to 100ms (10 Hz) via UBX-CFG-RATE
  // Class=0x06, ID=0x08, Len=6
  const uint8_t cfgRate[] = {
    0x06, 0x08,             // class, id
    0x06, 0x00,             // length = 6
    0x64, 0x00,             // measRate: 100 ms (10 Hz)
    0x01, 0x00,             // navRate: 1
    0x01, 0x00,             // timeRef: GPS time
  };
  sendUbx(cfgRate, sizeof(cfgRate));
  delay(100);

  Serial.println("GPS configured: 38400 baud, 10 Hz");
}

void initGps() {
  GNSS.begin(GNSS_BAUD_DEFAULT, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
  Serial.println("GNSS serial started at 9600, configuring 10 Hz...");
  delay(500);  // Wait for module to be ready
  configureGps10Hz();
  lastGpsDataMs = millis();
}

void pollGps() {
  bool gotData = false;
  while (GNSS.available() > 0) {
    gps.encode(GNSS.read());
    gotData = true;
#if defined(GNSS_DEBUG)
    gnssBytesSinceReport++;
#endif
  }
  if (gotData) {
    lastGpsDataMs = millis();
  } else if (millis() - lastGpsDataMs > 5000) {
    Serial.println("GNSS idle, restarting serial...");
    GNSS.end();
    delay(20);
    GNSS.begin(GNSS_BAUD_FAST, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
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
