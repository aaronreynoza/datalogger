#include "display.h"

#include <Wire.h>
#include <U8g2lib.h>

#include "gps.h"
#include "imu.h"
#include "logging.h"

static constexpr int I2C_SDA = 17;
static constexpr int I2C_SCL = 18;

static U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

static String gpsStatusLine() {
  if (!gps.location.isValid()) {
    String s = "GPS: NO FIX  sats:";
    s += gps.satellites.isValid() ? String(gps.satellites.value()) : "?";
    return s;
  }
  String s = "GPS: ";
  s += String(gps.location.lat(), 6);
  s += ",";
  s += String(gps.location.lng(), 6);
  s += " sats:";
  s += gps.satellites.isValid() ? String(gps.satellites.value()) : "?";
  return s;
}

static String imuStatusLine() {
  String s = "IMU T: ";
  if (!isnan(imuData.tempC)) {
    s += String(imuData.tempC, 1);
  } else {
    s += "N/A";
  }
  s += "C ";
  s += (loggingEnabled ? "LOG:ON" : "LOG:OFF");
  return s;
}

void initDisplay() {
  Wire.begin(I2C_SDA, I2C_SCL);
  u8g2.begin();
  u8g2.setI2CAddress(0x3C << 1);
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "T-Beam Logger");
  u8g2.drawStr(0, 26, "Display ready");
  u8g2.sendBuffer();
}

void updateDisplay(uint32_t nowMs) {
  static uint32_t lastUiMs = 0;
  if (nowMs - lastUiMs < 500) {
    return;
  }
  lastUiMs = nowMs;

  String l1 = gpsStatusLine();
  String l2 = imuStatusLine();

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, l1.c_str());
  u8g2.drawStr(0, 26, l2.c_str());
  u8g2.sendBuffer();
}
