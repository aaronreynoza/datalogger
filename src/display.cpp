#include "display.h"

#include <Wire.h>
#include <U8g2lib.h>

#include "gps.h"
#include "imu.h"
#include "pmu.h"
#include "logging.h"
#include "track.h"

static constexpr int I2C_SDA = 17;
static constexpr int I2C_SCL = 18;

static U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// Boot screen state
static constexpr int MAX_BOOT_ITEMS = 6;
struct BootItem {
  const char *label;
  bool ok;
};
static BootItem bootItems[MAX_BOOT_ITEMS];
static int bootItemCount = 0;

// ===================== Helpers =====================

static void formatLapTime(uint32_t ms, char *buf, size_t len) {
  uint32_t minutes = ms / 60000;
  uint32_t seconds = (ms % 60000) / 1000;
  uint32_t millis  = ms % 1000;
  snprintf(buf, len, "%u:%02lu.%03lu",
           static_cast<unsigned>(minutes),
           static_cast<unsigned long>(seconds),
           static_cast<unsigned long>(millis));
}

static int battPctFromVoltage(uint16_t mv) {
  if (mv >= 4200) return 100;
  if (mv <= 3300) return 0;
  return static_cast<int>((mv - 3300) * 100L / 900);
}

static void drawStatusBar(int y) {
  char buf[28];
  uint16_t battMv = pmuBattVoltageMv();
  int pct = battPctFromVoltage(battMv);
  uint32_t sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
  snprintf(buf, sizeof(buf), "%lusat %s %u.%02uV %d%%",
           static_cast<unsigned long>(sats),
           loggingEnabled ? "LOG" : "---",
           battMv / 1000, (battMv % 1000) / 10,
           pct);
  u8g2.drawStr(0, y, buf);
}

// ===================== Boot screen =====================

void initDisplay() {
  Wire.begin(I2C_SDA, I2C_SCL);
  u8g2.begin();
  u8g2.setI2CAddress(0x3C << 1);
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "T-BEAM LOGGER");
  u8g2.drawStr(0, 26, "Initializing...");
  u8g2.sendBuffer();
  bootItemCount = 0;
}

void displayBootStatus(const char *module, bool ok) {
  if (bootItemCount < MAX_BOOT_ITEMS) {
    bootItems[bootItemCount].label = module;
    bootItems[bootItemCount].ok = ok;
    bootItemCount++;
  }

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "T-BEAM LOGGER");

  // Draw boot items two per line, starting at y=26
  for (int i = 0; i < bootItemCount; i += 2) {
    int y = 26 + (i / 2) * 12;
    if (y > 60) break;

    char line[22];
    const char *s1 = bootItems[i].ok ? "OK" : "FAIL";
    if (i + 1 < bootItemCount) {
      const char *s2 = bootItems[i + 1].ok ? "OK" : "FAIL";
      snprintf(line, sizeof(line), "%-4s%-4s %-4s%s",
               bootItems[i].label, s1,
               bootItems[i + 1].label, s2);
    } else {
      snprintf(line, sizeof(line), "%-4s%s",
               bootItems[i].label, s1);
    }
    u8g2.drawStr(0, y, line);
  }
  u8g2.sendBuffer();
}

void displayBootDone() {
  delay(3000);
}

// ===================== Main display update =====================

void updateDisplay(uint32_t nowMs) {
  static uint32_t lastUiMs = 0;
  TrackUiState ts = getTrackUiState();
  // Faster refresh when recording (for blink) or racing (for timer)
  uint32_t interval = (ts.trackState == TRACK_STATE_RECORDING ||
                       ts.startPending ||
                       ts.trackState == TRACK_STATE_RACING) ? 200 : 500;
  if (nowMs - lastUiMs < interval) return;
  lastUiMs = nowMs;

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);

  // --- Recording (blink header to show activity) ---
  if (ts.trackState == TRACK_STATE_RECORDING || ts.startPending) {
    bool blink = (nowMs / 200) % 2 == 0;
    if (blink) {
      // Inverted header: white bar with black text
      u8g2.setDrawColor(1);
      u8g2.drawBox(0, 0, 128, 14);
      u8g2.setDrawColor(0);
      u8g2.drawStr(1, 12, " * RECORDING TRACK *");
      u8g2.setDrawColor(1);
    } else {
      u8g2.drawStr(0, 12, "  RECORDING TRACK");
    }
    if (ts.startPending) {
      u8g2.drawStr(0, 28, "Waiting for GPS...");
    } else if (ts.trackId == 0) {
      u8g2.drawStr(0, 28, "Start riding to");
      u8g2.drawStr(0, 40, "lock start point");
    } else {
      u8g2.drawStr(0, 28, "Ride a full lap");
      u8g2.drawStr(0, 40, "to set start/finish");
    }
    drawStatusBar(60);
    u8g2.sendBuffer();
    return;
  }

  // --- Racing (inverted header bar) ---
  if (ts.trackState == TRACK_STATE_RACING) {
    char buf[22];

    // Line 1: LAP count + speed — inverted bar
    double spd = gps.speed.isValid() ? gps.speed.kmph() : 0;
    snprintf(buf, sizeof(buf), " LAP %lu    %dkm/h",
             static_cast<unsigned long>(ts.lapCount),
             static_cast<int>(spd));
    u8g2.setDrawColor(1);
    u8g2.drawBox(0, 0, 128, 14);
    u8g2.setDrawColor(0);
    u8g2.drawStr(0, 12, buf);
    u8g2.setDrawColor(1);

    // Line 2: Current lap time (large-ish)
    char timeBuf[16];
    formatLapTime(ts.currentLapMs, timeBuf, sizeof(timeBuf));
    u8g2.setFont(u8g2_font_profont22_mn);
    // Center the time string
    int w = u8g2.getStrWidth(timeBuf);
    u8g2.drawStr((128 - w) / 2, 34, timeBuf);
    u8g2.setFont(u8g2_font_6x12_tf);

    // Line 3: Last lap
    if (ts.lastLapMs > 0) {
      formatLapTime(ts.lastLapMs, timeBuf, sizeof(timeBuf));
      snprintf(buf, sizeof(buf), "LAST %s", timeBuf);
      u8g2.drawStr(0, 48, buf);
    }

    // Line 4: Best lap
    if (ts.bestLapMs > 0) {
      formatLapTime(ts.bestLapMs, timeBuf, sizeof(timeBuf));
      snprintf(buf, sizeof(buf), "BEST %s", timeBuf);
      u8g2.drawStr(0, 60, buf);
    }

    u8g2.sendBuffer();
    return;
  }

  // --- Ready (track detected, waiting for start) ---
  if (ts.trackState == TRACK_STATE_READY) {
    char buf[22];
    u8g2.drawStr(0, 12, "TRACK DETECTED");
    snprintf(buf, sizeof(buf), "ID: %08lX",
             static_cast<unsigned long>(ts.trackId));
    u8g2.drawStr(0, 26, buf);
    u8g2.drawStr(0, 40, "Cross start >5km/h");
    drawStatusBar(60);
    u8g2.sendBuffer();
    return;
  }

  // --- Idle ---
  if (gps.location.isValid()) {
    char buf[22];
    snprintf(buf, sizeof(buf), "LAT  %.6f", gps.location.lat());
    u8g2.drawStr(0, 12, buf);
    snprintf(buf, sizeof(buf), "LON %.6f", gps.location.lng());
    u8g2.drawStr(0, 26, buf);
  } else {
    u8g2.drawStr(0, 12, "WAITING FOR GPS");
    char buf[22];
    uint32_t sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
    snprintf(buf, sizeof(buf), "Satellites: %lu",
             static_cast<unsigned long>(sats));
    u8g2.drawStr(0, 26, buf);
  }

  // IMU temp
  if (!isnan(imuData.tempC)) {
    char buf[22];
    snprintf(buf, sizeof(buf), "IMU: %.1fC", imuData.tempC);
    u8g2.drawStr(0, 40, buf);
  }

  drawStatusBar(60);
  u8g2.sendBuffer();
}
