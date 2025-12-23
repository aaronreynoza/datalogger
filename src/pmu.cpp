#include "pmu.h"

#include <Wire.h>
#include <XPowersLib.h>

static constexpr int PMU_SDA = 42;
static constexpr int PMU_SCL = 41;
static constexpr uint8_t PMU_ADDR = 0x34;

static TwoWire pmuWire(1);
static XPowersAXP2101 pmu;

void initPmu() {
  Serial.println("[PMU] Initializing AXP2101");

  if (!pmu.begin(pmuWire, PMU_ADDR, PMU_SDA, PMU_SCL)) {
    Serial.println("[PMU] AXP2101 not found");
    return;
  }

  Serial.println("[PMU] AXP2101 connected");

#if defined(XPOWERS_CHG_LED_CTRL_ON)
  pmu.setChargingLedMode(XPOWERS_CHG_LED_CTRL_ON);
#else
  pmu.setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);
#endif

  pmu.setALDO1Voltage(3300);
  pmu.enableALDO1(); // OLED/sensors

  pmu.setALDO3Voltage(3300);
  pmu.enableALDO3(); // LoRa rail (safe to enable)

  pmu.setALDO4Voltage(3300);
  pmu.enableALDO4(); // GPS rail

  pmu.setBLDO1Voltage(3300);
  pmu.enableBLDO1(); // SD rail (safe to enable)

  pmu.enableBattVoltageMeasure();
  pmu.enableVbusVoltageMeasure();
  pmu.enableSystemVoltageMeasure();
}
