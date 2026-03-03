#include "imu.h"

#include <SPI.h>

// HSPI pins shared by IMU and SD card
static const int HSPI_SCK_PIN  = 36;
static const int HSPI_MISO_PIN = 37;
static const int HSPI_MOSI_PIN = 35;
static const int IMU_CS_PIN    = 34;

static SPIClass hspi(HSPI);
static bool hspiStarted = false;

SPIClass &sharedHSPI() {
  if (!hspiStarted) {
    hspi.begin(HSPI_SCK_PIN, HSPI_MISO_PIN, HSPI_MOSI_PIN);
    hspiStarted = true;
  }
  return hspi;
}

static SensorQMI8658 qmi;
static bool imuReady = false;

ImuData imuData;

bool initImu() {
  Serial.println("[IMU] Initializing QMI8658 over SPI");

  pinMode(IMU_CS_PIN, OUTPUT);
  digitalWrite(IMU_CS_PIN, HIGH);

  if (!qmi.begin(sharedHSPI(), IMU_CS_PIN)) {
    Serial.println("[IMU] QMI8658 init FAILED");
    return false;
  }

  Serial.print("[IMU] QMI8658 ID: 0x");
  Serial.println(qmi.getChipID(), HEX);

  qmi.configAccelerometer(
      SensorQMI8658::ACC_RANGE_4G,
      SensorQMI8658::ACC_ODR_1000Hz,
      SensorQMI8658::LPF_MODE_0);

  qmi.configGyroscope(
      SensorQMI8658::GYR_RANGE_64DPS,
      SensorQMI8658::GYR_ODR_896_8Hz,
      SensorQMI8658::LPF_MODE_3);

  qmi.enableAccelerometer();
  qmi.enableGyroscope();

  imuReady = true;
  Serial.println("[IMU] QMI8658 configured");
  return true;
}

bool isImuReady() {
  return imuReady;
}

void updateImu() {
  if (qmi.getDataReady()) {
    if (qmi.getAccelerometer(imuData.acc.x, imuData.acc.y, imuData.acc.z) &&
        qmi.getGyroscope(imuData.gyr.x, imuData.gyr.y, imuData.gyr.z)) {
      imuData.tempC = qmi.getTemperature_C();
      imuData.timestamp = qmi.getTimestamp();
      imuData.hasData = true;
    } else {
      imuData.hasData = false;
    }
  } else {
    imuData.hasData = false;
  }
}
