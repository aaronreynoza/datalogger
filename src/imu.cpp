#include "imu.h"

#include <SPI.h>

// Pins from Meshtastic T-Beam S3 Supreme docs
static const int IMU_MOSI_PIN = 35;
static const int IMU_MISO_PIN = 37;
static const int IMU_SCK_PIN  = 36;
static const int IMU_CS_PIN   = 34;

static SPIClass imuSPI(HSPI);
static SensorQMI8658 qmi;

ImuData imuData;

void initImu() {
  Serial.println("Init QMI8658 over SPI (SensorLib 0.3.x API)");

  imuSPI.begin(IMU_SCK_PIN, IMU_MISO_PIN, IMU_MOSI_PIN, IMU_CS_PIN);
  pinMode(IMU_CS_PIN, OUTPUT);
  digitalWrite(IMU_CS_PIN, HIGH);

  if (!qmi.begin(imuSPI, IMU_CS_PIN)) {
    Serial.println("QMI8658 init FAILED");
    return;
  }

  Serial.print("QMI8658 ID: 0x");
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

  Serial.println("QMI8658 configured");
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
