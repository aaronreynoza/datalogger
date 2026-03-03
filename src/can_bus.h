#pragma once

#include <Arduino.h>

// CAN bus message IDs (from ecosystem strategy doc)
static constexpr uint32_t CAN_ID_QUICKSHIFTER = 0x100;
static constexpr uint32_t CAN_ID_DASHBOARD    = 0x101;
static constexpr uint32_t CAN_ID_INTAKE       = 0x102;
static constexpr uint32_t CAN_ID_SUSPENSION   = 0x103;
static constexpr uint32_t CAN_ID_ECU          = 0x200;

struct CanFrame {
  uint32_t id;
  uint8_t  len;
  uint8_t  data[8];
  uint32_t timestampMs;
};

// Stub — returns false until CAN hardware is connected
bool initCanBus();
bool isCanBusReady();
bool pollCanBus(CanFrame &frame);
