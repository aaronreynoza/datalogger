#include "can_bus.h"

// CAN bus is not yet connected. These stubs compile but do nothing.
// When MCP2515 or ESP32-C3 CAN controller is wired, implement here.

bool initCanBus() {
  Serial.println("CAN bus: not connected (stub)");
  return false;
}

bool isCanBusReady() {
  return false;
}

bool pollCanBus(CanFrame &frame) {
  (void)frame;
  return false;
}
