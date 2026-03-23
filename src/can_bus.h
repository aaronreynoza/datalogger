#pragma once

#include <Arduino.h>

// --- CAN bus hardware config ---
// NOTE: GPIO 4/5 are wired to LoRa SX1262 (BUSY/RST) on T-Beam Supreme.
// Use IO2/IO3 from the expansion header instead.
static constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_2;
static constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_3;

// --- Ring buffer config ---
static constexpr size_t CAN_RING_SIZE = 512;  // frames, power of 2

// --- OBD2 constants ---
static constexpr uint32_t OBD2_REQUEST_ID  = 0x7DF;  // broadcast functional address
static constexpr uint32_t OBD2_RESPONSE_ID = 0x7E8;  // ECU 1 response

// Standard OBD2 PIDs (Mode 01)
static constexpr uint8_t OBD2_PID_ENGINE_RPM    = 0x0C;
static constexpr uint8_t OBD2_PID_VEHICLE_SPEED  = 0x0D;
static constexpr uint8_t OBD2_PID_THROTTLE_POS   = 0x11;
static constexpr uint8_t OBD2_PID_COOLANT_TEMP   = 0x05;

// --- CAN frame (same struct, kept compatible) ---
struct CanFrame {
  uint32_t id;
  uint8_t  len;
  uint8_t  data[8];
  uint32_t timestampMs;
};

// --- Operating modes ---
enum CanMode : uint8_t {
  CAN_MODE_OFF = 0,
  CAN_MODE_LISTEN,     // passive sniff (TWAI_MODE_LISTEN_ONLY)
  CAN_MODE_OBD2,       // active OBD2 PID polling (TWAI_MODE_NORMAL)
};

// --- Diagnostic counters ---
struct CanStats {
  uint32_t framesReceived;   // total frames received by Core 0 task
  uint32_t framesDropped;    // ring buffer overflows
  uint32_t framesLogged;     // written to raw log file
  uint32_t busErrors;        // TWAI bus error count
  uint32_t uniqueIds;        // unique CAN IDs seen (capped at 64)
  uint32_t obd2Requests;     // OBD2 requests sent
  uint32_t obd2Responses;    // OBD2 responses received
  uint32_t txFailed;         // twai_transmit failures
  CanMode  mode;             // current operating mode
};

// --- Public API ---
bool initCanBus();                        // init TWAI + start Core 0 task
void stopCanBus();                        // stop task + uninstall TWAI
bool isCanBusReady();                     // TWAI installed and task running
CanMode getCanMode();                     // current mode
const CanStats& getCanStats();            // diagnostic counters

// Ring buffer consumer (called from Core 1 main loop)
bool canPopFrame(CanFrame &frame);        // non-blocking dequeue
size_t canAvailable();                    // frames waiting in ring buffer

// Raw log file management
bool startCanLog(uint32_t epoch);         // open /can-raw-YYYYMMDD-HHMM.bin
void stopCanLog();                        // flush + close
void flushCanLog();                       // periodic flush (call from main loop)
bool isCanLogActive();                    // file is open

// OBD2 mode (fallback)
void enableObd2Mode();                    // switch from listen to OBD2
void tickObd2(uint32_t nowMs);            // poll PIDs at 10 Hz (call from main loop)

// Main loop helpers
void drainCanToLog(uint16_t maxFrames);   // pop frames from ring -> raw log
void canCheckSpeed(uint32_t nowMs);       // auto-speed detection (500 -> 250 kbps)

// Diagnostics
void printCanTwaiStatus();                // print TWAI error counters + bus state
bool canLoopbackTest();                   // send+receive frame via NO_ACK mode (wiring test)
