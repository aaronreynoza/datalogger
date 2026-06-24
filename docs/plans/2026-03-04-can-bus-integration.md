# CAN Bus Integration — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add CAN bus data capture to the datalogger via ESP32-S3 native TWAI controller, with passive sniffing + OBD2 PID fallback, logging raw frames to a separate binary file on SD card.

**Architecture:** ESP32-S3 TWAI controller on GPIO 4 (TX) / GPIO 5 (RX) connected to SN65HVD230 CAN transceiver. A FreeRTOS task on Core 0 receives CAN frames into a lock-free SPSC ring buffer. Core 1 (main loop) drains the buffer, writes raw frames to a binary log file on SD, and prints diagnostics. OBD2 PID polling is a fallback mode activated when passive sniffing yields no frames after 3 seconds.

**Tech Stack:** ESP-IDF TWAI driver (`driver/twai.h`), FreeRTOS (`freertos/FreeRTOS.h`), Arduino framework on ESP32-S3

---

## Context for the Engineer

### Codebase Location
- Firmware repo: `/Users/aaronvaldez/repos/apexDirector/datalogger/`
- Build command: `pio run -e t-beams3-supreme`
- Flash command: `pio run -t upload -e t-beams3-supreme`
- Serial monitor: `pio device monitor -b 115200`

### Key Existing Files
- `src/can_bus.h` — Current stub: CanFrame struct + 3 function declarations
- `src/can_bus.cpp` — Current stub: all functions return false
- `src/main.cpp` — Main loop: GPS/IMU/Track at 100Hz, display at 500ms, housekeeping at 1Hz
- `src/race_logger.cpp` — IMU ring buffer pattern (600 samples, push/pop/peek)
- `src/atp_writer.h` — REC_CAN_FRAME = 0x05 reserved but not used yet
- `src/logging.h` — `diagLog()`, `diagLogf()`, `storageReady`, `loggingEnabled`

### Hardware Constraints
- Watchdog: 5s reboot on hang — NO blocking calls in main loop
- SD + IMU share HSPI — use `isWifiSdBusy()` guard before SD access
- Core 0 is completely unused (all current code runs on Core 1)
- `Serial.setTxTimeoutMs(0)` at end of setup — non-blocking serial
- Free GPIOs: 1-7, 10-16, 19-33 (we use GPIO 4 TX, GPIO 5 RX)

### What We're NOT Doing (Phase 1 only)
- NOT modifying ATP writer (no CAN records in ATP yet)
- NOT modifying race_logger pipeline
- NOT touching display or WiFi API
- NOT implementing CAN signal decoding (that's the binary config system, later)
- Raw frames go to a SEPARATE log file, not into the ATP session

---

### Task 1: Rewrite can_bus.h with TWAI types and API

**Files:**
- Modify: `src/can_bus.h`

**Step 1: Replace can_bus.h contents**

The existing header has a CanFrame struct and 3 stub function declarations. Replace the entire file with the new API that supports TWAI init, Core 0 task, ring buffer, raw logging, and OBD2 mode.

```cpp
#pragma once

#include <Arduino.h>

// --- CAN bus hardware config ---
static constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_4;
static constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_5;

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
```

**Step 2: Verify it compiles**

Run: `cd /Users/aaronvaldez/repos/apexDirector/datalogger && pio run -e t-beams3-supreme`
Expected: Compile error — can_bus.cpp still has old stubs. That's fine, confirms header is syntactically correct (linker error, not parse error).

**Step 3: Commit**

```bash
cd /Users/aaronvaldez/repos/apexDirector/datalogger
git add src/can_bus.h
git commit -m "feat(can): rewrite can_bus.h with TWAI types, ring buffer API, OBD2 constants"
```

---

### Task 2: Implement TWAI driver initialization + Core 0 receive task

**Files:**
- Modify: `src/can_bus.cpp`

**Step 1: Replace can_bus.cpp with full implementation**

This is the core of the CAN bus system. It handles:
1. TWAI driver install/start at 500 kbps LISTEN_ONLY
2. FreeRTOS task pinned to Core 0 that receives frames into ring buffer
3. Auto-speed detection (500 kbps -> 250 kbps fallback)
4. Lock-free SPSC ring buffer (producer on Core 0, consumer on Core 1)

```cpp
#include "can_bus.h"

#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "logging.h"

// ===================== Ring buffer (SPSC, lock-free) =====================
// Producer: Core 0 receive task
// Consumer: Core 1 main loop (canPopFrame)

static CanFrame canRing[CAN_RING_SIZE];
static volatile uint32_t canHead = 0;  // written by producer (Core 0)
static volatile uint32_t canTail = 0;  // written by consumer (Core 1)

static inline uint32_t ringMask(uint32_t v) { return v & (CAN_RING_SIZE - 1); }

static bool ringPush(const CanFrame &f) {
  uint32_t nextHead = canHead + 1;
  if (ringMask(nextHead) == ringMask(canTail)) return false;  // full
  canRing[ringMask(canHead)] = f;
  // Memory barrier: ensure frame is written before head advances
  __atomic_store_n(&canHead, nextHead, __ATOMIC_RELEASE);
  return true;
}

static bool ringPop(CanFrame &f) {
  uint32_t h = __atomic_load_n(&canHead, __ATOMIC_ACQUIRE);
  if (ringMask(canTail) == ringMask(h)) return false;  // empty
  f = canRing[ringMask(canTail)];
  canTail++;
  return true;
}

static size_t ringCount() {
  uint32_t h = __atomic_load_n(&canHead, __ATOMIC_ACQUIRE);
  return (h - canTail);  // works with wrapping
}

// ===================== State =====================

static CanStats stats = {};
static CanMode currentMode = CAN_MODE_OFF;
static TaskHandle_t canTaskHandle = nullptr;
static bool twaiInstalled = false;
static bool twaiStarted = false;

// Unique ID tracker (up to 64 IDs)
static uint32_t seenIds[64];
static uint8_t seenIdCount = 0;

static void trackUniqueId(uint32_t id) {
  for (uint8_t i = 0; i < seenIdCount; i++) {
    if (seenIds[i] == id) return;
  }
  if (seenIdCount < 64) {
    seenIds[seenIdCount++] = id;
    stats.uniqueIds = seenIdCount;
  }
}

// ===================== Raw log file =====================

static File canLogFile;
static bool canLogActive = false;
static uint32_t canLogFlushMs = 0;

// Raw log format: each frame = 17 bytes
// [4B bootMs little-endian][4B canId little-endian][1B dlc][8B data]
static constexpr size_t RAW_FRAME_SIZE = 17;

// Write buffer to reduce SD writes (512 bytes = ~30 frames)
static uint8_t canLogBuf[512];
static size_t canLogBufPos = 0;

static void canLogFlushBuf() {
  if (canLogBufPos > 0 && canLogActive) {
    canLogFile.write(canLogBuf, canLogBufPos);
    canLogBufPos = 0;
  }
}

static void canLogWriteFrame(const CanFrame &f) {
  if (!canLogActive) return;

  // Check if buffer has room
  if (canLogBufPos + RAW_FRAME_SIZE > sizeof(canLogBuf)) {
    canLogFlushBuf();
  }

  // Write to buffer
  uint8_t *p = canLogBuf + canLogBufPos;
  p[0] = f.timestampMs & 0xFF;
  p[1] = (f.timestampMs >> 8) & 0xFF;
  p[2] = (f.timestampMs >> 16) & 0xFF;
  p[3] = (f.timestampMs >> 24) & 0xFF;
  p[4] = f.id & 0xFF;
  p[5] = (f.id >> 8) & 0xFF;
  p[6] = (f.id >> 16) & 0xFF;
  p[7] = (f.id >> 24) & 0xFF;
  p[8] = f.len;
  memcpy(p + 9, f.data, 8);
  canLogBufPos += RAW_FRAME_SIZE;

  stats.framesLogged++;
}

// ===================== TWAI install/uninstall =====================

static bool installTwai(twai_mode_t mode, uint32_t speedKbps) {
  if (twaiInstalled) {
    twai_stop();
    twai_driver_uninstall();
    twaiInstalled = false;
    twaiStarted = false;
  }

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, mode);
  g_config.rx_queue_len = 32;  // default is 5, too low for automotive

  twai_timing_config_t t_config;
  if (speedKbps == 250) {
    t_config = TWAI_TIMING_CONFIG_250KBITS();
  } else {
    t_config = TWAI_TIMING_CONFIG_500KBITS();
  }

  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
  if (err != ESP_OK) {
    Serial.printf("[CAN] TWAI install failed: 0x%x\n", err);
    diagLogf("CAN twai install fail: 0x%x", err);
    return false;
  }
  twaiInstalled = true;

  err = twai_start();
  if (err != ESP_OK) {
    Serial.printf("[CAN] TWAI start failed: 0x%x\n", err);
    diagLogf("CAN twai start fail: 0x%x", err);
    twai_driver_uninstall();
    twaiInstalled = false;
    return false;
  }
  twaiStarted = true;

  Serial.printf("[CAN] TWAI started: %s mode, %lu kbps\n",
                mode == TWAI_MODE_LISTEN_ONLY ? "LISTEN" : "NORMAL",
                (unsigned long)speedKbps);
  diagLogf("CAN started: mode=%d speed=%lu",
           (int)mode, (unsigned long)speedKbps);
  return true;
}

// ===================== Core 0 receive task =====================

static void canReceiveTask(void *param) {
  (void)param;
  twai_message_t rxMsg;

  while (true) {
    // Block up to 10ms waiting for a frame
    esp_err_t err = twai_receive(&rxMsg, pdMS_TO_TICKS(10));

    if (err == ESP_OK) {
      CanFrame f;
      f.id = rxMsg.identifier;
      f.len = rxMsg.data_length_code;
      memcpy(f.data, rxMsg.data, 8);
      f.timestampMs = (uint32_t)millis();

      if (!ringPush(f)) {
        stats.framesDropped++;
      }
      stats.framesReceived++;
      trackUniqueId(f.id);

    } else if (err == ESP_ERR_TIMEOUT) {
      // No frame available — normal
    } else {
      stats.busErrors++;
    }

    // Yield to other Core 0 tasks (WiFi, etc. when enabled)
    taskYIELD();
  }
}

// ===================== Auto-speed detection =====================

// After starting in listen mode, if no frames arrive within 3 seconds,
// try 250 kbps. Called from main loop (Core 1), not from the task.
static uint32_t speedDetectStartMs = 0;
static bool speedDetectDone = false;
static uint32_t currentSpeedKbps = 500;

static void checkSpeedFallback(uint32_t nowMs) {
  if (speedDetectDone) return;
  if (currentMode == CAN_MODE_OFF) return;
  if (stats.framesReceived > 0) {
    speedDetectDone = true;
    Serial.printf("[CAN] Speed confirmed: %lu kbps (%lu frames)\n",
                  (unsigned long)currentSpeedKbps,
                  (unsigned long)stats.framesReceived);
    return;
  }
  if (nowMs - speedDetectStartMs > 3000 && currentSpeedKbps == 500) {
    Serial.println("[CAN] No frames at 500 kbps, trying 250 kbps...");
    diagLog("CAN: 500kbps timeout, trying 250kbps");
    currentSpeedKbps = 250;

    // Reinstall with 250 kbps (task keeps running, just no frames during switch)
    twai_mode_t mode = (currentMode == CAN_MODE_LISTEN)
                         ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL;
    installTwai(mode, 250);
    speedDetectStartMs = nowMs;

  } else if (nowMs - speedDetectStartMs > 3000 && currentSpeedKbps == 250) {
    speedDetectDone = true;
    Serial.println("[CAN] No frames at 250 kbps either. Bus may be empty or gateway blocking.");
    diagLog("CAN: no frames at 250kbps either");
  }
}

// ===================== OBD2 PID polling =====================

struct Obd2PidDef {
  uint8_t pid;
  const char *name;
};

static const Obd2PidDef obd2Pids[] = {
  {OBD2_PID_ENGINE_RPM,    "RPM"},
  {OBD2_PID_VEHICLE_SPEED,  "Speed"},
  {OBD2_PID_THROTTLE_POS,   "Throttle"},
  {OBD2_PID_COOLANT_TEMP,   "CoolantTemp"},
};
static constexpr uint8_t OBD2_PID_COUNT = sizeof(obd2Pids) / sizeof(obd2Pids[0]);

static uint8_t obd2PidIndex = 0;
static uint32_t lastObd2PollMs = 0;
static constexpr uint32_t OBD2_POLL_INTERVAL_MS = 100;  // 10 Hz cycle across all PIDs

static bool sendObd2Request(uint8_t pid) {
  if (!twaiStarted) return false;

  twai_message_t msg = {};
  msg.identifier = OBD2_REQUEST_ID;
  msg.data_length_code = 8;
  msg.data[0] = 0x02;  // number of additional bytes
  msg.data[1] = 0x01;  // Mode 01 (show current data)
  msg.data[2] = pid;
  // Bytes 3-7 padded with 0x00 (ISO 15765-2 padding)

  esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(10));
  if (err == ESP_OK) {
    stats.obd2Requests++;
    return true;
  }
  return false;
}

// ===================== Public API =====================

bool initCanBus() {
  memset(&stats, 0, sizeof(stats));
  canHead = 0;
  canTail = 0;
  seenIdCount = 0;
  speedDetectDone = false;
  currentSpeedKbps = 500;
  currentMode = CAN_MODE_OFF;

  // Start in LISTEN_ONLY mode at 500 kbps
  if (!installTwai(TWAI_MODE_LISTEN_ONLY, 500)) {
    return false;
  }

  currentMode = CAN_MODE_LISTEN;
  speedDetectStartMs = millis();

  // Create receive task on Core 0
  BaseType_t ret = xTaskCreatePinnedToCore(
    canReceiveTask,   // function
    "can_rx",         // name
    4096,             // stack (bytes)
    nullptr,          // param
    5,                // priority (same as WiFi tasks)
    &canTaskHandle,   // handle
    0                 // Core 0
  );

  if (ret != pdPASS) {
    Serial.println("[CAN] Failed to create Core 0 task");
    diagLog("CAN task create FAILED");
    twai_stop();
    twai_driver_uninstall();
    twaiInstalled = false;
    twaiStarted = false;
    currentMode = CAN_MODE_OFF;
    return false;
  }

  Serial.println("[CAN] Core 0 receive task started");
  diagLog("CAN task started on Core 0");
  return true;
}

void stopCanBus() {
  if (canTaskHandle) {
    vTaskDelete(canTaskHandle);
    canTaskHandle = nullptr;
  }
  if (twaiStarted) {
    twai_stop();
    twaiStarted = false;
  }
  if (twaiInstalled) {
    twai_driver_uninstall();
    twaiInstalled = false;
  }
  currentMode = CAN_MODE_OFF;
  stopCanLog();
  Serial.println("[CAN] Stopped");
}

bool isCanBusReady() {
  return twaiStarted && canTaskHandle != nullptr;
}

CanMode getCanMode() {
  return currentMode;
}

const CanStats& getCanStats() {
  return stats;
}

bool canPopFrame(CanFrame &frame) {
  return ringPop(frame);
}

size_t canAvailable() {
  return ringCount();
}

// ---- Raw log file ----

bool startCanLog(uint32_t epoch) {
  if (canLogActive) return true;
  if (!storageReady) return false;

  // Build filename from epoch (same pattern as atp_writer.cpp)
  int year = 1970, month = 1, day = 1, hour = 0, minute = 0;
  if (epoch) {
    uint32_t seconds = epoch;
    uint32_t days = seconds / 86400;
    uint32_t timeOfDay = seconds % 86400;
    hour = timeOfDay / 3600;
    minute = (timeOfDay % 3600) / 60;
    year = 1970;
    auto isLeap = [](int y) { return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0); };
    while (true) {
      uint16_t diy = isLeap(year) ? 366 : 365;
      if (days < diy) break;
      days -= diy;
      year++;
    }
    static const uint8_t daysInMonth[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    month = 1;
    for (int i = 0; i < 12; ++i) {
      uint8_t dim = daysInMonth[i];
      if (i == 1 && isLeap(year)) dim++;
      if (days < dim) { month = i + 1; day = days + 1; break; }
      days -= dim;
    }
  }

  char filename[40];
  snprintf(filename, sizeof(filename), "/can-raw-%04d%02d%02d-%02d%02d.bin",
           year, month, day, hour, minute);

  canLogFile = SD.open(filename, "w");
  if (!canLogFile) {
    Serial.printf("[CAN] Failed to create %s\n", filename);
    return false;
  }

  // Write simple header: magic + version + epoch
  const uint8_t magic[4] = {'C', 'A', 'N', '\0'};
  canLogFile.write(magic, 4);
  uint8_t ver = 1;
  canLogFile.write(&ver, 1);
  // 3 bytes padding for alignment
  uint8_t pad[3] = {0};
  canLogFile.write(pad, 3);
  // epoch as 4 bytes LE
  uint8_t eb[4];
  eb[0] = epoch & 0xFF; eb[1] = (epoch >> 8) & 0xFF;
  eb[2] = (epoch >> 16) & 0xFF; eb[3] = (epoch >> 24) & 0xFF;
  canLogFile.write(eb, 4);
  // bootMs as 4 bytes LE
  uint32_t bootMs = millis();
  eb[0] = bootMs & 0xFF; eb[1] = (bootMs >> 8) & 0xFF;
  eb[2] = (bootMs >> 16) & 0xFF; eb[3] = (bootMs >> 24) & 0xFF;
  canLogFile.write(eb, 4);
  canLogFile.flush();

  canLogActive = true;
  canLogBufPos = 0;
  canLogFlushMs = millis();

  Serial.printf("[CAN] Raw log started: %s\n", filename);
  diagLogf("CAN log: %s", filename);
  return true;
}

void stopCanLog() {
  if (!canLogActive) return;
  canLogFlushBuf();
  canLogFile.flush();
  canLogFile.close();
  canLogActive = false;
  Serial.printf("[CAN] Raw log closed: %lu frames\n",
                (unsigned long)stats.framesLogged);
}

void flushCanLog() {
  if (!canLogActive) return;
  uint32_t now = millis();
  if (now - canLogFlushMs >= 500) {
    canLogFlushBuf();
    canLogFile.flush();
    canLogFlushMs = now;
  }
}

bool isCanLogActive() {
  return canLogActive;
}

// ---- OBD2 mode ----

void enableObd2Mode() {
  if (currentMode == CAN_MODE_OBD2) return;

  Serial.println("[CAN] Switching to OBD2 mode (NORMAL)...");
  diagLog("CAN: switching to OBD2 mode");

  // Reinstall TWAI in NORMAL mode (can transmit)
  if (!installTwai(TWAI_MODE_NORMAL, currentSpeedKbps)) {
    Serial.println("[CAN] OBD2 mode switch failed");
    return;
  }
  currentMode = CAN_MODE_OBD2;
  obd2PidIndex = 0;
  lastObd2PollMs = millis();
}

void tickObd2(uint32_t nowMs) {
  if (currentMode != CAN_MODE_OBD2) return;
  if (nowMs - lastObd2PollMs < OBD2_POLL_INTERVAL_MS) return;

  lastObd2PollMs = nowMs;
  sendObd2Request(obd2Pids[obd2PidIndex].pid);
  obd2PidIndex = (obd2PidIndex + 1) % OBD2_PID_COUNT;
}
```

**Step 2: Verify it compiles**

Run: `cd /Users/aaronvaldez/repos/apexDirector/datalogger && pio run -e t-beams3-supreme`
Expected: PASS (compiles). If `driver/twai.h` not found, the ESP-IDF include path needs checking — it's at `tools/sdk/esp32s3/include/driver/include/driver/twai.h` and should be auto-resolved by the Arduino ESP32 framework.

**Step 3: Commit**

```bash
cd /Users/aaronvaldez/repos/apexDirector/datalogger
git add src/can_bus.cpp
git commit -m "feat(can): implement TWAI driver, Core 0 receive task, ring buffer, raw log, OBD2"
```

---

### Task 3: Integrate CAN bus into main.cpp

**Files:**
- Modify: `src/main.cpp`

**Step 1: Add CAN init to setup()**

After `initRaceLogger()` (line 211) and before the boot summary, add CAN bus initialization:

```cpp
// CAN bus (TWAI on GPIO 4/5 via SN65HVD230)
bool canOk = initCanBus();
displayBootStatus("CAN", canOk);
```

And in the boot summary (after line 218):

```cpp
Serial.print("  CAN:  "); Serial.println(canOk ? "OK" : "FAIL");
```

And in diagLog (after line 244):

```cpp
diagLogf("CAN=%s", canOk ? "OK" : "FAIL");
```

**Step 2: Add CAN processing to main loop**

Add a new section in the main loop. Place it after `flushRaceLogger(now)` (line 379) and before the track log flush (line 382). This section:
1. Drains CAN ring buffer and writes frames to raw log
2. Auto-starts CAN log when race logger starts (uses GPS epoch)
3. Checks speed fallback
4. Ticks OBD2 polling
5. Auto-switches to OBD2 if passive sniff yields nothing after 6 seconds

```cpp
  // ---- CAN bus processing ----
  if (isCanBusReady()) {
    // Auto-start CAN log when we have a GPS epoch
    // (even if not racing — we want raw CAN data from the entire session)
    static bool canLogStarted = false;
    if (!canLogStarted && latestGps.epoch > 0 && storageReady) {
      canLogStarted = startCanLog(latestGps.epoch);
    }

    // Drain ring buffer -> raw log file
    {
      CanFrame cf;
      uint16_t drained = 0;
      while (canPopFrame(cf) && drained < 100) {  // cap per loop to avoid starving IMU
        canLogWriteFrame(cf);
        drained++;
      }
    }

    // Flush raw log periodically
    flushCanLog();

    // Auto-speed detection (500 -> 250 kbps fallback)
    checkSpeedFallback(now);

    // OBD2 fallback: if still no frames after 6s, switch to active OBD2
    static bool obd2FallbackChecked = false;
    if (!obd2FallbackChecked && now > 6000) {
      const CanStats &cs = getCanStats();
      if (cs.framesReceived == 0 && cs.mode == CAN_MODE_LISTEN) {
        Serial.println("[CAN] No passive frames after 6s, switching to OBD2 mode");
        enableObd2Mode();
      }
      obd2FallbackChecked = true;
    }

    // Tick OBD2 polling (no-op if not in OBD2 mode)
    tickObd2(now);
  }
```

**Step 3: Add CAN stats to serial telemetry**

In `printTelemetry()` (after the RACE stats block, around line 156), add:

```cpp
  if (isCanBusReady()) {
    const CanStats &cs = getCanStats();
    Serial.print(" | CAN:");
    Serial.print(cs.mode == CAN_MODE_LISTEN ? "LSN" :
                 cs.mode == CAN_MODE_OBD2 ? "OBD" : "OFF");
    Serial.print(" rx=");
    Serial.print(cs.framesReceived);
    Serial.print(" ids=");
    Serial.print(cs.uniqueIds);
    if (cs.framesDropped > 0) {
      Serial.print(" drop=");
      Serial.print(cs.framesDropped);
    }
  }
```

**Step 4: Add CAN stop to race logger stop and serial commands**

In the serial command handler, add a 'K' command to manually switch to OBD2 mode:

```cpp
    case 'K':
    case 'k':
      if (isCanBusReady()) {
        enableObd2Mode();
        Serial.println("CAN: switched to OBD2 mode");
      } else {
        Serial.println("CAN: not ready");
      }
      break;
```

And update the command help line (line 172):

```cpp
Serial.println("Commands: D=toggle log, d=dump, c=clear, l=list, K=CAN OBD2 mode");
```

**Step 5: Stop CAN log when logging disabled**

In the 'D' toggle handler (around line 88-93), add `stopCanLog()` alongside `stopRaceLogger()`:

```cpp
    case 'D':
      if (!storageReady) {
        Serial.println("Storage not ready");
        break;
      }
      loggingEnabled = !loggingEnabled;
      if (!loggingEnabled) {
        stopRaceLogger();
        stopTrackLog();
        stopCanLog();
      }
```

Also in the 'c'/'C' handler (around line 100-105):

```cpp
    case 'c':
    case 'C':
      clearLogs();
      resetTrack();
      stopRaceLogger();
      stopCanLog();
      Serial.println("Logs cleared");
      break;
```

**Step 6: Make canLogWriteFrame and checkSpeedFallback accessible**

Since `canLogWriteFrame` and `checkSpeedFallback` are `static` in `can_bus.cpp`, they can't be called from `main.cpp`. We need to add a public wrapper function to the API. Add to `can_bus.h`:

```cpp
// Process CAN frames from ring buffer -> raw log (call from main loop)
void drainCanToLog(uint16_t maxFrames);

// Check if speed fallback is needed (call from main loop)
void canCheckSpeed(uint32_t nowMs);
```

And implement in `can_bus.cpp`:

```cpp
void drainCanToLog(uint16_t maxFrames) {
  CanFrame cf;
  uint16_t drained = 0;
  while (ringPop(cf) && drained < maxFrames) {
    canLogWriteFrame(cf);
    drained++;
  }
}

void canCheckSpeed(uint32_t nowMs) {
  checkSpeedFallback(nowMs);
}
```

Then simplify the main.cpp CAN section to use these wrapper functions instead of calling internal functions directly.

**Step 7: Verify it compiles**

Run: `cd /Users/aaronvaldez/repos/apexDirector/datalogger && pio run -e t-beams3-supreme`
Expected: PASS

**Step 8: Commit**

```bash
cd /Users/aaronvaldez/repos/apexDirector/datalogger
git add src/main.cpp src/can_bus.h src/can_bus.cpp
git commit -m "feat(can): integrate CAN bus into main loop with auto-log, speed detect, OBD2 fallback"
```

---

### Task 4: Add CAN status to boot display and housekeeping diagnostics

**Files:**
- Modify: `src/main.cpp`

**Step 1: Add CAN diagnostics to housekeeping (1 Hz)**

In the housekeeping section (around line 398-413), after the WiFi management block, add a 1-second CAN diagnostic print to serial (only when CAN has activity):

```cpp
    // CAN diagnostics (1 Hz)
    if (isCanBusReady()) {
      static uint32_t lastCanPrint = 0;
      static uint32_t lastRxCount = 0;
      const CanStats &cs = getCanStats();
      uint32_t newFrames = cs.framesReceived - lastRxCount;
      if (newFrames > 0 || cs.busErrors > 0) {
        if (now - lastCanPrint >= 5000) {  // every 5s when active
          Serial.printf("[CAN] %lu frames, %lu unique IDs, %lu errors, %lu dropped, mode=%s\n",
                        (unsigned long)cs.framesReceived,
                        (unsigned long)cs.uniqueIds,
                        (unsigned long)cs.busErrors,
                        (unsigned long)cs.framesDropped,
                        cs.mode == CAN_MODE_LISTEN ? "LISTEN" :
                        cs.mode == CAN_MODE_OBD2 ? "OBD2" : "OFF");
          lastCanPrint = now;
        }
      }
      lastRxCount = cs.framesReceived;
    }
```

**Step 2: Verify it compiles**

Run: `cd /Users/aaronvaldez/repos/apexDirector/datalogger && pio run -e t-beams3-supreme`
Expected: PASS

**Step 3: Commit**

```bash
cd /Users/aaronvaldez/repos/apexDirector/datalogger
git add src/main.cpp
git commit -m "feat(can): add CAN diagnostics to housekeeping and telemetry output"
```

---

### Task 5: Update CLAUDE.md and docs

**Files:**
- Modify: `CLAUDE.md`

**Step 1: Update CLAUDE.md**

In the architecture section, update can_bus.cpp/h description:

```
├── can_bus.cpp/h     — CAN bus via ESP32-S3 TWAI (GPIO 4/5), Core 0 task, raw logging
```

In the Hardware table, add:

```
| CAN Transceiver | SN65HVD230 | TWAI (GPIO) | TX=4, RX=5 |
```

In Key Timing, add:

```
- **CAN receive:** interrupt-driven (Core 0 task, 10ms poll)
- **CAN log flush:** 500 ms
- **OBD2 polling:** 100 ms per PID (10 Hz round-robin)
```

In SD Card Files, add:

```
| `can-raw-YYYYMMDD-HHMM.bin` | Raw CAN frames binary log |
```

In Future Work, update:

```
- [x] CAN bus integration (passive sniff + OBD2 fallback)
- [ ] CAN signal decoding (binary config from desktop app)
- [ ] CAN data in ATP format (REC_CAN_FRAME records)
```

**Step 2: Commit**

```bash
cd /Users/aaronvaldez/repos/apexDirector/datalogger
git add CLAUDE.md
git commit -m "docs: update CLAUDE.md with CAN bus integration details"
```

---

### Task 6: Build verification and flash test

**Step 1: Full clean build**

Run: `cd /Users/aaronvaldez/repos/apexDirector/datalogger && pio run -e t-beams3-supreme --clean`
Expected: PASS. Note the binary size — it should be well within the 8MB flash limit.

**Step 2: Check for warnings**

Review build output for any warnings related to CAN code. Fix any `-Wunused-variable`, `-Wuninitialized`, or `-Wsign-compare` warnings.

**Step 3: Flash to device**

Run: `pio run -t upload -e t-beams3-supreme`
Expected: Successful upload

**Step 4: Monitor serial output**

Run: `pio device monitor -b 115200`

Expected boot output should include:
```
==== ApexDirector Core Pro ====
Commands: D=toggle log, d=dump, c=clear, l=list, K=CAN OBD2 mode
...
[CAN] TWAI started: LISTEN mode, 500 kbps
[CAN] Core 0 receive task started
...
---- Boot summary ----
  PMU:  OK
  IMU:  OK
  SD:   OK
  CAN:  OK
----------------------
```

If CAN transceiver is NOT connected (bench test), expect:
- CAN: OK (TWAI installs regardless — it's just GPIO config)
- No CAN frames received
- After 3s: `[CAN] No frames at 500 kbps, trying 250 kbps...`
- After 6s: `[CAN] No passive frames after 6s, switching to OBD2 mode`
- OBD2 requests sent but no responses (no transceiver connected)

**Step 5: Commit final state**

```bash
cd /Users/aaronvaldez/repos/apexDirector/datalogger
git add -A
git commit -m "feat(can): CAN bus integration complete - TWAI + Core 0 + raw log + OBD2 fallback"
```

---

## Summary of Files Changed

| File | Action | Description |
|------|--------|-------------|
| `src/can_bus.h` | Rewrite | TWAI types, ring buffer API, OBD2 constants, stats struct |
| `src/can_bus.cpp` | Rewrite | TWAI driver, Core 0 task, SPSC ring buffer, raw log writer, OBD2 polling |
| `src/main.cpp` | Modify | CAN init in setup, drain+log in loop, telemetry output, serial commands |
| `CLAUDE.md` | Modify | Updated architecture, timing, file patterns, future work |

## What to Verify at the Track Tonight

1. **Serial output:** Watch for `[CAN]` messages during boot and driving
2. **Passive sniff result:** If frames appear, note the `ids=` count in telemetry
3. **OBD2 fallback:** If it auto-switches, watch for `obd2Requests` incrementing
4. **Raw log file:** After the session, check SD for `can-raw-*.bin` file and note its size
5. **No regression:** GPS + IMU + lap timing should work exactly as before (CAN is on Core 0, independent)

## Decoding the Raw Log File (post-test, on desktop)

The raw log file can be decoded with a simple script:
```python
# Read can-raw-*.bin
import struct
with open("can-raw-YYYYMMDD-HHMM.bin", "rb") as f:
    magic = f.read(4)   # b'CAN\x00'
    ver = f.read(1)     # b'\x01'
    f.read(3)           # padding
    epoch = struct.unpack("<I", f.read(4))[0]
    boot_ms = struct.unpack("<I", f.read(4))[0]
    print(f"Epoch: {epoch}, Boot: {boot_ms}")
    while True:
        data = f.read(17)
        if len(data) < 17: break
        ts, can_id, dlc = struct.unpack("<IIB", data[:9])
        payload = data[9:17]
        print(f"  {ts}ms  ID=0x{can_id:03X}  DLC={dlc}  {payload.hex()}")
```
