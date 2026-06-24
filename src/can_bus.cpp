#include "can_bus.h"

#include <cstring>
#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <SD.h>

#include <esp_task_wdt.h>
#include "logging.h"

// ===================== SPSC Lock-Free Ring Buffer =====================

static CanFrame canRing[CAN_RING_SIZE];
static volatile uint32_t canHead = 0;  // written by Core 0 producer
static volatile uint32_t canTail = 0;  // written by Core 1 consumer

static inline uint32_t ringMask(uint32_t idx) {
  return idx & (CAN_RING_SIZE - 1);
}

static bool ringPush(const CanFrame &frame) {
  uint32_t head = __atomic_load_n(&canHead, __ATOMIC_RELAXED);
  uint32_t tail = __atomic_load_n(&canTail, __ATOMIC_ACQUIRE);
  uint32_t next = head + 1;
  if (ringMask(next) == ringMask(tail) && (next - tail) >= CAN_RING_SIZE) {
    return false;  // full
  }
  canRing[ringMask(head)] = frame;
  __atomic_store_n(&canHead, next, __ATOMIC_RELEASE);
  return true;
}

static bool ringPop(CanFrame &frame) {
  uint32_t tail = __atomic_load_n(&canTail, __ATOMIC_RELAXED);
  uint32_t head = __atomic_load_n(&canHead, __ATOMIC_ACQUIRE);
  if (tail == head) return false;  // empty
  frame = canRing[ringMask(tail)];
  __atomic_store_n(&canTail, tail + 1, __ATOMIC_RELEASE);
  return true;
}

static uint32_t ringCount() {
  uint32_t head = __atomic_load_n(&canHead, __ATOMIC_ACQUIRE);
  uint32_t tail = __atomic_load_n(&canTail, __ATOMIC_ACQUIRE);
  return head - tail;
}

// ===================== Module State =====================

static CanStats stats;
static CanMode currentMode = CAN_MODE_OFF;
static bool twaiInstalled = false;
static TaskHandle_t canTaskHandle = nullptr;
static volatile bool taskRunning = false;
static volatile bool twaiReinstalling = false;  // pause Core 0 task during driver reinstall

// Unique CAN ID tracking
static constexpr size_t MAX_UNIQUE_IDS = 64;
static uint32_t uniqueIds[MAX_UNIQUE_IDS];
static uint32_t uniqueIdCount = 0;

// Auto-speed detection state
static uint32_t speedDetectStartMs = 0;
static uint32_t currentSpeedKbps = 500;
static uint8_t  speedAttempt = 0;  // 0=500kbps, 1=250kbps, 2=gave up
static bool     speedDetectDone = false;

// OBD2 state
static constexpr uint8_t OBD2_PID_LIST[] = {
  OBD2_PID_ENGINE_RPM,
  OBD2_PID_VEHICLE_SPEED,
  OBD2_PID_THROTTLE_POS,
  OBD2_PID_COOLANT_TEMP,
};
static constexpr uint8_t OBD2_PID_COUNT = sizeof(OBD2_PID_LIST) / sizeof(OBD2_PID_LIST[0]);
static uint8_t  obd2PidIndex = 0;
static uint32_t obd2LastPollMs = 0;

// ===================== Raw CAN Log File =====================

static File canLogFile;
static bool canLogActive = false;

static constexpr size_t CAN_LOG_BUF_SIZE = 512;
static uint8_t canLogBuf[CAN_LOG_BUF_SIZE];
static size_t  canLogBufPos = 0;
static uint32_t canLogLastFlushMs = 0;

// Raw log header: 16 bytes
// Bytes 0-3:   magic "CAN\0"
// Byte  4:     version (1)
// Bytes 5-7:   padding (0)
// Bytes 8-11:  epoch (uint32_t LE)
// Bytes 12-15: boot millis (uint32_t LE)

// Raw log frame: 17 bytes
// Bytes 0-3:  boot millis (uint32_t LE)
// Bytes 4-7:  CAN ID (uint32_t LE)
// Byte  8:    DLC
// Bytes 9-16: data[8]

static void canLogBufWriteU32(uint32_t v) {
  if (canLogBufPos + 4 > CAN_LOG_BUF_SIZE) return;
  canLogBuf[canLogBufPos++] = v & 0xFF;
  canLogBuf[canLogBufPos++] = (v >> 8) & 0xFF;
  canLogBuf[canLogBufPos++] = (v >> 16) & 0xFF;
  canLogBuf[canLogBufPos++] = (v >> 24) & 0xFF;
}

static void canLogFlushBuf() {
  if (canLogBufPos > 0 && canLogFile) {
    canLogFile.write(canLogBuf, canLogBufPos);
    canLogBufPos = 0;
  }
}

static void canLogWriteFrame(const CanFrame &frame) {
  // 17 bytes per frame — flush buffer if it won't fit
  if (canLogBufPos + 17 > CAN_LOG_BUF_SIZE) {
    canLogFlushBuf();
  }
  canLogBufWriteU32(frame.timestampMs);
  canLogBufWriteU32(frame.id);
  if (canLogBufPos < CAN_LOG_BUF_SIZE) {
    canLogBuf[canLogBufPos++] = frame.len;
  }
  size_t toCopy = 8;
  if (canLogBufPos + toCopy > CAN_LOG_BUF_SIZE) toCopy = CAN_LOG_BUF_SIZE - canLogBufPos;
  memcpy(&canLogBuf[canLogBufPos], frame.data, toCopy);
  canLogBufPos += toCopy;
}

// ===================== Unique ID Tracking =====================

static void trackUniqueId(uint32_t id) {
  for (uint32_t i = 0; i < uniqueIdCount; ++i) {
    if (uniqueIds[i] == id) return;
  }
  if (uniqueIdCount < MAX_UNIQUE_IDS) {
    uniqueIds[uniqueIdCount++] = id;
    stats.uniqueIds = uniqueIdCount;
  }
}

// ===================== TWAI Driver =====================

static bool installTwai(twai_mode_t mode, uint32_t speedKbps) {
  if (twaiInstalled) {
    twaiReinstalling = true;
    vTaskDelay(pdMS_TO_TICKS(20));  // let Core 0 task exit twai_receive
    twai_stop();
    twai_driver_uninstall();
    twaiInstalled = false;
  }

  twai_general_config_t gCfg = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, mode);
  gCfg.rx_queue_len = 32;

  twai_timing_config_t tCfg;
  if (speedKbps == 250) {
    tCfg = TWAI_TIMING_CONFIG_250KBITS();
  } else {
    tCfg = TWAI_TIMING_CONFIG_500KBITS();
    speedKbps = 500;  // default
  }

  twai_filter_config_t fCfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&gCfg, &tCfg, &fCfg);
  if (err != ESP_OK) {
    Serial.printf("[CAN] TWAI install failed: 0x%X\n", err);
    diagLogf("CAN twai install fail 0x%X", err);
    return false;
  }

  err = twai_start();
  if (err != ESP_OK) {
    Serial.printf("[CAN] TWAI start failed: 0x%X\n", err);
    diagLogf("CAN twai start fail 0x%X", err);
    twai_driver_uninstall();
    return false;
  }

  twaiInstalled = true;
  twaiReinstalling = false;
  currentSpeedKbps = speedKbps;
  Serial.printf("[CAN] TWAI started @ %u kbps (mode=%s)\n",
                speedKbps, mode == TWAI_MODE_LISTEN_ONLY ? "listen" : "normal");
  diagLogf("CAN started %ukbps", speedKbps);
  return true;
}

// ===================== Core 0 Receive Task =====================

static void canReceiveTask(void *param) {
  (void)param;
  taskRunning = true;

  while (taskRunning) {
    if (twaiReinstalling) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    twai_message_t rxMsg;
    esp_err_t err = twai_receive(&rxMsg, pdMS_TO_TICKS(10));

    if (err == ESP_OK) {
      CanFrame frame;
      frame.id = rxMsg.identifier;
      frame.len = rxMsg.data_length_code;
      memcpy(frame.data, rxMsg.data, 8);
      frame.timestampMs = millis();

      trackUniqueId(frame.id);
      stats.framesReceived++;

      if (!ringPush(frame)) {
        stats.framesDropped++;
      }
    } else if (err == ESP_ERR_TIMEOUT) {
      // No frame available — normal, just loop
    } else {
      stats.busErrors++;
    }

    taskYIELD();
  }

  vTaskDelete(nullptr);
}

// ===================== OBD2 Functions =====================

static void sendObd2Request(uint8_t pid) {
  twai_message_t txMsg;
  memset(&txMsg, 0, sizeof(txMsg));
  txMsg.identifier = OBD2_REQUEST_ID;
  txMsg.data_length_code = 8;
  txMsg.data[0] = 0x02;  // number of additional data bytes
  txMsg.data[1] = 0x01;  // Mode 01: show current data
  txMsg.data[2] = pid;
  // data[3..7] = 0 (padding)

  esp_err_t err = twai_transmit(&txMsg, pdMS_TO_TICKS(5));
  if (err == ESP_OK) {
    stats.obd2Requests++;
  } else {
    stats.txFailed++;
  }
}

// ===================== Epoch to Filename =====================

static void epochToFilename(uint32_t epochS, char *buf, size_t bufLen) {
  int year = 1970, month = 1, day = 1, hour = 0, minute = 0;
  if (epochS) {
    uint32_t seconds = epochS;
    uint32_t days = seconds / 86400;
    uint32_t timeOfDay = seconds % 86400;
    hour = timeOfDay / 3600;
    minute = (timeOfDay % 3600) / 60;

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
  snprintf(buf, bufLen, "/can-raw-%04d%02d%02d-%02d%02d.bin",
           year, month, day, hour, minute);
}

// ===================== Public API =====================

bool initCanBus() {
  memset(&stats, 0, sizeof(stats));
  stats.mode = CAN_MODE_LISTEN;
  currentMode = CAN_MODE_LISTEN;

  // Reset ring buffer
  __atomic_store_n(&canHead, 0, __ATOMIC_RELEASE);
  __atomic_store_n(&canTail, 0, __ATOMIC_RELEASE);

  // Reset unique ID tracking
  uniqueIdCount = 0;

  // Reset speed detection
  speedAttempt = 0;
  speedDetectDone = false;
  speedDetectStartMs = millis();

  // Install TWAI in listen-only mode at 500 kbps
  if (!installTwai(TWAI_MODE_LISTEN_ONLY, 500)) {
    currentMode = CAN_MODE_OFF;
    stats.mode = CAN_MODE_OFF;
    return false;
  }

  // Create receive task on Core 0
  BaseType_t result = xTaskCreatePinnedToCore(
    canReceiveTask,
    "canRx",
    4096,
    nullptr,
    5,           // priority
    &canTaskHandle,
    0            // Core 0
  );

  if (result != pdPASS) {
    Serial.println("[CAN] Failed to create receive task");
    diagLog("CAN task create failed");
    twai_stop();
    twai_driver_uninstall();
    twaiInstalled = false;
    currentMode = CAN_MODE_OFF;
    stats.mode = CAN_MODE_OFF;
    return false;
  }

  Serial.println("[CAN] Init OK — Core 0 receive task started");
  diagLog("CAN init OK");
  return true;
}

void stopCanBus() {
  // Signal task to stop
  taskRunning = false;

  // Wait for task to exit (up to 100ms)
  if (canTaskHandle) {
    vTaskDelay(pdMS_TO_TICKS(50));
    canTaskHandle = nullptr;
  }

  // Stop TWAI
  if (twaiInstalled) {
    twai_stop();
    twai_driver_uninstall();
    twaiInstalled = false;
  }

  // Close log if open
  if (canLogActive) {
    stopCanLog();
  }

  currentMode = CAN_MODE_OFF;
  stats.mode = CAN_MODE_OFF;
  Serial.println("[CAN] Stopped");
  diagLog("CAN stopped");
}

bool isCanBusReady() {
  return twaiInstalled && taskRunning;
}

CanMode getCanMode() {
  return currentMode;
}

const CanStats& getCanStats() {
  return stats;
}

// ===================== Ring Buffer Consumer =====================

bool canPopFrame(CanFrame &frame) {
  return ringPop(frame);
}

size_t canAvailable() {
  return (size_t)ringCount();
}

// ===================== Raw CAN Log Management =====================

bool startCanLog(uint32_t epoch) {
  if (canLogActive) stopCanLog();
  if (!storageReady) {
    Serial.println("[CAN] Cannot start log — storage not ready");
    return false;
  }

  char filename[40];
  epochToFilename(epoch, filename, sizeof(filename));

  canLogFile = SD.open(filename, "w");
  if (!canLogFile) {
    Serial.printf("[CAN] Failed to create %s\n", filename);
    diagLogf("CAN log create fail %s", filename);
    return false;
  }

  // Write 16-byte header
  canLogBufPos = 0;
  // Magic: "CAN\0"
  canLogBuf[canLogBufPos++] = 'C';
  canLogBuf[canLogBufPos++] = 'A';
  canLogBuf[canLogBufPos++] = 'N';
  canLogBuf[canLogBufPos++] = '\0';
  // Version
  canLogBuf[canLogBufPos++] = 1;
  // Padding
  canLogBuf[canLogBufPos++] = 0;
  canLogBuf[canLogBufPos++] = 0;
  canLogBuf[canLogBufPos++] = 0;
  // Epoch (uint32_t LE)
  canLogBufWriteU32(epoch);
  // Boot millis (uint32_t LE)
  canLogBufWriteU32(millis());

  canLogFlushBuf();
  canLogActive = true;
  canLogLastFlushMs = millis();

  Serial.printf("[CAN] Log started: %s\n", filename);
  diagLogf("CAN log %s", filename);
  return true;
}

void stopCanLog() {
  if (!canLogActive) return;
  canLogFlushBuf();
  if (canLogFile) {
    canLogFile.flush();
    canLogFile.close();
  }
  canLogActive = false;
  Serial.printf("[CAN] Log stopped — %u frames logged\n", stats.framesLogged);
  diagLogf("CAN log closed, %u frames", stats.framesLogged);
}

void flushCanLog() {
  if (!canLogActive) return;
  uint32_t now = millis();
  if (now - canLogLastFlushMs >= 500) {
    canLogFlushBuf();
    if (canLogFile) canLogFile.flush();
    canLogLastFlushMs = now;
  }
}

bool isCanLogActive() {
  return canLogActive;
}

// ===================== OBD2 Mode =====================

void enableObd2Mode() {
  if (currentMode == CAN_MODE_OBD2) return;

  // Reinstall TWAI in normal mode at 500 kbps (OBD2 standard speed).
  // Speed detection continues cycling and will reinstall at the right speed
  // if 500 kbps doesn't work.
  if (!installTwai(TWAI_MODE_NORMAL, 500)) {
    Serial.println("[CAN] OBD2 mode failed — TWAI reinstall error");
    return;
  }

  currentMode = CAN_MODE_OBD2;
  stats.mode = CAN_MODE_OBD2;
  obd2PidIndex = 0;
  obd2LastPollMs = 0;
  Serial.println("[CAN] Switched to OBD2 mode");
  diagLog("CAN OBD2 mode");
}

void tickObd2(uint32_t nowMs) {
  if (currentMode != CAN_MODE_OBD2) return;
  if (!twaiInstalled) return;

  // 100ms interval -> 10 Hz total, round-robin through PIDs
  if (nowMs - obd2LastPollMs < 100) return;
  obd2LastPollMs = nowMs;

  sendObd2Request(OBD2_PID_LIST[obd2PidIndex]);
  obd2PidIndex = (obd2PidIndex + 1) % OBD2_PID_COUNT;
}

// ===================== Main Loop Helpers =====================

void drainCanToLog(uint16_t maxFrames) {
  if (!canLogActive) {
    // Even if not logging, drain frames so ring doesn't fill
    CanFrame discard;
    uint16_t count = 0;
    while (count < maxFrames && ringPop(discard)) {
      count++;
    }
    return;
  }

  CanFrame frame;
  uint16_t count = 0;
  while (count < maxFrames && ringPop(frame)) {
    canLogWriteFrame(frame);
    stats.framesLogged++;
    count++;
  }
}

void canCheckSpeed(uint32_t nowMs) {
  if (speedDetectDone) return;
  if (!twaiInstalled) return;

  // Wait 3 seconds at each speed before trying the next
  if (nowMs - speedDetectStartMs < 3000) return;

  if (stats.framesReceived > 0) {
    // We're receiving frames — speed is correct
    speedDetectDone = true;
    Serial.printf("[CAN] Auto-detect: %u kbps confirmed (%u frames)\n",
                  currentSpeedKbps, stats.framesReceived);
    diagLogf("CAN speed %ukbps OK", currentSpeedKbps);
    return;
  }

  // Cycle between 500 and 250 kbps indefinitely until we get frames.
  // This handles the case where the car is turned on after the device boots.
  uint32_t nextSpeed = (currentSpeedKbps == 500) ? 250 : 500;
  Serial.printf("[CAN] No frames at %u kbps — trying %u kbps\n",
                currentSpeedKbps, nextSpeed);

  twai_mode_t mode = (currentMode == CAN_MODE_OBD2) ? TWAI_MODE_NORMAL : TWAI_MODE_LISTEN_ONLY;
  installTwai(mode, nextSpeed);
  speedDetectStartMs = nowMs;
  obd2LastPollMs = nowMs;  // prevent tickObd2 from TX-ing on the just-reinstalled driver
}

bool canLoopbackTest() {
  Serial.println("[CAN] === LOOPBACK SELF-TEST ===");
  Serial.println("[CAN] Stopping normal CAN, running test, then rebooting...");

  // Stop CAN receive task
  taskRunning = false;
  vTaskDelay(pdMS_TO_TICKS(100));
  esp_task_wdt_reset();

  if (twaiInstalled) {
    twai_stop();
    twai_driver_uninstall();
    twaiInstalled = false;
  }

  esp_task_wdt_reset();

  // Install in NO_ACK mode (self-test: TX without needing bus partner)
  twai_general_config_t gCfg = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NO_ACK);
  gCfg.rx_queue_len = 8;

  twai_timing_config_t tCfg = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t fCfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&gCfg, &tCfg, &fCfg);
  if (err != ESP_OK) {
    Serial.printf("[CAN] Test FAIL: TWAI install error 0x%X\n", err);
    delay(100);
    ESP.restart();
    return false;
  }

  err = twai_start();
  if (err != ESP_OK) {
    Serial.printf("[CAN] Test FAIL: TWAI start error 0x%X\n", err);
    twai_driver_uninstall();
    delay(100);
    ESP.restart();
    return false;
  }

  esp_task_wdt_reset();

  // Send a test frame with self-reception
  twai_message_t txMsg;
  memset(&txMsg, 0, sizeof(txMsg));
  txMsg.identifier = 0x7FF;
  txMsg.data_length_code = 4;
  txMsg.data[0] = 0xDE;
  txMsg.data[1] = 0xAD;
  txMsg.data[2] = 0xBE;
  txMsg.data[3] = 0xEF;
  txMsg.self = 1;  // self-reception request

  err = twai_transmit(&txMsg, pdMS_TO_TICKS(200));
  if (err != ESP_OK) {
    Serial.printf("[CAN] Test FAIL: TX error 0x%X (frame never sent)\n", err);
  } else {
    Serial.println("[CAN] Test TX sent (0x7FF: DEADBEEF)");
  }

  esp_task_wdt_reset();

  // Try to receive our own frame
  twai_message_t rxMsg;
  esp_err_t rxErr = twai_receive(&rxMsg, pdMS_TO_TICKS(500));

  esp_task_wdt_reset();

  // Get TWAI status for diagnosis
  twai_status_info_t info;
  twai_get_status_info(&info);
  Serial.printf("[CAN] Test TWAI: state=%d TEC=%u REC=%u busErr=%u txFail=%u\n",
    info.state, info.tx_error_counter, info.rx_error_counter,
    info.bus_error_count, info.tx_failed_count);

  if (rxErr == ESP_OK) {
    Serial.printf("[CAN] Test RX: id=0x%X dlc=%d data=%02X%02X%02X%02X\n",
      (unsigned)rxMsg.identifier, rxMsg.data_length_code,
      rxMsg.data[0], rxMsg.data[1], rxMsg.data[2], rxMsg.data[3]);
    if (rxMsg.identifier == 0x7FF && rxMsg.data[0] == 0xDE) {
      Serial.println("[CAN] Test PASS — ESP32 <-> transceiver wiring OK");
    } else {
      Serial.println("[CAN] Test FAIL — received unexpected data");
    }
  } else {
    Serial.printf("[CAN] Test FAIL — no frame received (err=0x%X)\n", rxErr);
    Serial.println("[CAN] Possible causes:");
    Serial.println("  1. IO2/IO3 not connected to SN65HVD230 CTX/CRX");
    Serial.println("  2. TX/RX wires swapped (IO2->CTX, IO3->CRX)");
    Serial.println("  3. SN65HVD230 no power (check 3.3V + GND)");
    Serial.println("  4. CANH/CANL shorted or open (need 60-120 ohm termination)");
  }

  // Clean reboot to restore normal operation
  Serial.println("[CAN] Rebooting to restore normal CAN...");
  Serial.flush();
  delay(100);
  ESP.restart();
  return false;  // unreachable
}

void printCanTwaiStatus() {
  if (!twaiInstalled) {
    Serial.println("[CAN] TWAI not installed");
    return;
  }
  twai_status_info_t info;
  if (twai_get_status_info(&info) != ESP_OK) {
    Serial.println("[CAN] Failed to get TWAI status");
    return;
  }
  const char *stateStr = "unknown";
  switch (info.state) {
    case TWAI_STATE_STOPPED:    stateStr = "STOPPED";    break;
    case TWAI_STATE_RUNNING:    stateStr = "RUNNING";    break;
    case TWAI_STATE_BUS_OFF:    stateStr = "BUS_OFF";    break;
    case TWAI_STATE_RECOVERING: stateStr = "RECOVERING"; break;
  }
  Serial.printf("[CAN] TWAI state=%s TEC=%u REC=%u | txQ=%u rxQ=%u | txFail=%u rxMiss=%u busErr=%u arbLost=%u\n",
    stateStr,
    info.tx_error_counter,
    info.rx_error_counter,
    info.msgs_to_tx,
    info.msgs_to_rx,
    info.tx_failed_count,
    info.rx_missed_count,
    info.bus_error_count,
    info.arb_lost_count);
}
