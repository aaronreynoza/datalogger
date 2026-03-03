#include "device_config.h"

#include <SD.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <cstring>

static const char *CONFIG_PATH = "/config.json";
static DeviceConfig config;

static void deriveSerial() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  snprintf(config.serial, sizeof(config.serial), "CP-%02X%02X%02X",
           mac[3], mac[4], mac[5]);
}

bool loadConfig() {
  memset(&config, 0, sizeof(config));
  deriveSerial();
  config.sessionType = 0;
  strlcpy(config.weather.conditions, "dry", sizeof(config.weather.conditions));

  File f = SD.open(CONFIG_PATH, "r");
  if (!f) {
    Serial.println("[CFG] No config.json — using defaults");
    return true;
  }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    Serial.print("[CFG] JSON parse error: ");
    Serial.println(err.c_str());
    return true;
  }

  if (doc["serial"].is<const char*>()) {
    strlcpy(config.serial, doc["serial"], sizeof(config.serial));
  }
  if (doc["driver"].is<const char*>()) {
    strlcpy(config.driverName, doc["driver"], sizeof(config.driverName));
  }
  if (doc["vehicle"].is<const char*>()) {
    strlcpy(config.vehicleName, doc["vehicle"], sizeof(config.vehicleName));
  }
  if (doc["session_type"].is<int>()) {
    config.sessionType = doc["session_type"];
  }
  if (doc["session_notes"].is<const char*>()) {
    strlcpy(config.sessionNotes, doc["session_notes"], sizeof(config.sessionNotes));
  }
  if (doc["weather_set"].is<bool>() && doc["weather_set"].as<bool>()) {
    config.weather.set = true;
    JsonObject w = doc["weather"];
    config.weather.ambientTempC = w["ambient_temp_c"] | 0.0f;
    config.weather.trackTempC = w["track_temp_c"] | 0.0f;
    const char *cond = w["conditions"] | "dry";
    strlcpy(config.weather.conditions, cond, sizeof(config.weather.conditions));
  }

  Serial.print("[CFG] Loaded — driver=\"");
  Serial.print(config.driverName);
  Serial.print("\" vehicle=\"");
  Serial.print(config.vehicleName);
  Serial.println("\"");
  return true;
}

void saveConfig() {
  JsonDocument doc;
  doc["serial"] = config.serial;
  doc["driver"] = config.driverName;
  doc["vehicle"] = config.vehicleName;
  doc["session_type"] = config.sessionType;
  doc["session_notes"] = config.sessionNotes;
  doc["weather_set"] = config.weather.set;
  if (config.weather.set) {
    JsonObject w = doc["weather"].to<JsonObject>();
    w["ambient_temp_c"] = config.weather.ambientTempC;
    w["track_temp_c"] = config.weather.trackTempC;
    w["conditions"] = config.weather.conditions;
  }

  File f = SD.open(CONFIG_PATH, "w");
  if (!f) {
    Serial.println("[CFG] Failed to write config.json");
    return;
  }
  serializeJsonPretty(doc, f);
  f.close();
}

const DeviceConfig &getConfig() {
  return config;
}

void setDriverName(const char *name) {
  strlcpy(config.driverName, name, sizeof(config.driverName));
  saveConfig();
}

void setVehicleName(const char *name) {
  strlcpy(config.vehicleName, name, sizeof(config.vehicleName));
  saveConfig();
}

void setSessionType(uint8_t type) {
  config.sessionType = type;
  saveConfig();
}

void setSessionNotes(const char *notes) {
  strlcpy(config.sessionNotes, notes, sizeof(config.sessionNotes));
  saveConfig();
}

void setWeather(float ambientC, float trackC, const char *conditions) {
  config.weather.ambientTempC = ambientC;
  config.weather.trackTempC = trackC;
  strlcpy(config.weather.conditions, conditions, sizeof(config.weather.conditions));
  config.weather.set = true;
  saveConfig();
}

void clearWeather() {
  config.weather.ambientTempC = 0;
  config.weather.trackTempC = 0;
  strlcpy(config.weather.conditions, "dry", sizeof(config.weather.conditions));
  config.weather.set = false;
  saveConfig();
}

uint8_t sessionTypeFromString(const char *str) {
  if (!str) return 1;
  if (strcmp(str, "practice") == 0) return 1;
  if (strcmp(str, "qualifying") == 0) return 2;
  if (strcmp(str, "race") == 0) return 3;
  if (strcmp(str, "test") == 0) return 4;
  return 1;  // default to practice
}
