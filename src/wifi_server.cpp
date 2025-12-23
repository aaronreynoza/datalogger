#include "wifi_server.h"

#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>

#include "logging.h"

static const char *WIFI_AP_SSID = "tbeam-telemetry";
static const char *WIFI_AP_PASS = "tbeam123";

static WebServer server(80);

static void handleRoot() {
  String html =
    "<html><body><h1>T-Beam Telemetry</h1>"
    "<p><a href=\"/logs\">List logs (JSON)</a></p>"
    "<p>Download: /log?file=gps_001.csv</p>"
    "</body></html>";
  server.send(200, "text/html", html);
}

static void handleLogs() {
  File root = LittleFS.open("/");
  if (!root) {
    server.send(500, "text/plain", "Failed to open root");
    return;
  }
  String json = "[";
  bool first = true;
  File file = root.openNextFile();
  while (file) {
    String name = file.name();
    if ((name.startsWith("/gps_") || name.startsWith("/imu_")) &&
        name.endsWith(".csv")) {
      if (!first) json += ",";
      if (name.startsWith("/")) name.remove(0,1);
      json += "\"" + name + "\"";
      first = false;
    }
    file = root.openNextFile();
  }
  root.close();
  json += "]";
  server.send(200, "application/json", json);
}

static void handleLogDownload() {
  if (!server.hasArg("file")) {
    server.send(400, "text/plain", "Missing ?file=");
    return;
  }
  String fname = server.arg("file");
  if (!fname.startsWith("/")) fname = "/" + fname;
  if (!LittleFS.exists(fname)) {
    server.send(404, "text/plain", "File not found");
    return;
  }
  File f = LittleFS.open(fname, "r");
  server.streamFile(f, "text/csv");
  f.close();
}

void setupWiFi() {
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS)) {
    Serial.println("WiFi AP start FAILED");
    return;
  }
  IPAddress ip = WiFi.softAPIP();
  Serial.print("WiFi AP: "); Serial.print(WIFI_AP_SSID);
  Serial.print("  pass="); Serial.println(WIFI_AP_PASS);
  Serial.print("Open: http://"); Serial.println(ip.toString());

  server.on("/", handleRoot);
  server.on("/logs", HTTP_GET, handleLogs);
  server.on("/log",  HTTP_GET, handleLogDownload);
  server.begin();
  Serial.println("HTTP server started on port 80");
}

void handleServer() {
  server.handleClient();
}
