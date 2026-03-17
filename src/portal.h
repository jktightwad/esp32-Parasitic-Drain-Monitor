#pragma once

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "storage.h"

// ===== PORTAL STATE =====
static AsyncWebServer portalServer(80);
static bool           portalActive    = false;
static volatile bool  credsUpdated    = false;  // set true when new WiFi saved
static volatile bool  configUpdated   = false;  // set true when device config saved

// ===== HTML =====
static const char PORTAL_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>VoltMon Setup</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { font-family: sans-serif; background: #f0f0f0; padding: 16px; color: #222; }
    h1 { font-size: 1.3em; margin-bottom: 4px; color: #1a73e8; }
    h2 { font-size: 1.05em; margin: 20px 0 8px; color: #444; border-bottom: 1px solid #ccc; padding-bottom: 4px; }
    .card { background: white; border-radius: 8px; padding: 16px; margin-bottom: 16px; box-shadow: 0 1px 3px rgba(0,0,0,0.1); }
    label { display: block; font-size: 0.85em; color: #555; margin-bottom: 4px; margin-top: 10px; }
    input[type=text], input[type=password] {
      width: 100%; padding: 10px; border: 1px solid #ccc;
      border-radius: 6px; font-size: 1em;
    }
    input[type=text]:focus, input[type=password]:focus {
      outline: none; border-color: #1a73e8;
    }
    button {
      width: 100%; padding: 12px; margin-top: 14px;
      background: #1a73e8; color: white; border: none;
      border-radius: 6px; font-size: 1em; cursor: pointer;
    }
    button:active { background: #1558b0; }
    button.danger { background: #d93025; }
    button.danger:active { background: #a50e0e; }
    .net-item {
      display: flex; justify-content: space-between; align-items: center;
      padding: 8px 0; border-bottom: 1px solid #eee; font-size: 0.95em;
    }
    .net-item:last-child { border-bottom: none; }
    .del-btn {
      background: #d93025; color: white; border: none;
      border-radius: 4px; padding: 4px 10px; font-size: 0.8em; cursor: pointer;
      width: auto; margin-top: 0;
    }
    .msg { padding: 10px; border-radius: 6px; margin-bottom: 12px; font-size: 0.9em; }
    .msg.ok  { background: #e6f4ea; color: #137333; }
    .msg.err { background: #fce8e6; color: #c5221f; }
    .optional { color: #888; font-size: 0.8em; }
    .subtitle { font-size: 0.8em; color: #888; margin-bottom: 8px; }
  </style>
</head>
<body>
  <div class="card">
    <h1>&#9889; VoltMon Setup</h1>
    <p class="subtitle" id="devname"></p>
    %MSG%
  </div>

  %FIRST_SETUP%

  <div class="card" id="wifi-section">
    <h2>WiFi Networks</h2>
    <div id="net-list">%NET_LIST%</div>
    <h2>Add Network</h2>
    <form action="/addwifi" method="POST">
      <label>Network Name (SSID)</label>
      <input type="text" name="ssid" placeholder="Your WiFi name" required>
      <label>Password</label>
      <input type="password" name="pass" placeholder="Your WiFi password">
      <button type="submit">Add Network</button>
    </form>
  </div>

  <div class="card">
    <h2>Adafruit IO <span class="optional">(optional)</span></h2>
    <form action="/saveaio" method="POST">
      <label>Username</label>
      <input type="text" name="aio_user" placeholder="Adafruit IO username" value="%AIO_USER%">
      <label>Key</label>
      <input type="password" name="aio_key" placeholder="Adafruit IO key" value="%AIO_KEY%">
      <button type="submit">Save Adafruit IO</button>
    </form>
  </div>

  <div class="card">
    <h2>Device Name</h2>
    <form action="/savename" method="POST">
      <label>Friendly Name</label>
      <input type="text" name="name" placeholder="e.g. John's Truck" value="%DEV_NAME%" required>
      <button type="submit">Save Name</button>
    </form>
  </div>

  <script>
    var n = "%DEV_NAME%";
    if (n) document.getElementById("devname").textContent = "Device: " + n;
  </script>
</body>
</html>
)rawhtml";

static const char FIRST_SETUP_HTML[] PROGMEM = R"rawhtml(
  <div class="card">
    <h2>&#x1F6E0; First Time Setup</h2>
    <p style="font-size:0.9em;color:#555;margin-bottom:8px;">
      Give your device a name and add your first WiFi network to get started.
    </p>
    <form action="/firstsetup" method="POST">
      <label>Device Name</label>
      <input type="text" name="name" placeholder="e.g. John's Truck" required>
      <label>WiFi Network Name (SSID)</label>
      <input type="text" name="ssid" placeholder="Your WiFi name" required>
      <label>WiFi Password</label>
      <input type="password" name="pass" placeholder="Your WiFi password">
      <label>Adafruit IO Username <span class="optional">(optional)</span></label>
      <input type="text" name="aio_user" placeholder="Leave blank to skip">
      <label>Adafruit IO Key <span class="optional">(optional)</span></label>
      <input type="password" name="aio_key" placeholder="Leave blank to skip">
      <button type="submit">Save &amp; Connect</button>
    </form>
  </div>
)rawhtml";

// ===== BUILD NET LIST HTML =====
static String buildNetList(const WiFiCredentials& creds) {
  if (creds.count == 0) {
    return "<p style='color:#888;font-size:0.9em;'>No networks saved.</p>";
  }
  String html = "";
  for (int i = 0; i < creds.count; i++) {
    html += "<div class='net-item'>";
    html += "<span>" + String(creds.networks[i].ssid) + "</span>";
    html += "<form action='/delwifi' method='POST' style='margin:0'>";
    html += "<input type='hidden' name='idx' value='" + String(i) + "'>";
    html += "<button class='del-btn' type='submit'>Remove</button>";
    html += "</form>";
    html += "</div>";
  }
  return html;
}

// ===== BUILD PAGE =====
static String buildPage(const DeviceConfig& cfg, const WiFiCredentials& creds,
                         const String& msg = "", bool isError = false) {
  String html = String(PORTAL_HTML);

  // Message
  if (msg.length() > 0) {
    String msgHtml = "<div class='msg " + String(isError ? "err" : "ok") + "'>" + msg + "</div>";
    html.replace("%MSG%", msgHtml);
  } else {
    html.replace("%MSG%", "");
  }

  // First setup block — only show if not yet configured
  if (!cfg.configured) {
    html.replace("%FIRST_SETUP%", String(FIRST_SETUP_HTML));
  } else {
    html.replace("%FIRST_SETUP%", "");
  }

  html.replace("%NET_LIST%",  buildNetList(creds));
  html.replace("%DEV_NAME%",  String(cfg.deviceName));
  html.replace("%AIO_USER%",  String(cfg.aioUsername));
  html.replace("%AIO_KEY%",   "");  // never pre-fill key for security

  return html;
}

// ===== START PORTAL =====
void startPortal(DeviceConfig& cfg, WiFiCredentials& creds) {
  if (portalActive) return;

  // Build AP name
  String apName = (strlen(cfg.deviceName) > 0)
    ? "VoltMon-" + String(cfg.deviceName)
    : "VoltMon-Initialize";

  WiFi.softAP(apName.c_str(), "voltmon");
  Serial.println("Portal AP started: " + apName);
  Serial.println("IP: " + WiFi.softAPIP().toString());

  // ===== ROUTES =====

  // Root
  portalServer.on("/", HTTP_GET, [&cfg, &creds](AsyncWebServerRequest* req) {
    req->send(200, "text/html", buildPage(cfg, creds));
  });

  // First setup POST
  portalServer.on("/firstsetup", HTTP_POST, [&cfg, &creds](AsyncWebServerRequest* req) {
    String name    = req->hasParam("name",     true) ? req->getParam("name",     true)->value() : "";
    String ssid    = req->hasParam("ssid",     true) ? req->getParam("ssid",     true)->value() : "";
    String pass    = req->hasParam("pass",     true) ? req->getParam("pass",     true)->value() : "";
    String aioUser = req->hasParam("aio_user", true) ? req->getParam("aio_user", true)->value() : "";
    String aioKey  = req->hasParam("aio_key",  true) ? req->getParam("aio_key",  true)->value() : "";

    if (name.length() == 0 || ssid.length() == 0) {
      req->send(200, "text/html", buildPage(cfg, creds, "Device name and WiFi SSID are required.", true));
      return;
    }

    strlcpy(cfg.deviceName,  name.c_str(),    sizeof(cfg.deviceName));
    strlcpy(cfg.aioUsername, aioUser.c_str(), sizeof(cfg.aioUsername));
    strlcpy(cfg.aioKey,      aioKey.c_str(),  sizeof(cfg.aioKey));
    cfg.configured = true;
    saveConfig(cfg);

    addWiFiNetwork(creds, ssid.c_str(), pass.c_str());

    credsUpdated  = true;
    configUpdated = true;

    req->send(200, "text/html", buildPage(cfg, creds, "Setup saved! Attempting to connect..."));
  });

  // Add WiFi POST
  portalServer.on("/addwifi", HTTP_POST, [&cfg, &creds](AsyncWebServerRequest* req) {
    String ssid = req->hasParam("ssid", true) ? req->getParam("ssid", true)->value() : "";
    String pass = req->hasParam("pass", true) ? req->getParam("pass", true)->value() : "";

    if (ssid.length() == 0) {
      req->send(200, "text/html", buildPage(cfg, creds, "SSID cannot be empty.", true));
      return;
    }

    if (creds.count >= MAX_WIFI_NETWORKS) {
      req->send(200, "text/html", buildPage(cfg, creds, "Maximum networks reached. Remove one first.", true));
      return;
    }

    addWiFiNetwork(creds, ssid.c_str(), pass.c_str());
    credsUpdated = true;
    req->send(200, "text/html", buildPage(cfg, creds, "Network added. Attempting to connect..."));
  });

  // Remove WiFi POST
  portalServer.on("/delwifi", HTTP_POST, [&cfg, &creds](AsyncWebServerRequest* req) {
    if (!req->hasParam("idx", true)) {
      req->send(200, "text/html", buildPage(cfg, creds, "Invalid request.", true));
      return;
    }
    int idx = req->getParam("idx", true)->value().toInt();
    removeWiFiNetwork(creds, idx);
    req->send(200, "text/html", buildPage(cfg, creds, "Network removed."));
  });

  // Save AIO POST
  portalServer.on("/saveaio", HTTP_POST, [&cfg, &creds](AsyncWebServerRequest* req) {
    String aioUser = req->hasParam("aio_user", true) ? req->getParam("aio_user", true)->value() : "";
    String aioKey  = req->hasParam("aio_key",  true) ? req->getParam("aio_key",  true)->value() : "";

    strlcpy(cfg.aioUsername, aioUser.c_str(), sizeof(cfg.aioUsername));
    if (aioKey.length() > 0) {
      strlcpy(cfg.aioKey, aioKey.c_str(), sizeof(cfg.aioKey));
    }
    saveConfig(cfg);
    configUpdated = true;
    req->send(200, "text/html", buildPage(cfg, creds, "Adafruit IO credentials saved."));
  });

  // Save device name POST
  portalServer.on("/savename", HTTP_POST, [&cfg, &creds](AsyncWebServerRequest* req) {
    String name = req->hasParam("name", true) ? req->getParam("name", true)->value() : "";
    if (name.length() == 0) {
      req->send(200, "text/html", buildPage(cfg, creds, "Name cannot be empty.", true));
      return;
    }
    strlcpy(cfg.deviceName, name.c_str(), sizeof(cfg.deviceName));
    saveConfig(cfg);
    configUpdated = true;

    // Update AP name to reflect new device name
    String apName = "VoltMon-" + name;
    WiFi.softAP(apName.c_str(), "voltmon");

    req->send(200, "text/html", buildPage(cfg, creds, "Device name saved."));
  });

  // 404
  portalServer.onNotFound([](AsyncWebServerRequest* req) {
    req->redirect("/");
  });

  portalServer.begin();
  portalActive = true;
}

// ===== STOP PORTAL =====
void stopPortal() {
  if (!portalActive) return;
  portalServer.end();
  WiFi.softAPdisconnect(true);
  portalActive = false;
  Serial.println("Portal stopped");
}