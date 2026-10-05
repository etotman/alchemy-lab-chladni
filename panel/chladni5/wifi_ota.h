// wifi_ota.h - Wi-Fi and over-the-air updates for the ESP32 panel, with
// automatic rollback, so the panel can be reflashed without reaching the
// USB port behind the rack.
//
//   wifi_ota::begin(...)  in setup(), after the display is up
//   wifi_ota::tick()      every pass of loop()
//
// Credentials come from wifi_secrets.h (copy wifi_secrets.example.h; it is
// git-ignored).  Without that file this compiles to nothing and the sketch
// runs without Wi-Fi.
//
// UPDATING.  The panel runs a small web server and the sender PUSHES the
// image to it - only outgoing connections from the computer:
//
//   curl -F "firmware=@<build>/chladni5.ino.bin" http://alchemy-panel.local/update
//
// or open http://alchemy-panel.local/ in a browser and use the form.  This
// replaced ArduinoOTA/espota (2026-10-04): espota has the BOARD connect
// back into the Mac for the data, and on this Mac every such connection was
// refused (error "connect back to the sender") although ping worked and the
// firewall was off - inbound connections from the LAN don't reach the
// process that sends the update.  No password: anyone on the home network
// can reflash the panel.  server.authenticate() is the place to change that.
//
// ROLLBACK.  The bootloader has CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE.  A
// freshly updated image boots PENDING_VERIFY; the Arduino core normally
// marks it valid straight away, but verifyRollbackLater() below takes that
// decision over: the image is marked valid only once it has joined Wi-Fi
// and is serving /update for the NEXT update.  If it crashes first, or is
// not online within kVerifyMs, it restarts and the bootloader boots the
// previous image again.  An image flashed over USB is never pending, so
// none of this applies to it.
//
// FLASH WRITES.  A flash write scrambles the panel while it runs (Arduino's
// prebuilt ESP-IDF lacks CONFIG_SPIRAM_XIP_FROM_PSRAM).  So Wi-Fi runs
// with persistent(false) - credentials stay in RAM, nothing goes to NVS -
// and the only writes are the update itself and the one-off mark-valid
// after it.  Both are moments when a scrambled screen does not matter.
//
// POWER SAVE OFF.  With the default modem sleep the radio dozes between
// beacons: pings ran 128 ms average / 304 ms worst and update traffic went
// missing.  Off, 3.6 ms.  The panel runs off rack power anyway.

#pragma once
#include <Arduino.h>

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#define PANEL_HAS_WIFI 1
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <Update.h>
#else
#define PANEL_HAS_WIFI 0
#endif
#include <esp_ota_ops.h>

/* Take the "is this image good?" decision away from the Arduino core. */
extern "C" bool verifyRollbackLater() { return true; }

namespace wifi_ota {

const char    *kHost     = "alchemy-panel";
const uint32_t kVerifyMs = 90000ul;   /* a pending image must be online by then */

enum State : uint8_t { NO_SECRETS, CONNECTING, ONLINE, UPDATING };

static State  g_state = NO_SECRETS;
static bool   g_served = false;
static void (*g_on_start)() = nullptr;
static void (*g_on_progress)(uint8_t pct) = nullptr;
static void (*g_on_fail)() = nullptr;
static const char *g_name = "panel";
static const char *(*g_status)() = nullptr;

/* Optional: text served at /status, for reading the panel's state from
 * the Mac (curl http://alchemy-panel.local/status) - it has no USB in the
 * rack. Call before begin(). */
inline void onStatus(const char *(*fn)()) { g_status = fn; }

static bool pendingVerify() {
  esp_ota_img_states_t s;
  return esp_ota_get_state_partition(esp_ota_get_running_partition(), &s) == ESP_OK
      && s == ESP_OTA_IMG_PENDING_VERIFY;
}

static void markValid(const char *why) {
  if (!pendingVerify()) return;
  esp_ota_mark_app_valid_cancel_rollback();
  Serial.printf("ota: this image marked valid (%s)\n", why);
}

inline State state() { return g_state; }

inline String address() {
#if PANEL_HAS_WIFI
  if (g_state == ONLINE || g_state == UPDATING) return WiFi.localIP().toString();
#endif
  return String();
}

#if PANEL_HAS_WIFI
static WebServer server(80);
static String    g_error;

static void fail(const char *what) {
  if (!g_error.length()) g_error = what;
  Serial.printf("ota: update failed (%s), carrying on with this image\n", g_error.c_str());
  if (Update.isRunning()) Update.abort();
  g_state = ONLINE;
  if (g_on_fail) g_on_fail();
}

static void handleRoot() {
  const esp_partition_t *run = esp_ota_get_running_partition();
  String page = F("<!doctype html><meta name=viewport content='width=device-width'>"
                  "<title>alchemy-panel</title><body style='font-family:sans-serif;margin:2em'>"
                  "<h2>alchemy-panel</h2><p>Running <b>");
  page += g_name;
  page += F("</b>, built " __DATE__ " " __TIME__ ", from ");
  page += run ? run->label : "?";
  page += F("</p><form method=post action=/update enctype=multipart/form-data>"
            "<input type=file name=firmware accept=.bin> <input type=submit value=Update>"
            "</form><p>Upload the sketch's <code>.ino.bin</code>. The panel reboots into it and "
            "rolls back by itself if the new image cannot get back online.</p>");
  server.send(200, "text/html", page);
}

/* Called repeatedly while the multipart body streams in, all inside one
 * server.handleClient() - so the loop (bus, screen) stops until it is done. */
static void handleUpload() {
  HTTPUpload &up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    g_error = "";
    g_state = UPDATING;
    Serial.printf("ota: receiving %s\n", up.filename.c_str());
    if (g_on_start) g_on_start();
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) g_error = Update.errorString();
  }
  else if (up.status == UPLOAD_FILE_WRITE && !g_error.length()) {
    if (Update.write(up.buf, up.currentSize) != up.currentSize) g_error = Update.errorString();
    else if (g_on_progress) g_on_progress((uint8_t)min<size_t>(99, up.totalSize / 12000));  /* ~1.2 MB image */
  }
  else if (up.status == UPLOAD_FILE_END && !g_error.length()) {
    if (!Update.end(true)) g_error = Update.errorString();
    else Serial.printf("ota: received %u bytes\n", (unsigned)up.totalSize);
  }
  else if (up.status == UPLOAD_FILE_ABORTED) {
    g_error = "upload aborted";
  }
}

static void handleUpdateDone() {
  if (g_error.length() || Update.hasError() || !Update.isFinished()) {
    server.send(500, "text/plain", String("update failed: ") + (g_error.length() ? g_error : Update.errorString()) + "\n");
    fail(g_error.length() ? g_error.c_str() : Update.errorString());
    return;
  }
  server.send(200, "text/plain", "update ok, rebooting into it\n");
  Serial.println(F("ota: update done, rebooting"));
  delay(300);
  ESP.restart();
}
#endif

/* on_start runs as an update begins (the bus and the screen stop from
 * then on); on_progress gets 0-99 as it streams; on_fail runs when an
 * update is abandoned and this image carries on, so the screen can be
 * repainted.  All may be null.  `name` is shown on the web page. */
inline void begin(const char *name, void (*on_start)() = nullptr,
                  void (*on_progress)(uint8_t) = nullptr, void (*on_fail)() = nullptr) {
  g_name = name;
  g_on_start = on_start;
  g_on_progress = on_progress;
  g_on_fail = on_fail;
  const esp_partition_t *run = esp_ota_get_running_partition();
  Serial.printf("ota: running from %s%s\n", run ? run->label : "?",
                pendingVerify() ? " - NEW IMAGE, pending verify" : "");
#if PANEL_HAS_WIFI
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setHostname(kHost);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  g_state = CONNECTING;
  Serial.printf("wifi: joining \"%s\"\n", WIFI_SSID);
#else
  Serial.println(F("wifi: no wifi_secrets.h - Wi-Fi and OTA off"));
  markValid("no Wi-Fi configured");
#endif
}

inline void tick() {
#if PANEL_HAS_WIFI
  const bool up = WiFi.status() == WL_CONNECTED;
  if (up && g_state == CONNECTING) {
    g_state = ONLINE;
    Serial.printf("wifi: online as %s.local, %s\n", kHost, WiFi.localIP().toString().c_str());
    if (!g_served) {
      MDNS.begin(kHost);
      MDNS.addService("http", "tcp", 80);
      server.on("/", HTTP_GET, handleRoot);
      server.on("/update", HTTP_POST, handleUpdateDone, handleUpload);
      server.on("/status", HTTP_GET, [] {
        server.send(200, "text/plain", g_status ? g_status() : "no status\n");
      });
      server.begin();
      g_served = true;
      Serial.printf("ota: send updates to http://%s.local/update\n", kHost);
      markValid("online and serving /update");
    }
  }
  else if (!up && g_state == ONLINE) {
    g_state = CONNECTING;
    Serial.println(F("wifi: lost, reconnecting"));
  }
  if (g_served) server.handleClient();
#endif
  if (pendingVerify() && millis() > kVerifyMs) {
    Serial.println(F("ota: new image never got online - restarting to roll back"));
    delay(100);
    esp_restart();
  }
}

}  // namespace wifi_ota
