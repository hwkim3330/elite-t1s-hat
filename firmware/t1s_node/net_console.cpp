// See net_console.h.
//
// WiFi: with an SSID saved (`wifi <ssid> <pass>`) the board joins that network as a station
// and keeps retrying. With none, it opens its own AP "t1s-<tag>" (password below) so a PC with
// WiFi can still reach it at 192.168.4.1. Either way the console is `nc <ip> 23` and OTA is
// `arduino-cli upload --protocol network --port <ip> ...` (or espota.py) with the OTA password.
//
// The T1S / W5500 side keeps its own addresses; WiFi is a separate subnet, so test traffic never
// takes the WiFi path by accident.
#include "net_console.h"

#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WiFi.h>
#include <freertos/stream_buffer.h>
#include <esp_heap_caps.h>

#ifndef T1S_AP_PASS
#define T1S_AP_PASS "t1s-bench"   // WPA2 needs >= 8 chars; change with -DT1S_AP_PASS=...
#endif
#ifndef T1S_OTA_PASS
#define T1S_OTA_PASS "t1s-ota"    // default; `ota <pass>` + reboot to change
#endif

ConsoleTee Con;

static StreamBufferHandle_t sOut = nullptr;   // printed text waiting for the TCP client
static StreamBufferHandle_t sIn = nullptr;    // bytes typed on the TCP client
static SemaphoreHandle_t sOutLock = nullptr;  // a stream buffer takes one writer at a time
static char sSsid[33] = "", sPass[65] = "", sOtaPass[33] = T1S_OTA_PASS, sHost[32] = "t1s-node";
static volatile bool sOtaBusy = false, sClient = false;
static bool sWifiOff = false;   // `wifi off`: no radio at all (saved)
static volatile uint8_t sOtaPct = 0;

size_t ConsoleTee::write(uint8_t c) { return write(&c, 1); }

void ConsoleTee::begin() {
  if (!sOutLock) sOutLock = xSemaphoreCreateMutex();
}

// One write = one printf line (Print::printf formats first, then writes once). The lock keeps
// lines from two tasks whole on USB serial too: without it, a zenoh retry message printed while
// `blast` reported cut the report in half (seen on hardware).
size_t ConsoleTee::write(const uint8_t *buf, size_t n) {
  const bool locked = sOutLock && xSemaphoreTake(sOutLock, pdMS_TO_TICKS(200)) == pdTRUE;
  Serial.write(buf, n);
  if (sOut && sClient) xStreamBufferSend(sOut, buf, n, 0);  // never blocks: full = dropped
  if (locked) xSemaphoreGive(sOutLock);
  return n;
}

int netConsoleRead() {
  uint8_t c;
  if (sIn && xStreamBufferReceive(sIn, &c, 1, 0) == 1) return c;
  return -1;
}

static void loadNet() {
  Preferences p;
  p.begin("net", true);
  p.getString("ssid", sSsid, sizeof(sSsid));
  p.getString("pass", sPass, sizeof(sPass));
  if (p.isKey("ota")) p.getString("ota", sOtaPass, sizeof(sOtaPass));
  sWifiOff = p.getBool("off", false);
  p.end();
}

static void saveNet() {
  Preferences p;
  p.begin("net", false);
  p.putString("ssid", sSsid);
  p.putString("pass", sPass);
  p.putString("ota", sOtaPass);
  p.putBool("off", sWifiOff);
  p.end();
}

static void startOta() {
  ArduinoOTA.setHostname(sHost);
  ArduinoOTA.setPassword(sOtaPass);
  ArduinoOTA.setMdnsEnabled(true);
  ArduinoOTA.onStart([]() { sOtaBusy = true; Serial.println("ota: start"); });
  ArduinoOTA.onProgress([](unsigned done, unsigned total) {
    const uint8_t pct = total ? done * 100 / total : 0;
    if (pct / 10 != sOtaPct / 10) Serial.printf("ota: %u %%\n", pct);
    sOtaPct = pct;
  });
  ArduinoOTA.onEnd([]() { Serial.println("ota: done, rebooting"); });
  ArduinoOTA.onError([](ota_error_t e) { sOtaBusy = false; Serial.printf("ota: error %u\n", (unsigned)e); });
  ArduinoOTA.begin();
  MDNS.addService("telnet", "tcp", 23);
}

static void netTask(void *) {
  WiFiServer server(23);
  WiFiClient client;
  bool otaUp = false;
  uint32_t tRetry = 0;
  for (;;) {
    const bool sta = sSsid[0] != 0;
    const bool up = sta ? WiFi.status() == WL_CONNECTED : (WiFi.getMode() & WIFI_AP) != 0;
    if (sta && !up && millis() - tRetry > 10000) {  // keep rejoining; the AP may come back
      tRetry = millis();
      WiFi.disconnect();
      WiFi.begin(sSsid, sPass);
    }
    if (up && !otaUp) {
      startOta();
      server.begin();
      server.setNoDelay(true);
      otaUp = true;
      Con.printf("net: %s %s, console tcp/23, ota as %s\n", sta ? "joined" : "AP",
                 (sta ? WiFi.localIP() : WiFi.softAPIP()).toString().c_str(), sHost);
    }
    if (otaUp) {
      ArduinoOTA.handle();
      if (server.hasClient()) {
        WiFiClient nc = server.accept();
        if (client && client.connected()) client.stop();  // newest client wins
        client = nc;
        client.setNoDelay(true);
        xStreamBufferReset(sOut);
        sClient = true;
        client.printf("== %s console (type `help`) ==\n", sHost);
      }
      if (client && client.connected()) {
        uint8_t b[256];
        size_t n;
        while ((n = xStreamBufferReceive(sOut, b, sizeof(b), 0)) > 0) client.write(b, n);
        while (client.available()) {
          uint8_t c = client.read();
          xStreamBufferSend(sIn, &c, 1, 0);
        }
      } else if (sClient) {
        sClient = false;
        client.stop();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(sOtaBusy ? 1 : 10));
  }
}

void netConsoleBegin(const char *tag) {
  loadNet();
  snprintf(sHost, sizeof(sHost), "t1s-%s", tag);
  if (sWifiOff) { Serial.println("net: wifi off (saved) -- no WiFi console, no OTA; `wifi ap` to turn it back on"); return; }
  sOut = xStreamBufferCreate(4096, 1);
  sIn = xStreamBufferCreate(256, 1);
  Con.begin();
  // the task first, then WiFi (see the header)
  if (!sOut || !sIn || !sOutLock || xTaskCreate(netTask, "net_con", 6144, nullptr, 2, nullptr) != pdPASS) {
    Serial.println("net: task/buffer alloc FAILED -- no WiFi console, no OTA");
    return;
  }
  WiFi.setHostname(sHost);
  if (sSsid[0]) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  // a sleeping station misses OTA/console packets
    WiFi.begin(sSsid, sPass);
    Serial.printf("net: joining \"%s\" as %s\n", sSsid, sHost);
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(sHost, T1S_AP_PASS);
    Serial.printf("net: no wifi saved -> AP \"%s\" (pass %s), 192.168.4.1\n", sHost, T1S_AP_PASS);
  }
}

bool netConsoleCommand(const char *cmd, const char *a, const char *b, int n) {
  if (!strcmp(cmd, "wifi")) {
    if (n >= 1 && !strcmp(a, "off")) { sWifiOff = true; }
    else if (n >= 1 && !strcmp(a, "ap")) { sSsid[0] = sPass[0] = 0; sWifiOff = false; }
    else if (n >= 1) {
      strncpy(sSsid, a, sizeof(sSsid) - 1);
      strncpy(sPass, n >= 2 ? b : "", sizeof(sPass) - 1);
      sWifiOff = false;
    } else { netConsolePrintStatus(); return true; }
    saveNet();
    Con.printf("wifi: %s saved, applied on reboot\n", sWifiOff ? "OFF" : sSsid[0] ? sSsid : "own AP");
    return true;
  }
  if (!strcmp(cmd, "ota") && n >= 1) {
    strncpy(sOtaPass, a, sizeof(sOtaPass) - 1);
    saveNet();
    Con.println("ota: password saved, applied on reboot");
    return true;
  }
  return false;
}

void netConsolePrintStatus() {
  Con.printf("heap: internal %u B free (DMA-capable %u, min ever %u), psram %u\n",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  if (sWifiOff) { Con.println("wifi: OFF (saved)"); return; }
  if (sSsid[0])
    Con.printf("wifi: station on \"%s\", %s, ip %s, rssi %d dBm\n", sSsid,
               WiFi.status() == WL_CONNECTED ? "connected" : "NOT connected", WiFi.localIP().toString().c_str(),
               WiFi.RSSI());
  else
    Con.printf("wifi: own AP \"%s\", ip %s, %d client(s)\n", sHost, WiFi.softAPIP().toString().c_str(),
               WiFi.softAPgetStationNum());
  Con.printf("ota: %s (hostname %s)  console tcp/23: %s\n", sOtaBusy ? "IN PROGRESS" : "ready", sHost,
             sClient ? "client connected" : "no client");
}
