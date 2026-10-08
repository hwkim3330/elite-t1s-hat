// Zone controller: this node drives MCU-less 10BASE-T1S endpoints (Microchip LAN866x) over OPEN Alliance TC18
// RCP, the way a car's zone controller drives its body endpoints, while a central computer only sends vehicle
// state.
//
//   state in    UDP 5008 (any interface: T1S or the board's RJ45), from the central computer, 20 Hz + on change
//   20 Hz loop  read the endpoint's thumbstick (MCP3204 over SPI) and proximity sensor (VCNL4200 over I2C) over
//               RCP; turn a stick flick into a gear request; draw the LED frame (zone_render.h) and send it
//   sensors out UDP 5009 to whoever sent the last state
//
// The endpoint sits in another IPv4 subnet than this node (e.g. 192.168.0.50 vs 192.168.100.65), so the zone gets
// an alias address (e.g. 192.168.0.65) handled below lwIP, without a second lwIP interface:
//  - ARP: answer "who has <alias>", resolve the endpoint's MAC with our own request
//  - UDP out: Ethernet/IPv4/UDP built here and handed to the MAC with esp_eth_transmit
//  - UDP in: frames to the alias are offered to zone::rx() from the driver's receive tap (before lwIP,
//    which drops them: not its address)
// Nothing is sent from the receive path itself (the driver's task): ARP replies are queued for the zone task.
//
// Packets (little-endian):
//   state   'VST2' u32 seq, u8 lamps (bit0 left lit, 1 right lit, 2 brake, 3 reverse, 4 hazard), char gear,
//           i16 speed x10 (km/h), u8 mode (0 manual, 1 autopilot, 2 autoware), u8 accel x100,
//           i16 steering wheel x10 (deg, + = left), u8 brake pedal x100                          = 17 bytes
//           ('VST1' = the first 14 bytes, no wheel / brake pedal)
//   sensors 'SNS1' u32 seq, u16 x, u16 y, u16 prox (0xFFFF = not read), u8 flags (bit0 I2C open, bit1 SPI
//           open, bit2 state fresh, bits 4-7 gear event counter), char last gear event (0 = none yet),
//           u16 frame loop work time this cycle (us), u16 failed RCP cycles so far               = 20 bytes
//           A new event = the counter changed, so a lost packet does not lose a gear change.
// Settings (`zone ip`, `zone on|off`, `zone bright`) are kept in NVS.
#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <esp_eth.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lwip/sockets.h>
#include "zone_render.h"
#include "zone_wtlv.h"

namespace zone {

static esp_eth_handle_t gZEth = nullptr;
static uint8_t gZMac[6];
static uint32_t gAlias = 0;          // network order
static uint32_t gTarget = 0;         // the endpoint (network order)
static uint8_t gTargetMac[6];
static volatile bool gTargetKnown = false;
static volatile uint32_t gArpAsked = 0, gArpAnswered = 0, gArpReplies = 0, gTxFrames = 0, gTxFail = 0, gRxUdp = 0;
// a pending ARP reply (filled by rx, sent by the task)
static volatile bool gArpPending = false;
static uint8_t gArpPeerMac[6];
static uint32_t gArpPeerIp = 0;
static uint16_t gIpId = 1;

// RCP: one request in flight. rx() copies a response only if its session is the one awaited.
static volatile uint16_t gRcpWant = 0;
static volatile uint32_t gRcpReplies = 0, gRcpLastLen = 0;
static volatile uint8_t gRcpLastRc = 0xFF;
static uint8_t gRcpLast[256];

// the loop
static volatile bool gWant = false;     // asked for (`zone on`, saved)
static volatile bool gRunning = false;  // the task's own view: handles may be open
static float gBright = 0.35f;
static Preferences gZPrefs;

static uint16_t csum(const uint8_t *p, int n) {
  uint32_t s = 0;
  for (int i = 0; i + 1 < n; i += 2) s += (p[i] << 8) | p[i + 1];
  if (n & 1) s += p[n - 1] << 8;
  while (s >> 16) s = (s & 0xFFFF) + (s >> 16);
  return (uint16_t)~s;
}

static bool tx(uint8_t *f, uint32_t len) {
  if (!gZEth) return false;
  const esp_err_t e = esp_eth_transmit(gZEth, f, len);
  if (e == ESP_OK) gTxFrames = gTxFrames + 1;
  else gTxFail = gTxFail + 1;
  return e == ESP_OK;
}

static void arp(uint16_t op, const uint8_t *dmac, uint32_t tip) {
  uint8_t f[60] = {0};
  memcpy(f, op == 1 ? (const uint8_t *)"\xff\xff\xff\xff\xff\xff" : dmac, 6);
  memcpy(f + 6, gZMac, 6);
  f[12] = 0x08; f[13] = 0x06;
  const uint8_t hdr[8] = {0, 1, 8, 0, 6, 4, 0, (uint8_t)op};
  memcpy(f + 14, hdr, 8);
  memcpy(f + 22, gZMac, 6);
  memcpy(f + 28, &gAlias, 4);
  if (op == 2) memcpy(f + 32, dmac, 6);
  memcpy(f + 38, &tip, 4);
  tx(f, 60);
}

// UDP to the endpoint from the alias. The payload sits after a 42-byte header in buf (caller provides room).
static bool udp(uint8_t *buf, uint16_t sport, uint16_t dport, uint16_t plen) {
  if (!gTargetKnown) return false;
  uint8_t *e = buf, *ip = buf + 14, *u = buf + 34;
  memcpy(e, gTargetMac, 6);
  memcpy(e + 6, gZMac, 6);
  e[12] = 0x08; e[13] = 0x00;
  const uint16_t tot = 20 + 8 + plen;
  ip[0] = 0x45; ip[1] = 0; ip[2] = tot >> 8; ip[3] = tot & 0xFF;
  ip[4] = gIpId >> 8; ip[5] = gIpId & 0xFF; gIpId++;
  ip[6] = 0x40; ip[7] = 0;                       // DF
  ip[8] = 64; ip[9] = 17; ip[10] = ip[11] = 0;
  memcpy(ip + 12, &gAlias, 4);
  memcpy(ip + 16, &gTarget, 4);
  const uint16_t c = csum(ip, 20);
  ip[10] = c >> 8; ip[11] = c & 0xFF;
  u[0] = sport >> 8; u[1] = sport & 0xFF; u[2] = dport >> 8; u[3] = dport & 0xFF;
  u[4] = (8 + plen) >> 8; u[5] = (8 + plen) & 0xFF; u[6] = u[7] = 0;   // UDP checksum optional on IPv4
  uint32_t len = 42 + plen;
  if (len < 60) { memset(buf + len, 0, 60 - len); len = 60; }
  return tx(buf, len);
}

// from the driver's receive tap: ARP for the alias / from the target, UDP to the alias
static void rx(esp_eth_handle_t h, const uint8_t *f, uint32_t len) {
  if (h != gZEth || !gAlias || len < 42) return;
  if (f[12] == 0x08 && f[13] == 0x06) {
    const uint16_t op = (f[20] << 8) | f[21];
    uint32_t sip, tip;
    memcpy(&sip, f + 28, 4);
    memcpy(&tip, f + 38, 4);
    if (op == 1 && tip == gAlias && !gArpPending) {
      memcpy(gArpPeerMac, f + 22, 6);
      gArpPeerIp = sip;
      gArpPending = true;
      gArpAsked = gArpAsked + 1;
    }
    if (sip == gTarget) {                         // a reply from the endpoint (or its own request): learn it
      memcpy(gTargetMac, f + 22, 6);
      gTargetKnown = true;
      if (op == 2) gArpReplies = gArpReplies + 1;
    }
    return;
  }
  if (f[12] == 0x08 && f[13] == 0x00 && f[23] == 17 && !memcmp(f + 30, &gAlias, 4)) {
    gRxUdp = gRxUdp + 1;
    const int ihl = (f[14] & 0x0F) * 4;
    const uint8_t *u = f + 14 + ihl;
    const uint16_t sport = (u[0] << 8) | u[1];
    if (sport == 49153 && len >= (uint32_t)(14 + ihl + 8 + 16)) {   // an RCP (SOME/IP) response
      const uint8_t *sip = u + 8;
      const uint16_t sess = (sip[10] << 8) | sip[11];
      if (!gRcpWant || sess != gRcpWant) return;   // late answer to a request we gave up on
      const uint32_t n = len - (14 + ihl + 8);
      gRcpLastLen = n < sizeof(gRcpLast) ? n : sizeof(gRcpLast);
      memcpy(gRcpLast, sip, gRcpLastLen);
      gRcpLastRc = sip[15];
      gRcpWant = 0;
      gRcpReplies = gRcpReplies + 1;
    }
  }
}

// ---- the LED panels: one 20x10 frame as RTP (RFC 4175 raw RGB, PT 96) to <target>:5001
static zr_fb gFb;
static uint32_t gRtpSeq = 0;
static bool sendFrame(const zr_fb fb) {
  static uint8_t buf[42 + 14 + 60 + 600];
  uint8_t *p = buf + 42;
  int n = 0;
  const uint32_t ts = (uint32_t)(esp_timer_get_time() / 10);   // 100 kHz, as the PC sends it
  p[n++] = 0x80; p[n++] = 0x80 | 96;
  p[n++] = gRtpSeq >> 8; p[n++] = gRtpSeq & 0xFF;
  p[n++] = ts >> 24; p[n++] = ts >> 16; p[n++] = ts >> 8; p[n++] = ts;
  p[n++] = 'Z'; p[n++] = 'O'; p[n++] = 'N'; p[n++] = 'E';          // SSRC
  p[n++] = gRtpSeq >> 24; p[n++] = gRtpSeq >> 16;                  // extended sequence (high 16 bits)
  for (int y = 0; y < ZR_H; y++) {
    p[n++] = 0; p[n++] = ZR_W * 3; p[n++] = 0; p[n++] = y; p[n++] = y < ZR_H - 1 ? 0x80 : 0; p[n++] = 0;
  }
  memcpy(p + n, fb, sizeof(zr_fb));
  n += sizeof(zr_fb);
  gRtpSeq++;
  return udp(buf, 5001, 5001, n);
}

// ---- RCP client (zone_wtlv.h): one request, wait for its response; rc 0 = OK, 0xFE = timeout
static uint16_t gSess = 1;
static uint32_t gRcpCalls = 0, gRcpTimeouts = 0, gRcpUsMax = 0;
static uint64_t gRcpUsSum = 0;
static uint8_t call(uint16_t method, const uint8_t *pl, int n, int ms = 300) {
  static uint8_t buf[42 + 16 + 96];
  if (n > 96) return 0xFD;
  gSess = gSess == 0xFFFF ? 1 : gSess + 1;
  const uint16_t sess = gSess;
  zw_header(buf + 42, method, sess, n);
  if (n) memcpy(buf + 58, pl, n);
  const uint32_t before = gRcpReplies;
  const int64_t t0 = esp_timer_get_time();
  gRcpWant = sess;
  udp(buf, 50000, 6800, 16 + n);
  gRcpCalls++;
  // poll every 1 ms tick (a round trip is ~1.9 ms on the bus)
  for (int i = 0; i < ms; i++) {
    if (gRcpReplies != before) {
      const uint32_t us = (uint32_t)(esp_timer_get_time() - t0);
      gRcpUsSum += us;
      if (us > gRcpUsMax) gRcpUsMax = us;
      return gRcpLastRc;
    }
    vTaskDelay(1);
  }
  gRcpWant = 0;
  gRcpTimeouts++;
  return 0xFE;
}
// field f of the last response's payload
static const uint8_t *field(int f, int &n) {
  return gRcpLastLen > 16 ? zw_field(gRcpLast + 16, gRcpLastLen - 16, f, &n) : nullptr;
}

static uint16_t gI2c = 0xFFFF, gSpi = 0xFFFF;
static uint32_t gWid[2];
static uint32_t gReopens = 0;
// open; if the peripheral is still held by a handle nobody closed (a client that died, or this node before a
// reset), close the handles in its range (I2C 0x100.., SPI 0x300..) and open again
static uint16_t openWithRecovery(uint16_t method, const uint8_t *pl, int n, uint16_t first, uint16_t closeMethod) {
  for (int attempt = 0; attempt < 2; attempt++) {
    const uint8_t rc = call(method, pl, n);
    int fn = 0;
    const uint8_t *v = rc == 0 ? field(0, fn) : nullptr;
    if (v && fn == 2) return (v[0] << 8) | v[1];
    if (rc != 5) {
      if (rc == 0xFE && attempt == 0) continue;     // lost: one retry
      break;
    }
    Con.printf("zone: open 0x%04x: not reachable (held by a stale handle) -- closing 0x%03x..0x%03x\n", method, first, first + 23);
    uint8_t c[4];
    for (uint16_t h = first; h < first + 24; h++) call(closeMethod, c, zw_close(c, h), 50);
  }
  return 0xFFFF;
}
static void sensorsOpen() {
  uint8_t p[48];
  int n;
  if (gI2c == 0xFFFF) {
    gI2c = openWithRecovery(ZW_OPEN_I2C, p, zw_open_i2c(p, 8, 9, 1), 0x100, ZW_CLOSE_I2C);
    gWid[0] = 0;
    if (gI2c != 0xFFFF) {
      const uint8_t init[3][3] = {{0x03, 0x02, 0x08}, {0x04, 0x01, 0x07}, {0x00, 0x00, 0x00}};   // PS fast, 16 bit, LED 200 mA
      for (auto &w : init) {
        n = zw_i2c_write(p, gI2c, 0x51, gWid[0]++, w, 3);
        call(ZW_WRITE_I2C, p, n);
      }
    }
  }
  if (gSpi == 0xFFFF) {
    const uint8_t pins[4] = {12, 13, 14, 15};
    call(ZW_RELEASE_PINS, p, zw_release(p, pins, 4));
    gSpi = openWithRecovery(ZW_OPEN_SPI, p, zw_open_spi(p, 12, 13, 14, 15, 1, 1923000), 0x300, ZW_CLOSE_SPI);
    gWid[1] = 0;
  }
  Con.printf("zone: sensors: i2c 0x%04x, spi 0x%04x%s\n", gI2c, gSpi, gI2c != 0xFFFF && gSpi != 0xFFFF ? "" : " -- retry in 2 s");
}
static void sensorsClose() {
  uint8_t p[4];
  if (gI2c != 0xFFFF) call(ZW_CLOSE_I2C, p, zw_close(p, gI2c));
  if (gSpi != 0xFFFF) call(ZW_CLOSE_SPI, p, zw_close(p, gSpi));
  gI2c = gSpi = 0xFFFF;
}
// one transfer with a WriteId: a lost request is sent again with the same WriteId (the endpoint takes a
// repeated one as a resend), so the sequence stays in step either way. rc 32 = out of step: reopen the handle.
static const uint8_t *xfer(int k, uint16_t method, uint8_t *p, int n, int &fn, int ms) {
  uint8_t rc = call(method, p, n, ms);
  if (rc == 0xFE) rc = call(method, p, n, ms);
  gWid[k]++;
  if (rc == 32) {
    Con.printf("zone: %s WriteId out of step -- reopening\n", k ? "spi" : "i2c");
    uint8_t c[4];
    call(k ? ZW_CLOSE_SPI : ZW_CLOSE_I2C, c, zw_close(c, k ? gSpi : gI2c), 50);
    (k ? gSpi : gI2c) = 0xFFFF;
    gReopens++;
  }
  return rc == 0 ? field(1, fn) : nullptr;
}
static int readProx(int ms = 300) {
  uint8_t p[48];
  const uint8_t reg = 0x08;
  int fn = 0;
  const uint8_t *v = xfer(0, ZW_WRITE_READ_I2C, p, zw_i2c_wr(p, gI2c, 0x51, 2, gWid[0], &reg, 1), fn, ms);
  return v && fn >= 2 ? v[0] | (v[1] << 8) : -1;
}
static int readAdc(int ch, int ms = 300) {
  uint8_t p[48];
  const uint8_t cmd[3] = {0x06, (uint8_t)(ch << 6), 0xFF};
  int fn = 0;
  const uint8_t *v = xfer(1, ZW_WRITE_READ_SPI, p, zw_spi_xfer(p, gSpi, gWid[1], cmd, 3), fn, ms);
  return v && fn >= 3 ? ((v[1] & 0x0F) << 8) | v[2] : -1;
}

// ---- state in / sensors out
static int gSock = -1;
static zr_state gSt;
static uint32_t gStAt = 0, gStCount = 0, gStBad = 0, gStSeq = 0;   // millis of the last state
static bool gStFresh = false;
static sockaddr_in gPeer;
static bool gPeerKnown = false;
static int64_t gEdgeL = 0, gEdgeR = 0;    // us: when that lamp's lit phase began (arrow sweep)
static uint8_t gPrevLamps = 0;
static int gX = -1, gY = -1, gProx = -1;
static bool gArmed = true;
static char gEvent = 0;
static uint8_t gEventCount = 0;
static uint32_t gSnsSeq = 0, gLoops = 0, gFails = 0, gBusyUs = 0, gBusyMax = 0, gOverruns = 0;
static uint64_t gBusySum = 0;
static uint32_t gFrames = 0;

// the vehicle state, from VST2 (stateIn) or from VSS signals over SOME/IP (sdv.h, another task): the frame task
// copies it under the same lock
static portMUX_TYPE gStMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t gVstAt = 0;          // millis of the last VST1/VST2: SNS1 goes back to its sender for 3 s after it
static void applyState(uint8_t lamps, char gear, double speed, uint8_t mode, double accel, double wheel, double brake) {
  const int64_t now = esp_timer_get_time();
  portENTER_CRITICAL(&gStMux);
  gSt.lamps = lamps & 0x0F;
  gSt.gear = gear;
  gSt.speed = speed;
  gSt.mode = mode;
  gSt.accel = accel;
  gSt.wheel_deg = wheel;
  gSt.brake = brake;
  if ((lamps & ZR_LEFT) && !(gPrevLamps & ZR_LEFT)) gEdgeL = now;
  if ((lamps & ZR_RIGHT) && !(gPrevLamps & ZR_RIGHT)) gEdgeR = now;
  gPrevLamps = lamps;
  gStAt = millis();
  gStCount++;
  portEXIT_CRITICAL(&gStMux);
}

static void stateIn() {
  if (gSock < 0) {
    gSock = socket(AF_INET, SOCK_DGRAM, 0);
    if (gSock < 0) return;
    sockaddr_in a = {};
    a.sin_family = AF_INET;
    a.sin_port = htons(5008);
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(gSock, (sockaddr *)&a, sizeof(a)) < 0) { close(gSock); gSock = -1; return; }
  }
  uint8_t b[64];
  sockaddr_in from;
  socklen_t fl = sizeof(from);
  int n;
  while ((n = recvfrom(gSock, b, sizeof(b), MSG_DONTWAIT, (sockaddr *)&from, &fl)) > 0) {
    const bool v2 = n >= 17 && !memcmp(b, "VST2", 4), v1 = n >= 14 && !memcmp(b, "VST1", 4);
    fl = sizeof(from);
    if (!v1 && !v2) { gStBad++; continue; }
    int16_t speed10, wheel10 = 0;
    memcpy(&gStSeq, b + 4, 4);
    memcpy(&speed10, b + 10, 2);
    if (v2) memcpy(&wheel10, b + 14, 2);
    applyState(b[8], (char)b[9], speed10 / 10.0, b[12], b[13] / 100.0, v2 ? wheel10 / 10.0 : 0, v2 ? b[16] / 100.0 : 0);
    gVstAt = millis();
    gPeer = from;
    gPeer.sin_port = htons(5009);
    gPeerKnown = true;
  }
}

static void sensorsOut(uint16_t busyUs) {
  if (!gPeerKnown || gSock < 0 || millis() - gVstAt > 3000) return;   // the VSS path (sdv.h) replaced VST2/SNS1
  uint8_t b[20];
  memcpy(b, "SNS1", 4);
  const uint32_t seq = gSnsSeq++;
  memcpy(b + 4, &seq, 4);
  const uint16_t x = gX < 0 ? 0xFFFF : gX, y = gY < 0 ? 0xFFFF : gY, pr = gProx < 0 ? 0xFFFF : gProx;
  const uint16_t fails = gFails > 0xFFFF ? 0xFFFF : gFails;
  memcpy(b + 8, &x, 2);
  memcpy(b + 10, &y, 2);
  memcpy(b + 12, &pr, 2);
  b[14] = (gI2c != 0xFFFF ? 1 : 0) | (gSpi != 0xFFFF ? 2 : 0) | (gStFresh ? 4 : 0) | (gEventCount << 4);
  b[15] = (uint8_t)gEvent;
  memcpy(b + 16, &busyUs, 2);
  memcpy(b + 18, &fails, 2);
  sendto(gSock, b, sizeof(b), 0, (sockaddr *)&gPeer, sizeof(gPeer));
}

// the gear selector: one flick = one shift (up D, down R, left P, right N), re-armed back inside the centre
// Only fresh samples from a fully successful cycle count, and a shift needs the same direction in two cycles in a
// row: a wrong gear sent to the car is worse than a late one (+50 ms).
static void gearFlick(int x, int y) {
  static char pending = 0;
  if (x < 0 || y < 0) { pending = 0; return; }
  const float jx = (x - 2048) / 2048.0f, jy = (y - 2048) / 2048.0f;
  const char g = jy > 0.6f ? 'D' : jy < -0.6f ? 'R' : jx < -0.6f ? 'P' : jx > 0.6f ? 'N' : 0;
  const bool confirmed = g && g == pending;
  pending = g;
  if (confirmed && gArmed) {
    gEvent = g;
    gEventCount = (gEventCount + 1) & 0x0F;
    gArmed = false;
    Con.printf("zone: gear %c (stick x %d y %d)\n", g, x, y);
  } else if (fabsf(jx) < 0.25f && fabsf(jy) < 0.25f) {
    gArmed = true;
  }
}

// ---- two tasks, so the LED panels never wait on RCP (a transfer can take up to 2 x its timeout when the bus
// loses frames, and a stale-handle recovery much longer):
//   zone      fixed 50 ms schedule: state in, ARP, the LED frame, sensors out (the latest readings)
//   zone_rcp  the sensor reads, paced to 50 ms when all is well; after a failed cycle it waits longer (100 ms,
//             doubling up to 800 ms) so a lossy bus is not loaded further; opens/closes the handles
static volatile uint32_t gRcpCycleUs = 0;
static uint32_t gRcpCycles = 0, gRcpCycleMax = 0;   // written by the RCP task only

static void rcpTask(void *) {
  uint32_t lastOpen = 0, run = 0, backoff = 0;
  for (;;) {
    const TickType_t t0 = xTaskGetTickCount();
    if (!gWant || !gTargetKnown) {
      if (gI2c != 0xFFFF || gSpi != 0xFFFF) sensorsClose();   // `zone off`: give the endpoint back
      gX = gY = gProx = -1;
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    const int64_t c0 = esp_timer_get_time();
    if ((gI2c == 0xFFFF || gSpi == 0xFFFF) && millis() - lastOpen > 2000) {
      lastOpen = millis();
      sensorsOpen();
    }
    bool ok = true;
    int x = -1, y = -1, pr = -1;
    if (gSpi != 0xFFFF) {
      x = readAdc(0, 25);
      if (x >= 0) y = readAdc(1, 25);
      ok = x >= 0 && y >= 0;
    }
    if (ok && gI2c != 0xFFFF) {
      pr = readProx(25);
      ok = pr >= 0;
    }
    if (ok) {
      gX = x; gY = y; gProx = pr;
      run = 0;
      backoff = 0;
    } else {
      if (run >= 3) gX = gY = gProx = -1;   // a single lost cycle keeps the last readings (no flicker on the panels)
      gFails++;
      backoff = backoff ? (backoff < 800 ? backoff * 2 : 800) : 100;
      if (++run > 10) {               // ~10 failed cycles in a row (seconds, with the back-off): reopen
        run = 0;
        Con.println("zone: no sensor answers -- reopening");
        gI2c = gSpi = 0xFFFF;
        gReopens++;
      }
    }
    gearFlick(ok ? x : -1, ok ? y : -1);
    const uint32_t us = (uint32_t)(esp_timer_get_time() - c0);
    gRcpCycles++;
    gRcpCycleUs = us;
    if (us > gRcpCycleMax) gRcpCycleMax = us;
    TickType_t wake = t0;
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(backoff ? backoff : 50));
  }
}

static void frameOnce() {
  zr_fb fb;
  gStFresh = gStCount && millis() - gStAt < 500;
  static bool wasFresh = false;
  if (gStFresh != wasFresh) {
    Con.printf("zone: vehicle state %s\n", gStFresh ? "arriving" : "lost (no update for 0.5 s): \"no link\" on the panels");
    wasFresh = gStFresh;
  }
  if (gStFresh) {
    portENTER_CRITICAL(&gStMux);
    zr_state s = gSt;
    const int64_t edgeL = gEdgeL, edgeR = gEdgeR;
    portEXIT_CRITICAL(&gStMux);
    const int prox = gProx;
    s.prox_m = prox < 0 ? -1 : rint(zr_prox_m(prox) * 100) / 100;   // the PC rounds to cm too
    const int64_t now = esp_timer_get_time();
    zr_car(fb, &s, (now - edgeL) / 1e6, (now - edgeR) / 1e6);
  } else {
    zr_nolink(fb, gFrames);
  }
  zr_scale(gFb, (const uint8_t(*)[ZR_W][3])fb, zr_bright(gBright));   // (a plain float was one step darker: 46 px)
  if (sendFrame(gFb)) gFrames++;
}

static void task(void *) {
  uint32_t lastArp = 0;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(50));
    if (xTaskGetTickCount() - wake > pdMS_TO_TICKS(50)) {   // fell behind: do not burst
      wake = xTaskGetTickCount();
      gOverruns++;
    }
    const int64_t t0 = esp_timer_get_time();
    stateIn();
    if (!gAlias) continue;
    if (gArpPending) {
      uint8_t m[6];
      memcpy(m, gArpPeerMac, 6);
      const uint32_t ip = gArpPeerIp;
      gArpPending = false;
      arp(2, m, ip);
      gArpAnswered = gArpAnswered + 1;
    }
    if (gTarget && (!gTargetKnown || millis() - lastArp > 30000) && millis() - lastArp > 1000) {
      arp(1, nullptr, gTarget);
      lastArp = millis();
    }
    if (!gWant) {
      if (gRunning) {                 // `zone off`: panels cleared here, handles closed by the RCP task
        zr_fb off = {};
        sendFrame(off);
        gRunning = false;
        Con.println("zone: stopped (panels cleared, RCP handles closed)");
      }
      continue;
    }
    if (!gTargetKnown) continue;
    if (!gRunning) {
      gRunning = true;
      Con.println("zone: running (20 Hz: state in UDP 5008, LED frame, sensors out UDP 5009; RCP sensor reads in their own task)");
    }
    frameOnce();
    const uint32_t us = (uint32_t)(esp_timer_get_time() - t0);
    gLoops++;
    gBusyUs = us;
    gBusySum += us;
    if (us > gBusyMax) gBusyMax = us;
    sensorsOut(us > 0xFFFF ? 0xFFFF : us);
  }
}

static void save() {
  gZPrefs.begin("zone", false);
  gZPrefs.putULong("alias", gAlias);
  gZPrefs.putULong("target", gTarget);
  gZPrefs.putBool("on", gWant);
  gZPrefs.putFloat("bright", gBright);
  gZPrefs.end();
}

static void begin(esp_eth_handle_t h) {
  gZEth = h;
  esp_eth_ioctl(h, ETH_CMD_G_MAC_ADDR, gZMac);
  gZPrefs.begin("zone", true);
  gAlias = gZPrefs.getULong("alias", 0);
  gTarget = gZPrefs.getULong("target", 0);
  gWant = gZPrefs.getBool("on", false) && gAlias && gTarget;
  gBright = gZPrefs.getFloat("bright", 0.35f);
  gZPrefs.end();
  static bool started = false;
  if (!started) {
    started = true;
    xTaskCreate(task, "zone", 6144, nullptr, 4, nullptr);
    xTaskCreate(rcpTask, "zone_rcp", 6144, nullptr, 3, nullptr);
  }
  if (gAlias)
    Con.printf("zone: alias " IPSTR " -> endpoint " IPSTR " (saved), loop %s\n", IP2STR((esp_ip4_addr_t *)&gAlias),
               IP2STR((esp_ip4_addr_t *)&gTarget), gWant ? "on" : "off");
}

static void status() {
  Con.printf("zone: %s, alias " IPSTR ", endpoint " IPSTR " %s %02x:%02x:%02x:%02x:%02x:%02x, bright %.2f\n",
             gRunning ? "running" : gWant ? "on (waiting for the endpoint)" : "off",
             IP2STR((esp_ip4_addr_t *)&gAlias), IP2STR((esp_ip4_addr_t *)&gTarget), gTargetKnown ? "at" : "(unresolved)",
             gTargetMac[0], gTargetMac[1], gTargetMac[2], gTargetMac[3], gTargetMac[4], gTargetMac[5], gBright);
  Con.printf("zone: arp asked %lu answered %lu, endpoint replies %lu; tx %lu (fail %lu) frames, udp to alias %lu\n",
             (unsigned long)gArpAsked, (unsigned long)gArpAnswered, (unsigned long)gArpReplies,
             (unsigned long)gTxFrames, (unsigned long)gTxFail, (unsigned long)gRxUdp);
  Con.printf("zone: frame loop %lu, work avg %lu us max %lu us, overruns %lu; LED frames %lu\n",
             (unsigned long)gLoops, (unsigned long)(gLoops ? gBusySum / gLoops : 0), (unsigned long)gBusyMax,
             (unsigned long)gOverruns, (unsigned long)gFrames);
  Con.printf("zone: rcp loop %lu, last %lu us max %lu us; sensors x %d y %d prox %d, i2c 0x%04x spi 0x%04x, "
             "failed cycles %lu, reopens %lu\n",
             (unsigned long)gRcpCycles, (unsigned long)gRcpCycleUs, (unsigned long)gRcpCycleMax, gX, gY, gProx, gI2c, gSpi,
             (unsigned long)gFails, (unsigned long)gReopens);
  Con.printf("zone: rcp calls %lu, timeouts %lu, round trip avg %lu us max %lu us; state in %lu (bad %lu), %s, "
             "sensors out %lu; last gear %c (#%u)\n",
             (unsigned long)gRcpCalls, (unsigned long)gRcpTimeouts,
             (unsigned long)(gRcpCalls > gRcpTimeouts ? gRcpUsSum / (gRcpCalls - gRcpTimeouts) : 0), (unsigned long)gRcpUsMax,
             (unsigned long)gStCount, (unsigned long)gStBad,
             gStCount ? (gStFresh ? "fresh" : "stale") : "none yet", (unsigned long)gSnsSeq, gEvent ? gEvent : '-', gEventCount);
}

// zone [status] | on | off | ip <alias> [endpoint] | bright <0..1> | reset | rcp | sense [n] [keep]
static void command(const char *a) {
  char s1[20], s2[20];
  float f;
  if (!*a || !strcmp(a, "status")) return status();
  const int nip = sscanf(a, "ip %19s %19s", s1, s2);
  if (nip >= 1) {
    IPAddress A, T(192, 168, 0, 50);
    if (!A.fromString(s1) || (nip == 2 && !T.fromString(s2))) { Con.println("zone: bad address"); return; }
    gAlias = (uint32_t)A;
    gTarget = (uint32_t)T;
    gTargetKnown = false;
    save();
    return status();
  }
  if (!strcmp(a, "on")) {
    if (!gAlias || !gTarget) { Con.println("zone: set the addresses first: zone ip <alias> [endpoint]"); return; }
    gWant = true;
    save();
    return status();
  }
  if (!strcmp(a, "off")) {
    gWant = false;
    save();
    for (int i = 0; i < 200 && (gRunning || gI2c != 0xFFFF || gSpi != 0xFFFF); i++) vTaskDelay(pdMS_TO_TICKS(10));
    return status();
  }
  if (sscanf(a, "bright %f", &f) == 1) {
    gBright = f < 0.02f ? 0.02f : f > 1 ? 1 : f;
    save();
    return status();
  }
  if (!strcmp(a, "reset")) {               // statistics
    gLoops = gFails = gOverruns = gBusyMax = gRcpCycles = gRcpCycleMax = gReopens = 0; gBusySum = 0;
    gRcpCalls = gRcpTimeouts = gRcpUsMax = 0; gRcpUsSum = 0;
    return status();
  }
  if (!strcmp(a, "rcp") || !strncmp(a, "sense", 5)) {
    if (gWant || gRunning || gI2c != 0xFFFF || gSpi != 0xFFFF) { Con.println("zone: the loop owns the endpoint -- `zone off` first"); return; }
    if (!gTargetKnown) { Con.println("zone: endpoint not resolved yet"); return; }
    if (!strcmp(a, "rcp")) {
      const uint8_t rc = call(ZW_GET_STATUS, nullptr, 0, 500);
      if (rc == 0xFE) { Con.println("zone: rcp GetStatus: no reply in 500 ms"); return; }
      Con.printf("zone: rcp GetStatus: reply rc %u, %lu B:", rc, (unsigned long)gRcpLastLen);
      for (uint32_t i = 16; i < gRcpLastLen && i < 64; i++) Con.printf(" %02x", gRcpLast[i]);
      Con.println();
      return;
    }
    const int cnt = atoi(a + 5) > 0 ? atoi(a + 5) : 10;   // zone sense [n] [keep]: read n times, 10 Hz
    sensorsOpen();
    for (int i = 0; i < cnt; i++) {
      const int64_t a0 = esp_timer_get_time();
      const int x = gSpi != 0xFFFF ? readAdc(0) : -1, y = gSpi != 0xFFFF ? readAdc(1) : -1, pr = gI2c != 0xFFFF ? readProx() : -1;
      Con.printf("zone: x %d y %d prox %d  (%lld us for 3 RCP round trips)\n", x, y, pr, (long long)(esp_timer_get_time() - a0));
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (strstr(a, "keep") == nullptr) sensorsClose();
    return;
  }
  Con.println("zone [status] | on | off | ip <alias> [endpoint] | bright <0..1> | reset | rcp | sense [n] [keep]");
}

}  // namespace zone
