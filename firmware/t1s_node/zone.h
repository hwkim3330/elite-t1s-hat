// Zone controller groundwork: talk to MCU-less 10BASE-T1S endpoints (Microchip LAN866x) that sit in another
// IPv4 subnet than the node itself, without a second lwIP interface.
//
// The node keeps its own address (lwIP); the zone gets an alias (e.g. 192.168.0.65) handled below lwIP:
//  - ARP: answer "who has <alias>", resolve the endpoint's MAC with our own request
//  - UDP out: Ethernet/IPv4/UDP built here and handed to the MAC with esp_eth_transmit
//  - UDP in: frames to the alias are offered to zone::rx() from the driver's receive tap (before lwIP,
//    which drops them: not its address)
// Nothing is sent from the receive path itself (the driver's task): ARP replies are queued for the zone task.
//
// First user: the endpoint's two 10x10 WS2812 panels, which take one 20x10 raw-RGB frame as RTP
// (RFC 4175, payload type 96) on UDP 5001; columns 0-9 = panel 1, 10-19 = panel 2.
#pragma once
#include <Arduino.h>
#include <esp_eth.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace zone {

static esp_eth_handle_t gZEth = nullptr;
static uint8_t gZMac[6];
static uint32_t gAlias = 0;          // network order
static uint32_t gTarget = 0;         // the endpoint (network order)
static uint8_t gTargetMac[6];
static volatile bool gTargetKnown = false;
static volatile uint32_t gRcpReplies = 0, gRcpLastLen = 0;
static volatile uint8_t gRcpLastRc = 0xFF;
static uint8_t gRcpLast[160];
static volatile uint16_t gRcpLastSess = 0;
static volatile uint32_t gArpAsked = 0, gArpAnswered = 0, gArpReplies = 0, gTxFrames = 0, gTxFail = 0, gRxUdp = 0;
// a pending ARP reply (filled by rx, sent by the task)
static volatile bool gArpPending = false;
static uint8_t gArpPeerMac[6];
static uint32_t gArpPeerIp = 0;

static volatile uint8_t gMode = 0;   // 0 off, 1 solid, 2 moving dot
static uint8_t gRgb[3] = {0, 40, 0};
static uint16_t gIpId = 1;

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

// UDP to the endpoint from the alias. payload is copied after a 42-byte header in buf (caller provides room).
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
      gRcpLastRc = sip[15];
      gRcpLastSess = (sip[10] << 8) | sip[11];
      gRcpLastLen = len - (14 + ihl + 8);
      memcpy(gRcpLast, sip, gRcpLastLen < sizeof(gRcpLast) ? gRcpLastLen : sizeof(gRcpLast));
      gRcpReplies = gRcpReplies + 1;
    }
  }
}

// ---- the LED panels: one 20x10 frame as RTP to <target>:5001
static uint8_t gFb[10][20][3];
static uint16_t gRtpSeq = 0;
static bool sendFrame() {
  static uint8_t buf[42 + 14 + 60 + 600];
  uint8_t *p = buf + 42;
  int n = 0;
  const uint32_t ts = (uint32_t)(esp_timer_get_time() / 10);
  p[n++] = 0x80; p[n++] = 0x80 | 96;
  p[n++] = gRtpSeq >> 8; p[n++] = gRtpSeq & 0xFF;
  p[n++] = ts >> 24; p[n++] = ts >> 16; p[n++] = ts >> 8; p[n++] = ts;
  p[n++] = 'Z'; p[n++] = 'O'; p[n++] = 'N'; p[n++] = 'E';          // SSRC
  p[n++] = 0; p[n++] = 0;                                          // extended sequence (high 16 bits)
  for (int y = 0; y < 10; y++) {
    p[n++] = 0; p[n++] = 60; p[n++] = 0; p[n++] = y; p[n++] = y < 9 ? 0x80 : 0; p[n++] = 0;
  }
  memcpy(p + n, gFb, 600);
  n += 600;
  gRtpSeq++;
  return udp(buf, 5001, 5001, n);
}

static void task(void *) {
  uint32_t lastArp = 0, k = 0;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(50));
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
    if (!gMode || !gTargetKnown) continue;
    memset(gFb, 0, sizeof(gFb));
    if (gMode == 1) {
      for (int y = 0; y < 10; y++)
        for (int x = 0; x < 20; x++) memcpy(gFb[y][x], gRgb, 3);
    } else {                                      // a dot running across both panels: shows the frame rate
      const int i = k++ % 200;
      memcpy(gFb[i / 20][i % 20], gRgb, 3);
    }
    sendFrame();
  }
}

static void begin(esp_eth_handle_t h) {
  gZEth = h;
  esp_eth_ioctl(h, ETH_CMD_G_MAC_ADDR, gZMac);
  static bool started = false;
  if (!started) {
    started = true;
    xTaskCreate(task, "zone", 4096, nullptr, 4, nullptr);
  }
}

static void status() {
  Con.printf("zone: alias " IPSTR ", endpoint " IPSTR " %s %02x:%02x:%02x:%02x:%02x:%02x, mode %u\n",
             IP2STR((esp_ip4_addr_t *)&gAlias), IP2STR((esp_ip4_addr_t *)&gTarget), gTargetKnown ? "at" : "(unresolved)",
             gTargetMac[0], gTargetMac[1], gTargetMac[2], gTargetMac[3], gTargetMac[4], gTargetMac[5], gMode);
  Con.printf("zone: arp asked %lu answered %lu, endpoint replies %lu; tx %lu (fail %lu) frames, udp to alias %lu\n",
             (unsigned long)gArpAsked, (unsigned long)gArpAnswered, (unsigned long)gArpReplies,
             (unsigned long)gTxFrames, (unsigned long)gTxFail, (unsigned long)gRxUdp);
}

// one SOME/IP request to the endpoint's RCP service (0xff10, UDP 6800), no payload
static void rcpPing(uint16_t method) {
  static uint8_t buf[42 + 16];
  static uint16_t sess = 1;
  uint8_t *p = buf + 42;
  const uint8_t h[16] = {0xFF, 0x10, (uint8_t)(method >> 8), (uint8_t)method, 0, 0, 0, 8,
                         0xAF, 0xFE, (uint8_t)(sess >> 8), (uint8_t)sess, 1, 1, 0, 0};
  memcpy(p, h, 16);
  sess++;
  udp(buf, 50000, 6800, 16);
}

// ---- RCP client: SOME/IP WTLV (tag = wire type << 12 | field number; 0/1/2 = u8/u16/u32, 6 = blob u16 len)
static int put8(uint8_t *p, int f, uint8_t v) { p[0] = 0x00; p[1] = f; p[2] = v; return 3; }
static int put16(uint8_t *p, int f, uint16_t v) { p[0] = 0x10; p[1] = f; p[2] = v >> 8; p[3] = v; return 4; }
static int put32(uint8_t *p, int f, uint32_t v) { p[0] = 0x20; p[1] = f; p[2] = v >> 24; p[3] = v >> 16; p[4] = v >> 8; p[5] = v; return 6; }
static int putBlob(uint8_t *p, int f, const uint8_t *d, int n) { p[0] = 0x60; p[1] = f; p[2] = n >> 8; p[3] = n; memcpy(p + 4, d, n); return 4 + n; }
// find field f in the reply's WTLV payload: returns a pointer to its value (blob: after the length), n = size
static const uint8_t *field(int f, int &n) {
  const uint8_t *p = gRcpLast + 16, *end = gRcpLast + (gRcpLastLen < sizeof(gRcpLast) ? gRcpLastLen : sizeof(gRcpLast));
  while (p + 2 <= end) {
    const int t = p[0] >> 4, id = ((p[0] & 0x0F) << 8) | p[1];
    p += 2;
    int len = t == 6 ? ((p[0] << 8) | p[1]) : (1 << t);
    if (t == 6) p += 2;
    if (id == f) { n = len; return p; }
    p += len;
  }
  return nullptr;
}
static uint16_t gSess = 1;
// one request, wait for its response; rc 0 = OK, 0xFE = timeout
static uint8_t call(uint16_t method, const uint8_t *pl, int n, int ms = 300) {
  static uint8_t buf[42 + 16 + 64];
  const uint16_t sess = gSess++;
  const uint32_t len = 8 + n;
  const uint8_t h[16] = {0xFF, 0x10, (uint8_t)(method >> 8), (uint8_t)method, (uint8_t)(len >> 24), (uint8_t)(len >> 16),
                         (uint8_t)(len >> 8), (uint8_t)len, 0xAF, 0xFE, (uint8_t)(sess >> 8), (uint8_t)sess, 1, 1, 0, 0};
  memcpy(buf + 42, h, 16);
  memcpy(buf + 58, pl, n);
  const uint32_t before = gRcpReplies;
  udp(buf, 50000, 6800, 16 + n);
  for (int i = 0; i < ms / 2; i++) {
    if (gRcpReplies != before && gRcpLastSess == sess) return gRcpLastRc;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  return 0xFE;
}
static uint16_t gI2c = 0xFFFF, gSpi = 0xFFFF;
static uint32_t gWid[2];
static uint16_t openWithRecovery(uint16_t method, const uint8_t *pl, int n, uint16_t first, uint16_t closeMethod) {
  for (int attempt = 0; attempt < 2; attempt++) {
    uint8_t rc = call(method, pl, n);
    int fn;
    const uint8_t *v = rc == 0 ? field(0, fn) : nullptr;
    if (v && fn == 2) return (v[0] << 8) | v[1];
    if (rc != 5) break;                             // NOT_REACHABLE: still held by a client that died
    uint8_t c[4];
    for (uint16_t h = first; h < first + 24; h++) call(closeMethod, c, put16(c, 0, h), 100);
  }
  return 0xFFFF;
}
static bool sensorsOpen() {
  uint8_t p[32];
  int n;
  if (gI2c == 0xFFFF) {
    n = put8(p, 0, 8); n += put8(p + n, 1, 9); n += put8(p + n, 2, 1);
    gI2c = openWithRecovery(0x1200, p, n, 0x100, 0x1202);
    gWid[0] = 0;
    if (gI2c != 0xFFFF) {
      const uint8_t init[3][3] = {{0x03, 0x02, 0x08}, {0x04, 0x01, 0x07}, {0x00, 0x00, 0x00}};   // PS fast, 16 bit, LED 200 mA
      for (auto &w : init) {
        n = put16(p, 0, gI2c); n += put16(p + n, 1, 0x51); n += put32(p + n, 2, gWid[0]++); n += putBlob(p + n, 3, w, 3);
        call(0x1204, p, n);
      }
    }
  }
  if (gSpi == 0xFFFF) {
    const uint8_t pins[4] = {12, 13, 14, 15};
    n = putBlob(p, 0, pins, 4);
    call(0x1105, p, n);
    n = put8(p, 0, 12); n += put8(p + n, 1, 13); n += put8(p + n, 2, 14); n += put8(p + n, 3, 15);
    n += put8(p + n, 4, 1); n += put32(p + n, 5, 1923000);
    gSpi = openWithRecovery(0x1500, p, n, 0x300, 0x1502);
    gWid[1] = 0;
  }
  return gI2c != 0xFFFF && gSpi != 0xFFFF;
}
static int readProx() {
  uint8_t p[32], reg = 0x08;
  int n = put16(p, 0, gI2c); n += put16(p + n, 1, 0x51); n += put16(p + n, 2, 2); n += put32(p + n, 3, gWid[0]++);
  n += putBlob(p + n, 4, &reg, 1);
  if (call(0x1208, p, n)) return -1;
  int fn;
  const uint8_t *v = field(1, fn);
  return v && fn >= 2 ? v[0] | (v[1] << 8) : -1;
}
static int readAdc(int ch) {
  uint8_t p[32];
  const uint8_t cmd[3] = {0x06, (uint8_t)(ch << 6), 0xFF};
  int n = put16(p, 0, gSpi); n += put16(p + n, 1, 3); n += put32(p + n, 2, gWid[1]++); n += putBlob(p + n, 3, cmd, 3);
  if (call(0x1508, p, n)) return -1;
  int fn;
  const uint8_t *v = field(1, fn);
  return v && fn >= 3 ? ((v[1] & 0x0F) << 8) | v[2] : -1;
}
static void sensorsClose() {
  uint8_t p[4];
  if (gI2c != 0xFFFF) call(0x1202, p, put16(p, 0, gI2c));
  if (gSpi != 0xFFFF) call(0x1502, p, put16(p, 0, gSpi));
  gI2c = gSpi = 0xFFFF;
}

// zone [status] | ip <alias> <endpoint> | led <r> <g> <b> | dot <r> <g> <b> | off
static void command(const char *a) {
  int r, g, b;
  char s1[20], s2[20];
  if (!*a || !strcmp(a, "status")) return status();
  if (sscanf(a, "ip %19s %19s", s1, s2) == 2) {
    IPAddress A, T;
    if (!A.fromString(s1) || !T.fromString(s2)) { Con.println("zone: bad address"); return; }
    gAlias = (uint32_t)A;
    gTarget = (uint32_t)T;
    gTargetKnown = false;
    return status();
  }
  if (sscanf(a, "led %d %d %d", &r, &g, &b) == 3 || sscanf(a, "dot %d %d %d", &r, &g, &b) == 3) {
    gRgb[0] = r; gRgb[1] = g; gRgb[2] = b;
    gMode = a[0] == 'l' ? 1 : 2;
    return status();
  }
  if (!strcmp(a, "rcp")) {
    const uint32_t before = gRcpReplies;
    rcpPing(0x1002);
    for (int i = 0; i < 50 && gRcpReplies == before; i++) vTaskDelay(pdMS_TO_TICKS(10));
    if (gRcpReplies == before) { Con.println("zone: rcp GetStatus: no reply in 500 ms"); return; }
    Con.printf("zone: rcp GetStatus: reply rc %u, %lu B:", gRcpLastRc, (unsigned long)gRcpLastLen);
    for (uint32_t i = 16; i < gRcpLastLen && i < 64; i++) Con.printf(" %02x", gRcpLast[i]);
    Con.println();
    return;
  }
  if (!strncmp(a, "sense", 5)) {             // zone sense [n]: open the stick + proximity over RCP and read n times
    const int cnt = atoi(a + 5) > 0 ? atoi(a + 5) : 10;
    const bool ok = sensorsOpen();
    Con.printf("zone: sensors i2c 0x%04x spi 0x%04x %s\n", gI2c, gSpi, ok ? "open" : "NOT all open");
    const int64_t t0 = esp_timer_get_time();
    for (int i = 0; i < cnt; i++) {
      const int64_t a0 = esp_timer_get_time();
      const int x = gSpi != 0xFFFF ? readAdc(0) : -1, y = gSpi != 0xFFFF ? readAdc(1) : -1, pr = gI2c != 0xFFFF ? readProx() : -1;
      Con.printf("zone: x %d y %d prox %d  (%lld us for 3 RCP round trips)\n", x, y, pr, (long long)(esp_timer_get_time() - a0));
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    (void)t0;
    if (strstr(a, "keep") == nullptr) sensorsClose();
    return;
  }
  if (!strcmp(a, "off")) {
    gMode = 0;
    memset(gFb, 0, sizeof(gFb));
    if (gTargetKnown) sendFrame();
    return status();
  }
  Con.println("zone [status] | ip <alias> <endpoint> | led <r> <g> <b> | dot <r> <g> <b> | off | rcp");
}

}  // namespace zone
