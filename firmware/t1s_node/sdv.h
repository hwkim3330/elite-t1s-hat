// SDV signal API: COVESA VSS signals over SOME/IP on 10BASE-T1S, with this node as the gateway for hosts that have
// no T1S of their own (a laptop on the USB cable, a PC on the RJ45). Small, static, no malloc.
//
// The dictionary (vss_dict.h) is generated from sdv/signals.vss; the same wire is implemented in Python in
// sdv/python/t1s_sdv (wire.py documents every byte). In short:
//   SOME/IP service 0x5653, this node = instance 1, endpoint UDP 30501; event 0x8000|id, set method id,
//   get method 0x4000|id; event group = id >> 8; several SOME/IP messages per datagram.
//   SOME/IP-SD on UDP 30490 / 224.224.224.245: OfferService every second (TTL 3 s), SubscribeEventgroup to every
//   other instance offered (renewed with each offer), SubscribeEventgroupAck to subscribers. Events go unicast to
//   each subscriber; a provider may also send them to 224.224.224.246:30501, which this node joins (so it hears the
//   vehicle state even while it cannot transmit: PLCA ID outside the coordinator's node count).
//   Host channel: UDP 30500 (any interface) or USB serial (0x01 COBS(frame + CRC16) 0x00 between the text lines):
//   HELLO, SUB/UNSUB by id list, PUB, GET (from the cache), SET (local actuator, or a SOME/IP request to the
//   signal's provider, answered when its response comes), PING; DATA frames carry every update a host subscribed to.
//
// Modes (`sdv off|listen|on`, saved): listen = receive and serve hosts, never transmit on T1S (sound mode with the
// node outside the PLCA count); on = also offer, subscribe and publish on T1S.
// The zone controller (zone.h) takes its vehicle state from here when VSS signals arrive, and its sensors are
// published here; a stick flick is a `set` of SelectedGear to whoever provides it.
#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <lwip/sockets.h>
#include "vss_dict.h"
#include "zone.h"

namespace sdv {

constexpr uint16_t kService = 0x5653, kInstance = 1, kClient = 0x0065;
constexpr uint16_t kSdPort = 30490, kEvPort = 30501, kHostPort = 30500;
constexpr int kSubs = 6, kOffers = 6, kHosts = 4, kPend = 6;
constexpr uint8_t kModeOff = 0, kModeListen = 1, kModeOn = 2;

struct Sig { uint8_t v[4]; uint32_t at; uint32_t ip; uint16_t port; uint8_t has, own, dirty; };
struct Sub { uint32_t ip; uint16_t port; uint32_t groups; uint32_t until; };
struct Offer { uint16_t inst; uint32_t ip; uint16_t sdPort, evPort; uint32_t until; };
struct Host { uint8_t kind; uint32_t ip; uint16_t port; uint32_t seen; uint32_t want; uint8_t buf[200]; int n; };   // kind 1 serial, 2 udp
struct Pend { uint16_t sess; int8_t host; uint8_t hseq, tries; uint16_t id; int64_t t0, sent; uint8_t val[4]; uint32_t ip; uint16_t port; };

static Sig gS[VSS_N];
static Sub gSub[kSubs];
static Offer gOff[kOffers];
static Host gH[kHosts];          // [0] = USB serial
static Pend gP[kPend];
static int gSd = -1, gEv = -1, gHs = -1;
static uint32_t gIp = 0;         // the T1S address (network order)
static uint8_t gMode = kModeListen;
static uint16_t gSess = 0, gSdSess = 0;
static QueueHandle_t gSerialQ = nullptr;      // host frames from the USB serial (loop()) to the task
struct SerialFrame { uint8_t n; uint8_t b[160]; };
// statistics
static uint32_t gRxEv = 0, gTxEv = 0, gTxDg = 0, gSdRx = 0, gSdTx = 0, gHostRx = 0, gHostTx = 0, gSetOk = 0, gSetFail = 0;
static uint32_t gCntRx = 0, gCntLost = 0, gCntLast = 0, gPings = 0, gBadCrc = 0;
static uint32_t gLoopMaxUs = 0;
static volatile uint32_t gBlastHz = 0, gBlastUntil = 0;
static uint8_t gLastEvCount = 0xFF;
static Preferences gPrefsS;

static int idx(uint16_t id) {
  int lo = 0, hi = VSS_N - 1;
  while (lo <= hi) {
    const int m = (lo + hi) / 2;
    if (VSS_SIGS[m].id == id) return m;
    if (VSS_SIGS[m].id < id) lo = m + 1; else hi = m - 1;
  }
  return -1;
}
static inline uint16_t be16(const uint8_t *p) { return (p[0] << 8) | p[1]; }
static inline uint32_t be32(const uint8_t *p) { return ((uint32_t)p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3]; }
static inline void put16(uint8_t *p, uint16_t v) { p[0] = v >> 8; p[1] = v; }
static inline void put32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }

// value <-> number (the table keeps the wire bytes)
static double num(int k) {
  const uint8_t *v = gS[k].v;
  switch (VSS_SIGS[k].type) {
    case VT_BOOL: case VT_U8: return v[0];
    case VT_I8: return (int8_t)v[0];
    case VT_U16: return be16(v);
    case VT_I16: return (int16_t)be16(v);
    case VT_U32: return be32(v);
    case VT_I32: return (int32_t)be32(v);
    default: { uint32_t u = be32(v); float f; memcpy(&f, &u, 4); return f; }
  }
}
static void encode(int k, double x, uint8_t *v) {
  switch (VSS_SIGS[k].type) {
    case VT_BOOL: v[0] = x != 0; break;
    case VT_U8: case VT_I8: v[0] = (uint8_t)(int)lrint(x); break;
    case VT_U16: case VT_I16: put16(v, (uint16_t)(int32_t)lrint(x)); break;
    case VT_U32: case VT_I32: put32(v, (uint32_t)(int64_t)llrint(x)); break;
    default: { float f = x; uint32_t u; memcpy(&u, &f, 4); put32(v, u); }
  }
}

// ---------------------------------------------------------------- hosts
static uint8_t gCobs[400];
static void serialOut(const uint8_t *f, int n) {
  // 0x01, COBS(f + CRC-16/CCITT), 0x00 -- one write under the console's lock, so it never lands inside a text line
  uint8_t raw[220];
  if (n > (int)sizeof(raw) - 2) return;
  memcpy(raw, f, n);
  uint16_t c = 0xFFFF;
  for (int i = 0; i < n; i++) {
    c ^= raw[i] << 8;
    for (int b = 0; b < 8; b++) c = c & 0x8000 ? (c << 1) ^ 0x1021 : c << 1;
  }
  raw[n] = c >> 8; raw[n + 1] = c; n += 2;
  int o = 0;
  gCobs[o++] = 0x01;
  int code = o++, len = 1;
  for (int i = 0; i < n; i++) {
    if (raw[i] == 0) { gCobs[code] = len; code = o++; len = 1; }
    else {
      gCobs[o++] = raw[i];
      if (++len == 255) { gCobs[code] = len; code = o++; len = 1; }
    }
  }
  gCobs[code] = len;
  gCobs[o++] = 0x00;
  Con.frame(gCobs, o);
}
static void hostSend(int h, const uint8_t *f, int n) {
  if (gH[h].kind == 1) serialOut(f, n);
  else if (gH[h].kind == 2 && gHs >= 0) {
    sockaddr_in a = {};
    a.sin_family = AF_INET; a.sin_port = gH[h].port; a.sin_addr.s_addr = gH[h].ip;
    sendto(gHs, f, n, 0, (sockaddr *)&a, sizeof(a));
  }
  gHostTx++;
}
static void hostFlush(int h) {
  Host &H = gH[h];
  if (H.n > 6) hostSend(h, H.buf, H.n);
  H.n = 0;
}
static void hostQueue(int k) {          // a value update for every host that wants it
  const uint32_t now = millis();
  for (int h = 0; h < kHosts; h++) {
    Host &H = gH[h];
    if (!H.kind || !(H.want & (1u << (k & 31))) || now - H.seen > 12000) continue;
    const int sz = VSS_SIGS[k].size;
    if (H.n + 2 + sz > (int)sizeof(H.buf)) hostFlush(h);
    if (!H.n) { H.buf[0] = 0x88; H.buf[1] = 0; put32(H.buf + 2, now); H.n = 6; }
    put16(H.buf + H.n, VSS_SIGS[k].id);
    memcpy(H.buf + H.n + 2, gS[k].v, sz);
    H.n += 2 + sz;
  }
}

// ---------------------------------------------------------------- SOME/IP out
static uint8_t gTx[600];
static void sendTo(int sock, uint32_t ip, uint16_t port, const uint8_t *b, int n) {
  sockaddr_in a = {};
  a.sin_family = AF_INET; a.sin_port = htons(port); a.sin_addr.s_addr = ip;
  if (sendto(sock, b, n, 0, (sockaddr *)&a, sizeof(a)) == n) gTxDg++;
}
static int someip(uint8_t *p, uint16_t method, uint8_t mt, uint16_t client, uint16_t sess, uint8_t rc, const uint8_t *pl, int n,
                  uint16_t service = kService) {
  put16(p, service); put16(p + 2, method); put32(p + 4, 8 + n); put16(p + 8, client); put16(p + 10, sess);
  p[12] = 1; p[13] = 1; p[14] = mt; p[15] = rc;
  if (n) memcpy(p + 16, pl, n);
  return 16 + n;
}
// every dirty own signal to every subscriber of its group (one datagram per subscriber)
static void publishDirty() {
  if (gMode != kModeOn || gEv < 0) { for (auto &s : gS) s.dirty = 0; return; }
  const uint32_t now = millis();
  for (auto &S : gSub) {
    if (!S.ip || (int32_t)(S.until - now) < 0) continue;
    int n = 0;
    for (int k = 0; k < VSS_N; k++) {
      if (!gS[k].dirty || !(S.groups & (1u << (VSS_SIGS[k].id >> 8)))) continue;
      if (n + 20 > (int)sizeof(gTx)) { sendTo(gEv, S.ip, S.port, gTx, n); n = 0; }
      gSess = gSess == 0xFFFF ? 1 : gSess + 1;
      n += someip(gTx + n, 0x8000 | VSS_SIGS[k].id, 0x02, 0, gSess, 0, gS[k].v, VSS_SIGS[k].size);
      gTxEv++;
    }
    if (n) sendTo(gEv, S.ip, S.port, gTx, n);
  }
  for (auto &s : gS) s.dirty = 0;
}
// set a signal this node publishes (bus subscribers + hosts)
static void own(uint16_t id, double x, bool force = false) {
  const int k = idx(id);
  if (k < 0) return;
  uint8_t v[4];
  encode(k, x, v);
  if (!force && gS[k].has && !memcmp(v, gS[k].v, VSS_SIGS[k].size)) return;
  memcpy(gS[k].v, v, 4);
  gS[k].has = gS[k].own = gS[k].dirty = 1;
  gS[k].at = millis();
  gS[k].ip = 0;
  hostQueue(k);
}

// ---------------------------------------------------------------- SD
static int sdHeader(uint8_t *p) {
  gSdSess = gSdSess == 0xFFFF ? 1 : gSdSess + 1;
  put16(p, 0xFFFF); put16(p + 2, 0x8100); put16(p + 8, 0); put16(p + 10, gSdSess);
  p[12] = 1; p[13] = 1; p[14] = 0x02; p[15] = 0;
  p[16] = 0xC0; p[17] = p[18] = p[19] = 0;
  return 24;   // + entries length at 20
}
static int ipv4Option(uint8_t *p, uint32_t ip, uint16_t port) {
  put16(p, 9); p[2] = 0x04; p[3] = 0; memcpy(p + 4, &ip, 4); p[8] = 0; p[9] = 0x11; put16(p + 10, port);
  return 12;
}
static int entry(uint8_t *p, uint8_t type, uint16_t inst, uint32_t ttl, uint16_t group, bool opt, bool service) {
  p[0] = type; p[1] = 0; p[2] = 0; p[3] = opt ? 0x10 : 0;
  put16(p + 4, kService); put16(p + 6, inst); p[8] = 1;
  p[9] = ttl >> 16; p[10] = ttl >> 8; p[11] = ttl;
  if (service) put32(p + 12, 0); else { p[12] = 0; p[13] = 0; put16(p + 14, group); }
  return 16;
}
static int sdFinish(uint8_t *p, int entriesEnd, int optLen) {
  put32(p + 20, entriesEnd - 24);
  put32(p + entriesEnd, optLen);
  const int total = entriesEnd + 4 + optLen;
  put32(p + 4, total - 8);
  return total;
}
static uint32_t wantGroups() {        // the vehicle state for the zone + whatever the hosts subscribed to
  uint32_t g = (1u << 1) | (1u << 2) | (1u << 3) | (1u << 5);
  const uint32_t now = millis();
  for (auto &H : gH)
    if (H.kind && now - H.seen < 12000)
      for (int k = 0; k < VSS_N; k++)
        if (H.want & (1u << (k & 31))) g |= 1u << (VSS_SIGS[k].id >> 8);
  return g;
}
static void sdOffer() {
  uint8_t b[64];
  int n = sdHeader(b);
  n += entry(b + n, 0x01, kInstance, 3, 0, true, true);
  const int e = n;
  const int ol = ipv4Option(b + e + 4, gIp, kEvPort);
  n = sdFinish(b, e, ol);
  static const uint32_t mc = inet_addr("224.224.224.245");
  sendTo(gSd, mc, kSdPort, b, n);
  gSdTx++;
}
static void sdSubscribe(const Offer &o) {
  uint8_t b[24 + 16 * 8 + 4 + 12];
  int n = sdHeader(b);
  const uint32_t g = wantGroups();
  for (int grp = 1; grp < 32 && n < 24 + 16 * 8; grp++)
    if (g & (1u << grp)) n += entry(b + n, 0x06, o.inst, 3, grp, true, false);
  const int e = n;
  const int ol = ipv4Option(b + e + 4, gIp, kEvPort);
  n = sdFinish(b, e, ol);
  sendTo(gSd, o.ip, o.sdPort, b, n);
  gSdTx++;
}
static void sdRx(const uint8_t *d, int len, uint32_t ip, uint16_t port) {
  if (len < 28 || be16(d) != 0xFFFF || be16(d + 2) != 0x8100) return;
  gSdRx++;
  const uint32_t ne = be32(d + 20);
  if (24 + ne + 4 > (uint32_t)len) return;
  const uint8_t *opts = d + 24 + ne + 4;
  const uint32_t no = be32(d + 24 + ne);
  uint8_t ack[24 + 16 * 8 + 4];
  int an = sdHeader(ack);
  const uint32_t now = millis();
  for (uint32_t o = 0; o + 16 <= ne; o += 16) {
    const uint8_t *e = d + 24 + o;
    if (be16(e + 4) != kService) continue;
    const uint8_t type = e[0];
    const uint16_t inst = be16(e + 6);
    const uint32_t ttl = (e[9] << 16) | (e[10] << 8) | e[11];
    // the first option the entry points at, if it is an IPv4 endpoint
    uint32_t eip = 0; uint16_t eport = 0;
    if (e[3] >> 4) {
      uint32_t off = 0;
      for (int i = 0; i < e[1] && off + 3 <= no; i++) off += 3 + be16(opts + off);
      if (off + 12 <= no && opts[off + 2] == 0x04) { memcpy(&eip, opts + off + 4, 4); eport = be16(opts + off + 10); }
    }
    if (type == 0x01 && inst != kInstance) {
      Offer *slot = nullptr;
      for (auto &x : gOff) if (x.inst == inst && x.ip == ip) slot = &x;
      if (!slot) for (auto &x : gOff) if (!x.ip || (int32_t)(x.until - now) < 0) { slot = &x; break; }
      if (!slot) continue;
      if (!ttl) { slot->ip = 0; continue; }
      *slot = {inst, ip, port, eport ? eport : kEvPort, now + ttl * 1000};
      if (gMode == kModeOn) sdSubscribe(*slot);
    } else if (type == 0x06 && inst == kInstance && eip) {
      const uint16_t grp = be16(e + 14);
      Sub *slot = nullptr;
      for (auto &x : gSub) if (x.ip == eip && x.port == eport) slot = &x;
      if (!slot) for (auto &x : gSub) if (!x.ip || (int32_t)(x.until - now) < 0) { slot = &x; *slot = {eip, eport, 0, 0}; break; }
      bool ok = slot != nullptr && grp < 32;
      if (ok && ttl) {
        const bool first = !(slot->groups & (1u << grp));
        slot->groups |= 1u << grp;
        slot->until = now + ttl * 1000;
        if (first)                                   // initial values of that group
          for (int k = 0; k < VSS_N; k++) if (gS[k].own && gS[k].has && (VSS_SIGS[k].id >> 8) == grp) gS[k].dirty = 1;
      } else if (ok) slot->groups &= ~(1u << grp);
      if (an + 16 <= (int)sizeof(ack) - 4) an += entry(ack + an, 0x07, inst, ok ? ttl : 0, grp, false, false);
    }
  }
  if (an > 24 && gMode == kModeOn) {
    const int n = sdFinish(ack, an, 0);
    sendTo(gSd, ip, port, ack, n);
    gSdTx++;
  }
}

// ---------------------------------------------------------------- events, requests, responses
static void hostReply(int8_t h, uint8_t op, uint8_t seq, const uint8_t *b, int n) {
  if (h < 0 || h >= kHosts || !gH[h].kind) return;
  uint8_t f[64];
  f[0] = op; f[1] = seq;
  memcpy(f + 2, b, n);
  hostSend(h, f, 2 + n);
}
static void setReply(Pend &p, uint8_t rc) {
  const uint32_t us = (uint32_t)(esp_timer_get_time() - p.t0);
  rc == 0 ? gSetOk++ : gSetFail++;
  uint8_t r[7];
  r[0] = rc; put16(r + 1, p.id); put32(r + 3, us);
  hostReply(p.host, 0x86, p.hseq, r, 7);
  if (p.host < 0) Con.printf("sdv: set 0x%04x -> rc %u in %lu us\n", p.id, rc, (unsigned long)us);
  p.sess = 0;
}
static bool localSet(int k, const uint8_t *v) {      // actuators this node owns
  if (VSS_SIGS[k].id == VSS_PRIVATE_ZONE_BRIGHTNESS) {
    const int pct = v[0] > 100 ? 100 : v[0] < 2 ? 2 : v[0];
    zone::gBright = pct / 100.0f;
    own(VSS_PRIVATE_ZONE_BRIGHTNESS, pct);
    return true;
  }
  return false;
}
static void echoIfPing(int k) {
  if (VSS_SIGS[k].id == VSS_PRIVATE_TEST_PING) { own(VSS_PRIVATE_TEST_PONG, num(k), true); gPings++; }
  if (VSS_SIGS[k].id == VSS_PRIVATE_TEST_COUNTER && !gS[k].own) {
    const uint32_t c = be32(gS[k].v);
    if (gCntRx && c > gCntLast + 1 && c - gCntLast < 100000) gCntLost += c - gCntLast - 1;
    gCntLast = c;
    gCntRx++;
  }
}
static bool gVehicleNew = false;
static void evRx(const uint8_t *d, int len, uint32_t ip, uint16_t port) {
  for (int o = 0; o + 16 <= len;) {
    const uint8_t *m = d + o;
    const uint32_t ml = be32(m + 4);
    if (ml < 8 || o + 8 + (int)ml > len) break;
    o += 8 + ml;
    if (be16(m) != kService) continue;
    const uint16_t method = be16(m + 2), sess = be16(m + 10), client = be16(m + 8);
    const uint8_t mt = m[14], rc = m[15];
    const uint8_t *pl = m + 16;
    const int pn = ml - 8;
    const int k = idx(method & 0x3FFF);
    if (mt == 0x02 && (method & 0x8000)) {                  // notification
      if (k < 0 || pn < VSS_SIGS[k].size || gS[k].own) continue;
      memcpy(gS[k].v, pl, VSS_SIGS[k].size);
      gS[k].has = 1; gS[k].at = millis(); gS[k].ip = ip; gS[k].port = port;
      gRxEv++;
      if ((VSS_SIGS[k].id >> 8) <= 2) gVehicleNew = true;
      hostQueue(k);
      echoIfPing(k);
    } else if (mt == 0x80 || mt == 0x81) {                   // a response to our set
      for (auto &p : gP) if (p.sess && p.sess == sess) setReply(p, mt == 0x80 ? 0 : (rc ? rc : 1));
    } else if (mt == 0x00 && k >= 0 && gMode == kModeOn) {  // a request to us
      uint8_t out[24];
      int n;
      if (method & 0x4000) {                                // get
        n = gS[k].has ? someip(out, method, 0x80, client, sess, 0, gS[k].v, VSS_SIGS[k].size)
                      : someip(out, method, 0x81, client, sess, 0x20, nullptr, 0);
      } else {                                              // set
        const bool ok = pn >= VSS_SIGS[k].size && localSet(k, pl);
        n = someip(out, method, ok ? 0x80 : 0x81, client, sess, ok ? 0 : VSS_SIGS[k].actuator ? 0x22 : 0x21, nullptr, 0);
      }
      sendTo(gEv, ip, port, out, n);
    }
  }
}
// a set for a signal someone else provides: a SOME/IP request to where its values come from
static void remoteSet(int8_t host, uint8_t hseq, int k, const uint8_t *v) {
  Pend *p = nullptr;
  for (auto &x : gP) if (!x.sess) { p = &x; break; }
  uint32_t ip = gS[k].ip;
  uint16_t port = gS[k].port;
  if (!ip) for (auto &o : gOff) if (o.ip && (int32_t)(o.until - millis()) > 0) { ip = o.ip; port = o.evPort; break; }
  if (!p || !ip || gMode != kModeOn) {
    Pend tmp = {}; tmp.host = host; tmp.hseq = hseq; tmp.id = VSS_SIGS[k].id; tmp.t0 = esp_timer_get_time();
    setReply(tmp, gMode != kModeOn ? 0x04 : 0x22);          // E_NOT_READY (listen mode) / no provider
    return;
  }
  gSess = gSess == 0xFFFF ? 1 : gSess + 1;
  *p = {gSess, host, hseq, 0, VSS_SIGS[k].id, esp_timer_get_time(), 0, {0}, ip, port};
  memcpy(p->val, v, VSS_SIGS[k].size);
}
static void pendTick() {
  const int64_t now = esp_timer_get_time();
  for (auto &p : gP) {
    if (!p.sess || now - p.sent < 150000) continue;
    if (p.tries >= 3) { setReply(p, 0xFE); continue; }       // 3 tries, 150 ms apart
    const int k = idx(p.id);
    uint8_t out[24];
    const int n = someip(out, p.id, 0x00, kClient, p.sess, 0, p.val, VSS_SIGS[k].size);
    sendTo(gEv, p.ip, p.port, out, n);
    p.sent = now;
    p.tries++;
  }
}

// ---------------------------------------------------------------- host frames
static void hostFrame(int h, const uint8_t *f, int n) {
  if (n < 2) return;
  gHostRx++;
  gH[h].seen = millis();
  const uint8_t op = f[0], seq = f[1];
  const uint8_t *b = f + 2;
  n -= 2;
  uint8_t r[16];
  if (op == 0x01) {
    r[0] = 1; put16(r + 1, VSS_DICT_HASH); put16(r + 3, kInstance); put32(r + 5, millis());
    hostReply(h, 0x81, seq, r, 9);
  } else if ((op == 0x02 || op == 0x03) && n >= 2) {
    const int cnt = be16(b);
    uint32_t mask = 0;
    if (!cnt) mask = 0xFFFFFFFF;
    for (int i = 0; i < cnt && 2 + 2 * i + 1 < n; i++) { const int k = idx(be16(b + 2 + 2 * i)); if (k >= 0) mask |= 1u << (k & 31); }
    if (op == 0x02) gH[h].want |= mask; else gH[h].want &= ~mask;
    r[0] = 0;
    hostReply(h, op | 0x80, seq, r, 1);
    if (op == 0x02) {                                         // the current values at once
      for (int k = 0; k < VSS_N; k++) if (gS[k].has && (mask & (1u << (k & 31)))) hostQueue(k);
    }
  } else if (op == 0x04) {                                    // publish: these become this node's signals
    for (int o = 0; o + 2 <= n;) {
      const int k = idx(be16(b + o));
      if (k < 0 || o + 2 + VSS_SIGS[k].size > n) break;
      memcpy(gS[k].v, b + o + 2, VSS_SIGS[k].size);
      gS[k].has = gS[k].own = gS[k].dirty = 1; gS[k].at = millis(); gS[k].ip = 0;
      o += 2 + VSS_SIGS[k].size;
      for (int x = 0; x < kHosts; x++) if (x != h) { const uint32_t w = gH[x].want; (void)w; }
      hostQueue(k);
      echoIfPing(k);
    }
  } else if (op == 0x05 && n >= 2) {
    const int k = idx(be16(b));
    r[0] = k < 0 ? 0x03 : gS[k].has ? 0 : 0x20; put16(r + 1, be16(b)); put32(r + 3, k >= 0 ? millis() - gS[k].at : 0);
    int rn = 7;
    if (!r[0]) { memcpy(r + 7, gS[k].v, VSS_SIGS[k].size); rn += VSS_SIGS[k].size; }
    hostReply(h, 0x85, seq, r, rn);
  } else if (op == 0x06 && n >= 2) {
    const int k = idx(be16(b));
    if (k < 0 || n < 2 + VSS_SIGS[k].size) { r[0] = 0x03; put16(r + 1, be16(b)); put32(r + 3, 0); hostReply(h, 0x86, seq, r, 7); return; }
    if (!VSS_SIGS[k].actuator) { r[0] = 0x21; put16(r + 1, be16(b)); put32(r + 3, 0); hostReply(h, 0x86, seq, r, 7); return; }
    if (localSet(k, b + 2)) { r[0] = 0; put16(r + 1, be16(b)); put32(r + 3, 0); hostReply(h, 0x86, seq, r, 7); gSetOk++; return; }
    remoteSet(h, seq, k, b + 2);
  } else if (op == 0x07 && n >= 4) {
    memcpy(r, b, 4); put32(r + 4, (uint32_t)esp_timer_get_time());
    hostReply(h, 0x87, seq, r, 8);
  }
}
// from loop(): one serial frame (between 0x01 and 0x00, still COBS-encoded)
static void serialBytes(const uint8_t *c, int n) {
  SerialFrame f;
  int o = 0, i = 0;
  while (i < n) {
    const uint8_t code = c[i];
    if (!code || i + code > n) { gBadCrc++; return; }
    for (int j = 1; j < code; j++) { if (o >= (int)sizeof(f.b)) { gBadCrc++; return; } f.b[o++] = c[i + j]; }
    i += code;
    if (code < 255 && i < n) { if (o >= (int)sizeof(f.b)) { gBadCrc++; return; } f.b[o++] = 0; }
  }
  if (o < 4) { gBadCrc++; return; }
  uint16_t crc = 0xFFFF;
  for (int k = 0; k < o - 2; k++) {
    crc ^= f.b[k] << 8;
    for (int b = 0; b < 8; b++) crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  if (crc != be16(f.b + o - 2)) { gBadCrc++; return; }
  f.n = o - 2;
  if (gSerialQ) xQueueSend(gSerialQ, &f, 0);
}

// ---------------------------------------------------------------- the zone's side
static void zoneTick() {
  using namespace zone;
  if (gVehicleNew) {                                         // vehicle state from VSS -> the LED picture
    gVehicleNew = false;
    auto b = [](uint16_t id) { const int k = idx(id); return k >= 0 && gS[k].has && num(k) != 0; };
    auto v = [](uint16_t id, double d) { const int k = idx(id); return k >= 0 && gS[k].has ? num(k) : d; };
    const uint8_t lamps = (b(VSS_PRIVATE_LIGHTS_DIRECTIONINDICATOR_LEFT_ISLIT) ? ZR_LEFT : 0) |
                          (b(VSS_PRIVATE_LIGHTS_DIRECTIONINDICATOR_RIGHT_ISLIT) ? ZR_RIGHT : 0) |
                          (b(VSS_BODY_LIGHTS_BRAKE_ISACTIVE) ? 4 : 0) | (b(VSS_BODY_LIGHTS_BACKUP_ISON) ? 8 : 0) |
                          (b(VSS_BODY_LIGHTS_HAZARD_ISSIGNALING) ? 16 : 0);
    const int g = (int)v(VSS_POWERTRAIN_TRANSMISSION_SELECTEDGEAR, 126);
    const char gear = g == 126 ? 'P' : g == 127 ? 'D' : g == 0 ? 'N' : g < 0 ? 'R' : 'D';
    if (idx(VSS_SPEED) >= 0 && gS[idx(VSS_SPEED)].has)
      applyState(lamps, gear, v(VSS_SPEED, 0), (uint8_t)v(VSS_PRIVATE_DRIVINGMODE, 0), v(VSS_CHASSIS_ACCELERATOR_PEDALPOSITION, 0) / 100.0,
                 v(VSS_CHASSIS_STEERINGWHEEL_ANGLE, 0), v(VSS_CHASSIS_BRAKE_PEDALPOSITION, 0) / 100.0);
  }
  if (!gWant) return;
  const double pm = gProx < 0 ? -1 : rint(zr_prox_m(gProx) * 100) / 100;
  own(VSS_PRIVATE_ZONE_STICK_X, gX < 0 ? 0xFFFF : gX);
  own(VSS_PRIVATE_ZONE_STICK_Y, gY < 0 ? 0xFFFF : gY);
  own(VSS_PRIVATE_ZONE_PROXIMITY_RAW, gProx < 0 ? 0xFFFF : gProx);
  own(VSS_PRIVATE_EXTERNAL_PROXIMITY_REAR, pm);
  own(VSS_PRIVATE_ZONE_LOOPWORKTIME, gBusyUs > 0xFFFF ? 0xFFFF : gBusyUs);
  own(VSS_PRIVATE_ZONE_RCPFAILURES, gFails > 0xFFFF ? 0xFFFF : gFails);
  own(VSS_PRIVATE_ZONE_BRIGHTNESS, lrint(gBright * 100));
  if (gLastEvCount == 0xFF) gLastEvCount = gEventCount;
  if (gEventCount != gLastEvCount) {                         // a stick flick: tell the car (set) and anyone listening
    gLastEvCount = gEventCount;
    own(VSS_PRIVATE_ZONE_STICK_GEARFLICK, (uint8_t)gEvent, true);
    const int k = idx(VSS_POWERTRAIN_TRANSMISSION_SELECTEDGEAR);
    uint8_t v[4] = {(uint8_t)(gEvent == 'P' ? 126 : gEvent == 'D' ? 127 : gEvent == 'N' ? 0 : -1)};
    remoteSet(-1, 0, k, v);
  }
}

// ---------------------------------------------------------------- the task
static int udpSock(uint16_t port, const char *group) {
  const int s = socket(AF_INET, SOCK_DGRAM, 0);
  if (s < 0) return -1;
  int one = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  sockaddr_in a = {};
  a.sin_family = AF_INET; a.sin_port = htons(port); a.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(s, (sockaddr *)&a, sizeof(a)) < 0) { close(s); return -1; }
  if (group) {
    ip_mreq m = {};
    m.imr_multiaddr.s_addr = inet_addr(group);
    m.imr_interface.s_addr = gIp;
    if (setsockopt(s, IPPROTO_IP, IP_ADD_MEMBERSHIP, &m, sizeof(m)) < 0) Con.printf("sdv: join %s failed\n", group);
    in_addr ifa; ifa.s_addr = gIp;
    setsockopt(s, IPPROTO_IP, IP_MULTICAST_IF, &ifa, sizeof(ifa));
    uint8_t ttl = 1;
    setsockopt(s, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));
  }
  return s;
}

static void task(void *) {
  static uint8_t rx[700];
  gSd = udpSock(kSdPort, "224.224.224.245");
  gEv = udpSock(kEvPort, "224.224.224.246");
  gHs = udpSock(kHostPort, nullptr);
  Con.printf("sdv: %s, SOME/IP 0x%04x.%u on UDP %u (SD %u), hosts on UDP %u and USB serial, dictionary 0x%04x (%d signals)\n",
             gMode == kModeOn ? "on" : gMode == kModeListen ? "listen (no T1S transmit)" : "off", kService, kInstance, kEvPort,
             kSdPort, kHostPort, VSS_DICT_HASH, VSS_N);
  uint32_t tOffer = 0, tZone = 0, tCycle = 0, tBlast = 0;
  uint32_t blastN = 0;
  for (;;) {
    fd_set fs;
    FD_ZERO(&fs);
    int mx = -1;
    for (int s : {gSd, gEv, gHs}) if (s >= 0) { FD_SET(s, &fs); mx = s > mx ? s : mx; }
    timeval tv = {0, 2000};
    const int r = mx >= 0 ? select(mx + 1, &fs, nullptr, nullptr, &tv) : (vTaskDelay(2), 0);
    const int64_t t0 = esp_timer_get_time();
    if (gMode == kModeOff) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
    sockaddr_in from;
    socklen_t fl;
    int n;
    if (r > 0) {
      for (int s : {gSd, gEv, gHs}) {
        if (s < 0) continue;
        for (int i = 0; i < 8; i++) {
          fl = sizeof(from);
          n = recvfrom(s, rx, sizeof(rx), MSG_DONTWAIT, (sockaddr *)&from, &fl);
          if (n <= 0) break;
          if (from.sin_addr.s_addr == gIp) continue;       // our own multicast
          if (s == gSd) sdRx(rx, n, from.sin_addr.s_addr, ntohs(from.sin_port));
          else if (s == gEv) evRx(rx, n, from.sin_addr.s_addr, ntohs(from.sin_port));
          else {
            int h = -1;
            for (int k = 1; k < kHosts; k++) if (gH[k].kind == 2 && gH[k].ip == from.sin_addr.s_addr && gH[k].port == from.sin_port) h = k;
            if (h < 0) for (int k = 1; k < kHosts; k++) if (!gH[k].kind || millis() - gH[k].seen > 12000) { h = k; gH[k] = {}; break; }
            if (h < 0) continue;
            gH[h].kind = 2; gH[h].ip = from.sin_addr.s_addr; gH[h].port = from.sin_port;
            hostFrame(h, rx, n);
          }
        }
      }
    }
    SerialFrame sf;
    while (gSerialQ && xQueueReceive(gSerialQ, &sf, 0) == pdTRUE) { gH[0].kind = 1; hostFrame(0, sf.b, sf.n); }
    const uint32_t now = millis();
    if (now - tOffer >= 1000) {
      tOffer = now;
      if (gMode == kModeOn) sdOffer();
    }
    if (now - tZone >= 50 || gVehicleNew) {
      if (now - tZone >= 50) tZone = now;
      zoneTick();
    }
    if (now - tCycle >= 1000) {                                // all own values again once a second (fields)
      tCycle = now;
      for (auto &s : gS) if (s.own && s.has) s.dirty = 1;
    }
    if (gBlastHz && (int32_t)(gBlastUntil - now) > 0) {        // `sdv blast`: Test.Counter at a fixed rate
      const uint32_t due = (uint32_t)((uint64_t)(now - tBlast) * gBlastHz / 1000);
      for (uint32_t i = 0; i < due && i < 20; i++) own(VSS_PRIVATE_TEST_COUNTER, ++blastN, true);
      if (due) tBlast += due * 1000 / gBlastHz;
    } else if (gBlastHz) {
      Con.printf("sdv: blast done, %lu sent\n", (unsigned long)blastN);
      gBlastHz = 0;
    } else { tBlast = now; blastN = blastN; }
    pendTick();
    publishDirty();
    for (int h = 0; h < kHosts; h++) hostFlush(h);
    const uint32_t us = (uint32_t)(esp_timer_get_time() - t0);
    if (us > gLoopMaxUs) gLoopMaxUs = us;
  }
}

static void status() {
  Con.printf("sdv: %s, ip " IPSTR ", dictionary 0x%04x, %d signals\n",
             gMode == kModeOn ? "on" : gMode == kModeListen ? "listen" : "off", IP2STR((esp_ip4_addr_t *)&gIp), VSS_DICT_HASH, VSS_N);
  const uint32_t now = millis();
  for (auto &o : gOff) if (o.ip && (int32_t)(o.until - now) > 0)
    Con.printf("sdv: offer 0x%04x.%u from " IPSTR ":%u, events :%u\n", kService, o.inst, IP2STR((esp_ip4_addr_t *)&o.ip), o.sdPort, o.evPort);
  for (auto &s : gSub) if (s.ip && (int32_t)(s.until - now) > 0)
    Con.printf("sdv: subscriber " IPSTR ":%u groups 0x%08lx\n", IP2STR((esp_ip4_addr_t *)&s.ip), s.port, (unsigned long)s.groups);
  for (int h = 0; h < kHosts; h++) if (gH[h].kind && now - gH[h].seen < 12000)
    Con.printf("sdv: host %d %s " IPSTR ":%u wants 0x%08lx, seen %lu ms ago\n", h, gH[h].kind == 1 ? "usb" : "udp",
               IP2STR((esp_ip4_addr_t *)&gH[h].ip), ntohs(gH[h].port), (unsigned long)gH[h].want, (unsigned long)(now - gH[h].seen));
  Con.printf("sdv: events rx %lu tx %lu in %lu datagrams; sd rx %lu tx %lu; host frames rx %lu tx %lu (bad %lu); set ok %lu fail %lu\n",
             (unsigned long)gRxEv, (unsigned long)gTxEv, (unsigned long)gTxDg, (unsigned long)gSdRx, (unsigned long)gSdTx,
             (unsigned long)gHostRx, (unsigned long)gHostTx, (unsigned long)gBadCrc, (unsigned long)gSetOk, (unsigned long)gSetFail);
  Con.printf("sdv: test counter rx %lu lost %lu last %lu; pings echoed %lu; loop work max %lu us\n", (unsigned long)gCntRx,
             (unsigned long)gCntLost, (unsigned long)gCntLast, (unsigned long)gPings, (unsigned long)gLoopMaxUs);
}
static void values() {
  const uint32_t now = millis();
  for (int k = 0; k < VSS_N; k++) {
    if (!gS[k].has) continue;
    Con.printf("sdv: %-58s %10.2f %s (%s, %lu ms)\n", VSS_SIGS[k].path, num(k), VSS_SIGS[k].unit,
               gS[k].own ? "own" : "bus", (unsigned long)(now - gS[k].at));
  }
}

static void saveMode() { gPrefsS.begin("sdv", false); gPrefsS.putUChar("mode", gMode); gPrefsS.end(); }

static void begin(uint32_t ip) {
  gIp = ip;
  gPrefsS.begin("sdv", true);
  gMode = gPrefsS.getUChar("mode", kModeListen);
  gPrefsS.end();
  gSerialQ = xQueueCreate(4, sizeof(SerialFrame));
  xTaskCreate(task, "sdv", 6144, nullptr, 4, nullptr);
}

// sdv [status] | values | on | listen | off | blast <hz> <s> | reset
static void command(const char *a) {
  unsigned hz = 0, secs = 0;
  if (!*a || !strcmp(a, "status")) return status();
  if (!strcmp(a, "values")) return values();
  if (!strcmp(a, "on") || !strcmp(a, "listen") || !strcmp(a, "off")) {
    gMode = !strcmp(a, "on") ? kModeOn : !strcmp(a, "listen") ? kModeListen : kModeOff;
    saveMode();
    return status();
  }
  if (sscanf(a, "blast %u %u", &hz, &secs) >= 1) {
    gBlastUntil = millis() + (secs ? secs : 5) * 1000;
    gBlastHz = hz > 5000 ? 5000 : hz;
    Con.printf("sdv: Test.Counter at %u Hz for %u s\n", (unsigned)gBlastHz, secs ? secs : 5);
    return;
  }
  if (!strcmp(a, "reset")) {
    gRxEv = gTxEv = gTxDg = gSdRx = gSdTx = gHostRx = gHostTx = gSetOk = gSetFail = gCntRx = gCntLost = gCntLast = gPings = gBadCrc = 0;
    gLoopMaxUs = 0;
    return status();
  }
  Con.println("sdv [status] | values | on | listen | off | blast <hz> [s] | reset");
}

}  // namespace sdv
