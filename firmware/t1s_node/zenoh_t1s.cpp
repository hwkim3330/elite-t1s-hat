// Zenoh-pico over the board's wired network. Compiled in only with -DT1S_WITH_ZENOH (plus
// --library <zenoh-pico> and -DZENOH_ARDUINO_ESP32); otherwise this file is a stub and
// t1s_node builds and runs exactly as before. An explicit flag, not __has_include: Arduino's
// library resolver only follows plain #include lines.
//
// Kept out of the .ino on purpose: Arduino's prototype pass (-CC) corrupts zenoh-pico's
// ##-heavy macros, and plain .cpp files in the sketch folder skip that pass.
//
// Transport. Default: **peer over UDP multicast** (ZENOH_LOCATOR, udp/224.0.0.224:7447), so
// two boards talk Zenoh with no router and no PC. -DZENOH_ROUTER='"udp/<ip>:7447"' connects to a
// zenohd instead (the 2026-10-01 bench). lwIP sends multicast out of the default netif, which is
// the wired one (Ethernet outranks the WiFi AP), so Zenoh rides T1S / the W5500, never WiFi.
//
// Keys (<node> = t1s-hat-<PLCA id> on the HAT node, t1s-eth-<mac> on a W5500-only board):
//   t1s/<node>/hello    2 Hz   seq, uptime, PLCA as configured
//   t1s/<node>/signal  20 Hz   a synthetic sensor value (sine) -- a steady small-message load
//   test/ping/<node>    5 Hz   "<seq> <t_us>" (rate: `zenoh ping <hz>`)
//   test/pong/<node>           the echo of our ping, published by any other board
//   test/stats/<node>   1 Hz   RTT over the last second, measured here from the pongs
//   t1s/<node>/bulk            `zenoh blast`: seq-numbered payloads, counted by every other board
//   t1s/<node>/cmd      sub    anything sent here is printed on the console
//   t1s/<node>/config   sub    a console command (plca/csma/ip/spi/mode/save/reboot/status/counters), run by
//                              the console task; the result comes back on t1s/<node>/config/ack
// Every board echoes every other board's ping, so with two boards each measures its own RTT.
#include "zenoh_t1s.h"

#include <Arduino.h>
#include "net_console.h"

#ifdef T1S_WITH_ZENOH
#include <algorithm>
#include <cmath>
#include <esp_timer.h>
#include <zenoh-pico.h>

#ifndef ZENOH_LOCATOR
// `#iface=` is required by zenoh-pico's multicast locator check; the ESP32 port ignores its value.
#define ZENOH_LOCATOR "udp/224.0.0.224:7447#iface=eth"
#endif

static bool sUp = false, sWanted = true;
static char sNode[24] = "t1s-node";
static bool sFixedName = false;
static z_owned_session_t sSession;
static z_owned_publisher_t sHello, sSignal, sPing, sStats, sAck, sBulk;
static z_owned_subscriber_t sPong, sCmd, sCfg, sPingAll, sBulkAll;
static portMUX_TYPE sMbx = portMUX_INITIALIZER_UNLOCKED;
static char sCfgIn[100], sAckOut[200];
static volatile bool sCfgPending = false, sAckPending = false;
static uint32_t sPingSeq = 0, sHelloSeq = 0, sPongs = 0, sSent = 0, sEchoed = 0;
static uint32_t sLastPongMs = 0;
static volatile uint16_t sPingHz = 5;
static volatile bool sPaused = false;  // `zenoh pause`: no periodic traffic, no echoes (clean UDP tests)

struct Rtt { uint32_t n = 0; int64_t sum = 0, mn = INT64_MAX, mx = 0, jit = 0, last = 0; };
static Rtt sRtt, sRttLast;   // current window, last finished window (for `zenoh` status)
// every RTT, for `zenoh rtts` (the report's distribution)
static int32_t sRttRing[600];
static volatile uint32_t sRttN = 0;

// pings from other boards, waiting to be echoed by the zenoh task (callbacks only queue)
struct Echo { char who[24]; char body[40]; };
static Echo sEchoQ[8];
static volatile uint8_t sEchoHead = 0, sEchoTail = 0;
static TaskHandle_t sZTask = nullptr;  // woken by a ping callback, so the echo goes out at once

// bulk receive side (any other board's t1s/*/bulk)
struct BulkRx { uint32_t msgs = 0, bytes = 0, minSeq = UINT32_MAX, maxSeq = 0; int64_t t0 = 0, t1 = 0; };
static BulkRx sBulkRx;
static volatile bool sBulkReset = false;
// bulk send request, carried out by the zenoh task
static volatile uint32_t sBlastSecs = 0, sBlastBytes = 0;
static volatile bool sBlastBatch = false;

static void put(z_owned_publisher_t &p, const char *s) {
  z_owned_bytes_t b;
  z_bytes_copy_from_str(&b, s);
  if (z_publisher_put(z_publisher_loan(&p), z_bytes_move(&b), NULL) == 0) sSent++;
}

static size_t copyPayload(const z_loaned_sample_t *sample, char *buf, size_t n) {
  z_owned_string_t v;
  z_bytes_to_string(z_sample_payload(sample), &v);
  size_t len = z_string_len(z_string_loan(&v));
  if (len >= n) len = n - 1;
  memcpy(buf, z_string_data(z_string_loan(&v)), len);
  buf[len] = 0;
  z_string_drop(z_string_move(&v));
  return len;
}

// key segment `idx` (0-based, '/'-separated) of a sample's key
static void keySegment(const z_loaned_sample_t *sample, int idx, char *out, size_t n) {
  z_view_string_t ks;
  z_keyexpr_as_view_string(z_sample_keyexpr(sample), &ks);
  const char *k = z_string_data(z_view_string_loan(&ks));
  const size_t len = z_string_len(z_view_string_loan(&ks));
  size_t i = 0;
  for (int seg = 0; seg < idx && i < len; i++)
    if (k[i] == '/') seg++;
  size_t j = i;
  while (j < len && k[j] != '/') j++;
  const size_t m = j - i < n - 1 ? j - i : n - 1;
  memcpy(out, k + i, m);
  out[m] = 0;
}

static void onPong(z_loaned_sample_t *sample, void *) {
  char buf[48];
  copyPayload(sample, buf, sizeof(buf));
  unsigned seq;
  long long t0;
  if (sscanf(buf, "%u %lld", &seq, &t0) != 2) return;
  const int64_t rtt = esp_timer_get_time() - t0;
  if (rtt < 0) return;
  Rtt &r = sRtt;
  if (r.n) r.jit += llabs(rtt - r.last);
  r.n++; r.sum += rtt; r.mn = min(r.mn, rtt); r.mx = max(r.mx, rtt); r.last = rtt;
  sRttRing[sRttN % 600] = (int32_t)rtt;
  sRttN = sRttN + 1;
  sPongs++;
  sLastPongMs = millis();
}

// Zenoh peers this node has heard (their pings on test/ping/<who>): the tablet's Zenoh view
struct Peer { char name[24]; uint32_t pings, lastMs; };
static Peer sPeers[8];
static uint8_t sNPeers = 0;

static void notePeer(const char *who) {
  portENTER_CRITICAL(&sMbx);
  int i = 0;
  while (i < sNPeers && strcmp(sPeers[i].name, who)) i++;
  if (i == sNPeers && sNPeers < 8) { strncpy(sPeers[i].name, who, sizeof(sPeers[i].name) - 1); sPeers[i].pings = 0; sNPeers++; }
  if (i < sNPeers) { sPeers[i].pings++; sPeers[i].lastMs = millis(); }
  portEXIT_CRITICAL(&sMbx);
}

void zenohT1sTele(char *out, size_t n) {
  const int32_t last = sRttN ? sRttRing[(sRttN - 1) % 600] : -1;
  int o = snprintf(out, n, "z %d %d %lu %lu %ld %u ", sUp ? 1 : 0, sPaused ? 1 : 0, (unsigned long)sSent,
                   (unsigned long)sPongs, (long)last, (unsigned)sNPeers);
  portENTER_CRITICAL(&sMbx);
  for (int i = 0; i < sNPeers && o < (int)n - 1; i++)
    o += snprintf(out + o, n - o, "%s%s:%lu:%lu", i ? "," : "", sPeers[i].name, (unsigned long)sPeers[i].pings,
                  (unsigned long)(millis() - sPeers[i].lastMs));
  portEXIT_CRITICAL(&sMbx);
  if (!sNPeers && o < (int)n - 1) snprintf(out + o, n - o, "-");
}

static void onPingAll(z_loaned_sample_t *sample, void *) {
  Echo e;
  keySegment(sample, 2, e.who, sizeof(e.who));  // test/ping/<who>
  if (!strcmp(e.who, sNode)) return;             // our own ping, looped back by multicast
  notePeer(e.who);
  copyPayload(sample, e.body, sizeof(e.body));
  const uint8_t next = (sEchoHead + 1) % 8;
  if (next == sEchoTail) return;  // queue full: dropped, the sender counts it as lost
  sEchoQ[sEchoHead] = e;
  sEchoHead = next;
  if (sZTask) xTaskNotifyGive(sZTask);  // echo now, not on the next 2 ms pass
}

static void onBulk(z_loaned_sample_t *sample, void *) {
  char who[24];
  keySegment(sample, 1, who, sizeof(who));       // t1s/<who>/bulk
  if (!strcmp(who, sNode)) return;
  const z_loaned_bytes_t *p = z_sample_payload(sample);
  const size_t len = z_bytes_len(p);
  uint32_t seq = 0;
  z_bytes_reader_t rd = z_bytes_get_reader(p);
  if (z_bytes_reader_read(&rd, (uint8_t *)&seq, 4) != 4) return;
  if (sBulkReset) { sBulkRx = BulkRx(); sBulkReset = false; }
  BulkRx &b = sBulkRx;
  const int64_t now = esp_timer_get_time();
  if (!b.msgs) b.t0 = now;
  b.t1 = now;
  b.msgs++;
  b.bytes += len;
  if (seq < b.minSeq) b.minSeq = seq;
  if (seq > b.maxSeq) b.maxSeq = seq;
}

static void onCmd(z_loaned_sample_t *sample, void *) {
  char buf[96];
  copyPayload(sample, buf, sizeof(buf));
  Con.printf("zenoh: cmd \"%s\"\n", buf);
}

static void onConfig(z_loaned_sample_t *sample, void *) {
  char buf[sizeof(sCfgIn)];
  copyPayload(sample, buf, sizeof(buf));
  portENTER_CRITICAL(&sMbx);
  if (!sCfgPending) {
    memcpy(sCfgIn, buf, sizeof(sCfgIn));
    sCfgPending = true;
  }
  portEXIT_CRITICAL(&sMbx);
}

static bool declarePub(z_owned_publisher_t &p, const char *ke, bool express = false) {
  z_view_keyexpr_t k;
  z_view_keyexpr_from_str_unchecked(&k, ke);
  z_publisher_options_t o;
  z_publisher_options_default(&o);
  o.is_express = express;  // latency-critical keys skip batching
  return z_declare_publisher(z_session_loan(&sSession), &p, z_view_keyexpr_loan(&k), &o) == 0;
}

static bool declareSub(z_owned_subscriber_t &s, const char *ke, void (*cb)(z_loaned_sample_t *, void *)) {
  z_view_keyexpr_t k;
  z_view_keyexpr_from_str_unchecked(&k, ke);
  z_owned_closure_sample_t c;
  z_closure_sample(&c, cb, NULL, NULL);
  return z_declare_subscriber(z_session_loan(&sSession), &s, z_view_keyexpr_loan(&k), z_closure_sample_move(&c), NULL) == 0;
}

static const char *where() {
#ifdef ZENOH_ROUTER
  return ZENOH_ROUTER;
#else
  return ZENOH_LOCATOR;
#endif
}

static bool start() {
  z_owned_config_t cfg;
  z_config_default(&cfg);
  zp_config_insert(z_config_loan_mut(&cfg), Z_CONFIG_MODE_KEY, "peer");
#ifdef ZENOH_ROUTER
  zp_config_insert(z_config_loan_mut(&cfg), Z_CONFIG_CONNECT_KEY, ZENOH_ROUTER);
#else
  zp_config_insert(z_config_loan_mut(&cfg), Z_CONFIG_LISTEN_KEY, ZENOH_LOCATOR);
#endif
  Con.printf("zenoh: opening %s as %s ...\n", where(), sNode);
  const z_result_t rc = z_open(&sSession, z_config_move(&cfg), NULL);
  if (rc < 0) { Con.printf("zenoh: open FAILED (%d)\n", (int)rc); return false; }
  char ke[64];
  bool ok = true;
  snprintf(ke, sizeof(ke), "t1s/%s/hello", sNode);   ok &= declarePub(sHello, ke);
  snprintf(ke, sizeof(ke), "t1s/%s/signal", sNode);  ok &= declarePub(sSignal, ke);
  snprintf(ke, sizeof(ke), "test/ping/%s", sNode);   ok &= declarePub(sPing, ke, true);
  snprintf(ke, sizeof(ke), "test/stats/%s", sNode);  ok &= declarePub(sStats, ke);
  snprintf(ke, sizeof(ke), "t1s/%s/bulk", sNode);    ok &= declarePub(sBulk, ke);
  snprintf(ke, sizeof(ke), "test/pong/%s", sNode);   ok &= declareSub(sPong, ke, onPong);
  ok &= declareSub(sPingAll, "test/ping/*", onPingAll);
  ok &= declareSub(sBulkAll, "t1s/*/bulk", onBulk);
  snprintf(ke, sizeof(ke), "t1s/%s/cmd", sNode);     ok &= declareSub(sCmd, ke, onCmd);
  snprintf(ke, sizeof(ke), "t1s/%s/config", sNode);  ok &= declareSub(sCfg, ke, onConfig);
  snprintf(ke, sizeof(ke), "t1s/%s/config/ack", sNode); ok &= declarePub(sAck, ke);
  Con.printf("zenoh: session %s\n", ok ? "up" : "declare FAILED");
  return ok;
}

// `zenoh blast <sec> <bytes> [batch]`: publish seq-numbered payloads as fast as zenoh-pico takes them.
// Unbatched, every put is its own UDP datagram and costs ~1 ms on the sender (that, not the
// receiver, held bulk near 1000 msg/s). `batch` wraps the run in zp_batch_start/stop, so puts
// are packed into a datagram until it is full. The report gives the mean time per put call.
static void runBlast(uint32_t secs, uint32_t bytes, bool batch) {
  static uint8_t buf[1024];
  bytes = bytes < 8 ? 8 : bytes > sizeof(buf) ? sizeof(buf) : bytes;
  memset(buf, 0xC3, sizeof(buf));
  uint32_t n = 0, failed = 0;
  int64_t inPut = 0;
  if (batch) zp_batch_start(z_session_loan(&sSession));
  const int64_t t0 = esp_timer_get_time(), tEnd = t0 + (int64_t)secs * 1000000;
  while (esp_timer_get_time() < tEnd) {
    memcpy(buf, &n, 4);
    z_owned_bytes_t b;
    z_bytes_copy_from_buf(&b, buf, bytes);
    const int64_t tp = esp_timer_get_time();
    const bool ok = z_publisher_put(z_publisher_loan(&sBulk), z_bytes_move(&b), NULL) == 0;
    inPut += esp_timer_get_time() - tp;
    if (ok) n++;
    else { failed++; vTaskDelay(1); }
    if ((n & 15) == 0) taskYIELD();
  }
  if (batch) zp_batch_stop(z_session_loan(&sSession));   // flushes what is left
  const double s = (esp_timer_get_time() - t0) / 1e6;
  Con.printf("zblast: %lu x %lu B in %.2f s = %.2f Mbit/s payload, %.0f msg/s, %lu failed, %s, put %.0f us\n",
             (unsigned long)n, (unsigned long)bytes, s, n * bytes * 8 / s / 1e6, n / s, (unsigned long)failed,
             batch ? "batched" : "unbatched", n + failed ? (double)inPut / (n + failed) : 0.0);
}

void zenohT1sLoop(bool netUp, int plcaId, int plcaCount) {
  if (!sWanted || !netUp) return;
  if (!sUp) {
    // retry 3 s, 6, 12 ... up to 30 s apart, so a failing open does not fill the console
    static uint32_t tTry = 0, gap = 3000;
    static uint16_t fails = 0;
    if (millis() - tTry < gap) return;
    tTry = millis();
    if (fails) gap = gap * 2 > 30000 ? 30000 : gap * 2;
    fails++;
    if (!sFixedName) snprintf(sNode, sizeof(sNode), "t1s-hat-%d", plcaId < 0 ? 0 : plcaId);
    sUp = start();
    if (sUp) { gap = 3000; fails = 0; }
    return;
  }
  if (sAckPending) {
    char out[sizeof(sAckOut)];
    portENTER_CRITICAL(&sMbx);
    memcpy(out, sAckOut, sizeof(out));
    sAckPending = false;
    portEXIT_CRITICAL(&sMbx);
    put(sAck, out);
  }
  if (sBlastSecs) {
    const uint32_t s = sBlastSecs, by = sBlastBytes;
    sBlastSecs = 0;
    runBlast(s, by, sBlastBatch);
  }
  // paused: bulk tests still run (above), nothing periodic and no echoes compete with them
  if (sPaused) { sEchoTail = sEchoHead; return; }
  while (sEchoTail != sEchoHead) {  // echo other boards' pings on their pong key
    const Echo &e = sEchoQ[sEchoTail];
    char ke[48];
    snprintf(ke, sizeof(ke), "test/pong/%s", e.who);
    z_view_keyexpr_t k;
    z_view_keyexpr_from_str_unchecked(&k, ke);
    z_owned_bytes_t b;
    z_bytes_copy_from_str(&b, e.body);
    z_put_options_t po;
    z_put_options_default(&po);
    po.is_express = true;
    if (z_put(z_session_loan(&sSession), z_view_keyexpr_loan(&k), z_bytes_move(&b), &po) == 0) sEchoed++;
    sEchoTail = (sEchoTail + 1) % 8;
  }
  static uint32_t tPing = 0, tHello = 0, tSig = 0, tStats = 0;
  const uint32_t now = millis();
  char b[96];
  if (sPingHz && now - tPing >= 1000u / sPingHz) {
    tPing = now;
    snprintf(b, sizeof(b), "%u %lld", (unsigned)sPingSeq++, (long long)esp_timer_get_time());
    put(sPing, b);
  }
  if (now - tSig >= 50) {
    tSig = now;
    snprintf(b, sizeof(b), "%.3f", 20.0 + 5.0 * sin(now / 1000.0));
    put(sSignal, b);
  }
  if (now - tHello >= 500) {
    tHello = now;
    snprintf(b, sizeof(b), "seq=%u up=%lus plca=%d/%d", (unsigned)sHelloSeq++, (unsigned long)(now / 1000),
             plcaId, plcaCount);
    put(sHello, b);
  }
  if (now - tStats >= 1000) {
    tStats = now;
    Rtt &r = sRtt;
    if (r.n) {
      snprintf(b, sizeof(b), "n=%u avg=%.2fms min=%.2fms max=%.2fms jitter=%.2fms", (unsigned)r.n,
               r.sum / 1000.0 / r.n, r.mn / 1000.0, r.mx / 1000.0, r.n > 1 ? r.jit / 1000.0 / (r.n - 1) : 0.0);
      put(sStats, b);
    }
    sRttLast = r;
    sRtt = Rtt();
  }
}

void zenohT1sPrintStatus() {
  if (!sUp) { Con.printf("zenoh: %s as %s\n", sWanted ? "connecting" : "off", sNode); return; }
  const Rtt &r = sRttLast;
  Con.printf("zenoh: up as %s via %s, ping %u Hz, sent %lu, pongs %lu, echoed %lu, last pong %lu ms ago", sNode,
             where(), (unsigned)sPingHz, (unsigned long)sSent, (unsigned long)sPongs, (unsigned long)sEchoed,
             (unsigned long)(millis() - sLastPongMs));
  if (r.n) Con.printf(", rtt avg %.2f min %.2f max %.2f ms (n=%u)", r.sum / 1000.0 / r.n, r.mn / 1000.0, r.mx / 1000.0, (unsigned)r.n);
  Con.println();
}

// zenoh [status] | ping <hz> | rtts [reset] | blast <sec> <bytes> | sink [reset]
void zenohT1sCommand(const char *args) {
  char a[16] = {}, b[16] = {}, c[16] = {}, d[16] = {};
  const int n = sscanf(args ? args : "", "%15s %15s %15s %15s", a, b, c, d);
  if (n < 1 || !strcmp(a, "status")) { zenohT1sPrintStatus(); return; }
  if (!strcmp(a, "pause") || !strcmp(a, "resume")) {
    sPaused = !strcmp(a, "pause");
    Con.printf("zenoh: %s\n", sPaused ? "paused (no periodic traffic, no echoes)" : "resumed");
  } else if (!strcmp(a, "ping") && n >= 2) {
    sPingHz = constrain(atoi(b), 0, 200);
    Con.printf("zenoh: ping %u Hz\n", (unsigned)sPingHz);
  } else if (!strcmp(a, "rtts")) {
    const uint32_t total = sRttN, m = total < 600 ? total : 600;
    static int32_t v[600], s[600];
    for (uint32_t i = 0; i < m; i++) v[i] = sRttRing[(total - m + i) % 600];
    memcpy(s, v, m * sizeof(int32_t));
    std::sort(s, s + m);
    if (m) Con.printf("zrtt: n %lu min %ld p50 %ld p99 %ld max %ld us (pongs %lu)\n", (unsigned long)m, (long)s[0],
                      (long)s[m / 2], (long)s[m * 99 / 100], (long)s[m - 1], (unsigned long)sPongs);
    else Con.println("zrtt: n 0");
    char line[256];
    for (uint32_t i = 0; i < m; i += 25) {
      int o = snprintf(line, sizeof(line), "zrtts:");
      for (uint32_t k = i; k < m && k < i + 25; k++) o += snprintf(line + o, sizeof(line) - o, " %ld", (long)v[k]);
      Con.println(line);
    }
    if (n >= 2 && !strcmp(b, "reset")) { sRttN = 0; Con.println("zrtt: reset"); }
  } else if (!strcmp(a, "blast") && n >= 2) {
    if (!sUp) { Con.println("zblast: no session"); return; }
    sBlastBytes = n >= 3 ? atoi(c) : 256;
    sBlastBatch = n >= 4 && !strcmp(d, "batch");
    sBlastSecs = constrain(atoi(b), 1, 120);
    Con.printf("zblast: queued %lu s x %lu B%s\n", (unsigned long)sBlastSecs, (unsigned long)sBlastBytes,
               sBlastBatch ? ", batched" : "");
  } else if (!strcmp(a, "sink")) {
    const BulkRx r = sBulkRx;
    const double s = r.msgs > 1 ? (r.t1 - r.t0) / 1e6 : 0;
    const uint32_t expect = r.msgs ? r.maxSeq - r.minSeq + 1 : 0;
    Con.printf("zsink: %lu msgs, %lu B in %.2f s = %.2f Mbit/s payload, %.0f msg/s, seq %lu..%lu expected %lu lost %ld\n",
               (unsigned long)r.msgs, (unsigned long)r.bytes, s, s > 0 ? r.bytes * 8 / s / 1e6 : 0.0,
               s > 0 ? r.msgs / s : 0.0, (unsigned long)(r.msgs ? r.minSeq : 0), (unsigned long)r.maxSeq,
               (unsigned long)expect, (long)expect - (long)r.msgs);
    if (n >= 2 && !strcmp(b, "reset")) { sBulkReset = true; Con.println("zsink: reset"); }
  } else {
    Con.println("zenoh [status] | pause | resume | ping <hz> | rtts [reset] | blast <sec> <bytes> [batch] | sink [reset]");
  }
}

bool zenohT1sAvailable() { return true; }

bool zenohT1sTakeConfig(char *out, size_t n) {
  bool got = false;
  portENTER_CRITICAL(&sMbx);
  if (sCfgPending) {
    strncpy(out, sCfgIn, n - 1);
    out[n - 1] = 0;
    sCfgPending = false;
    got = true;
  }
  portEXIT_CRITICAL(&sMbx);
  return got;
}

void zenohT1sAck(const char *text) {
  portENTER_CRITICAL(&sMbx);
  strncpy(sAckOut, text, sizeof(sAckOut) - 1);
  sAckOut[sizeof(sAckOut) - 1] = 0;
  sAckPending = true;
  portEXIT_CRITICAL(&sMbx);
}

struct TaskArgs { volatile bool *up; const uint8_t *id, *cnt; uint8_t off; };
static TaskArgs sArgs;

static void zenohTask(void *) {
  for (;;) {
    const uint8_t id = *sArgs.id;
    zenohT1sLoop(*sArgs.up, id == sArgs.off ? -1 : id, *sArgs.cnt);
    // sleep up to 2 ms, or until a ping arrives (its echo should not wait for the next pass)
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2));
  }
}

void zenohT1sStartTask(volatile bool *netUp, const uint8_t *plcaId, const uint8_t *plcaCount, uint8_t plcaOff,
                       const char *name) {
  if (name) {
    strncpy(sNode, name, sizeof(sNode) - 1);
    sFixedName = true;
  }
  sArgs = {netUp, plcaId, plcaCount, plcaOff};
  xTaskCreate(zenohTask, "zenoh_t1s", 12288, nullptr, 3, &sZTask);
}

#else  // built without T1S_WITH_ZENOH

void zenohT1sLoop(bool, int, int) {}
void zenohT1sPrintStatus() { Con.println("zenoh: not built in (compile with zenoh-pico)"); }
void zenohT1sCommand(const char *) { zenohT1sPrintStatus(); }
bool zenohT1sAvailable() { return false; }
void zenohT1sStartTask(volatile bool *, const uint8_t *, const uint8_t *, uint8_t, const char *) {}
bool zenohT1sTakeConfig(char *, size_t) { return false; }
void zenohT1sAck(const char *) {}
void zenohT1sTele(char *out, size_t n) { snprintf(out, n, "z 0 0 0 0 -1 0 -"); }

#endif
