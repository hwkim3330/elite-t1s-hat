// Zenoh-pico over the T1S bus. Compiled in only with -DT1S_WITH_ZENOH (plus
// --library <zenoh-pico> and -DZENOH_ARDUINO_ESP32); otherwise this file is a stub and
// t1s_node builds and runs exactly as before. An explicit flag, not __has_include: Arduino's
// library resolver only follows plain #include lines.
//
// Kept out of the .ino on purpose: Arduino's prototype pass (-CC) corrupts zenoh-pico's
// ##-heavy macros, and plain .cpp files in the sketch folder skip that pass.
//
// Once the T1S netif is up the node opens a session to the router at ZENOH_LOCATOR and:
//   t1s/<node>/hello    2 Hz   seq, uptime, PLCA as configured
//   t1s/<node>/signal  20 Hz   a synthetic sensor value (sine) — a steady small-message load
//   test/ping/<node>    5 Hz   "<seq> <t_us>"; a peer echoes it on test/pong/<node>
//   test/stats/<node>   1 Hz   RTT over the last second, measured here from the pongs
//   t1s/<node>/cmd      sub    anything sent here is printed on the console
//   t1s/<node>/config   sub    a console command (plca/csma/ip/spi/save/reboot/status), run by
//                              the console task; the result comes back on t1s/<node>/config/ack
// <node> is "t1s-hat-<PLCA id>".
#include "zenoh_t1s.h"

#include <Arduino.h>

#ifdef T1S_WITH_ZENOH
#include <cmath>
#include <esp_timer.h>
#include <zenoh-pico.h>

#ifndef ZENOH_LOCATOR
#define ZENOH_LOCATOR "udp/192.168.100.50:7447"
#endif

static bool sUp = false, sWanted = true;
static char sNode[24] = "t1s-hat-0";
static z_owned_session_t sSession;
static z_owned_publisher_t sHello, sSignal, sPing, sStats, sAck;
static z_owned_subscriber_t sPong, sCmd, sCfg;
// one-slot mailboxes between the zenoh task and the console task
static portMUX_TYPE sMbx = portMUX_INITIALIZER_UNLOCKED;
static char sCfgIn[100], sAckOut[200];
static volatile bool sCfgPending = false, sAckPending = false;
static uint32_t sPingSeq = 0, sHelloSeq = 0, sPongs = 0, sSent = 0;
static uint32_t sLastPongMs = 0;

struct Rtt { uint32_t n = 0; int64_t sum = 0, mn = INT64_MAX, mx = 0, jit = 0, last = 0; };
static Rtt sRtt, sRttLast;   // current window, last finished window (for `zenoh` status)

static void put(z_owned_publisher_t &p, const char *s) {
  z_owned_bytes_t b;
  z_bytes_copy_from_str(&b, s);
  if (z_publisher_put(z_publisher_loan(&p), z_bytes_move(&b), NULL) == 0) sSent++;
}

static void onPong(z_loaned_sample_t *sample, void *) {
  z_owned_string_t v;
  z_bytes_to_string(z_sample_payload(sample), &v);
  char buf[48];
  size_t len = z_string_len(z_string_loan(&v));
  if (len >= sizeof(buf)) len = sizeof(buf) - 1;
  memcpy(buf, z_string_data(z_string_loan(&v)), len);
  buf[len] = 0;
  z_string_drop(z_string_move(&v));
  unsigned seq;
  long long t0;
  if (sscanf(buf, "%u %lld", &seq, &t0) != 2) return;
  const int64_t rtt = esp_timer_get_time() - t0;
  if (rtt < 0) return;
  Rtt &r = sRtt;
  if (r.n) r.jit += llabs(rtt - r.last);
  r.n++; r.sum += rtt; r.mn = min(r.mn, rtt); r.mx = max(r.mx, rtt); r.last = rtt;
  sPongs++;
  sLastPongMs = millis();
}

static void onCmd(z_loaned_sample_t *sample, void *) {
  z_owned_string_t v;
  z_bytes_to_string(z_sample_payload(sample), &v);
  Serial.printf("zenoh: cmd \"%.*s\"\n", (int)z_string_len(z_string_loan(&v)), z_string_data(z_string_loan(&v)));
  z_string_drop(z_string_move(&v));
}

static void onConfig(z_loaned_sample_t *sample, void *) {
  z_owned_string_t v;
  z_bytes_to_string(z_sample_payload(sample), &v);
  size_t len = z_string_len(z_string_loan(&v));
  if (len >= sizeof(sCfgIn)) len = sizeof(sCfgIn) - 1;
  portENTER_CRITICAL(&sMbx);
  if (!sCfgPending) {
    memcpy(sCfgIn, z_string_data(z_string_loan(&v)), len);
    sCfgIn[len] = 0;
    sCfgPending = true;
  }
  portEXIT_CRITICAL(&sMbx);
  z_string_drop(z_string_move(&v));
}

static bool declarePub(z_owned_publisher_t &p, const char *ke) {
  z_view_keyexpr_t k;
  z_view_keyexpr_from_str_unchecked(&k, ke);
  return z_declare_publisher(z_session_loan(&sSession), &p, z_view_keyexpr_loan(&k), NULL) == 0;
}

static bool declareSub(z_owned_subscriber_t &s, const char *ke, void (*cb)(z_loaned_sample_t *, void *)) {
  z_view_keyexpr_t k;
  z_view_keyexpr_from_str_unchecked(&k, ke);
  z_owned_closure_sample_t c;
  z_closure_sample(&c, cb, NULL, NULL);
  return z_declare_subscriber(z_session_loan(&sSession), &s, z_view_keyexpr_loan(&k), z_closure_sample_move(&c), NULL) == 0;
}

static bool start() {
  z_owned_config_t cfg;
  z_config_default(&cfg);
  zp_config_insert(z_config_loan_mut(&cfg), Z_CONFIG_MODE_KEY, "peer");
  zp_config_insert(z_config_loan_mut(&cfg), Z_CONFIG_CONNECT_KEY, ZENOH_LOCATOR);
  Serial.printf("zenoh: opening %s as %s ...\n", ZENOH_LOCATOR, sNode);
  if (z_open(&sSession, z_config_move(&cfg), NULL) < 0) { Serial.println("zenoh: open FAILED"); return false; }
  char ke[64];
  bool ok = true;
  snprintf(ke, sizeof(ke), "t1s/%s/hello", sNode);   ok &= declarePub(sHello, ke);
  snprintf(ke, sizeof(ke), "t1s/%s/signal", sNode);  ok &= declarePub(sSignal, ke);
  snprintf(ke, sizeof(ke), "test/ping/%s", sNode);   ok &= declarePub(sPing, ke);
  snprintf(ke, sizeof(ke), "test/stats/%s", sNode);  ok &= declarePub(sStats, ke);
  snprintf(ke, sizeof(ke), "test/pong/%s", sNode);   ok &= declareSub(sPong, ke, onPong);
  snprintf(ke, sizeof(ke), "t1s/%s/cmd", sNode);     ok &= declareSub(sCmd, ke, onCmd);
  snprintf(ke, sizeof(ke), "t1s/%s/config", sNode);  ok &= declareSub(sCfg, ke, onConfig);
  snprintf(ke, sizeof(ke), "t1s/%s/config/ack", sNode); ok &= declarePub(sAck, ke);
  Serial.printf("zenoh: session %s\n", ok ? "up" : "declare FAILED");
  return ok;
}

void zenohT1sLoop(bool netUp, int plcaId, int plcaCount) {
  if (!sWanted || !netUp) return;
  if (!sUp) {
    static uint32_t tTry = 0;
    if (millis() - tTry < 3000) return;
    tTry = millis();
    snprintf(sNode, sizeof(sNode), "t1s-hat-%d", plcaId < 0 ? 0 : plcaId);
    sUp = start();
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
  static uint32_t tPing = 0, tHello = 0, tSig = 0, tStats = 0;
  const uint32_t now = millis();
  char b[96];
  if (now - tPing >= 200) {
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
  if (!sUp) { Serial.printf("zenoh: %s\n", sWanted ? "connecting" : "off"); return; }
  const Rtt &r = sRttLast;
  Serial.printf("zenoh: up as %s via %s, sent %lu, pongs %lu, last pong %lu ms ago", sNode, ZENOH_LOCATOR,
                (unsigned long)sSent, (unsigned long)sPongs, (unsigned long)(millis() - sLastPongMs));
  if (r.n) Serial.printf(", rtt avg %.2f min %.2f max %.2f ms (n=%u)", r.sum / 1000.0 / r.n, r.mn / 1000.0, r.mx / 1000.0, (unsigned)r.n);
  Serial.println();
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
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void zenohT1sStartTask(volatile bool *netUp, const uint8_t *plcaId, const uint8_t *plcaCount, uint8_t plcaOff) {
  sArgs = {netUp, plcaId, plcaCount, plcaOff};
  xTaskCreate(zenohTask, "zenoh_t1s", 12288, nullptr, 3, nullptr);
}

#else  // built without T1S_WITH_ZENOH

void zenohT1sLoop(bool, int, int) {}
void zenohT1sPrintStatus() { Serial.println("zenoh: not built in (compile with zenoh-pico)"); }
bool zenohT1sAvailable() { return false; }
void zenohT1sStartTask(volatile bool *, const uint8_t *, const uint8_t *, uint8_t) {}
bool zenohT1sTakeConfig(char *, size_t) { return false; }
void zenohT1sAck(const char *) {}

#endif
