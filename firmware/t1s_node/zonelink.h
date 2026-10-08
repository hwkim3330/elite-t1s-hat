// ZoneLink: the zone controller on Zenoh with COVESA VSS keys (lower-case, '/' for '.'), through a zenohd router on
// the bench host (`zenoh router udp/<host>:7447`). Payload: one little-endian scalar per key.
//   in   vehicle/speed f32, .../selectedgear i8 (126 P, 127 D, 0 N, -1 R), steeringwheel/angle i16, accelerator and
//        brake pedalposition u8 %, private/drivingmode u8, lamps u8 0/1 -> the LED picture (zone.h applyState)
//   out  vehicle/private/zone/stick/x|y u16, vehicle/private/zone/proximity/raw u16 (0xFFFF = not read),
//        vehicle/private/external/proximity_rear f32 m (-1 = not read), and per stick flick
//        vehicle/powertrain/transmission/selectedgear/target i8 + vehicle/private/zone/stick/gearflick u8 (ASCII)
// Priorities: actuator requests and proximity RealTime (1), stick InteractiveHigh (2).
// Without Zenoh state for 1 s the zone falls back to VST2/SNS1 by itself (zone.h zlFresh).
#pragma once
#include "zone.h"
#include "zenoh_t1s.h"

namespace zl {
enum { SPEED, GEAR, WHEEL, ACC, BRK, MODE, LLIT, RLIT, BLAMP, BACKUP, HAZ, NV };
static const char *const kIn[NV] = {
    "speed", "powertrain/transmission/selectedgear", "chassis/steeringwheel/angle", "chassis/accelerator/pedalposition",
    "chassis/brake/pedalposition", "private/drivingmode", "private/lights/directionindicator/left/islit",
    "private/lights/directionindicator/right/islit", "body/lights/brake/isactive", "body/lights/backup/ison",
    "body/lights/hazard/issignaling"};
static double gV[NV] = {0, 126};
static volatile uint32_t gRx = 0, gTx = 0;
enum { P_X, P_Y, P_RAW, P_PROX, P_GEAR, P_FLICK, NP };
static int gPub[NP];

static void onVehicle(const char *key, size_t kl, const uint8_t *p, size_t n) {
  if (kl < 8 || kl > 80) return;
  const char *k = key + 8;                          // after "vehicle/"
  const size_t l = kl - 8;
  for (int i = 0; i < NV; i++) {
    if (strlen(kIn[i]) != l || memcmp(kIn[i], k, l)) continue;
    if (i == SPEED && n == 4) { float f; memcpy(&f, p, 4); gV[i] = f; }
    else if (i == GEAR && n >= 1) gV[i] = (int8_t)p[0];
    else if (i == WHEEL && n >= 2) { int16_t v; memcpy(&v, p, 2); gV[i] = v; }
    else if (n >= 1) gV[i] = p[0];
    gRx = gRx + 1;
    const int g = (int)gV[GEAR];
    const uint8_t lamps = (gV[LLIT] ? ZR_LEFT : 0) | (gV[RLIT] ? ZR_RIGHT : 0) | (gV[BLAMP] ? 4 : 0) | (gV[BACKUP] ? 8 : 0) |
                          (gV[HAZ] ? 16 : 0);
    zone::gZlAt = millis();
    zone::applyState(lamps, g == 126 ? 'P' : g == 127 ? 'D' : g == 0 ? 'N' : g < 0 ? 'R' : 'D', gV[SPEED], (uint8_t)gV[MODE],
                     gV[ACC] / 100.0, gV[WHEEL], gV[BRK] / 100.0);
    return;
  }
}

static void task(void *) {
  uint16_t last[3] = {0, 0, 0};
  float lastProx = -2;
  uint8_t seen = zone::gEventCount;
  uint32_t tAll = 0;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(50));
    if (!zone::gWant) continue;
    const bool all = millis() - tAll >= 1000;
    if (all) tAll = millis();
    const uint16_t v[3] = {(uint16_t)(zone::gX < 0 ? 0xFFFF : zone::gX), (uint16_t)(zone::gY < 0 ? 0xFFFF : zone::gY),
                           (uint16_t)(zone::gProx < 0 ? 0xFFFF : zone::gProx)};
    for (int i = 0; i < 3; i++)
      if (all || v[i] != last[i]) { if (zenohT1sPut(gPub[P_X + i], &v[i], 2)) gTx = gTx + 1; last[i] = v[i]; }
    const float pm = zone::gProx < 0 ? -1.0f : (float)(rint(zr_prox_m(zone::gProx) * 100) / 100);
    if (all || pm != lastProx) { if (zenohT1sPut(gPub[P_PROX], &pm, 4)) gTx = gTx + 1; lastProx = pm; }
    if (zone::gEventCount != seen) {
      seen = zone::gEventCount;
      if (zone::zlFresh()) {                        // otherwise SNS1 carries it (fallback)
        const char e = zone::gEvent;
        const int8_t g = e == 'P' ? 126 : e == 'D' ? 127 : e == 'N' ? 0 : -1;
        zenohT1sPut(gPub[P_GEAR], &g, 1);
        zenohT1sPut(gPub[P_FLICK], &e, 1);
        gTx = gTx + 2;
      }
    }
  }
}

static void begin() {
  zenohT1sAddSub("vehicle/**", onVehicle);
  gPub[P_X] = zenohT1sAddPub("vehicle/private/zone/stick/x", 2);
  gPub[P_Y] = zenohT1sAddPub("vehicle/private/zone/stick/y", 2);
  gPub[P_RAW] = zenohT1sAddPub("vehicle/private/zone/proximity/raw", 1);
  gPub[P_PROX] = zenohT1sAddPub("vehicle/private/external/proximity_rear", 1);
  gPub[P_GEAR] = zenohT1sAddPub("vehicle/powertrain/transmission/selectedgear/target", 1);
  gPub[P_FLICK] = zenohT1sAddPub("vehicle/private/zone/stick/gearflick", 1);
  xTaskCreate(task, "zonelink", 3072, nullptr, 2, nullptr);
}

static void status() {
  Con.printf("zonelink: vehicle keys in %lu, last %ld ms ago (%s), keys out %lu\n", (unsigned long)gRx,
             zone::gZlAt ? (long)(millis() - zone::gZlAt) : -1L, zone::zlFresh() ? "state from Zenoh" : "fallback VST2/SNS1",
             (unsigned long)gTx);
}
}  // namespace zl
