// The zone controller's picture for the endpoint's two 10x10 LED panels (one 20x10 frame), plain C so the
// same code is tested on a PC (tests/zone_render_test.c) and runs on the ESP32.
//
// "car" view: left panel = gear letter, right panel = speed (two 3x5 digits); while an indicator lamp is lit
// a sequential amber arrow sweeps outward over its panel (one column per 25 ms from the lit edge); the two
// bottom rows red when braking, white in reverse; a red bar along the top row grows as something nears the
// proximity sensor; cyan dots in the bottom corners while Autoware / the autopilot drives.
// In the pixels left over: the pedals as bars on the two outer columns, growing upward from row 7 to row 1
// (column 0 = brake, red; column 19 = accelerator, green; 7 px = fully pressed), and the steering wheel as a
// 2-px marker on row 1 (columns 9-10 at 0 deg, one column per 450/8 deg, left turn = left; white, violet past
// +-90 deg). A panel showing an indicator arrow shows neither its pedal bar nor the wheel marker.
// "no link" view: no vehicle state for a while -- a dim dashed line, so a stale picture never looks live.
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ZR_W 20
#define ZR_H 10

typedef uint8_t zr_fb[ZR_H][ZR_W][3];

enum { ZR_LEFT = 1, ZR_RIGHT = 2, ZR_BRAKE = 4, ZR_REVERSE = 8, ZR_HAZARD = 16 };
enum { ZR_MANUAL = 0, ZR_AUTOPILOT = 1, ZR_AUTOWARE = 2 };

typedef struct {
  uint8_t lamps;      // ZR_LEFT/RIGHT = that lamp is lit right now; BRAKE, REVERSE
  char gear;          // 'P' 'R' 'N' 'D'
  double speed;       // km/h (negative in reverse)
  uint8_t mode;       // ZR_MANUAL / ZR_AUTOPILOT / ZR_AUTOWARE
  double prox_m;      // proximity, metres (>= 1 or < 0: no bar)
  double wheel_deg;   // steering wheel angle, degrees, + = left (the simulator's convention)
  double accel;       // accelerator pedal 0..1
  double brake;       // brake pedal 0..1
} zr_state;

static const uint8_t ZR_AMBER[3] = {255, 110, 0}, ZR_RED[3] = {255, 0, 0}, ZR_WHITE[3] = {200, 200, 200},
                     ZR_CYAN[3] = {0, 160, 255}, ZR_GREEN[3] = {0, 220, 60}, ZR_VIOLET[3] = {170, 60, 255};

// gear letters as drawn by the bench's reference (PIL's default bitmap font): width, 10 rows of bits (bit x)
typedef struct { char c; uint8_t w; uint16_t rows[10]; uint8_t rgb[3]; } zr_glyph;
static const zr_glyph ZR_GEARS[4] = {
  {'P', 7, {0, 0, 30, 34, 34, 34, 30, 2, 2, 2}, {255, 60, 60}},
  {'R', 7, {0, 0, 30, 34, 34, 34, 30, 50, 34, 34}, {230, 230, 230}},
  {'N', 9, {0, 0, 38, 38, 38, 42, 42, 42, 50, 50}, {255, 170, 0}},
  {'D', 8, {0, 0, 30, 34, 66, 66, 66, 66, 34, 30}, {40, 255, 90}},
};
static const char *ZR_DIGITS[10] = {   // 3x5, row-major
  "111101101101111", "010110010010111", "111001111100111", "111001111001111", "101101111001001",
  "111100111001111", "111100111101111", "111001010010010", "111101111101111", "111101111001111"};
static const char *ZR_ARROW[10] = {"....#.....", "...##.....", "..###.....", ".#########", "##########",
                                   "##########", ".#########", "..###.....", "...##.....", "....#....."};

static inline void zr_set(zr_fb fb, int y, int x, const uint8_t c[3]) { memcpy(fb[y][x], c, 3); }
static inline int zr_off(zr_fb fb, int y, int x) { return !fb[y][x][0] && !fb[y][x][1] && !fb[y][x][2]; }

// the arrow sweep: `since` = seconds since that lamp's lit edge
static inline void zr_arrow(zr_fb fb, int panel, double since) {
  int cols = (int)(since / 0.025) + 1;
  if (cols > 10) cols = 10;
  for (int y = 0; y < 10; y++)
    for (int x = 0; x < 10; x++)
      if (ZR_ARROW[y][x] == '#' && (9 - x) < cols) zr_set(fb, y, panel == 0 ? x : ZR_W - 1 - x, ZR_AMBER);
}

static inline void zr_car(zr_fb fb, const zr_state *s, double since_l, double since_r) {
  memset(fb, 0, sizeof(zr_fb));
  if (s->lamps & ZR_LEFT) zr_arrow(fb, 0, since_l);
  else
    for (int i = 0; i < 4; i++)
      if (ZR_GEARS[i].c == s->gear) {
        const zr_glyph *g = &ZR_GEARS[i];
        const int ox = (10 - g->w) / 2 + 1;
        for (int y = 0; y < 10; y++)
          for (int x = 0; x < g->w; x++)
            if ((g->rows[y] >> x) & 1 && x + ox >= 0 && x + ox < 10) zr_set(fb, y, x + ox, g->rgb);
      }
  if (s->lamps & ZR_RIGHT) zr_arrow(fb, 1, since_r);
  else {
    int v = (int)rint((double)s->speed);           // Python's round(): half to even
    if (v < 0) v = -v;
    if (v > 99) v = 99;
    const int d[2] = {v / 10, v % 10};
    for (int i = 0; i < 2; i++)
      for (int k = 0; k < 15; k++)
        if (ZR_DIGITS[d[i]][k] == '1') zr_set(fb, 2 + k / 3, 11 + i * 4 + k % 3, ZR_WHITE);
  }
  // pedal bars (outer columns) and the wheel marker (row 1), kept off a panel that shows an arrow
  const int arrowL = (s->lamps & ZR_LEFT) != 0, arrowR = (s->lamps & ZR_RIGHT) != 0;
  const double pedal[2] = {s->brake, s->accel};
  for (int i = 0; i < 2; i++) {
    if (i == 0 ? arrowL : arrowR) continue;
    double v = pedal[i] < 0 ? 0 : pedal[i] > 1 ? 1 : pedal[i];
    const int n = (int)rint(v * 7);
    for (int k = 0; k < n; k++) zr_set(fb, 7 - k, i == 0 ? 0 : ZR_W - 1, i == 0 ? ZR_RED : ZR_GREEN);
  }
  {
    double w = s->wheel_deg / 450.0;
    if (w < -1) w = -1;
    if (w > 1) w = 1;
    const int k = -(int)rint(w * 8);
    const uint8_t *c = fabs(s->wheel_deg) > 90 ? ZR_VIOLET : ZR_WHITE;
    for (int x = 9 + k; x <= 10 + k; x++)
      if (!(x < 10 ? arrowL : arrowR)) zr_set(fb, 1, x, c);
  }
  if (s->lamps & (ZR_BRAKE | ZR_REVERSE)) {
    const uint8_t *c = (s->lamps & ZR_REVERSE) ? ZR_WHITE : ZR_RED;
    for (int y = 8; y < 10; y++)
      for (int x = 0; x < ZR_W; x++)
        if (zr_off(fb, y, x)) zr_set(fb, y, x, c);
  }
  if (s->prox_m >= 0 && s->prox_m < 1.0) {
    const int n = (int)rint((1.0 - (double)s->prox_m) * ZR_W);
    for (int x = 0; x < n && x < ZR_W; x++) zr_set(fb, 0, x, ZR_RED);
  }
  if (s->mode == ZR_AUTOPILOT || s->mode == ZR_AUTOWARE) {
    zr_set(fb, 9, 0, ZR_CYAN); zr_set(fb, 9, 9, ZR_CYAN); zr_set(fb, 9, 10, ZR_CYAN); zr_set(fb, 9, 19, ZR_CYAN);
  }
}

// no vehicle state: a dim dashed line across the middle, moving so the panels show the node is alive.
// tick = frame count at 20 frames/s (the dashes move one pixel every 4 frames)
static inline void zr_nolink(zr_fb fb, uint32_t tick) {
  static const uint8_t dim[3] = {40, 0, 0};
  memset(fb, 0, sizeof(zr_fb));
  for (int x = 0; x < ZR_W; x++)
    if ((x + tick / 4) % 4 < 2) { zr_set(fb, 4, x, dim); zr_set(fb, 5, x, dim); }
}

// the brightness setting (kept as a float) as the factor the PC uses: hundredths, as a double. 0.35f is
// 0.34999999, which would make 200 x b = 69 where the PC gets 70.
static inline double zr_bright(float b) { return rint((double)b * 100) / 100.0; }

// brightness as the bench applies it: each channel int(c * b)
static inline void zr_scale(zr_fb out, const zr_fb in, double b) {
  for (int y = 0; y < ZR_H; y++)
    for (int x = 0; x < ZR_W; x++)
      for (int k = 0; k < 3; k++) out[y][x][k] = (uint8_t)((double)in[y][x][k] * b);
}

// proximity counts (VCNL4200, 16 bit, 200 mA) -> a rough distance: under 4 counts = nothing (2 m),
// else 3/sqrt(counts) clamped to 0.03..2 m (the bench's reference; not a measurement)
static inline double zr_prox_m(int raw) {
  if (raw < 4) return 2.0;
  double m = 3.0 / sqrt((double)raw);
  if (m < 0.03) m = 0.03;
  if (m > 2.0) m = 2.0;
  return m;
}

#ifdef __cplusplus
}
#endif
