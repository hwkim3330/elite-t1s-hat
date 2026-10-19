// The zone controller's picture for the endpoint's two 10x10 LED panels (one 20x10 frame), plain C so the
// same code is tested on a PC (tests/zone_render_test.c) and runs on the ESP32. Fonts, icons and colours come from
// zone_glyphs.h, generated from glyphs/zone_glyphs.txt (the bench console's Python renderer uses the same source).
//
// "car" view (zr_car):
//   row 0        the steering wheel as a 2-px marker (columns 9-10 at 0 deg, one column per 450/8 deg, left turn =
//                left; white, violet past +-90 deg); a red proximity bar from the left wins over it, longer the
//                nearer (from 1 m)
//   left panel   the gear letter, 5x7 (columns 5-9, rows 1-7), and two 3x3 tell-tale slots (columns 1-3):
//                rows 1-3 the lamps, one at a time: high beam (blue; centre dot = auto high beam drives it) >
//                rear fog (amber) > front fog (green) > low beam (green; centre dot = auto high beam ready);
//                rows 5-7 hazard (red triangle) > interior light (white)
//   right panel  the speed in km/h, 3x5 digits on rows 1-5: 0-99 on columns 11-13 / 15-17, 100-199 with a
//                one-column '1' on column 10 and the others on 12-14 / 16-18; the assist line on row 7
//                (columns 11-17): cyan while the autopilot / Autoware drives, else green with SCC or lane keeping
//   columns 0/19 the pedals as bars growing upward from row 7 to row 1 (brake red, accelerator green; 7 px = fully
//                pressed)
//   rows 8-9     red when braking, white in reverse, where nothing else is lit
// While an indicator lamp is lit its panel shows only a sequential amber arrow sweeping outward (one column per
// 25 ms from the lit edge).
// "no link" view (zr_nolink): no vehicle state for a while -- a dim dashed line, so a stale picture never looks live.
// Text (zr_text): ASCII in the 5x8 text font, its 8-row cell on rows 1-8, scrolled by the caller.
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "zone_glyphs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ZR_W 20
#define ZR_H 10

typedef uint8_t zr_fb[ZR_H][ZR_W][3];

enum { ZR_LEFT = 1, ZR_RIGHT = 2, ZR_BRAKE = 4, ZR_REVERSE = 8, ZR_HAZARD = 16, ZR_HIGH_BEAM = 32, ZR_LOW_BEAM = 64,
       ZR_INTERIOR = 128 };
// tell-tale states beside the lamps (state_in `tells`)
enum { ZR_HBA = 1, ZR_HBA_ACTIVE = 2, ZR_FOG_FRONT = 4, ZR_FOG_REAR = 8, ZR_SCC = 16, ZR_LKA = 32 };
enum { ZR_MANUAL = 0, ZR_AUTOPILOT = 1, ZR_AUTOWARE = 2 };

typedef struct {
  uint8_t lamps;      // ZR_LEFT/RIGHT = that lamp is lit right now; BRAKE, REVERSE, HAZARD (the switch), HIGH_BEAM, ...
  uint8_t tells;      // ZR_HBA (auto high beam on), HBA_ACTIVE (it drives the high beam), FOG_FRONT/REAR, SCC, LKA
  char gear;          // 'P' 'R' 'N' 'D'
  double speed;       // km/h (negative in reverse)
  uint8_t mode;       // ZR_MANUAL / ZR_AUTOPILOT / ZR_AUTOWARE
  double prox_m;      // proximity, metres (>= 1 or < 0: no bar)
  double wheel_deg;   // steering wheel angle, degrees, + = left (the simulator's convention)
  double accel;       // accelerator pedal 0..1
  double brake;       // brake pedal 0..1
} zr_state;

static inline void zr_set(zr_fb fb, int y, int x, const uint8_t c[3]) { memcpy(fb[y][x], c, 3); }
static inline int zr_off(zr_fb fb, int y, int x) { return !fb[y][x][0] && !fb[y][x][1] && !fb[y][x][2]; }

// a glyph with its top-left at (ox, oy), clipped to the frame
static inline void zr_blit(zr_fb fb, const zg_glyph *g, int ox, int oy, const uint8_t c[3]) {
  if (!g) return;
  for (int y = 0; y < g->h; y++)
    for (int x = 0; x < g->w; x++)
      if ((g->rows[y] >> x) & 1 && ox + x >= 0 && ox + x < ZR_W && oy + y >= 0 && oy + y < ZR_H) zr_set(fb, oy + y, ox + x, c);
}

// the arrow sweep: `since` = seconds since that lamp's lit edge
static inline void zr_arrow(zr_fb fb, int panel, double since) {
  int cols = (int)(since / 0.025) + 1;
  if (cols > 10) cols = 10;
  const zg_glyph *g = ZG_ARROW_LEFT;
  for (int y = 0; y < g->h; y++)
    for (int x = 0; x < g->w; x++)
      if ((g->rows[y] >> x) & 1 && (9 - x) < cols) zr_set(fb, y, panel == 0 ? x : ZR_W - 1 - x, ZG_AMBER);
}

static inline const zg_glyph *zr_digit(int d) { return &ZG_DIGIT[d]; }   // the digit font is in order 0-9

static inline void zr_speed(zr_fb fb, double speed) {
  int v = (int)rint(speed);                       // Python's round(): half to even
  if (v < 0) v = -v;
  if (v > 199) v = 199;
  const int x = v >= 100 ? 16 : 15;
  zr_blit(fb, zr_digit(v % 10), x, 1, ZG_WHITE);
  if (v >= 10) zr_blit(fb, zr_digit(v / 10 % 10), x - 4, 1, ZG_WHITE);
  if (v >= 100) zr_blit(fb, &ZG_NARROW[0], 10, 1, ZG_WHITE);
}

static inline void zr_tells(zr_fb fb, const zr_state *s) {
  int dot = 0;
  if (s->lamps & ZR_HIGH_BEAM) { zr_blit(fb, ZG_ICON_HIGH, 1, 1, ZG_TT_BLUE); dot = (s->tells & ZR_HBA_ACTIVE) != 0; }
  else if (s->tells & ZR_FOG_REAR) zr_blit(fb, ZG_ICON_FOG, 1, 1, ZG_TT_AMBER);
  else if (s->tells & ZR_FOG_FRONT) zr_blit(fb, ZG_ICON_FOG, 1, 1, ZG_TT_GREEN);
  else if (s->lamps & ZR_LOW_BEAM) { zr_blit(fb, ZG_ICON_LOW, 1, 1, ZG_TT_GREEN); dot = (s->tells & ZR_HBA) != 0; }
  if (dot) zr_set(fb, 2, 2, ZG_TT_DOT);
  if (s->lamps & ZR_HAZARD) zr_blit(fb, ZG_ICON_HAZARD, 1, 5, ZG_TT_RED);
  else if (s->lamps & ZR_INTERIOR) zr_blit(fb, ZG_ICON_INTERIOR, 1, 5, ZG_TT_WHITE);
}

static inline void zr_assist(zr_fb fb, const zr_state *s) {
  const uint8_t *c = (s->mode == ZR_AUTOPILOT || s->mode == ZR_AUTOWARE) ? ZG_TT_CYAN
                     : (s->tells & (ZR_SCC | ZR_LKA)) ? ZG_TT_GREEN : 0;
  if (c)
    for (int x = 11; x <= 17; x++) zr_set(fb, 7, x, c);
}

// proximity warning: a red bar on the top row, longer the closer (from 1 m), the same in both states
static inline void zr_prox_bar(zr_fb fb, double prox_m) {
  if (prox_m >= 0 && prox_m < 1.0) {
    const int n = (int)rint((1.0 - prox_m) * ZR_W);
    for (int x = 0; x < n && x < ZR_W; x++) zr_set(fb, 0, x, ZG_RED);
  }
}

static inline void zr_car(zr_fb fb, const zr_state *s, double since_l, double since_r) {
  memset(fb, 0, sizeof(zr_fb));
  const int arrowL = (s->lamps & ZR_LEFT) != 0, arrowR = (s->lamps & ZR_RIGHT) != 0;
  if (arrowL) zr_arrow(fb, 0, since_l);
  else {
    const char name[2] = {s->gear, 0};
    const zg_glyph *g = zg_find(ZG_GEAR, ZG_GEAR_COUNT, name);
    const uint8_t *c = s->gear == 'P' ? ZG_GEAR_P : s->gear == 'R' ? ZG_GEAR_R : s->gear == 'N' ? ZG_GEAR_N : ZG_GEAR_D;
    zr_blit(fb, g, 5, 1, c);
    zr_tells(fb, s);
  }
  if (arrowR) zr_arrow(fb, 1, since_r);
  else {
    zr_speed(fb, s->speed);
    zr_assist(fb, s);
  }
  // pedal bars (outer columns) and the wheel marker (row 0), kept off a panel that shows an arrow
  const double pedal[2] = {s->brake, s->accel};
  for (int i = 0; i < 2; i++) {
    if (i == 0 ? arrowL : arrowR) continue;
    double v = pedal[i] < 0 ? 0 : pedal[i] > 1 ? 1 : pedal[i];
    const int n = (int)rint(v * 7);
    for (int k = 0; k < n; k++) zr_set(fb, 7 - k, i == 0 ? 0 : ZR_W - 1, i == 0 ? ZG_RED : ZG_GREEN);
  }
  {
    double w = s->wheel_deg / 450.0;
    if (w < -1) w = -1;
    if (w > 1) w = 1;
    const int k = -(int)rint(w * 8);
    const uint8_t *c = fabs(s->wheel_deg) > 90 ? ZG_VIOLET : ZG_WHITE;
    for (int x = 9 + k; x <= 10 + k; x++)
      if (!(x < 10 ? arrowL : arrowR)) zr_set(fb, 0, x, c);
  }
  if (s->lamps & (ZR_BRAKE | ZR_REVERSE)) {
    const uint8_t *c = (s->lamps & ZR_REVERSE) ? ZG_WHITE : ZG_RED;
    for (int y = 8; y < 10; y++)
      for (int x = 0; x < ZR_W; x++)
        if (zr_off(fb, y, x)) zr_set(fb, y, x, c);
  }
  zr_prox_bar(fb, s->prox_m);
}

// text in the text font: each glyph's ink width, one dark column between glyphs
static inline int zr_text_width(const char *s) {
  int w = 0;
  for (; *s; s++) w += zg_text(*s)->w + 1;
  return w ? w - 1 : 0;
}

// s with its first column at x = off (may be off the panels), the 8-row cell on rows 1-8
static inline void zr_text(zr_fb fb, const char *s, int off, const uint8_t c[3]) {
  memset(fb, 0, sizeof(zr_fb));
  for (int x = off; *s && x < ZR_W; s++) {
    const zg_glyph *g = zg_text(*s);
    if (x + g->w > 0) zr_blit(fb, g, x, 1, c);
    x += g->w + 1;
  }
}

// no vehicle state: a dim dashed line across the middle, moving so the panels show the node is alive.
// tick = frame count at 20 frames/s (the dashes move one pixel every 4 frames). The proximity warning is the
// zone's own sensor, so it stays on without the PC (prox_m < 0: none)
static inline void zr_nolink(zr_fb fb, uint32_t tick, double prox_m) {
  memset(fb, 0, sizeof(zr_fb));
  for (int x = 0; x < ZR_W; x++)
    if ((x + tick / 4) % 4 < 2) { zr_set(fb, 4, x, ZG_DIM); zr_set(fb, 5, x, ZG_DIM); }
  zr_prox_bar(fb, prox_m);
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
