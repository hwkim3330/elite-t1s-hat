// Host build of the zone controller's LED renderer and RCP encoder, for comparison with a reference.
//   cc -O1 -Wall -I../t1s_node -o zone_render_test zone_render_test.c -lm
//   render: stdin lines "lamps gear speed mode prox_m since_l since_r bright wheel_deg accel brake"
//                                                                               -> one line of 1200 hex chars
//   no link: "nolink <tick> <bright>"                                           -> the same
//   wtlv:   stdin lines "wtlv <kind> args..."                                    -> the payload bytes in hex
//   header: "wtlv header <method> <session> <payload length>"                   -> the 16 SOME/IP header bytes
// The comparison with the Python references is driven from the bench's tests/compare_zone_c.py.
#include <stdio.h>
#include <string.h>
#include "zone_render.h"
#include "zone_wtlv.h"

static void hex(const uint8_t *p, int n) {
  for (int i = 0; i < n; i++) printf("%02x", p[i]);
  printf("\n");
}

int main(void) {
  char line[512];
  while (fgets(line, sizeof line, stdin)) {
    if (!strncmp(line, "wtlv ", 5)) {
      uint8_t b[256];
      int n = 0, h, a, r, w, x[8];
      unsigned u;
      char kind[32];
      sscanf(line + 5, "%31s", kind);
      if (!strcmp(kind, "open_i2c") && sscanf(line + 5, "%*s %d %d %d", &x[0], &x[1], &x[2]) == 3)
        n = zw_open_i2c(b, x[0], x[1], x[2]);
      else if (!strcmp(kind, "i2c_wr") && sscanf(line + 5, "%*s %d %d %d %u %d", &h, &a, &r, &u, &w) == 5) {
        const uint8_t reg = (uint8_t)w;
        n = zw_i2c_wr(b, h, a, r, u, &reg, 1);
      } else if (!strcmp(kind, "i2c_write") && sscanf(line + 5, "%*s %d %d %u %d %d %d", &h, &a, &u, &x[0], &x[1], &x[2]) == 6) {
        const uint8_t d[3] = {(uint8_t)x[0], (uint8_t)x[1], (uint8_t)x[2]};
        n = zw_i2c_write(b, h, a, u, d, 3);
      } else if (!strcmp(kind, "open_spi") && sscanf(line + 5, "%*s %d %d %d %d %d %u", &x[0], &x[1], &x[2], &x[3], &x[4], &u) == 6)
        n = zw_open_spi(b, x[0], x[1], x[2], x[3], x[4], u);
      else if (!strcmp(kind, "spi_xfer") && sscanf(line + 5, "%*s %d %u %d %d %d", &h, &u, &x[0], &x[1], &x[2]) == 5) {
        const uint8_t d[3] = {(uint8_t)x[0], (uint8_t)x[1], (uint8_t)x[2]};
        n = zw_spi_xfer(b, h, u, d, 3);
      } else if (!strcmp(kind, "release") && sscanf(line + 5, "%*s %d %d %d %d", &x[0], &x[1], &x[2], &x[3]) == 4) {
        const uint8_t pins[4] = {(uint8_t)x[0], (uint8_t)x[1], (uint8_t)x[2], (uint8_t)x[3]};
        n = zw_release(b, pins, 4);
      } else if (!strcmp(kind, "close") && sscanf(line + 5, "%*s %d", &h) == 1)
        n = zw_close(b, h);
      else if (!strcmp(kind, "header") && sscanf(line + 5, "%*s %d %d %d", &x[0], &x[1], &x[2]) == 3)
        n = zw_header(b, x[0], x[1], x[2]);
      hex(b, n);
      continue;
    }
    zr_state s;
    char gear;
    unsigned lamps, mode, tick;
    double since_l, since_r, bright, speed, prox, wheel, accel, brake;
    zr_fb fb, out;
    if (sscanf(line, "nolink %u %lf", &tick, &bright) == 2) {
      zr_nolink(fb, tick);
      zr_scale(out, (const uint8_t(*)[ZR_W][3])fb, bright);
      hex(&out[0][0][0], sizeof out);
      continue;
    }
    if (sscanf(line, "%u %c %lf %u %lf %lf %lf %lf %lf %lf %lf", &lamps, &gear, &speed, &mode, &prox, &since_l, &since_r, &bright,
               &wheel, &accel, &brake) != 11) continue;
    s.lamps = (uint8_t)lamps; s.gear = gear; s.speed = speed; s.mode = (uint8_t)mode; s.prox_m = prox;
    s.wheel_deg = wheel; s.accel = accel; s.brake = brake;
    zr_car(fb, &s, since_l, since_r);
    zr_scale(out, (const uint8_t(*)[ZR_W][3])fb, bright);
    hex(&out[0][0][0], sizeof out);
  }
  return 0;
}
