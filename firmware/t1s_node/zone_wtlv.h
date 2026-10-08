// OPEN Alliance TC18 Remote Control Protocol (RCP) requests as the LAN866x endpoints take them: SOME/IP
// (service 0xff10, client 0xaffe, protocol/interface version 1) with a WTLV payload -- each field a 16-bit tag,
// wire type << 12 | field number (0, 1, 2 ... in order), wire types 0/1/2 = u8/u16/u32, 6 = blob with a
// 16-bit length. Plain C (host-tested in tests/zone_render_test.c).
// Return codes seen: 0 OK, 5 NOT_REACHABLE (peripheral not opened / still held by another handle),
// 32 WriteId out of sequence (each handle's WriteId starts at 0 and steps by 1), 33 I2C address NACK.
#pragma once
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
  ZW_OPEN_I2C = 0x1200, ZW_CLOSE_I2C = 0x1202, ZW_WRITE_I2C = 0x1204, ZW_WRITE_READ_I2C = 0x1208,
  ZW_RELEASE_PINS = 0x1105, ZW_OPEN_SPI = 0x1500, ZW_CLOSE_SPI = 0x1502, ZW_WRITE_READ_SPI = 0x1508,
  ZW_GET_STATUS = 0x1002,
};

static inline int zw_u8(uint8_t *p, int f, uint8_t v) { p[0] = (f >> 8) & 0x0F; p[1] = f; p[2] = v; return 3; }
static inline int zw_u16(uint8_t *p, int f, uint16_t v) { p[0] = 0x10 | ((f >> 8) & 0x0F); p[1] = f; p[2] = v >> 8; p[3] = v; return 4; }
static inline int zw_u32(uint8_t *p, int f, uint32_t v) {
  p[0] = 0x20 | ((f >> 8) & 0x0F); p[1] = f; p[2] = v >> 24; p[3] = v >> 16; p[4] = v >> 8; p[5] = v; return 6;
}
static inline int zw_blob(uint8_t *p, int f, const uint8_t *d, int n) {
  p[0] = 0x60 | ((f >> 8) & 0x0F); p[1] = f; p[2] = n >> 8; p[3] = n; memcpy(p + 4, d, n); return 4 + n;
}

// the 16-byte SOME/IP header in front of a payload of n bytes
static inline int zw_header(uint8_t *p, uint16_t method, uint16_t session, int n) {
  const uint32_t len = 8 + n;
  const uint8_t h[16] = {0xFF, 0x10, (uint8_t)(method >> 8), (uint8_t)method, (uint8_t)(len >> 24), (uint8_t)(len >> 16),
                         (uint8_t)(len >> 8), (uint8_t)len, 0xAF, 0xFE, (uint8_t)(session >> 8), (uint8_t)session, 1, 1, 0, 0};
  memcpy(p, h, 16);
  return 16;
}

static inline int zw_open_i2c(uint8_t *p, int sda, int scl, int speed) {
  int n = zw_u8(p, 0, sda); n += zw_u8(p + n, 1, scl); return n + zw_u8(p + n, 2, speed);
}
static inline int zw_i2c_wr(uint8_t *p, uint16_t h, uint16_t addr, uint16_t rd, uint32_t wid, const uint8_t *wr, int nw) {
  int n = zw_u16(p, 0, h); n += zw_u16(p + n, 1, addr); n += zw_u16(p + n, 2, rd); n += zw_u32(p + n, 3, wid);
  return n + zw_blob(p + n, 4, wr, nw);
}
static inline int zw_i2c_write(uint8_t *p, uint16_t h, uint16_t addr, uint32_t wid, const uint8_t *wr, int nw) {
  int n = zw_u16(p, 0, h); n += zw_u16(p + n, 1, addr); n += zw_u32(p + n, 2, wid); return n + zw_blob(p + n, 3, wr, nw);
}
static inline int zw_open_spi(uint8_t *p, int miso, int sck, int cs, int mosi, int mode, uint32_t hz) {
  int n = zw_u8(p, 0, miso); n += zw_u8(p + n, 1, sck); n += zw_u8(p + n, 2, cs); n += zw_u8(p + n, 3, mosi);
  n += zw_u8(p + n, 4, mode); return n + zw_u32(p + n, 5, hz);
}
static inline int zw_spi_xfer(uint8_t *p, uint16_t h, uint32_t wid, const uint8_t *wr, int nw) {
  int n = zw_u16(p, 0, h); n += zw_u16(p + n, 1, nw); n += zw_u32(p + n, 2, wid); return n + zw_blob(p + n, 3, wr, nw);
}
static inline int zw_release(uint8_t *p, const uint8_t *pins, int n) { return zw_blob(p, 0, pins, n); }
static inline int zw_close(uint8_t *p, uint16_t h) { return zw_u16(p, 0, h); }

// field f of a WTLV reply payload: pointer to its value (a blob: after the length), *len = its size; or 0
static inline const uint8_t *zw_field(const uint8_t *p, int n, int f, int *len) {
  const uint8_t *end = p + n;
  while (p + 2 <= end) {
    const int t = p[0] >> 4, id = ((p[0] & 0x0F) << 8) | p[1];
    int l;
    p += 2;
    if (t == 6) { if (p + 2 > end) return 0; l = (p[0] << 8) | p[1]; p += 2; }
    else if (t <= 3) l = 1 << t;
    else return 0;
    if (p + l > end) return 0;
    if (id == f) { *len = l; return p; }
    p += l;
  }
  return 0;
}

#ifdef __cplusplus
}
#endif
