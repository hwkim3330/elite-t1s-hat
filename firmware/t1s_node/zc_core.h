// zc_core.h -- the "ZC" core frame: one fixed 20-byte header in front of every zone message, version 1.0.
// Plain C99 (and C++), header only, no allocation, no Arduino: the same file builds on the ESP and on a PC
// (firmware/tests/zc_core_test.c checks it against the golden vectors in firmware/tests/zc_vectors.txt; the bench
// console's Python encoder is checked against the same vectors).
//
// Byte order: LITTLE-endian everywhere (not network order), like the older VST2/SNS1/T1S-EVT messages.
// One UDP datagram = one message; the payload length is the datagram length minus 20.
//
//   off size field
//    0   2   magic     5A 43 ("ZC")
//    2   1   version   high nibble major (1), low nibble minor (0). Another major: drop the message
//    3   1   type      1 STATE, 2 EVENT, 3 CMD, 4 ACK, 5 DIAG, 6 HEARTBEAT
//    4   2   id        signal or bundle id (0x8000.. = bundle, 0x0100..0x7FFF = signal, 0x0001..0x00FF = core)
//    6   1   class     bits 0-1 deadline class: 0 EVENT (<= 5 ms), 1 PERIODIC (20-50 ms), 2 DIAGNOSTIC; bits 2-7 0
//    7   1   flags     bit0 ACK_REQ, bit1 TS_VALID, bit2 AUTH, bit3 E2E, bit4 EDGE, bit5 SAFE; bits 6-7 0
//    8   4   sequence  u32, +1 per message, per (sender, id)
//   12   8   timestamp u64: gPTP time in ns when TS_VALID, else the sender's local monotonic microseconds
//   20   n   payload   fixed layout per id (below)
//
// Messages of phase 1 (same UDP ports as the legacy formats; a receiver tells them apart by the first bytes, see
// zc_wire_of()):
//   0x8001 zone.body.state_in    STATE PERIODIC, PC -> zone, UDP 5008, 9 B:
//          lamps u8 (bit0 left lit, 1 right lit, 2 brake, 3 reverse, 4 hazard) | gear u8 ASCII P R N D |
//          speed i16 km/h x10 | mode u8 (0 manual, 1 autopilot, 2 autoware) | accel u8 % | wheel i16 deg x10, + = left |
//          brake u8 %                                        flags.EDGE = sent off the 50 ms grid because of a change
//   0x8002 zone.body.sensors_out STATE PERIODIC, zone -> PC, UDP 5009, 8 B:
//          x u16 | y u16 | prox u16 (0xFFFF = not read) | flags u8 (bit0 I2C open, bit1 SPI open, bits 4-7 gear event
//          counter) | last gear event u8 ASCII (0 = none)     timestamp = when the sensors were read;
//                                                              flags.SAFE = the zone has no fresh vehicle state
//   0x0010 zone.diag             DIAG DIAGNOSTIC, zone -> PC, UDP 5009, 1 Hz, 12 B:
//          busy_us u16 (last cycle) | busy_max_us u16 (since the last diag) | failed_cycles u32 | overruns u32
//   0x0110 body.lights.indicator EVENT EVENT, PC -> zone, UDP 5006, flags ACK_REQ, 1 B:
//          code u8: 0 off, 1 left, 2 right, 3 hazard, 4 horn, 5 flash
//   ACK                          ACK, same id as the message it answers, back to its sender, 8 B:
//          acked sequence u32 | result u8 (0 accepted, 1 out of range, 2 too old, 3 auth failed, 4 refused: safe state,
//          5 unknown id) | 3 bytes 0
//          The header's own sequence is the ACK sender's counter for that id; the timestamp is the ACK's send time.
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define ZC_HDR_LEN 20
#define ZC_MAGIC0 0x5A
#define ZC_MAGIC1 0x43
#define ZC_VERSION 0x10          // 1.0
#define ZC_MAJOR(v) ((uint8_t)(v) >> 4)

enum { ZC_STATE = 1, ZC_EVENT = 2, ZC_CMD = 3, ZC_ACK = 4, ZC_DIAG = 5, ZC_HEARTBEAT = 6 };
enum { ZC_CLASS_EVENT = 0, ZC_CLASS_PERIODIC = 1, ZC_CLASS_DIAGNOSTIC = 2 };
enum {
  ZC_F_ACK_REQ = 0x01, ZC_F_TS_VALID = 0x02, ZC_F_AUTH = 0x04, ZC_F_E2E = 0x08, ZC_F_EDGE = 0x10, ZC_F_SAFE = 0x20
};
enum { ZC_ACK_OK = 0, ZC_ACK_RANGE = 1, ZC_ACK_STALE = 2, ZC_ACK_AUTH = 3, ZC_ACK_SAFE = 4, ZC_ACK_UNKNOWN = 5 };

#define ZC_ID_ZONE_DIAG 0x0010
#define ZC_ID_INDICATOR 0x0110
#define ZC_ID_STATE_IN 0x8001
#define ZC_ID_SENSORS_OUT 0x8002

#define ZC_LEN_STATE_IN 9
#define ZC_LEN_SENSORS_OUT 8
#define ZC_LEN_ZONE_DIAG 12
#define ZC_LEN_INDICATOR 1
#define ZC_LEN_ACK 8

// freshness limits of phase 1 (a catalog will hold these later)
#define ZC_STATE_IN_MAX_AGE_MS 500     // vehicle state: older than this -> safe state ("no link" on the panels)
#define ZC_EVENT_MAX_AGE_NS 5000000ULL // an EVENT stamped (gPTP) more than 5 ms from the receiver's clock: result 2

// decode errors
enum { ZC_OK = 0, ZC_E_SHORT = -1, ZC_E_MAGIC = -2, ZC_E_MAJOR = -3, ZC_E_PAYLOAD = -4 };

typedef struct {
  uint8_t version, type;
  uint16_t id;
  uint8_t cls, flags;
  uint32_t seq;
  uint64_t ts;
} zc_hdr_t;

typedef struct {
  uint8_t lamps, gear;
  int16_t speed10;
  uint8_t mode, accel;
  int16_t wheel10;
  uint8_t brake;
} zc_state_in_t;

typedef struct {
  uint16_t x, y, prox;
  uint8_t flags, gear_event;
} zc_sensors_out_t;

typedef struct {
  uint16_t busy_us, busy_max_us;
  uint32_t failed_cycles, overruns;
} zc_zone_diag_t;

typedef struct {
  uint32_t seq;
  uint8_t result;
} zc_ack_t;

// ---- little-endian access, byte by byte: any host byte order, any alignment
static inline void zc_w16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void zc_w32(uint8_t *p, uint32_t v) { zc_w16(p, (uint16_t)v); zc_w16(p + 2, (uint16_t)(v >> 16)); }
static inline void zc_w64(uint8_t *p, uint64_t v) { zc_w32(p, (uint32_t)v); zc_w32(p + 4, (uint32_t)(v >> 32)); }
static inline uint16_t zc_r16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t zc_r32(const uint8_t *p) { return (uint32_t)zc_r16(p) | ((uint32_t)zc_r16(p + 2) << 16); }
static inline uint64_t zc_r64(const uint8_t *p) { return (uint64_t)zc_r32(p) | ((uint64_t)zc_r32(p + 4) << 32); }

// ---- header
static inline zc_hdr_t zc_hdr(uint8_t type, uint16_t id, uint8_t cls, uint8_t flags, uint32_t seq, uint64_t ts) {
  zc_hdr_t h;
  h.version = ZC_VERSION; h.type = type; h.id = id; h.cls = cls; h.flags = flags; h.seq = seq; h.ts = ts;
  return h;
}
static inline int zc_put_hdr(uint8_t *b, const zc_hdr_t *h) {
  b[0] = ZC_MAGIC0; b[1] = ZC_MAGIC1;
  b[2] = h->version; b[3] = h->type;
  zc_w16(b + 4, h->id);
  b[6] = h->cls & 0x03; b[7] = h->flags;
  zc_w32(b + 8, h->seq);
  zc_w64(b + 12, h->ts);
  return ZC_HDR_LEN;
}
static inline int zc_is_zc(const uint8_t *b, size_t n) { return n >= 2 && b[0] == ZC_MAGIC0 && b[1] == ZC_MAGIC1; }
// ZC_OK, or ZC_E_SHORT / ZC_E_MAGIC / ZC_E_MAJOR. A newer minor version is accepted (fields are only ever appended).
static inline int zc_get_hdr(const uint8_t *b, size_t n, zc_hdr_t *h) {
  if (n < ZC_HDR_LEN) return n >= 2 && !zc_is_zc(b, n) ? ZC_E_MAGIC : ZC_E_SHORT;
  if (!zc_is_zc(b, n)) return ZC_E_MAGIC;
  if (ZC_MAJOR(b[2]) != ZC_MAJOR(ZC_VERSION)) return ZC_E_MAJOR;
  h->version = b[2]; h->type = b[3];
  h->id = zc_r16(b + 4);
  h->cls = b[6] & 0x03; h->flags = b[7];
  h->seq = zc_r32(b + 8);
  h->ts = zc_r64(b + 12);
  return ZC_OK;
}

// ---- payloads: put returns the payload length; get returns ZC_OK or ZC_E_PAYLOAD (too short)
static inline int zc_put_state_in(uint8_t *p, const zc_state_in_t *s) {
  p[0] = s->lamps; p[1] = s->gear;
  zc_w16(p + 2, (uint16_t)s->speed10);
  p[4] = s->mode; p[5] = s->accel;
  zc_w16(p + 6, (uint16_t)s->wheel10);
  p[8] = s->brake;
  return ZC_LEN_STATE_IN;
}
static inline int zc_get_state_in(const uint8_t *p, size_t n, zc_state_in_t *s) {
  if (n < ZC_LEN_STATE_IN) return ZC_E_PAYLOAD;
  s->lamps = p[0]; s->gear = p[1];
  s->speed10 = (int16_t)zc_r16(p + 2);
  s->mode = p[4]; s->accel = p[5];
  s->wheel10 = (int16_t)zc_r16(p + 6);
  s->brake = p[8];
  return ZC_OK;
}
static inline int zc_put_sensors_out(uint8_t *p, const zc_sensors_out_t *s) {
  zc_w16(p, s->x); zc_w16(p + 2, s->y); zc_w16(p + 4, s->prox);
  p[6] = s->flags; p[7] = s->gear_event;
  return ZC_LEN_SENSORS_OUT;
}
static inline int zc_get_sensors_out(const uint8_t *p, size_t n, zc_sensors_out_t *s) {
  if (n < ZC_LEN_SENSORS_OUT) return ZC_E_PAYLOAD;
  s->x = zc_r16(p); s->y = zc_r16(p + 2); s->prox = zc_r16(p + 4);
  s->flags = p[6]; s->gear_event = p[7];
  return ZC_OK;
}
static inline int zc_put_zone_diag(uint8_t *p, const zc_zone_diag_t *d) {
  zc_w16(p, d->busy_us); zc_w16(p + 2, d->busy_max_us);
  zc_w32(p + 4, d->failed_cycles); zc_w32(p + 8, d->overruns);
  return ZC_LEN_ZONE_DIAG;
}
static inline int zc_get_zone_diag(const uint8_t *p, size_t n, zc_zone_diag_t *d) {
  if (n < ZC_LEN_ZONE_DIAG) return ZC_E_PAYLOAD;
  d->busy_us = zc_r16(p); d->busy_max_us = zc_r16(p + 2);
  d->failed_cycles = zc_r32(p + 4); d->overruns = zc_r32(p + 8);
  return ZC_OK;
}
static inline int zc_put_indicator(uint8_t *p, uint8_t code) { p[0] = code; return ZC_LEN_INDICATOR; }
static inline int zc_get_indicator(const uint8_t *p, size_t n, uint8_t *code) {
  if (n < ZC_LEN_INDICATOR) return ZC_E_PAYLOAD;
  *code = p[0];
  return ZC_OK;
}
static inline int zc_put_ack(uint8_t *p, const zc_ack_t *a) {
  zc_w32(p, a->seq);
  p[4] = a->result; p[5] = p[6] = p[7] = 0;
  return ZC_LEN_ACK;
}
static inline int zc_get_ack(const uint8_t *p, size_t n, zc_ack_t *a) {
  if (n < ZC_LEN_ACK) return ZC_E_PAYLOAD;
  a->seq = zc_r32(p);
  a->result = p[4];
  return ZC_OK;
}

// ---- whole messages: header + payload into b (room for 20 + payload), returns the datagram length
static inline int zc_msg_state_in(uint8_t *b, uint8_t flags, uint32_t seq, uint64_t ts, const zc_state_in_t *s) {
  const zc_hdr_t h = zc_hdr(ZC_STATE, ZC_ID_STATE_IN, ZC_CLASS_PERIODIC, flags, seq, ts);
  return zc_put_hdr(b, &h) + zc_put_state_in(b + ZC_HDR_LEN, s);
}
static inline int zc_msg_sensors_out(uint8_t *b, uint8_t flags, uint32_t seq, uint64_t ts, const zc_sensors_out_t *s) {
  const zc_hdr_t h = zc_hdr(ZC_STATE, ZC_ID_SENSORS_OUT, ZC_CLASS_PERIODIC, flags, seq, ts);
  return zc_put_hdr(b, &h) + zc_put_sensors_out(b + ZC_HDR_LEN, s);
}
static inline int zc_msg_zone_diag(uint8_t *b, uint8_t flags, uint32_t seq, uint64_t ts, const zc_zone_diag_t *d) {
  const zc_hdr_t h = zc_hdr(ZC_DIAG, ZC_ID_ZONE_DIAG, ZC_CLASS_DIAGNOSTIC, flags, seq, ts);
  return zc_put_hdr(b, &h) + zc_put_zone_diag(b + ZC_HDR_LEN, d);
}
static inline int zc_msg_indicator(uint8_t *b, uint8_t flags, uint32_t seq, uint64_t ts, uint8_t code) {
  const zc_hdr_t h = zc_hdr(ZC_EVENT, ZC_ID_INDICATOR, ZC_CLASS_EVENT, flags, seq, ts);
  return zc_put_hdr(b, &h) + zc_put_indicator(b + ZC_HDR_LEN, code);
}
// an ACK for message (id, class) with sequence acked; seq/ts/flags are the ACK's own
static inline int zc_msg_ack(uint8_t *b, uint16_t id, uint8_t cls, uint8_t flags, uint32_t seq, uint64_t ts,
                             uint32_t acked, uint8_t result) {
  const zc_hdr_t h = zc_hdr(ZC_ACK, id, cls, flags, seq, ts);
  const zc_ack_t a = {acked, result};
  return zc_put_hdr(b, &h) + zc_put_ack(b + ZC_HDR_LEN, &a);
}

// ---- which format a datagram on the zone ports is. The four never overlap:
//   VST1/VST2 "VST" ..., SNS1 "SNS1", legacy T1S-EVT = 12 B with bytes 1-3 "TVE" (u32 LE 0x455654cc), ZC = 5A 43
enum { ZC_WIRE_UNKNOWN = 0, ZC_WIRE_ZC = 1, ZC_WIRE_VST = 2, ZC_WIRE_SNS = 3, ZC_WIRE_EVT = 4 };
static inline int zc_wire_of(const uint8_t *b, size_t n) {
  if (n >= ZC_HDR_LEN && zc_is_zc(b, n)) return ZC_WIRE_ZC;
  if (n >= 14 && b[0] == 'V' && b[1] == 'S' && b[2] == 'T' && (b[3] == '1' || b[3] == '2')) return ZC_WIRE_VST;
  if (n >= 16 && !memcmp(b, "SNS1", 4)) return ZC_WIRE_SNS;
  if (n == 12 && b[1] == 'T' && b[2] == 'V' && b[3] == 'E') return ZC_WIRE_EVT;
  return ZC_WIRE_UNKNOWN;
}

// ---- receive side: sequence tracking per (sender, id)
typedef struct {
  uint32_t last;       // last sequence accepted
  uint8_t have;        // 0 until the first message
  uint32_t gaps;       // messages missing between accepted ones (forward jumps)
  uint32_t dups;       // same or older sequence (repeat, reorder)
} zc_seq_t;
// 1 = new (take it), 0 = repeat / older (a retransmission: answer, do not act again). A jump backwards by more than
// 1000 is taken as a restarted sender, not as an old message.
static inline int zc_seq_take(zc_seq_t *s, uint32_t seq) {
  if (!s->have) { s->have = 1; s->last = seq; return 1; }
  const int32_t d = (int32_t)(seq - s->last);
  if (d > 0) { s->gaps += (uint32_t)(d - 1); s->last = seq; return 1; }
  if (d < -1000) { s->last = seq; return 1; }
  s->dups++;
  return 0;
}
// |now - ts| <= max (both in the same unit)
static inline int zc_ts_fresh(uint64_t now, uint64_t ts, uint64_t max) {
  return (now >= ts ? now - ts : ts - now) <= max;
}
