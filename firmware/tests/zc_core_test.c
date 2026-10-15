// Host test of the ZC core frame (../t1s_node/zc_core.h) against the golden vectors, plus a line mode that the
// bench console's tests drive to compare this C code with its Python encoder on random messages.
//   cc -O1 -Wall -Wextra -Werror -I../t1s_node -o zc_core_test zc_core_test.c
//   ./zc_core_test zc_vectors.txt        check every vector (encode -> same bytes, decode -> same fields), exit 1 on a miss
//   ./zc_core_test -                     stdin lines:
//       enc <kind> key=value ...         -> hex of the message (kinds and keys as in zc_vectors.txt)
//       dec <hex>                        -> "err=<n>" or "key=value ..." (header, then the payload fields of its id)
//       wire <hex>                       -> zc_wire_of()
//       seq <n> <n> ...                  -> zc_seq_take() on each: "1 0 1 ... gaps=<g> dups=<d>"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "zc_core.h"

#define MAXK 24
typedef struct { char k[24]; long long v; } kv_t;
typedef struct { kv_t a[MAXK]; int n; } kvs_t;

static int get(const kvs_t *m, const char *k, long long *v) {
  for (int i = 0; i < m->n; i++)
    if (!strcmp(m->a[i].k, k)) { *v = m->a[i].v; return 1; }
  return 0;
}
static long long need(const kvs_t *m, const char *k, int *bad) {
  long long v = 0;
  if (!get(m, k, &v)) { fprintf(stderr, "missing key %s\n", k); *bad = 1; }
  return v;
}
// "k=v k=v ..." up to (not including) "hex="
static void parse_kv(char *s, kvs_t *m) {
  m->n = 0;
  for (char *t = strtok(s, " \t\r\n"); t && m->n < MAXK; t = strtok(NULL, " \t\r\n")) {
    char *e = strchr(t, '=');
    if (!e) continue;
    *e = 0;
    snprintf(m->a[m->n].k, sizeof m->a[m->n].k, "%s", t);
    m->a[m->n].v = (long long)strtoull(e + 1, NULL, 0);
    if (e[1] == '-') m->a[m->n].v = strtoll(e + 1, NULL, 0);
    m->n++;
  }
}
static int unhex(const char *s, uint8_t *b, int max) {
  int n = 0, hi = -1;
  for (; *s && n < max; s++) {
    int d = *s >= '0' && *s <= '9' ? *s - '0' : *s >= 'a' && *s <= 'f' ? *s - 'a' + 10 : *s >= 'A' && *s <= 'F' ? *s - 'A' + 10 : -1;
    if (d < 0) continue;
    if (hi < 0) hi = d;
    else { b[n++] = (uint8_t)(hi << 4 | d); hi = -1; }
  }
  return n;
}
static void hexs(const uint8_t *b, int n, char *out) {
  for (int i = 0; i < n; i++) sprintf(out + 2 * i, "%02x", b[i]);
  out[2 * n] = 0;
}

// encode from fields; returns the length or -1
static int enc(const char *kind, const kvs_t *m, uint8_t *b) {
  int bad = 0;
  long long ver = ZC_VERSION;
  get(m, "version", &ver);
  zc_hdr_t h = zc_hdr((uint8_t)need(m, "type", &bad), (uint16_t)need(m, "id", &bad), (uint8_t)need(m, "cls", &bad),
                      (uint8_t)need(m, "flags", &bad), (uint32_t)need(m, "seq", &bad), (uint64_t)need(m, "ts", &bad));
  h.version = (uint8_t)ver;
  int n = zc_put_hdr(b, &h);
  uint8_t *p = b + n;
  if (!strcmp(kind, "state_in")) {
    zc_state_in_t s = {(uint8_t)need(m, "lamps", &bad), (uint8_t)need(m, "gear", &bad), (int16_t)need(m, "speed10", &bad),
                       (uint8_t)need(m, "mode", &bad), (uint8_t)need(m, "accel", &bad), (int16_t)need(m, "wheel10", &bad),
                       (uint8_t)need(m, "brake", &bad)};
    n += zc_put_state_in(p, &s);
  } else if (!strcmp(kind, "sensors_out")) {
    zc_sensors_out_t s = {(uint16_t)need(m, "x", &bad), (uint16_t)need(m, "y", &bad), (uint16_t)need(m, "prox", &bad),
                          (uint8_t)need(m, "sflags", &bad), (uint8_t)need(m, "gear_event", &bad)};
    n += zc_put_sensors_out(p, &s);
  } else if (!strcmp(kind, "zone_diag")) {
    zc_zone_diag_t d = {(uint16_t)need(m, "busy_us", &bad), (uint16_t)need(m, "busy_max_us", &bad),
                        (uint32_t)need(m, "failed_cycles", &bad), (uint32_t)need(m, "overruns", &bad)};
    n += zc_put_zone_diag(p, &d);
  } else if (!strcmp(kind, "indicator")) {
    n += zc_put_indicator(p, (uint8_t)need(m, "code", &bad));
  } else if (!strcmp(kind, "ack")) {
    zc_ack_t a = {(uint32_t)need(m, "acked", &bad), (uint8_t)need(m, "result", &bad)};
    n += zc_put_ack(p, &a);
  } else {
    fprintf(stderr, "unknown kind %s\n", kind);
    return -1;
  }
  return bad ? -1 : n;
}

// decode to "key=value ..." (the same keys as the vectors); returns ZC_OK or an error
static int dec(const uint8_t *b, int n, char *out, size_t cap) {
  zc_hdr_t h;
  int e = zc_get_hdr(b, (size_t)n, &h);
  if (e) return e;
  int o = snprintf(out, cap, "version=%u type=%u id=%u cls=%u flags=%u seq=%lu ts=%llu", h.version, h.type, h.id, h.cls,
                   h.flags, (unsigned long)h.seq, (unsigned long long)h.ts);
  const uint8_t *p = b + ZC_HDR_LEN;
  const size_t pn = (size_t)(n - ZC_HDR_LEN);
  if (h.type == ZC_ACK) {
    zc_ack_t a;
    if ((e = zc_get_ack(p, pn, &a))) return e;
    snprintf(out + o, cap - o, " acked=%lu result=%u", (unsigned long)a.seq, a.result);
  } else if (h.id == ZC_ID_STATE_IN) {
    zc_state_in_t s;
    if ((e = zc_get_state_in(p, pn, &s))) return e;
    snprintf(out + o, cap - o, " lamps=%u gear=%u speed10=%d mode=%u accel=%u wheel10=%d brake=%u", s.lamps, s.gear, s.speed10,
             s.mode, s.accel, s.wheel10, s.brake);
  } else if (h.id == ZC_ID_SENSORS_OUT) {
    zc_sensors_out_t s;
    if ((e = zc_get_sensors_out(p, pn, &s))) return e;
    snprintf(out + o, cap - o, " x=%u y=%u prox=%u sflags=%u gear_event=%u", s.x, s.y, s.prox, s.flags, s.gear_event);
  } else if (h.id == ZC_ID_ZONE_DIAG) {
    zc_zone_diag_t d;
    if ((e = zc_get_zone_diag(p, pn, &d))) return e;
    snprintf(out + o, cap - o, " busy_us=%u busy_max_us=%u failed_cycles=%lu overruns=%lu", d.busy_us, d.busy_max_us,
             (unsigned long)d.failed_cycles, (unsigned long)d.overruns);
  } else if (h.id == ZC_ID_INDICATOR) {
    uint8_t c;
    if ((e = zc_get_indicator(p, pn, &c))) return e;
    snprintf(out + o, cap - o, " code=%u", c);
  }
  return ZC_OK;
}

// every key of the vector line must come back from dec() with the same value
static int same_fields(const kvs_t *want, const char *got) {
  char tmp[512];
  snprintf(tmp, sizeof tmp, "%s", got);
  kvs_t g;
  parse_kv(tmp, &g);
  for (int i = 0; i < want->n; i++) {
    long long v;
    if (!get(&g, want->a[i].k, &v)) { fprintf(stderr, "  decode lacks %s\n", want->a[i].k); return 0; }
    if (v != want->a[i].v) { fprintf(stderr, "  %s: decoded %lld, vector %lld\n", want->a[i].k, v, want->a[i].v); return 0; }
  }
  return 1;
}

static int run_vectors(const char *path) {
  FILE *f = fopen(path, "r");
  if (!f) { perror(path); return 2; }
  char line[1024];
  int total = 0, fail = 0;
  while (fgets(line, sizeof line, f)) {
    if (line[0] == '#' || line[0] == '\n') continue;
    char *hx = strstr(line, "hex=");
    if (!hx) continue;
    uint8_t want[256], got[256];
    const int wn = unhex(hx + 4, want, sizeof want);
    *hx = 0;
    char kind[32] = "", name[64] = "";
    int skip = 0;
    sscanf(line, "%31s %63s %n", kind, name, &skip);
    kvs_t m;
    parse_kv(line + skip, &m);
    total++;
    int ok = 1;
    if (!strcmp(kind, "reject")) {
      long long err = 0;
      get(&m, "err", &err);
      char out[512];
      const int e = dec(want, wn, out, sizeof out);
      if (e != err) { fprintf(stderr, "  decode gave %d, want %lld\n", e, err); ok = 0; }
    } else if (!strcmp(kind, "wire")) {
      long long w = -1;
      get(&m, "wire", &w);
      const int e = zc_wire_of(want, (size_t)wn);
      if (e != w) { fprintf(stderr, "  zc_wire_of %d, want %lld\n", e, w); ok = 0; }
    } else {
      const int gn = enc(kind, &m, got);
      if (gn != wn || memcmp(got, want, (size_t)wn)) {
        char a[600], b[600];
        hexs(got, gn < 0 ? 0 : gn, a);
        hexs(want, wn, b);
        fprintf(stderr, "  encode %s\n  vector %s\n", a, b);
        ok = 0;
      }
      char out[512];
      const int e = dec(want, wn, out, sizeof out);
      if (e) { fprintf(stderr, "  decode error %d\n", e); ok = 0; }
      else if (!same_fields(&m, out)) ok = 0;
    }
    printf("%s %s %s\n", ok ? "ok  " : "FAIL", kind, name);
    fail += !ok;
  }
  fclose(f);
  printf("%d vectors, %d failed\n", total, fail);
  return fail || !total ? 1 : 0;
}

static int run_lines(void) {
  char line[1024];
  while (fgets(line, sizeof line, stdin)) {
    uint8_t b[256];
    char out[600];
    if (!strncmp(line, "enc ", 4)) {
      char kind[32] = "";
      int skip = 0;
      sscanf(line + 4, "%31s %n", kind, &skip);
      kvs_t m;
      parse_kv(line + 4 + skip, &m);
      const int n = enc(kind, &m, b);
      if (n < 0) printf("error\n");
      else { hexs(b, n, out); printf("%s\n", out); }
    } else if (!strncmp(line, "dec ", 4)) {
      const int n = unhex(line + 4, b, sizeof b);
      const int e = dec(b, n, out, sizeof out);
      if (e) printf("err=%d\n", e);
      else printf("%s\n", out);
    } else if (!strncmp(line, "seq ", 4)) {      // seq <n> <n> ...: zc_seq_take on each -> "1 0 1 ... gaps=<g> dups=<d>"
      zc_seq_t s = {0, 0, 0, 0};
      char *e = line + 4;
      for (;;) {
        char *q;
        const unsigned long v = strtoul(e, &q, 0);
        if (q == e) break;
        printf("%d ", zc_seq_take(&s, (uint32_t)v));
        e = q;
      }
      printf("gaps=%lu dups=%lu\n", (unsigned long)s.gaps, (unsigned long)s.dups);
    } else if (!strncmp(line, "wire ", 5)) {
      const int n = unhex(line + 5, b, sizeof b);
      printf("%d\n", zc_wire_of(b, (size_t)n));
    }
    fflush(stdout);
  }
  return 0;
}

int main(int argc, char **argv) {
  if (argc > 1 && strcmp(argv[1], "-")) return run_vectors(argv[1]);
  // a few fixed checks of the receive helpers
  if (argc == 1) {
    zc_seq_t s = {0, 0, 0, 0};
    int ok = zc_seq_take(&s, 10) == 1 && zc_seq_take(&s, 10) == 0 && zc_seq_take(&s, 13) == 1 && s.gaps == 2 &&
             zc_seq_take(&s, 12) == 0 && s.dups == 2 && zc_seq_take(&s, 5000) == 1 && s.gaps == 4988 &&
             zc_seq_take(&s, 1) == 1 /* far back: a restarted sender */ && s.last == 1;
    s.last = 0xFFFFFFFEu;
    ok = ok && zc_seq_take(&s, 0xFFFFFFFFu) == 1 && zc_seq_take(&s, 0) == 1 && s.gaps == 4988 /* wrap, no gap */ &&
         zc_ts_fresh(100, 95, 5) && !zc_ts_fresh(100, 94, 5) && zc_ts_fresh(95, 100, 5);
    printf("%s receive helpers\n", ok ? "ok  " : "FAIL");
    return ok ? 0 : 1;
  }
  return run_lines();
}
