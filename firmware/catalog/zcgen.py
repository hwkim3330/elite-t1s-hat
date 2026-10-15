#!/usr/bin/env python3
"""ZC signal catalogue -> code. The catalogue (a YAML file, schema at the top of vehicle_body.yaml) is the one place
the ids, payload layouts, deadline classes, rates and freshness limits of the ZC frame are written down.

    python3 zcgen.py [catalogue.yaml] [--c FILE] [--vectors FILE] [--md FILE] [--py FILE] [--lua FILE] [--check]

Defaults (relative to this folder): vehicle_body.yaml -> ../t1s_node/zc_catalog.h, ../tests/zc_catalog_vectors.txt,
CATALOG.md. --py / --lua are written only when given (the bench console keeps them). --check writes nothing and
exits 1 if a target differs from what would be generated (for tests / CI).

Outputs
  C        C99 + C++, header only, no allocation: per item a packed-layout struct, zc_put_<c> / zc_get_<c> /
           zc_msg_<c>, zc_safe_<c> (the catalogue's safe values), ZC_ID_* / ZC_LEN_* / ZC_PERIOD_MS_* /
           ZC_MAX_AGE_MS_* / enum and bit constants, a const table zc_catalog[] with zc_catalog_find(id), and
           ZC_CATALOG_HASH / ZC_CATALOG_VERSION for the HEARTBEAT. Built on zc_core.h (header, ACK, sequence).
  vectors  golden vectors from each item's `examples`, in the format of tests/zc_vectors.txt
  Python   the same tables as data (struct formats, fields, enums, bits, profile maps) for the console / gateway
  Lua      the table the Wireshark dissector (t1s_bench.lua) reads
  docs     CATALOG.md: one table per item, the profile mappings, the hash
"""
import argparse
import json
import os
import pprint
import struct
import sys
import zlib

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
TYPES = {"state": 1, "event": 2, "cmd": 3, "ack": 4, "diag": 5, "heartbeat": 6}
CLASSES = {"event": 0, "periodic": 1, "diagnostic": 2}
CLASS_LIMIT = {0: 64, 1: 256, 2: 1024}          # payload ceiling per deadline class (a bounded frame keeps the worst PLCA access bounded)
DIRS = ("zone->gw", "gw->zone", "node->gw", "gw->node")
# wire type -> (struct code, size, C type, signed)
WT = {"u8": ("B", 1, "uint8_t", False), "i8": ("b", 1, "int8_t", True), "u16": ("H", 2, "uint16_t", False),
      "i16": ("h", 2, "int16_t", True), "u32": ("I", 4, "uint32_t", False), "i32": ("i", 4, "int32_t", True),
      "u64": ("Q", 8, "uint64_t", False), "i64": ("q", 8, "int64_t", True), "f32": ("f", 4, "float", True),
      "char": ("B", 1, "uint8_t", False)}
PROFILES = ("vss", "ros2", "uorb", "ngva")      # vss: implemented by the gateway; the rest: hooks (validated, carried)
VSS_TYPES = {"bool": "<B", "u8": "<B", "i8": "<b", "u16": "<H", "i16": "<h", "u32": "<I", "i32": "<i", "f32": "<f"}
WIRE_KEYS = ("id", "name", "type", "class", "period_ms", "max_age_ms", "ack", "dir", "port", "edge")
FIELD_WIRE_KEYS = ("name", "type", "count", "scale", "unit", "enum", "bits", "invalid", "safe", "pad")


class CatalogError(ValueError):
    pass


# ------------------------------------------------------------------ load + validate
def load(path):
    with open(path) as f:
        cat = yaml.safe_load(f)
    items = []
    for inc in cat.get("include", []):
        with open(os.path.join(os.path.dirname(path), inc)) as f:
            items += yaml.safe_load(f)["items"]
    items += cat["items"]
    cat["items"] = sorted((norm_item(it) for it in items), key=lambda i: i["id"])
    seen_id, seen_c, seen_n = {}, {}, {}
    for it in cat["items"]:
        for k, d in (("id", seen_id), ("c", seen_c), ("name", seen_n)):
            if it[k] in d:
                raise CatalogError("duplicate %s %r (%s and %s)" % (k, it[k], d[it[k]], it["name"]))
            d[it[k]] = it["name"]
    cat["profile"] = cat.get("profile", "vss")
    cat["hash"] = catalog_hash(cat)
    return cat


def norm_item(it):
    it = dict(it)
    name = it.get("name", "?")
    need = ("id", "name", "c", "type", "class", "dir", "fields")
    for k in need:
        if k not in it:
            raise CatalogError("%s: missing %s" % (name, k))
    if it["type"] not in TYPES or it["type"] == "ack":
        raise CatalogError("%s: type %r" % (name, it["type"]))
    if it["class"] not in CLASSES:
        raise CatalogError("%s: class %r" % (name, it["class"]))
    if it["dir"] not in DIRS:
        raise CatalogError("%s: dir %r" % (name, it["dir"]))
    i = it["id"]
    if not 0 < i <= 0xFFFF:
        raise CatalogError("%s: id 0x%x out of range" % (name, i))
    if it["type"] == "heartbeat" and i != 0x0001:
        raise CatalogError("%s: HEARTBEAT is id 0x0001" % name)
    if not it["c"].isidentifier() or it["c"].lower() != it["c"]:
        raise CatalogError("%s: c name %r" % (name, it["c"]))
    it.setdefault("period_ms", 0)
    it.setdefault("max_age_ms", 3 * it["period_ms"] if it["period_ms"] else 5)
    it.setdefault("ack", False)
    it.setdefault("edge", False)
    it.setdefault("port", 0)
    it["fields"] = [norm_field(name, f) for f in it["fields"]]
    off = 0
    for f in it["fields"]:
        f["offset"] = off
        off += f["size"]
    it["len"] = off
    cls = CLASSES[it["class"]]
    if off > CLASS_LIMIT[cls]:
        raise CatalogError("%s: payload %d B over the %s ceiling %d B" % (name, off, it["class"], CLASS_LIMIT[cls]))
    for p in it.get("map", {}) or {}:
        if p not in PROFILES:
            raise CatalogError("%s: unknown profile %r" % (name, p))
    names = [f["name"] for f in it["fields"] if f["type"] is not None]
    for ex in it.get("examples", []):
        miss = set(names) - set(ex)
        if miss:
            raise CatalogError("%s: example lacks %s" % (name, sorted(miss)))
    return it


def norm_field(item, f):
    f = dict(f)
    n = f.get("name")
    if not n or not n.isidentifier():
        raise CatalogError("%s: field name %r" % (item, n))
    if n in ("version", "type", "id", "cls", "flags", "seq", "ts", "kind"):
        raise CatalogError("%s: field name %r is a header key" % (item, n))
    if "pad" in f:
        f.update(type=None, count=f["pad"], size=f["pad"])
        return f
    if f.get("type") not in WT:
        raise CatalogError("%s.%s: type %r" % (item, n, f.get("type")))
    f.setdefault("count", 1)
    for k in ("enum", "bits"):
        for lab in f.get(k) or {}:
            if not isinstance(lab, str) or not lab.isidentifier():     # YAML 1.1 reads a bare off / on / no as a bool
                raise CatalogError("%s.%s: %s label %r (quote it)" % (item, n, k, lab))
    size = WT[f["type"]][1]
    f["size"] = size * f["count"]
    bits = f.get("bits")
    if bits:
        width = size * 8
        nb = {}
        for lab, b in bits.items():
            first, w = (b, 1) if isinstance(b, int) else tuple(b)
            if first < 0 or first + w > width:
                raise CatalogError("%s.%s: bit %s outside %d bits" % (item, n, lab, width))
            nb[lab] = [first, w]
        f["bits"] = nb
    for p in (f.get("map") or {}):
        if p not in PROFILES:
            raise CatalogError("%s.%s: unknown profile %r" % (item, n, p))
    vss = (f.get("map") or {}).get("vss")
    if vss:
        if "type" in vss and vss["type"] not in VSS_TYPES:
            raise CatalogError("%s.%s: vss type %r" % (item, n, vss["type"]))
        if "bits" in vss and not set(vss["bits"]) <= set(f.get("bits") or {}):
            raise CatalogError("%s.%s: vss bits not in bits" % (item, n))
        if "one_of" in vss and not set(vss["one_of"]) <= set(f.get("enum") or {}):
            raise CatalogError("%s.%s: vss one_of labels not in enum" % (item, n))
        if "values" in vss and not set(vss["values"]) <= set(f.get("enum") or {}):
            raise CatalogError("%s.%s: vss values labels not in enum" % (item, n))
    return f


def canonical(cat):
    """the wire contract only (see the schema): what the hash covers"""
    out = []
    for it in cat["items"]:
        d = {k: it[k] for k in WIRE_KEYS}
        d["fields"] = [{k: f[k] for k in FIELD_WIRE_KEYS if k in f and f[k] is not None} for f in it["fields"]]
        out.append(d)
    return json.dumps({"catalog": cat["catalog"], "version": cat["version"], "items": out}, sort_keys=True,
                      separators=(",", ":"))


def catalog_hash(cat):
    return zlib.crc32(canonical(cat).encode()) & 0xFFFFFFFF


# ------------------------------------------------------------------ helpers
def fmt_of(it):
    s = "<"
    for f in it["fields"]:
        if f["type"] is None:
            s += "%dx" % f["count"]
        else:
            s += (str(f["count"]) if f["count"] > 1 else "") + WT[f["type"]][0]
    return s


def up(s):
    return s.upper()


def cnum(v, t):
    if t in ("u64", "i64"):
        return "%dULL" % v if v >= 0 else "(%dLL)" % v
    if t == "f32":
        return repr(float(v)) + "f"
    return "%d" % v if v >= 0 else "(%d)" % v


def wire_value(f, v):
    """an example value as the raw integer / list"""
    return v


def encode_example(it, ex):
    hdr = struct.pack("<2sBBHBBIQ", b"ZC", 0x10, TYPES[it["type"]], it["id"], CLASSES[it["class"]], ex.get("flags", 0),
                      ex.get("seq", 0) & 0xFFFFFFFF, ex.get("ts", 0) & 0xFFFFFFFFFFFFFFFF)
    vals = []
    for f in it["fields"]:
        if f["type"] is None:
            continue
        v = ex[f["name"]]
        vals += list(v) if f["count"] > 1 else [v]
    return hdr + struct.pack(fmt_of(it), *vals)


# ------------------------------------------------------------------ C
def gen_c(cat):
    L = []
    w = L.append
    w("// zc_catalog.h -- GENERATED by catalog/zcgen.py from catalog/%s (catalogue %s v%d). Do not edit: change the"
      % (os.path.basename(cat["path"]), cat["catalog"], cat["version"]))
    w("// YAML and run `python3 catalog/zcgen.py` (tests check that this file is current).")
    w("// Payload structs, encode/decode and the const table of every item; the 20-byte header, ACK, sequence tracking")
    w("// and wire_of() are in zc_core.h. Little-endian, byte by byte (any host order, any alignment), no allocation.")
    w("#pragma once")
    w('#include "zc_core.h"')
    w("")
    w("#define ZC_CATALOG_NAME \"%s\"" % cat["catalog"])
    w("#define ZC_CATALOG_VERSION %d" % cat["version"])
    w("#define ZC_CATALOG_HASH 0x%08XUL   // CRC-32 of the wire contract (zcgen.py canonical()), sent in HEARTBEAT"
      % cat["hash"])
    w("")
    w("// ---- per item: id, length, header type / class, period and freshness")
    for it in cat["items"]:
        C = up(it["c"])
        w("// 0x%04X %s: %s %s, %s, port %s%s" % (it["id"], it["name"], it["type"].upper(), it["class"].upper(), it["dir"],
                                                 it["port"] or "-", ", ACK" if it["ack"] else ""))
        w("#define ZC_ID_%s 0x%04X" % (C, it["id"]))
        w("#define ZC_LEN_%s %d" % (C, it["len"]))
        w("#define ZC_TYPE_%s %d" % (C, TYPES[it["type"]]))
        w("#define ZC_CLASS_%s %d" % (C, CLASSES[it["class"]]))
        w("#define ZC_PERIOD_MS_%s %d" % (C, it["period_ms"]))
        w("#define ZC_MAX_AGE_MS_%s %d" % (C, it["max_age_ms"]))
        if it["port"]:
            w("#define ZC_PORT_%s %d" % (C, it["port"]))
        for f in it["fields"]:
            F = up(f["name"])
            for lab, v in (f.get("enum") or {}).items():
                w("#define ZC_%s_%s_%s %d" % (C, F, up(lab), v))
            for lab, (b0, bw) in (f.get("bits") or {}).items():
                if bw == 1:
                    w("#define ZC_%s_%s_%s (1u << %d)" % (C, F, up(lab), b0))
                else:
                    w("#define ZC_%s_%s_%s_SHIFT %d" % (C, F, up(lab), b0))
                    w("#define ZC_%s_%s_%s_MASK 0x%Xu" % (C, F, up(lab), ((1 << bw) - 1) << b0))
            if "invalid" in f:
                w("#define ZC_%s_%s_INVALID %s" % (C, F, cnum(f["invalid"], f["type"])))
        w("")
    w("// ---- payload structs (field order = wire order; padding in the struct is irrelevant, the codecs go by bytes)")
    for it in cat["items"]:
        w("typedef struct {")
        for f in it["fields"]:
            if f["type"] is None:
                continue
            arr = "[%d]" % f["count"] if f["count"] > 1 else ""
            unit = (" %s" % f["unit"]) if f.get("unit") else ""
            sc = (" x %g" % f["scale"]) if f.get("scale") else ""
            w("  %s %s%s;%s" % (WT[f["type"]][2], f["name"], arr, ("  //" + sc + unit) if sc or unit else ""))
        w("} zc_%s_t;" % it["c"])
    w("")
    rd = {1: ("", ""), 2: ("zc_r16", "zc_w16"), 4: ("zc_r32", "zc_w32"), 8: ("zc_r64", "zc_w64")}
    w("static inline float zc_rf32(const uint8_t *p) { uint32_t u = zc_r32(p); float f; memcpy(&f, &u, 4); return f; }")
    w("static inline void zc_wf32(uint8_t *p, float f) { uint32_t u; memcpy(&u, &f, 4); zc_w32(p, u); }")
    w("")
    w("// ---- codecs: put returns the payload length; get returns ZC_OK or ZC_E_PAYLOAD (too short)")
    for it in cat["items"]:
        c, C = it["c"], up(it["c"])
        puts, gets = [], []
        for f in it["fields"]:
            if f["type"] is None:
                puts.append("  memset(p + %d, 0, %d);" % (f["offset"], f["count"]))
                continue
            t, sz = f["type"], WT[f["type"]][1]
            ct = WT[t][2]
            for k in range(f["count"]):
                off = f["offset"] + k * sz
                ref = "s->%s%s" % (f["name"], "[%d]" % k if f["count"] > 1 else "")
                if t == "f32":
                    puts.append("  zc_wf32(p + %d, %s);" % (off, ref))
                    gets.append("  %s = zc_rf32(p + %d);" % (ref, off))
                elif sz == 1:
                    puts.append("  p[%d] = (uint8_t)%s;" % (off, ref))
                    gets.append("  %s = (%s)p[%d];" % (ref, ct, off))
                else:
                    r, wr = rd[sz]
                    ut = "uint%d_t" % (sz * 8)
                    puts.append("  %s(p + %d, (%s)%s);" % (wr, off, ut, ref))
                    gets.append("  %s = (%s)%s(p + %d);" % (ref, ct, r, off))
        w("static inline int zc_put_%s(uint8_t *p, const zc_%s_t *s) {" % (c, c))
        L.extend(puts)
        w("  return ZC_LEN_%s;" % C)
        w("}")
        w("static inline int zc_get_%s(const uint8_t *p, size_t n, zc_%s_t *s) {" % (c, c))
        w("  if (n < ZC_LEN_%s) return ZC_E_PAYLOAD;" % C)
        L.extend(gets)
        w("  return ZC_OK;")
        w("}")
        w("// header + payload into b (room for ZC_HDR_LEN + ZC_LEN_%s), returns the datagram length" % C)
        w("static inline int zc_msg_%s(uint8_t *b, uint8_t flags, uint32_t seq, uint64_t ts, const zc_%s_t *s) {" % (c, c))
        w("  const zc_hdr_t h = zc_hdr(ZC_TYPE_%s, ZC_ID_%s, ZC_CLASS_%s, flags, seq, ts);" % (C, C, C))
        w("  return zc_put_hdr(b, &h) + zc_put_%s(b + ZC_HDR_LEN, s);" % c)
        w("}")
        safes = []
        for f in it["fields"]:
            sv = f.get("safe")
            if sv is None or sv == "hold":
                continue
            if sv == "invalid":
                safes.append("  s->%s = ZC_%s_%s_INVALID;" % (f["name"], C, up(f["name"])))
            elif isinstance(sv, dict):
                for lab, on in sv.items():
                    m = "ZC_%s_%s_%s" % (C, up(f["name"]), up(lab))
                    safes.append("  s->%s = (%s)(%s);" % (f["name"], WT[f["type"]][2],
                                                         "s->%s | %s" % (f["name"], m) if on else "s->%s & ~%s" % (f["name"], m)))
            else:
                safes.append("  s->%s = %s;" % (f["name"], cnum(sv, f["type"])))
        w("// the catalogue's safe values (fields marked hold, or without one, keep their last value)")
        w("static inline void zc_safe_%s(zc_%s_t *s) {" % (c, c))
        L.extend(safes or ["  (void)s;"])
        w("}")
        w("")
    w("// ---- the table: what a receiver needs to know about an id without its codec")
    w("enum { ZC_DIR_ZONE_TO_GW = 0, ZC_DIR_GW_TO_ZONE = 1, ZC_DIR_NODE_TO_GW = 2, ZC_DIR_GW_TO_NODE = 3 };")
    w("typedef struct {")
    w("  uint16_t id;")
    w("  uint8_t type, cls, ack, dir;")
    w("  uint16_t len, period_ms, max_age_ms, port;")
    w("  const char *name;")
    w("} zc_item_t;")
    w("static const zc_item_t zc_catalog[] = {")
    for it in cat["items"]:
        w("    {0x%04X, %d, %d, %d, %d, %d, %d, %d, %d, \"%s\"}," % (
            it["id"], TYPES[it["type"]], CLASSES[it["class"]], int(it["ack"]), DIRS.index(it["dir"]), it["len"],
            it["period_ms"], it["max_age_ms"], it["port"], it["name"]))
    w("};")
    w("#define ZC_CATALOG_COUNT %d" % len(cat["items"]))
    w("static inline const zc_item_t *zc_catalog_find(uint16_t id) {")
    w("  for (int i = 0; i < ZC_CATALOG_COUNT; i++)")
    w("    if (zc_catalog[i].id == id) return &zc_catalog[i];")
    w("  return NULL;")
    w("}")
    w("")
    return "\n".join(L) + "\n"


def gen_c_test(cat):
    """zc_catalog_test.inc: enc/dec by kind name over the generated codecs, for the host test program"""
    L = ["// GENERATED by catalog/zcgen.py -- the host test's per-kind encode/decode over the generated codecs.",
         "// Included by tests/zc_core_test.c after its kv helpers (get, need)."]
    w = L.append
    w("static int zc_cat_enc(const char *kind, const kvs_t *m, uint8_t *p, int *bad) {")
    for it in cat["items"]:
        c = it["c"]
        w('  if (!strcmp(kind, "%s")) {' % c)
        w("    zc_%s_t s;" % c)
        w("    memset(&s, 0, sizeof s);")
        for f in it["fields"]:
            if f["type"] is None:
                continue
            ct = WT[f["type"]][2]
            for k in range(f["count"]):
                key = f["name"] + ("%d" % k if f["count"] > 1 else "")
                ref = f["name"] + ("[%d]" % k if f["count"] > 1 else "")
                w('    s.%s = (%s)need(m, "%s", bad);' % (ref, ct, key))
        w("    return zc_put_%s(p, &s);" % c)
        w("  }")
    w("  return -1;")
    w("}")
    w("// payload fields of id as \" key=value ...\"; ZC_OK, ZC_E_PAYLOAD, or 1 = not in the catalogue")
    w("static int zc_cat_dec(uint16_t id, const uint8_t *p, size_t n, char *out, size_t cap) {")
    w("  int o = 0, e;")
    w("  (void)o;")
    for it in cat["items"]:
        c = it["c"]
        w("  if (id == ZC_ID_%s) {" % up(c))
        w("    zc_%s_t s;" % c)
        w("    if ((e = zc_get_%s(p, n, &s))) return e;" % c)
        for f in it["fields"]:
            if f["type"] is None:
                continue
            for k in range(f["count"]):
                key = f["name"] + ("%d" % k if f["count"] > 1 else "")
                ref = "s." + f["name"] + ("[%d]" % k if f["count"] > 1 else "")
                if f["type"] == "f32":
                    w('    o += snprintf(out + o, cap - o, " %s=%%.9g", (double)%s);' % (key, ref))
                elif WT[f["type"]][3]:
                    w('    o += snprintf(out + o, cap - o, " %s=%%lld", (long long)%s);' % (key, ref))
                else:
                    w('    o += snprintf(out + o, cap - o, " %s=%%llu", (unsigned long long)%s);' % (key, ref))
        w("    return ZC_OK;")
        w("  }")
    w("  return 1;")
    w("}")
    return "\n".join(L) + "\n"


# ------------------------------------------------------------------ vectors
def flat_example(it, ex):
    kv = {"type": TYPES[it["type"]], "id": it["id"], "cls": CLASSES[it["class"]], "flags": ex.get("flags", 0),
          "seq": ex.get("seq", 0), "ts": ex.get("ts", 0)}
    for f in it["fields"]:
        if f["type"] is None:
            continue
        v = ex[f["name"]]
        if f["count"] > 1:
            for k, x in enumerate(v):
                kv["%s%d" % (f["name"], k)] = x
        else:
            kv[f["name"]] = v
    return kv


def gen_vectors(cat):
    L = ["# GENERATED by catalog/zcgen.py from the `examples` of catalogue %s v%d (hash 0x%08x)."
         % (cat["catalog"], cat["version"], cat["hash"]),
         "# Same format as zc_vectors.txt (which stays hand-written). Arrays are flattened: servo100..servo107.",
         "# Bytes by zcgen.py's own packer (struct + the YAML layout); the hand-written vectors and the old",
         "# hand-coded motor-node codecs are the independent references."]
    for it in cat["items"]:
        for i, ex in enumerate(it.get("examples", [])):
            kv = flat_example(it, ex)
            L.append("%s ex%d %s hex=%s" % (it["c"], i, " ".join("%s=%s" % (k, v) for k, v in kv.items()),
                                            encode_example(it, ex).hex()))
    return "\n".join(L) + "\n"


# ------------------------------------------------------------------ Python
def py_items(cat):
    out = {}
    for it in cat["items"]:
        out[it["id"]] = {
            "name": it["name"], "c": it["c"], "type": TYPES[it["type"]], "cls": CLASSES[it["class"]],
            "period_ms": it["period_ms"], "max_age_ms": it["max_age_ms"], "ack": it["ack"], "dir": it["dir"],
            "port": it["port"], "edge": it["edge"], "len": it["len"], "fmt": fmt_of(it),
            "fields": [{k: f[k] for k in ("name", "type", "count", "offset", "size", "scale", "unit", "enum", "bits",
                                          "invalid", "safe", "map") if f.get(k) is not None} for f in it["fields"]],
            "map": it.get("map") or {}, "flags_map": it.get("flags_map") or {}}
    return out


def gen_py(cat):
    body = pprint.pformat(py_items(cat), width=118, sort_dicts=True)
    return ('"""GENERATED by elite-t1s-hat firmware/catalog/zcgen.py from catalogue %s v%d -- do not edit.\n'
            'The tables of every ZC item (ids, struct formats, fields, enums, bits, safe values, profile maps); the\n'
            'codec on top of them is tools/zc_core.py, the Zenoh gateway tools/zcgw.py."""\n'
            "CATALOG = %r\nVERSION = %d\nHASH = 0x%08x\nPROFILE = %r\n\n"
            "ITEMS = %s\n" % (cat["catalog"], cat["version"], cat["catalog"], cat["version"], cat["hash"],
                               cat["profile"], body))


# ------------------------------------------------------------------ Lua
def lua_val(v):
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (int, float)):
        return repr(v)
    if isinstance(v, str):
        return json.dumps(v)
    if isinstance(v, list):
        return "{" + ", ".join(lua_val(x) for x in v) + "}"
    if isinstance(v, dict):
        return "{" + ", ".join("[%s] = %s" % (json.dumps(k) if isinstance(k, str) else k, lua_val(x)) for k, x in v.items()) + "}"
    return "nil"


def gen_lua(cat):
    L = ["-- GENERATED by elite-t1s-hat firmware/catalog/zcgen.py from catalogue %s v%d (hash 0x%08x) -- do not edit."
         % (cat["catalog"], cat["version"], cat["hash"]),
         "-- Read by t1s_bench.lua (dofile next to it). fields: name, type, offset, count, scale, unit, enum (raw -> label), bits.",
         "return {", "  catalog = %s, version = %d, hash = 0x%08x," % (json.dumps(cat["catalog"]), cat["version"], cat["hash"]),
         "  items = {"]
    for it in cat["items"]:
        fs = []
        for f in it["fields"]:
            if f["type"] is None:
                continue
            d = {"name": f["name"], "type": f["type"], "offset": f["offset"], "count": f["count"]}
            for k in ("scale", "unit"):
                if f.get(k) is not None:
                    d[k] = f[k]
            if f.get("enum"):
                d["enum"] = {v: k for k, v in f["enum"].items()}
            if f.get("bits"):
                d["bits"] = f["bits"]
            fs.append(d)
        L.append("    [0x%04x] = {name = %s, c = %s, type = %d, cls = %d, len = %d, period_ms = %d, max_age_ms = %d, fields = %s},"
                 % (it["id"], json.dumps(it["name"]), json.dumps(it["c"]), TYPES[it["type"]], CLASSES[it["class"]],
                    it["len"], it["period_ms"], it["max_age_ms"], lua_val(fs)))
    L += ["  },", "}"]
    return "\n".join(L) + "\n"


# ------------------------------------------------------------------ docs
def vss_key(path):
    return "vehicle/" + path[len("Vehicle."):].lower().replace(".", "/")


def gen_md(cat):
    L = ["# ZC signal catalogue `%s` v%d" % (cat["catalog"], cat["version"]), "",
         "Generated by `zcgen.py` from `%s` (+ `core.yaml`); do not edit. Catalogue hash (HEARTBEAT): **0x%08x**."
         % (os.path.basename(cat["path"]), cat["hash"]), "",
         "Every message is the 20-byte ZC header (`t1s_node/zc_core.h`) + the payload below, little-endian, packed.",
         "Profile: **%s** (gateway: Zenoh key = `vehicle/<lower-case VSS path>`). Hooks carried for: ros2, uorb, ngva." % cat["profile"],
         "", "| id | name | type | class | period / max age | dir, port | ack | payload |", "|---|---|---|---|---|---|---|---|"]
    for it in cat["items"]:
        L.append("| `0x%04X` | %s | %s | %s | %s / %d ms | %s, %s | %s | %d B |" % (
            it["id"], it["name"], it["type"].upper(), it["class"], ("%d ms" % it["period_ms"]) if it["period_ms"] else "on change",
            it["max_age_ms"], it["dir"], it["port"] or "-", "yes" if it["ack"] else "", it["len"]))
    for it in cat["items"]:
        L += ["", "## `0x%04X` %s (`%s`)" % (it["id"], it["name"], it["c"]), ""]
        if it.get("doc"):
            L += [" ".join(it["doc"].split()), ""]
        L += ["| off | size | field | type | scale / unit | values | safe | VSS |", "|---|---|---|---|---|---|---|---|"]
        for f in it["fields"]:
            if f["type"] is None:
                L.append("| %d | %d | (reserved) | 0 | | | | |" % (f["offset"], f["size"]))
                continue
            vals = []
            if f.get("enum"):
                vals.append(", ".join("%s=0x%X" % (k, v) for k, v in f["enum"].items()))
            if f.get("bits"):
                vals.append(", ".join("%s: bit %d%s" % (k, b[0], "-%d" % (b[0] + b[1] - 1) if b[1] > 1 else "")
                                      for k, b in f["bits"].items()))
            if "invalid" in f:
                vals.append("invalid 0x%X" % f["invalid"])
            vm = (f.get("map") or {}).get("vss") or {}
            if "key" in vm:
                vss = "`%s`%s" % (vss_key(vm["key"]) + ("/target" if vm.get("target") else ""), " (%s)" % vm.get("type", "") if vm.get("type") else "")
            elif "bits" in vm:
                vss = "<br>".join("%s: `%s`" % (k, vss_key(p)) for k, p in vm["bits"].items())
            elif "one_of" in vm:
                vss = "<br>".join("%s: `%s`" % (k, vss_key(p)) for k, p in vm["one_of"].items())
            else:
                vss = ""
            ty = f["type"] + ("[%d]" % f["count"] if f["count"] > 1 else "")
            su = " ".join(x for x in (("x %g" % f["scale"]) if f.get("scale") else "", f.get("unit", "")) if x)
            L.append("| %d | %d | %s | %s | %s | %s | %s | %s |" % (f["offset"], f["size"], f["name"], ty, su, "; ".join(vals),
                                                                   json.dumps(f["safe"]) if "safe" in f else "", vss))
        fm = (it.get("flags_map") or {}).get("vss")
        if fm:
            L += ["", "Header flags on Zenoh: " + ", ".join("%s → `%s`" % (k, vss_key(v)) for k, v in fm.items())]
    L += ["", "Items without a VSS mapping (and the core items) are published by the gateway as",
          "`zc/<sender>/<item name>/<field>`.", ""]
    return "\n".join(L)


# ------------------------------------------------------------------ main
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("catalog", nargs="?", default=os.path.join(HERE, "vehicle_body.yaml"))
    ap.add_argument("--c", default=os.path.join(HERE, "..", "t1s_node", "zc_catalog.h"))
    ap.add_argument("--c-test", default=os.path.join(HERE, "..", "tests", "zc_catalog_test.inc"))
    ap.add_argument("--vectors", default=os.path.join(HERE, "..", "tests", "zc_catalog_vectors.txt"))
    ap.add_argument("--md", default=os.path.join(HERE, "CATALOG.md"))
    ap.add_argument("--py", action="append", default=[])
    ap.add_argument("--lua", action="append", default=[])
    ap.add_argument("--extra-c", action="append", default=[], help="another copy of zc_catalog.h (e.g. a node's sketch folder)")
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args(argv)
    try:
        cat = load(a.catalog)
    except CatalogError as e:
        print("catalogue error: %s" % e, file=sys.stderr)
        return 2
    cat["path"] = a.catalog
    c = gen_c(cat)
    jobs = [(a.c, c), (a.c_test, gen_c_test(cat)), (a.vectors, gen_vectors(cat)), (a.md, gen_md(cat))]
    jobs += [(p, c) for p in a.extra_c] + [(p, gen_py(cat)) for p in a.py] + [(p, gen_lua(cat)) for p in a.lua]
    stale = 0
    for path, text in jobs:
        try:
            old = open(path).read()
        except OSError:
            old = None
        if old == text:
            continue
        if a.check:
            print("out of date: %s" % path)
            stale += 1
        else:
            with open(path, "w") as f:
                f.write(text)
            print("wrote %s" % os.path.relpath(path))
    print("catalogue %s v%d: %d items, hash 0x%08x" % (cat["catalog"], cat["version"], len(cat["items"]), cat["hash"]))
    return 1 if stale else 0


if __name__ == "__main__":
    sys.exit(main())
