"""Wire formats of the T1S SDV signal API (pure functions, no sockets).

SOME/IP on the bus (big-endian):
  service 0x5653, major 1; one signal per SOME/IP message, several messages per UDP datagram
  event  = 0x8000 | id   (NOTIFICATION, payload = the value)
  set    = id            (REQUEST, payload = the value -> RESPONSE, return code; E_OK = 0)
  get    = 0x4000 | id   (REQUEST, empty -> RESPONSE, payload = the value)
  event group = id >> 8 (the dictionary groups a VSS branch per high byte)
SOME/IP-SD: UDP 30490, multicast 224.224.224.245; OfferService (cyclic), SubscribeEventgroup / Ack, IPv4 endpoint
option. Notifications: unicast to each subscriber, or (a provider in multicast mode) one datagram to
224.224.224.246:30501 that every node on the bus can take, also one that cannot transmit (PLCA ID outside the count).

Host channel (a host talking to the gateway ESP, over USB serial or UDP 30500), little framing, same value bytes:
  op u8, seq u8, body
  0x01 HELLO                         -> 0x81 ver u8, dict hash u16, instance u16, uptime ms u32
  0x02 SUB  n u16, ids u16*n (0 = all) -> 0x82 rc u8
  0x03 UNSUB (same)                   -> 0x83 rc u8
  0x04 PUB  (id u16, value)*          (no answer, as a zenoh put)
  0x05 GET  id u16                    -> 0x85 rc u8, id u16, age ms u32, value
  0x06 SET  id u16, value             -> 0x86 rc u8, id u16, round trip us u32
  0x07 PING t u32                     -> 0x87 t u32, gateway us u32
  0x88 DATA (gateway -> host) ms u32, (id u16, value)*
  on USB serial a frame is 0x01, COBS(frame + CRC-16/CCITT), 0x00 -- the text console never carries 0x00 or 0x01,
  so text and frames share the port (inside a frame only 0x00 ends it: COBS code bytes may be 0x01).
"""
import struct

from .vss_dict import HASH, SIGNALS

SERVICE = 0x5653
MAJOR = 1
SD_PORT = 30490
SD_MCAST = "224.224.224.245"
EVT_PORT = 30501          # the ESP's SOME/IP endpoint; also the multicast event port
EVT_MCAST = "224.224.224.246"
HOST_PORT = 30500

FMT = {"bool": "?", "u8": "B", "i8": "b", "u16": "H", "i16": "h", "u32": "I", "i32": "i", "f32": "f"}
BY_ID = {s[0]: s for s in SIGNALS}
BY_PATH = {s[1]: s for s in SIGNALS}
RC = {0: "E_OK", 1: "E_NOT_OK", 2: "E_UNKNOWN_SERVICE", 3: "E_UNKNOWN_METHOD", 4: "E_NOT_READY", 5: "E_NOT_REACHABLE",
      6: "E_TIMEOUT", 0x20: "no value yet", 0x21: "not an actuator", 0x22: "no provider", 0xFE: "timeout"}

MT_REQUEST, MT_NOTIFICATION, MT_RESPONSE, MT_ERROR = 0x00, 0x02, 0x80, 0x81


def sid(x):
    """signal id from an id or a path"""
    if isinstance(x, int):
        return x
    return BY_PATH[x][0]


def ids_for(prefix):
    """every signal id whose path is `prefix` or below it ("" or "Vehicle" = all)"""
    p = prefix.rstrip(".")
    return [s[0] for s in SIGNALS if not p or p == "Vehicle" or s[1] == p or s[1].startswith(p + ".")]


def size(i):
    return struct.calcsize(">" + FMT[BY_ID[i][2]])


def enc(i, v):
    t = BY_ID[i][2]
    if t != "f32" and t != "bool":
        v = int(round(v))
    return struct.pack(">" + FMT[t], v)


def dec(i, b):
    return struct.unpack(">" + FMT[BY_ID[i][2]], b[:size(i)])[0]


def fmt_value(i, v):
    s = BY_ID.get(i)
    if not s:
        return repr(v)
    if s[2] == "f32":
        v = "%.2f" % v
    elif s[0] == 0x0102:
        v = {126: "P", 127: "D", 0: "N", -1: "R"}.get(v, v)
    elif s[0] == 0x0405:
        v = chr(v) if 32 < v < 127 else v
    return "%s=%s%s" % (s[1], v, (" " + s[3]) if s[3] and s[3] not in ("percent",) else "%" if s[3] == "percent" else "")


GEAR_VSS = {"P": 126, "D": 127, "N": 0, "R": -1}
GEAR_CHAR = {v: k for k, v in GEAR_VSS.items()}


# ---------------------------------------------------------------- SOME/IP
def someip(method, payload=b"", mt=MT_NOTIFICATION, client=0, session=0, rc=0, service=SERVICE):
    return struct.pack(">HHIHHBBBB", service, method, 8 + len(payload), client, session, 1, MAJOR, mt, rc) + payload


def someip_split(d):
    """-> [(service, method, client, session, msg type, rc, payload)] for every SOME/IP message in a datagram"""
    out, o = [], 0
    while o + 16 <= len(d):
        srv, m, ln, cl, ss, pv, iv, mt, rc = struct.unpack(">HHIHHBBBB", d[o:o + 16])
        if ln < 8 or o + 8 + ln > len(d):
            break
        out.append((srv, m, cl, ss, mt, rc, d[o + 16:o + 8 + ln]))
        o += 8 + ln
    return out


def event(i, v, session=0):
    return someip(0x8000 | i, enc(i, v), MT_NOTIFICATION, 0, session)


# ---------------------------------------------------------------- SOME/IP-SD
def sd_entry_service(typ, instance, ttl, minor=0, nopt=1):
    return struct.pack(">BBBBHHB", typ, 0, 0, nopt << 4, SERVICE, instance, MAJOR) + ttl.to_bytes(3, "big") + struct.pack(">I", minor)


def sd_entry_eventgroup(typ, instance, ttl, group, nopt=1):
    return struct.pack(">BBBBHHB", typ, 0, 0, nopt << 4, SERVICE, instance, MAJOR) + ttl.to_bytes(3, "big") + struct.pack(">BBH", 0, 0, group)


def sd_opt_ipv4(ip, port, proto=0x11):
    return struct.pack(">HBB4sBBH", 9, 0x04, 0, bytes(int(x) for x in ip.split(".")), 0, proto, port)


def sd_message(entries, options, session):
    """entries: list of 16-byte entries that each reference option 0 (one endpoint per message)"""
    e = b"".join(entries)
    o = b"".join(options)
    body = struct.pack(">B3xI", 0xC0, len(e)) + e + struct.pack(">I", len(o)) + o
    return someip(0x8100, body, MT_NOTIFICATION, 0, session, service=0xFFFF)


def sd_parse(payload):
    """-> [(type, instance, major, ttl, group or minor, (ip, port) or None)]"""
    out = []
    if len(payload) < 8:
        return out
    ne = struct.unpack(">I", payload[4:8])[0]
    ents = payload[8:8 + ne]
    rest = payload[8 + ne:]
    opts = []
    if len(rest) >= 4:
        no = struct.unpack(">I", rest[:4])[0]
        ob, o = rest[4:4 + no], 0
        while o + 3 <= len(ob):
            ln, typ = struct.unpack(">HB", ob[o:o + 3])
            if typ == 0x04 and ln >= 9:
                ip = ".".join(str(x) for x in ob[o + 4:o + 8])
                opts.append((ip, struct.unpack(">H", ob[o + 10:o + 12])[0]))
            else:
                opts.append(None)
            o += 3 + ln
    for k in range(0, len(ents) - 15, 16):
        e = ents[k:k + 16]
        typ, i1, i2, no, srv, inst, maj = struct.unpack(">BBBBHHB", e[:9])
        ttl = int.from_bytes(e[9:12], "big")
        last = struct.unpack(">H", e[14:16])[0] if typ in (0x06, 0x07) else struct.unpack(">I", e[12:16])[0]
        ep = opts[i1] if (no >> 4) and i1 < len(opts) else None
        if srv == SERVICE:
            out.append((typ, inst, maj, ttl, last, ep))
    return out


SD_NAMES = {0x00: "Find", 0x01: "Offer", 0x06: "Subscribe", 0x07: "SubscribeAck"}


def describe(d, sport, dport):
    """one line for a console / log: what a UDP payload carries, decoded with the dictionary; None if not ours"""
    msgs = someip_split(d)
    if not msgs:
        return None
    if msgs[0][0] == 0xFFFF and msgs[0][1] == 0x8100:
        parts = []
        for typ, inst, maj, ttl, last, ep in sd_parse(msgs[0][6]):
            n = SD_NAMES.get(typ, "0x%02x" % typ)
            if typ == 0x07 and ttl == 0:
                n = "SubscribeNack"
            if typ == 0x01 and ttl == 0:
                n = "StopOffer"
            parts.append("%s %04x.%d%s%s" % (n, SERVICE, inst, " group %d" % last if typ in (6, 7) else "",
                                             " @%s:%d" % ep if ep else ""))
        return "SD", "; ".join(parts) or "SD (other service)"
    if msgs[0][0] != SERVICE:
        return None
    parts = []
    for srv, m, cl, ss, mt, rc, pl in msgs:
        i = m & 0x3FFF
        name = BY_ID[i][1] if i in BY_ID else "0x%04x" % i
        try:
            if mt == MT_NOTIFICATION and m & 0x8000:
                parts.append(fmt_value(i, dec(i, pl)))
            elif mt == MT_REQUEST and not m & 0x4000:
                parts.append("SET " + fmt_value(i, dec(i, pl)))
            elif mt == MT_REQUEST:
                parts.append("GET " + name)
            elif mt == MT_RESPONSE and m & 0x4000 and not m & 0x8000:
                parts.append("GET-> " + fmt_value(i, dec(i, pl)))
            else:
                parts.append("%s-> %s %s" % ("SET" if not m & 0x4000 else "GET", name, RC.get(rc, rc)))
        except (struct.error, KeyError):
            parts.append("%s ?" % name)
    return "VSS", " | ".join(parts)


# ---------------------------------------------------------------- host channel
def crc16(b, c=0xFFFF):
    for x in b:
        c ^= x << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def cobs_enc(b):
    out, blk = bytearray(), bytearray()
    for x in b:
        if x == 0:
            out.append(len(blk) + 1); out += blk; blk = bytearray()
        else:
            blk.append(x)
            if len(blk) == 254:
                out.append(255); out += blk; blk = bytearray()
    out.append(len(blk) + 1); out += blk
    return bytes(out)


def cobs_dec(b):
    out, i = bytearray(), 0
    while i < len(b):
        c = b[i]
        if c == 0 or i + c > len(b):
            raise ValueError("cobs")
        out += b[i + 1:i + c]
        i += c
        if c < 255 and i < len(b):
            out.append(0)
    return bytes(out)


def serial_frame(f):
    return b"\x01" + cobs_enc(f + struct.pack(">H", crc16(f))) + b"\x00"


def serial_unframe(body):
    """body = the bytes between 0x01 and 0x00 -> frame, or None if the CRC fails"""
    try:
        d = cobs_dec(body)
    except ValueError:
        return None
    if len(d) < 4 or crc16(d[:-2]) != struct.unpack(">H", d[-2:])[0]:
        return None
    return d[:-2]


class Demux:
    """splits a serial byte stream into text lines and host frames"""

    def __init__(self):
        self.text, self.frame, self.in_frame = bytearray(), bytearray(), False

    def feed(self, data):
        """-> (lines, frames)"""
        lines, frames = [], []
        for x in data:
            if self.in_frame:
                if x == 0:
                    f = serial_unframe(bytes(self.frame))
                    if f:
                        frames.append(f)
                    self.in_frame = False
                elif len(self.frame) > 1100:     # no end seen: the frame was cut, drop it
                    self.in_frame = False
                else:
                    self.frame.append(x)
            elif x == 1:
                self.in_frame, self.frame = True, bytearray()
            elif x == 10:
                lines.append(bytes(self.text).decode(errors="replace").rstrip("\r"))
                self.text = bytearray()
            elif x:
                self.text.append(x)
        return lines, frames


def values_pack(pairs):
    return b"".join(struct.pack(">H", i) + enc(i, v) for i, v in pairs)


def values_unpack(b):
    out, o = [], 0
    while o + 2 <= len(b):
        i = struct.unpack(">H", b[o:o + 2])[0]
        if i not in BY_ID:
            break
        n = size(i)
        out.append((i, dec(i, b[o + 2:o + 2 + n])))
        o += 2 + n
    return out


def describe_host(f):
    op = f[0]
    names = {1: "HELLO", 0x81: "HELLO>", 2: "SUB", 0x82: "SUB>", 3: "UNSUB", 4: "PUB", 5: "GET", 0x85: "GET>",
             6: "SET", 0x86: "SET>", 7: "PING", 0x87: "PONG", 0x88: "DATA"}
    s = names.get(op, "op 0x%02x" % op)
    if op in (4, 0x88):
        s += " " + " ".join(fmt_value(i, v) for i, v in values_unpack(f[2 + (4 if op == 0x88 else 0):]))
    return s


__all__ = [n for n in dir() if not n.startswith("_")] + ["HASH", "SIGNALS"]
