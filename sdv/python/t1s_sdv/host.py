"""A host behind the gateway ESP: pub/sub/get/set VSS signals on the T1S bus with no T1S hardware or driver on the
host. The ESP's USB serial (macOS /dev/cu.usbmodem*, Linux /dev/ttyACM*, Windows COMx) or UDP 30500 (its RJ45, or
the T1S address from a PC on the bus, or a console that relays its serial port).

    from t1s_sdv.host import Session
    s = Session("serial:/dev/cu.usbmodem101")         # or "udp:192.168.100.65", "udp:127.0.0.1:30500" (console relay)
    s.subscribe("Vehicle.Body.Lights", lambda path, value: print(path, value))
    print(s.get("Vehicle.Speed"))                      # (value, age s) or raises
    s.set("Vehicle.Private.Zone.Brightness", 35)       # -> round trip s, raises on a refusal
    s.publish("Vehicle.Private.Test.Counter", 7)

The serial port is opened without touching DTR/RTS (pyserial's open leaves both asserted, as the OS opens the port):
the ESP32-S3's USB serial resets the chip on a DTR/RTS low/high pattern. Text the firmware prints still arrives and
goes to `on_text` (default: dropped), so the console keeps working on the same port.
"""
import socket
import struct
import threading
import time

from . import wire as w


class SdvError(Exception):
    pass


class Session:
    def __init__(self, url, on_text=None, timeout=0.5):
        self.url, self.on_text, self.timeout = url, on_text, timeout
        self.seq = 0
        self.wait = {}                 # seq -> [event, reply]
        self.listeners = []            # (ids, cb)
        self.lock = threading.Lock()
        self.stats = {"rx_frames": 0, "data": 0, "values": 0, "bad": 0}
        kind, _, rest = url.partition(":")
        if kind == "serial":
            import serial
            self.ser = serial.Serial()
            self.ser.port, self.ser.baudrate, self.ser.timeout = rest, 115200, 0.05
            self.ser.open()
            self.udp = None
            threading.Thread(target=self._rx_serial, daemon=True).start()
        elif kind == "udp":
            host, _, port = rest.partition(":")
            self.addr = (host, int(port or w.HOST_PORT))
            self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp.connect(self.addr)
            self.ser = None
            threading.Thread(target=self._rx_udp, daemon=True).start()
        else:
            raise ValueError("url: serial:<port> or udp:<host>[:port]")
        self.info = self.hello()
        threading.Thread(target=self._keepalive, daemon=True).start()

    # ------------------------------------------------------------ API
    def hello(self):
        r = self._call(0x01, b"")
        ver, h, inst, up = struct.unpack(">BHHI", r[:9])
        if h != w.HASH:
            print("t1s_sdv: the gateway's dictionary is 0x%04x, this library's 0x%04x -- regenerate (sdv/gen.py)" % (h, w.HASH))
        return {"version": ver, "dict": h, "instance": inst, "uptime_s": up / 1000}

    def subscribe(self, prefix, cb):
        """cb(path, value) for every update of a signal under `prefix` ("Vehicle" = all)"""
        ids = w.ids_for(prefix) if isinstance(prefix, str) else [w.sid(prefix)]
        if not ids:
            raise SdvError("no signal under " + prefix)
        self.listeners.append((set(ids), cb))
        self._sub_send()
        return ids

    def unsubscribe_all(self):
        self.listeners = []
        self._call(0x03, struct.pack(">H", 0))

    def publish(self, x, value):
        i = w.sid(x)
        self._send(0x04, w.values_pack([(i, value)]))

    def publish_many(self, pairs):
        self._send(0x04, w.values_pack([(w.sid(x), v) for x, v in pairs]))

    def get(self, x):
        """-> (value, age in s); raises SdvError if the gateway has none"""
        i = w.sid(x)
        r = self._call(0x05, struct.pack(">H", i))
        rc, rid, age = struct.unpack(">BHI", r[:7])
        if rc:
            raise SdvError("get %s: %s" % (w.BY_ID[i][1], w.RC.get(rc, rc)))
        return w.dec(i, r[7:]), age / 1000

    def set(self, x, value, timeout=1.0):
        """actuator request through the gateway; -> round trip in s (host -> gateway -> provider -> back)"""
        i = w.sid(x)
        t0 = time.perf_counter()
        r = self._call(0x06, struct.pack(">H", i) + w.enc(i, value), timeout)
        rc = r[0]
        if rc:
            raise SdvError("set %s: %s" % (w.BY_ID[i][1], w.RC.get(rc, "rc %d" % rc)))
        return time.perf_counter() - t0

    def ping(self):
        t0 = time.perf_counter()
        self._call(0x07, struct.pack(">I", int(t0 * 1e6) & 0xFFFFFFFF))
        return time.perf_counter() - t0

    def close(self):
        if self.ser:
            self.ser.close()
        if self.udp:
            self.udp.close()

    # ------------------------------------------------------------ internals
    def _sub_send(self):
        ids = sorted(set().union(*[l[0] for l in self.listeners])) if self.listeners else []
        if ids:
            self._call(0x02, struct.pack(">H", len(ids)) + b"".join(struct.pack(">H", i) for i in ids))

    def _keepalive(self):
        # the gateway drops a UDP host after 10 s of silence; serial hosts are kept, but renewing is harmless
        while True:
            time.sleep(4)
            try:
                if self.listeners:
                    self._sub_send()
            except (SdvError, OSError):
                pass

    def _send(self, op, body, s=None):
        if s is None:
            with self.lock:
                self.seq = (self.seq + 1) & 0xFF
                s = self.seq
        f = bytes((op, s)) + body
        if self.ser:
            self.ser.write(w.serial_frame(f))
        else:
            self.udp.send(f)
        return s

    def _call(self, op, body, timeout=None):
        ev = threading.Event()
        with self.lock:
            self.seq = (self.seq + 1) & 0xFF
            s = self.seq
            self.wait[s] = [ev, None]
        self._send(op, body, s)
        ok = ev.wait(timeout or self.timeout)
        r = self.wait.pop(s, [None, None])[1]
        if not ok or r is None:
            raise SdvError("no answer from the gateway (op 0x%02x)" % op)
        return r

    def _frame(self, f):
        if len(f) < 2:
            return
        self.stats["rx_frames"] += 1
        op, s = f[0], f[1]
        if op == 0x88:
            self.stats["data"] += 1
            for i, v in w.values_unpack(f[6:]):
                self.stats["values"] += 1
                for ids, cb in self.listeners:
                    if i in ids:
                        try:
                            cb(w.BY_ID[i][1], v)
                        except Exception as e:
                            print("t1s_sdv listener: %r" % e)
            return
        p = self.wait.get(s)
        if p and op & 0x80:
            p[1] = f[2:]
            p[0].set()

    def _rx_serial(self):
        dm = w.Demux()
        while True:
            try:
                d = self.ser.read(4096)
            except Exception:
                return
            if not d:
                continue
            lines, frames = dm.feed(d)
            for f in frames:
                self._frame(f)
            if self.on_text:
                for l in lines:
                    self.on_text(l)

    def _rx_udp(self):
        while True:
            try:
                self._frame(self.udp.recv(2048))
            except OSError:
                return
