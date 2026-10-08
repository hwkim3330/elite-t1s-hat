"""A SOME/IP VSS node on an IP interface (a Linux PC on the T1S bus through a USB T1S adapter, or any LAN): offers
its signals, subscribes to the other nodes' event groups, publishes, answers get/set. Same wire as the ESP gateway.

    from t1s_sdv.node import Node
    n = Node("192.168.100.70", instance=2, port=30502, multicast=True)
    n.subscribe("Vehicle.Private.Zone", lambda i, v, src: print(i, v))
    n.publish("Vehicle.Speed", 42.0)
    n.on_set("Vehicle.Powertrain.Transmission.SelectedGear", lambda v: 0)     # rc 0 = done
    n.set("Vehicle.Private.Zone.Brightness", 35)                              # -> (rc, seconds)

SD: offers go to the multicast group from an unicast socket of its own; the offer's endpoint option is the event
port. Subscribe/ack are answered to the sender's address and port (so two nodes can share one PC address: each
has its own SD socket). Notifications: unicast to each subscriber, or with multicast=True one datagram to
224.224.224.246:30501 (a node that cannot transmit, e.g. outside the PLCA node count, still gets them)."""
import socket
import struct
import threading
import time

from . import wire as w


class Node:
    def __init__(self, ip, instance, port, client=None, multicast=False, cycle=None, offer_every=1.0, ttl=3):
        self.ip, self.instance, self.port, self.multicast = ip, instance, port, multicast
        self.client = client if client is not None else int(ip.split(".")[-1]) | 0x0100
        self.cycle, self.offer_every, self.ttl = cycle, offer_every, ttl
        self.lock = threading.Lock()
        self.values = {}            # id -> (value, time, src)
        self.own = {}               # id -> value, the signals this node publishes
        self.subs_in = {}           # (ip, port) -> {group: expiry}   who subscribed to us
        self.offers = {}            # instance -> (sd addr, evt endpoint, expiry)
        self.listeners = []         # (ids set, callback)
        self.setters = {}           # id -> handler(value) -> rc
        self.pending = {}           # session -> [event, reply]
        self.session = 0
        self.stats = {"rx_events": 0, "tx_events": 0, "tx_datagrams": 0, "sd_rx": 0, "subscribers": 0}
        self.want_groups = set()
        # sockets
        self.sd_mc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sd_mc.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sd_mc.bind(("", w.SD_PORT))
        self.sd_mc.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(w.SD_MCAST) + socket.inet_aton(ip))
        self.sd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sd.bind((ip, 0))
        self.sd.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(ip))
        self.sd.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)
        self.ev = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ev.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.ev.bind(("", port))
        self.ev.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(ip))
        self.ev.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 0)
        if port == w.EVT_PORT:
            self.ev.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(w.EVT_MCAST) + socket.inet_aton(ip))
        self.mc_rx = None
        if port != w.EVT_PORT:      # multicast events from other providers arrive on 30501
            self.mc_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.mc_rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.mc_rx.bind((w.EVT_MCAST, w.EVT_PORT))
            self.mc_rx.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton(w.EVT_MCAST) + socket.inet_aton(ip))
        self.running = True
        for f, s in ((self._sd_rx, self.sd_mc), (self._sd_rx, self.sd), (self._ev_rx, self.ev)) + \
                ((((self._ev_rx, self.mc_rx),) if self.mc_rx else ())):
            threading.Thread(target=f, args=(s,), daemon=True).start()
        threading.Thread(target=self._timer, daemon=True).start()

    # ------------------------------------------------------------ API
    def publish(self, x, value, flush=True):
        i = w.sid(x)
        with self.lock:
            self.own[i] = value
            self.values[i] = (value, time.time(), "self")
        if flush:
            self._send_events([(i, value)])

    def publish_many(self, pairs):
        pairs = [(w.sid(x), v) for x, v in pairs]
        now = time.time()
        with self.lock:
            for i, v in pairs:
                self.own[i] = v
                self.values[i] = (v, now, "self")
        self._send_events(pairs)

    def subscribe(self, prefix, cb):
        ids = set(w.ids_for(prefix) if isinstance(prefix, str) else [w.sid(prefix)])
        self.listeners.append((ids, cb))
        self.want_groups |= {i >> 8 for i in ids}
        self._subscribe_all()
        return ids

    def on_set(self, x, handler):
        self.setters[w.sid(x)] = handler

    def latest(self, x):
        return self.values.get(w.sid(x))

    def get(self, x, timeout=0.3):
        i = w.sid(x)
        r = self._request(i, 0x4000 | i, b"", timeout)
        if r is None:
            return 0xFE, None
        rc, pl = r
        return rc, (w.dec(i, pl) if rc == 0 and pl else None)

    def set(self, x, value, timeout=0.3, retries=2):
        i = w.sid(x)
        t0 = time.perf_counter()
        for _ in range(retries + 1):
            r = self._request(i, i, w.enc(i, value), timeout)
            if r is not None:
                return r[0], time.perf_counter() - t0
        return 0xFE, time.perf_counter() - t0

    def close(self):
        self.running = False
        try:
            self.sd.sendto(w.sd_message([w.sd_entry_service(0x01, self.instance, 0)], [w.sd_opt_ipv4(self.ip, self.port)], self._sess()),
                           (w.SD_MCAST, w.SD_PORT))
        except OSError:
            pass
        for s in (self.sd_mc, self.sd, self.ev, self.mc_rx):
            if s:
                s.close()

    # ------------------------------------------------------------ internals
    def _sess(self):
        self.session = self.session % 0xFFFF + 1
        return self.session

    def _provider(self, i):
        v = self.values.get(i)
        if v and isinstance(v[2], tuple):
            return v[2]
        now = time.time()
        for inst, (sd, ep, exp) in self.offers.items():
            if exp > now and ep:
                return ep
        return None

    def _request(self, i, method, payload, timeout):
        dst = self._provider(i)
        if not dst:
            return None
        s = self._sess()
        ev = threading.Event()
        self.pending[s] = [ev, None]
        self.ev.sendto(w.someip(method, payload, w.MT_REQUEST, self.client, s), dst)
        ev.wait(timeout)
        return self.pending.pop(s, [None, None])[1]

    def _send_events(self, pairs):
        if not pairs:
            return
        now = time.time()
        if self.multicast:
            dsts = {(w.EVT_MCAST, w.EVT_PORT): None}
        else:
            dsts = {a: g for a, g in self.subs_in.items() if any(e > now for e in g.values())}
        for a, g in dsts.items():
            d = b"".join(w.event(i, v, self._sess()) for i, v in pairs if g is None or g.get(i >> 8, 0) > now)
            if d:
                self._sendto_split(d, a)

    def _sendto_split(self, d, a):
        """whole SOME/IP messages, at most ~1400 B per datagram"""
        msgs, o, chunk = [], 0, b""
        while o < len(d):
            ln = struct.unpack(">I", d[o + 4:o + 8])[0] + 8
            m = d[o:o + ln]
            if len(chunk) + len(m) > 1400:
                msgs.append(chunk)
                chunk = b""
            chunk += m
            o += ln
        msgs.append(chunk)
        for c in msgs:
            try:
                self.ev.sendto(c, a)
                self.stats["tx_datagrams"] += 1
                self.stats["tx_events"] += len(w.someip_split(c))
            except OSError:
                pass

    def _offer(self):
        groups = {i >> 8 for i in self.own} | {i >> 8 for i in self.setters}
        if not groups and not self.own:
            return
        self.sd.sendto(w.sd_message([w.sd_entry_service(0x01, self.instance, self.ttl)], [w.sd_opt_ipv4(self.ip, self.port)], self._sess()),
                       (w.SD_MCAST, w.SD_PORT))

    def _subscribe_to(self, inst, sd_addr):
        if not self.want_groups:
            return
        ents = [w.sd_entry_eventgroup(0x06, inst, self.ttl, g) for g in sorted(self.want_groups)]
        self.sd.sendto(w.sd_message(ents, [w.sd_opt_ipv4(self.ip, self.port)], self._sess()), sd_addr)

    def _subscribe_all(self):
        now = time.time()
        for inst, (sd, ep, exp) in list(self.offers.items()):
            if exp > now:
                self._subscribe_to(inst, sd)

    def _timer(self):
        t_offer = t_cycle = 0.0
        while self.running:
            time.sleep(0.01)
            now = time.time()
            if now - t_offer >= self.offer_every:
                t_offer = now
                try:
                    self._offer()
                except OSError:
                    pass
                self.stats["subscribers"] = sum(1 for g in self.subs_in.values() if any(e > now for e in g.values()))
            if self.cycle and now - t_cycle >= self.cycle:
                t_cycle = now
                with self.lock:
                    pairs = list(self.own.items())
                self._send_events(pairs)

    def _sd_rx(self, s):
        while self.running:
            try:
                d, a = s.recvfrom(1500)
            except OSError:
                return
            if a[0] == self.ip and a[1] == self.sd.getsockname()[1]:
                continue
            msgs = w.someip_split(d)
            if not msgs or msgs[0][0] != 0xFFFF:
                continue
            self.stats["sd_rx"] += 1
            acks = []
            for typ, inst, maj, ttl, last, ep in w.sd_parse(msgs[0][6]):
                if typ == 0x01 and inst != self.instance:
                    if ttl == 0:
                        self.offers.pop(inst, None)
                        continue
                    self.offers[inst] = (a, ep, time.time() + ttl)
                    self._subscribe_to(inst, a)     # renewed with every offer (TTL 3 s, offers every second)
                elif typ == 0x06 and inst == self.instance and ep:
                    g = self.subs_in.setdefault(ep, {})
                    first = g.get(last, 0) < time.time()
                    if ttl:
                        g[last] = time.time() + ttl
                    else:
                        g.pop(last, None)
                    acks.append(w.sd_entry_eventgroup(0x07, inst, ttl, last, nopt=0))
                    if ttl and first:         # initial values of that group (fields)
                        with self.lock:
                            init = [(i, v) for i, v in self.own.items() if i >> 8 == last]
                        if init and not self.multicast:
                            self._sendto_split(b"".join(w.event(i, v, self._sess()) for i, v in init), ep)
            if acks:
                s2 = struct.pack(">B3xI", 0xC0, len(b"".join(acks))) + b"".join(acks) + struct.pack(">I", 0)
                self.sd.sendto(w.someip(0x8100, s2, w.MT_NOTIFICATION, 0, self._sess(), service=0xFFFF), a)

    def _ev_rx(self, s):
        while self.running:
            try:
                d, a = s.recvfrom(1500)
            except OSError:
                return
            if a[0] == self.ip:
                continue
            for srv, m, cl, ss, mt, rc, pl in w.someip_split(d):
                if srv != w.SERVICE:
                    continue
                i = m & 0x3FFF
                if mt == w.MT_NOTIFICATION and m & 0x8000 and i in w.BY_ID:
                    try:
                        v = w.dec(i, pl)
                    except struct.error:
                        continue
                    self.stats["rx_events"] += 1
                    src = (a[0], w.EVT_PORT) if a[1] == w.EVT_PORT else a
                    self.values[i] = (v, time.time(), src)
                    for ids, cb in self.listeners:
                        if i in ids:
                            try:
                                cb(i, v, a)
                            except Exception as e:      # a listener's bug must not stop the receiver
                                print("t1s_sdv listener: %r" % e)
                elif mt in (w.MT_RESPONSE, w.MT_ERROR) and ss in self.pending:
                    p = self.pending[ss]
                    p[1] = (rc, pl)
                    p[0].set()
                elif mt == w.MT_REQUEST and i in w.BY_ID:
                    if m & 0x4000:          # get
                        v = self.values.get(i)
                        out = w.someip(m, w.enc(i, v[0]) if v else b"", w.MT_RESPONSE if v else w.MT_ERROR, cl, ss, 0 if v else 0x20)
                    else:                   # set
                        h = self.setters.get(i)
                        if not h:
                            rc2 = 0x21 if w.BY_ID[i][4] != "a" else 0x22
                        else:
                            try:
                                rc2 = h(w.dec(i, pl)) or 0
                            except Exception:
                                rc2 = 1
                        out = w.someip(m, b"", w.MT_RESPONSE if rc2 == 0 else w.MT_ERROR, cl, ss, rc2)
                    s.sendto(out, a) if s is self.ev else self.ev.sendto(out, a)
